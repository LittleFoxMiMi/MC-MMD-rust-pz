//! MMD 关节（约束）封装
//!
//! 移植自 babylon-mmd 的关节构建逻辑。
//! 使用 Bullet3 btGeneric6DofSpringConstraint。

use glam::{Mat4, Quat, Vec3};

use mmd::pmx::joint::Joint as PmxJoint;
use mmd::pmx::rigid_body::{RigidBody, RigidBodyMode};
use std::collections::HashSet;

use super::bullet_ffi::{BulletConstraint, BulletRigidBody, BT_CONSTRAINT_STOP_ERP};

/// Recognize the RGBA six-body/nine-joint mechanism without model or bone names.
/// A kinematic anchor drives four unbound helpers and one bound dynamic output.
/// Keep the exception local: ordinary hair/accessory chains need frame offsets.
pub(crate) fn legacy_frame_joints(bodies: &[RigidBody], joints: &[PmxJoint]) -> HashSet<usize> {
    let zero_spring = |j: &PmxJoint| j.position_spring.iter().chain(&j.rotation_spring)
        .all(|&v| v == 0.0);
    let mut adjacency = vec![HashSet::new(); bodies.len()];
    for j in joints.iter().filter(|j| zero_spring(j)) {
        let (a, b) = (j.rigid_body_a_index as usize, j.rigid_body_b_index as usize);
        if a < bodies.len() && b < bodies.len() && a != b {
            adjacency[a].insert(b); adjacency[b].insert(a);
        }
    }
    let mut selected = HashSet::new();
    for (anchor, body) in bodies.iter().enumerate() {
        if body.mode != RigidBodyMode::Static { continue; }
        let helpers: Vec<_> = adjacency[anchor].iter().copied().filter(|&i|
            bodies[i].bone_index < 0 && bodies[i].mode != RigidBodyMode::Static).collect();
        if helpers.len() != 4 || helpers.iter().filter(|&&i|
            bodies[i].mode == RigidBodyMode::DynamicWithBonePosition).count() != 1 { continue; }
        for &output in &adjacency[anchor] {
            if bodies[output].bone_index < 0 || bodies[output].mode != RigidBodyMode::Dynamic { continue; }
            let mut members = HashSet::from([anchor, output]);
            members.extend(helpers.iter().copied());
            let internal: Vec<_> = joints.iter().enumerate().filter(|(_, j)|
                members.contains(&(j.rigid_body_a_index as usize)) &&
                members.contains(&(j.rigid_body_b_index as usize))).collect();
            if internal.len() != 9 || internal.iter().any(|(_, j)| !zero_spring(j) ||
                j.type_ != mmd::pmx::joint::JointType::Spring6DOF) { continue; }
            let mut degrees = vec![0; bodies.len()];
            let mut pairs = HashSet::new();
            for (_, j) in &internal {
                let (a,b) = (j.rigid_body_a_index as usize, j.rigid_body_b_index as usize);
                pairs.insert((a.min(b),a.max(b))); degrees[a]+=1; degrees[b]+=1;
            }
            let mut helper_degrees: Vec<_> = helpers.iter().map(|&i| degrees[i]).collect();
            helper_degrees.sort_unstable();
            if pairs.len() == 9 && degrees[anchor] == 5 && degrees[output] == 4 &&
                helper_degrees == [2,2,2,3] && helpers.iter().any(|&i|
                    bodies[i].mode == RigidBodyMode::DynamicWithBonePosition && degrees[i] == 3) {
                selected.extend(internal.iter().map(|(i,_)| *i));
            }
        }
    }
    selected
}

fn pmx_joint_rotation(rotation: [f32; 3]) -> Quat {
    // Bullet setEulerZYX(x,y,z) applies X, then Y, then Z to a vector.
    // glam's intrinsic ZYX takes its arguments in Z,Y,X order.
    Quat::from_euler(glam::EulerRot::ZYX, rotation[2], rotation[1], rotation[0])
}

/// MMD 关节数据
pub struct MmdJointData {
    /// 关节名称
    pub name: String,
    /// 刚体 A 索引
    pub rigid_body_a_index: i32,
    /// 刚体 B 索引
    pub rigid_body_b_index: i32,
    /// Bullet3 约束
    pub constraint: Option<BulletConstraint>,
}

impl MmdJointData {
    /// 从 PMX 关节数据创建 Bullet3 6DOF 弹簧约束
    ///
    /// 移植自 babylon-mmd buildPhysics() 关节部分。
    /// 欧拉角与 saba btMatrix3x3::setEulerZYX 一致（Rz * Ry * Rx）。
    pub fn from_pmx(
        pmx_joint: &PmxJoint,
        rb_a: &BulletRigidBody,
        rb_b: &BulletRigidBody,
        rb_a_initial_transform: Mat4,
        rb_b_initial_transform: Mat4,
    ) -> Self {
        let position = Vec3::new(
            pmx_joint.position[0],
            pmx_joint.position[1],
            pmx_joint.position[2],
        );

        let rotation = pmx_joint_rotation(pmx_joint.rotation);

        let joint_transform = Mat4::from_rotation_translation(rotation, position);

        // frameA = rbA_initialTransform.inverse() * jointTransform
        // frameB = rbB_initialTransform.inverse() * jointTransform
        let frame_a = rb_a_initial_transform.inverse() * joint_transform;
        let frame_b = rb_b_initial_transform.inverse() * joint_transform;

        // 创建 Bullet3 6DOF 弹簧约束
        let constraint = BulletConstraint::new_6dof_spring(rb_a, rb_b, frame_a, frame_b, true);

        // 配置约束参数（仅在创建成功时）
        if let Some(ref c) = constraint {
            c.use_frame_offset(true);
            for axis in 0..6 {
                c.set_param(BT_CONSTRAINT_STOP_ERP, 0.475, axis);
            }

            c.set_linear_lower_limit(
                pmx_joint.position_min[0],
                pmx_joint.position_min[1],
                pmx_joint.position_min[2],
            );
            c.set_linear_upper_limit(
                pmx_joint.position_max[0],
                pmx_joint.position_max[1],
                pmx_joint.position_max[2],
            );
            c.set_angular_lower_limit(
                pmx_joint.rotation_min[0],
                pmx_joint.rotation_min[1],
                pmx_joint.rotation_min[2],
            );
            c.set_angular_upper_limit(
                pmx_joint.rotation_max[0],
                pmx_joint.rotation_max[1],
                pmx_joint.rotation_max[2],
            );

            for i in 0..3 {
                let stiffness = pmx_joint.position_spring[i];
                if stiffness.abs() > 1e-6 {
                    c.set_stiffness(i as i32, stiffness);
                    c.enable_spring(i as i32, true);
                } else {
                    c.enable_spring(i as i32, false);
                }
            }

            for i in 0..3 {
                let stiffness = pmx_joint.rotation_spring[i];
                c.set_stiffness(i as i32 + 3, stiffness);
                c.enable_spring(i as i32 + 3, true);
            }
        }

        Self {
            name: pmx_joint.local_name.clone(),
            rigid_body_a_index: pmx_joint.rigid_body_a_index,
            rigid_body_b_index: pmx_joint.rigid_body_b_index,
            constraint,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mechanism() -> (Vec<RigidBody>, Vec<PmxJoint>) {
        use mmd::pmx::{rigid_body::RigidBodyShape, joint::JointType};
        let bodies = (0..7).map(|i| RigidBody {
            local_name: format!("anonymous-{i}"), universal_name: String::new(),
            bone_index: if i == 0 || i >= 5 { i } else { -1 },
            group: 0, un_collision_group_flag: 0, shape: RigidBodyShape::Sphere,
            size: [0.3,0.0,0.0], position: [0.0;3], rotation: [0.0;3], mass: 1.0,
            move_attenuation: 0.9, rotation_attenuation: 0.9, repulsion: 0.0, friction: 0.5,
            mode: if i == 0 { RigidBodyMode::Static } else if i == 2 {
                RigidBodyMode::DynamicWithBonePosition } else { RigidBodyMode::Dynamic },
        }).collect();
        let joints = [(0,1),(1,5),(0,2),(0,3),(3,2),(2,5),(0,4),(4,5),(0,5),
            // Ordinary bound accessory attached to the same anchor stays modern.
            (0,6)].iter().map(|&(a,b)| PmxJoint {
            local_name: String::new(), universal_name: String::new(), type_: JointType::Spring6DOF,
            rigid_body_a_index: a, rigid_body_b_index: b,
            position: [0.0;3], rotation: [0.0;3], position_min: [0.0;3], position_max: [0.0;3],
            rotation_min: [-0.5;3], rotation_max: [0.5;3],
            position_spring: [0.0;3], rotation_spring: [0.0;3],
        }).collect();
        (bodies,joints)
    }

    #[test]
    fn rgba_exception_is_local_and_independent_of_names_order_and_edge_direction() {
        let (mut bodies, mut joints) = mechanism();
        assert_eq!(legacy_frame_joints(&bodies,&joints), (0..9).collect());
        let count = bodies.len(); bodies.reverse(); joints.reverse();
        for j in &mut joints {
            let a = count as i32 - 1 - j.rigid_body_a_index;
            j.rigid_body_a_index = count as i32 - 1 - j.rigid_body_b_index;
            j.rigid_body_b_index = a;
        }
        assert_eq!(legacy_frame_joints(&bodies,&joints), (1..10).collect());
    }

    #[test]
    fn ordinary_or_incomplete_network_does_not_get_legacy_frames() {
        let (bodies,joints) = mechanism();
        let mut spring = joints.clone(); spring[0].rotation_spring[0] = 1.0;
        assert!(legacy_frame_joints(&bodies,&spring).is_empty());
        let mut missing = joints.clone(); missing.remove(4);
        assert!(legacy_frame_joints(&bodies,&missing).is_empty());
        let mut bound = bodies.clone(); bound[1].bone_index = 10;
        assert!(legacy_frame_joints(&bound,&joints).is_empty());
        let mut chain = joints.clone(); chain[4].rigid_body_b_index = 1;
        assert!(legacy_frame_joints(&bodies,&chain).is_empty());
    }

    #[test]
    fn joint_axes_match_bullet_set_euler_zyx() {
        for [x, y, z] in [[0.3_f32, -0.6, 0.8], [-1.570451, -3.1413395, 3.1413395]] {
            let (sx, cx) = x.sin_cos();
            let (sy, cy) = y.sin_cos();
            let (sz, cz) = z.sin_cos();
            // Columns of Bullet's explicit btMatrix3x3::setEulerZYX formula.
            let expected = glam::Mat3::from_cols(
                Vec3::new(cy * cz, cy * sz, -sy),
                Vec3::new(sy * sx * cz - cx * sz, sy * sx * sz + cx * cz, cy * sx),
                Vec3::new(sy * cx * cz + sx * sz, sy * cx * sz - sx * cz, cy * cx),
            );
            let actual = glam::Mat3::from_quat(pmx_joint_rotation([x, y, z]));
            assert!(actual.abs_diff_eq(expected, 1e-6));
        }
    }
}
