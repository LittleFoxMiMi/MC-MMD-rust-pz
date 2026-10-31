//! MMD 关节（约束）封装
//!
//! 移植自 babylon-mmd 的关节构建逻辑。
//! 使用 Bullet3 btGeneric6DofSpringConstraint。

use glam::{Mat4, Quat, Vec3};

use mmd::pmx::joint::Joint as PmxJoint;

use super::bullet_ffi::{BulletConstraint, BulletRigidBody, BT_CONSTRAINT_STOP_ERP};

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
