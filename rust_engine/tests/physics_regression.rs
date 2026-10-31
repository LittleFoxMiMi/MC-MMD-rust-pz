use glam::{Mat4, Quat, Vec3};
use mmd_engine::physics::bullet_ffi::{BulletRigidBody, BulletShape, BulletWorld, RigidBodyInfo};
use mmd_engine::skeleton::{BoneLink, BoneSet};
use std::collections::HashSet;

fn info(kinematic: bool, position: Vec3) -> RigidBodyInfo {
    RigidBodyInfo {
        mass: if kinematic { 0.0 } else { 1.0 },
        linear_damping: 0.0,
        angular_damping: 0.0,
        friction: 0.5,
        restitution: 0.0,
        additional_damping: false,
        is_kinematic: kinematic,
        disable_deactivation: true,
        no_contact_response: false,
        initial_transform: Mat4::from_translation(position),
    }
}

fn falling_ball(filter: bool, mask: i32) -> f32 {
    let world = BulletWorld::new(0.0, -10.0, 0.0).unwrap();
    let box_shape = BulletShape::r#box(5.0, 0.5, 5.0).unwrap();
    let ball_shape = BulletShape::sphere(0.5).unwrap();
    let floor = BulletRigidBody::new(&info(true, Vec3::ZERO), &box_shape).unwrap();
    let ball = BulletRigidBody::new(&info(false, Vec3::new(0.0, 3.0, 0.0)), &ball_shape).unwrap();
    world.set_kinematic_filter(filter);
    world.add_rigid_body(&floor, 1, 0xffff);
    world.add_rigid_body(&ball, 2, mask);
    for _ in 0..120 {
        world.step(1.0 / 60.0, 1, 1.0 / 60.0);
    }
    let y = ball.get_position().y;
    world.remove_rigid_body(&ball);
    world.remove_rigid_body(&floor);
    y
}

#[test]
fn body_collision_respects_pmx_masks() {
    let supported = falling_ball(false, 0xffff);
    let filtered = falling_ball(true, 0xffff);
    let masked = falling_ball(false, 0xfffe);
    assert!(
        (supported - 1.0).abs() < 0.02,
        "body collider missed: {supported}"
    );
    assert!(
        filtered < -10.0 && masked < -10.0,
        "filter/mask ignored: {filtered}, {masked}"
    );
}

#[test]
fn reset_clears_interpolated_pose_and_velocity() {
    let world = BulletWorld::new(0.0, 0.0, 0.0).unwrap();
    let shape = BulletShape::sphere(0.5).unwrap();
    let ball = BulletRigidBody::new(&info(false, Vec3::ZERO), &shape).unwrap();
    world.add_rigid_body(&ball, 1, 0xffff);
    ball.set_linear_velocity(10.0, 0.0, 0.0);
    for _ in 0..10 {
        world.step(1.0 / 60.0, 1, 1.0 / 60.0);
    }
    let target = Vec3::new(20.0, 4.0, -3.0);
    ball.reset_transform(Mat4::from_translation(target));
    world.step(0.0, 1, 1.0 / 60.0);
    assert!((ball.get_position() - target).length() < 1e-5);
    world.step(1.0 / 60.0, 1, 1.0 / 60.0);
    assert!((ball.get_position() - target).length() < 1e-5);
    world.remove_rigid_body(&ball);
}

#[test]
fn mode_two_follows_updated_parent_but_mode_one_keeps_world_position() {
    let mut bones = BoneSet::new();
    // Deliberately place child before parent, with reversed deform levels.
    for (name, parent, position, level) in [
        ("mode2-child", 2, Vec3::new(0.0, 2.0, 0.0), 0),
        ("mode1-child", 2, Vec3::new(1.0, 1.0, 0.0), 0),
        ("mode2-parent", 3, Vec3::Y, 5),
        ("root", -1, Vec3::ZERO, 10),
        ("plain-child", 0, Vec3::new(0.0, 3.0, 0.0), 0),
    ] {
        let mut b = BoneLink::new(name.into());
        b.parent_index = parent;
        b.initial_position = position;
        b.transform_level = level;
        bones.add_bone(b);
    }
    bones.build_hierarchy();
    bones.set_physics_bone_indices(&HashSet::from([0, 1, 2]));
    let parent = Mat4::from_rotation_translation(Quat::from_rotation_z(0.7), Vec3::Y);
    let child =
        Mat4::from_rotation_translation(Quat::from_rotation_x(0.3), Vec3::new(99.0, 99.0, 99.0));
    let free = Mat4::from_translation(Vec3::new(4.0, 5.0, 6.0));
    bones.apply_physics_transforms(
        &[(0, child), (1, free), (2, parent)],
        &HashSet::from([0, 2]),
    );
    let expected = parent.transform_point3(Vec3::Y);
    assert!((bones.get_global_transform(0).w_axis.truncate() - expected).length() < 1e-5);
    assert!(
        (bones.get_global_transform(1).w_axis.truncate() - free.w_axis.truncate()).length() < 1e-5
    );
    let plain = bones.get_global_transform(0).transform_point3(Vec3::Y);
    assert!((bones.get_global_transform(4).w_axis.truncate() - plain).length() < 1e-5);
    let local = bones.link(0).unwrap().local_to_parent;
    assert!((local.w_axis.truncate() - Vec3::Y).length() < 1e-5);
    assert!((parent * local - child).x_axis.length() < 1e-5);
}

#[test]
fn movement_inertia_responds_to_acceleration_and_stop_at_any_frame_rate() {
    use mmd::pmx::rigid_body::{RigidBody, RigidBodyMode, RigidBodyShape};
    use mmd_engine::physics::{MMDPhysics, PhysicsConfig};
    let rigid = RigidBody {
        local_name: "inertia probe".into(),
        universal_name: String::new(),
        bone_index: 0,
        group: 0,
        un_collision_group_flag: 0,
        shape: RigidBodyShape::Sphere,
        size: [0.5, 0.0, 0.0],
        position: [0.0; 3],
        rotation: [0.0; 3],
        mass: 1.0,
        move_attenuation: 0.0,
        rotation_attenuation: 0.0,
        repulsion: 0.0,
        friction: 0.0,
        mode: RigidBodyMode::Dynamic,
    };
    let config = PhysicsConfig {
        gravity_y: 0.0,
        inertia_strength: 1.0,
        ..PhysicsConfig::default()
    };
    for fps in [30, 60, 120] {
        let mut physics = MMDPhysics::new().unwrap();
        physics.build_physics(&[rigid.clone()], &[], &[Mat4::IDENTITY]);
        physics.set_gravity(0.0, 0.0, 0.0);
        physics.sync_bodies_with_model_velocity(&[Mat4::IDENTITY], 0.0, Mat4::IDENTITY, &config);
        let dt = 1.0 / fps as f32;
        for frame in 1..=fps {
            let position = Vec3::new(frame as f32 * dt, 0.0, 0.0);
            physics.sync_bodies_with_model_velocity(
                &[Mat4::IDENTITY],
                dt,
                Mat4::from_translation(position),
                &config,
            );
            physics.step_simulation(dt, &config);
            let velocity = physics.rigid_bodies[0]
                .bullet_body
                .as_ref()
                .unwrap()
                .get_linear_velocity();
            assert!(
                (velocity.x + 1.0).abs() < 1e-4,
                "constant-speed drag at {fps} FPS: {velocity}"
            );
        }
        physics.sync_bodies_with_model_velocity(
            &[Mat4::IDENTITY],
            dt,
            Mat4::from_translation(Vec3::X),
            &config,
        );
        physics.step_simulation(dt, &config);
        let velocity = physics.rigid_bodies[0]
            .bullet_body
            .as_ref()
            .unwrap()
            .get_linear_velocity();
        assert!(
            velocity.length() < 1e-4,
            "stop response at {fps} FPS: {velocity}"
        );
    }
}

#[test]
fn weapon_kinematic_velocity_and_dynamic_travel_follow_render_time() {
    use mmd::pmx::rigid_body::{RigidBody, RigidBodyMode, RigidBodyShape};
    use mmd_engine::physics::{MMDPhysics, PhysicsConfig};
    let anchor = RigidBody {
        local_name: "moving weapon anchor".into(),
        universal_name: String::new(), bone_index: 0, group: 0,
        un_collision_group_flag: 0xffff, shape: RigidBodyShape::Sphere,
        size: [0.1, 0.0, 0.0], position: [0.0; 3], rotation: [0.0; 3],
        mass: 0.0, move_attenuation: 0.0, rotation_attenuation: 0.0,
        repulsion: 0.0, friction: 0.0, mode: RigidBodyMode::Static,
    };
    let mut dynamic = anchor.clone();
    dynamic.bone_index = -1;dynamic.position = [0.0, 10.0, 0.0];
    dynamic.mass = 1.0;dynamic.mode = RigidBodyMode::Dynamic;
    let config = PhysicsConfig { gravity_y: 0.0, ..PhysicsConfig::default() };
    for fps in [30, 60, 100, 144, 240] {
        let mut physics = MMDPhysics::new().unwrap();
        physics.build_physics(&[anchor.clone(), dynamic.clone()], &[], &[Mat4::IDENTITY]);
        physics.set_gravity(0.0, 0.0, 0.0);
        physics.simulate_weapon(&[Mat4::IDENTITY], Mat4::IDENTITY, 0.0, &config);
        let speed = 50.0;
        physics.rigid_bodies[1].bullet_body.as_ref().unwrap().set_linear_velocity(speed, 0.0, 0.0);
        for tick in 1..=fps*2 {
            let dt = 1.0 / fps as f32;
            let position = speed * tick as f32 * dt;
            let frame = Mat4::from_translation(Vec3::new(position, 0.0, 0.0));
            physics.simulate_weapon(&[Mat4::IDENTITY], frame, dt, &config);
            let anchor_body = physics.rigid_bodies[0].bullet_body.as_ref().unwrap();
            assert!((anchor_body.get_linear_velocity().x - speed).abs() < 0.02,
                "uneven kinematic velocity at {fps} FPS tick {tick}: {:?}", anchor_body.get_linear_velocity());
            assert!((anchor_body.get_position().x - position).abs() < 0.001);
            let body = physics.rigid_bodies[1].bullet_body.as_ref().unwrap();
            assert!((body.get_linear_velocity().x - speed).abs() < 0.001,
                "weapon owner travel was clamped at {fps} FPS");
            let before = body.get_position();
            physics.simulate_weapon(&[Mat4::IDENTITY], frame, 0.0, &config);
            let body = physics.rigid_bodies[1].bullet_body.as_ref().unwrap();
            assert!((body.get_position() - before).length() < 0.00001,
                "zero-time upload advanced a dynamic body");
        }
        physics.reset();
        physics.simulate_weapon(&[Mat4::IDENTITY], Mat4::IDENTITY, 0.0, &config);
        assert!(physics.rigid_bodies[1].bullet_body.as_ref().unwrap().get_linear_velocity().length() < 0.00001);
    }
}
