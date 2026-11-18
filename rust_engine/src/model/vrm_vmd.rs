use crate::{
    animation::{Motion, VmdAnimation},
    morph::MorphManager,
    skeleton::BoneManager,
};

use glam::{Quat, Vec3};

use super::vrm_morph::{is_standard_vmd_expression, VMD_EXPRESSIONS};

const IK_NAMES: [(&str, &str); 4] = [
    ("左足ＩＫ", "左足IK"),
    ("右足ＩＫ", "右足IK"),
    ("左つま先ＩＫ", "左つま先IK"),
    ("右つま先ＩＫ", "右つま先IK"),
];

fn primary_alias(name: &str) -> Option<&'static str> {
    match name {
        "左足D" => Some("左足"),
        "右足D" => Some("右足"),
        "左ひざD" => Some("左ひざ"),
        "右ひざD" => Some("右ひざ"),
        "左足首D" => Some("左足首"),
        "右足首D" => Some("右足首"),
        _ => IK_NAMES
            .iter()
            .find_map(|&(full, half)| (name == half).then_some(full)),
    }
}

const ROOT_NAMES: [&str; 3] = ["全ての親", "センター", "グルーブ"];

/// PMX VMD rotations are deltas from the source's rest, which a VMD does not
/// contain. Use a conventional A-pose profile measured from a representative
/// MMD rig (the local 少女 PMX), rather than applying deltas to the VRM T-pose.
/// Only directions are transferred: segment lengths and shoe stance stay with
/// the VRM. All vectors here are in the engine's native (+Z forward) basis.
pub(super) struct RestBasis {
    basis: Vec<Quat>,
    parent: Vec<Quat>,
    neutral: Vec<(usize, Quat)>,
}

impl RestBasis {
    pub(super) fn from_bind(bones: &BoneManager) -> Self {
        let index = |name: &str| bones.find_bone_by_name(name);
        let direction = |i, j| {
            let a = bones.get_bone(i)?;
            let mut cursor = bones.get_bone(j)?;
            let delta = cursor.initial_position - a.initial_position;
            // Ignore same-name helpers outside the mapped limb's hierarchy.
            while cursor.parent_index >= 0 {
                if cursor.parent_index as usize == i {
                    return delta.try_normalize();
                }
                cursor = bones.get_bone(cursor.parent_index as usize)?;
            }
            None
        };
        let mut overrides = vec![None; bones.bone_count()];
        for (side, sign) in [("左", 1.0), ("右", -1.0)] {
            for (name, child, reference, sagittal_only) in [
                ("腕", "ひじ", Vec3::new(sign * 1.916, -1.745, -0.054), false),
                (
                    "ひじ",
                    "手首",
                    Vec3::new(sign * 1.881, -1.713, 0.116),
                    false,
                ),
                ("足", "ひざ", Vec3::new(0.0, -4.930, -0.190), true),
                ("ひざ", "足首", Vec3::new(0.0, -5.143, -0.429), true),
            ] {
                let (Some(i), Some(j)) = (
                    index(&format!("{side}{name}")),
                    index(&format!("{side}{child}")),
                ) else {
                    continue;
                };
                if let Some(bind) = direction(i, j) {
                    let target = if sagittal_only {
                        // Match knee/ankle pitch without narrowing the VRM's
                        // bind stance to the reference rig's shoe spacing.
                        reference.normalize() * (1.0 - bind.x * bind.x).max(0.0).sqrt()
                            + Vec3::X * bind.x
                    } else {
                        reference.normalize()
                    };
                    overrides[i] = Some(Quat::from_rotation_arc(bind, target));
                }
            }
            if let Some(ankle) = index(&format!("{side}足首")) {
                // Retarget leg pitch without imposing the reference model's shoe angle.
                overrides[ankle] = Some(Quat::IDENTITY);
            }
        }
        let inherited = |mut i: i32| {
            while let Some(bone) = bones.get_bone(i as usize) {
                if let Some(q) = overrides[i as usize] {
                    return q;
                }
                i = bone.parent_index;
            }
            Quat::IDENTITY
        };
        let mut rest = Self {
            basis: Vec::new(),
            parent: Vec::new(),
            neutral: Vec::new(),
        };
        for i in 0..bones.bone_count() {
            let child = inherited(i as i32);
            let parent = inherited(bones.get_bone(i).unwrap().parent_index);
            rest.basis.push(child);
            rest.parent.push(parent);
            if child != parent {
                rest.neutral
                    .push((i, (parent.inverse() * child).normalize()));
            }
        }
        rest
    }

    fn rotation(&self, i: usize, raw: Quat) -> Quat {
        (self.parent[i].inverse() * raw * self.basis[i]).normalize()
    }
}

fn takes_translation(name: &str) -> bool {
    ROOT_NAMES.contains(&name) || IK_NAMES.iter().any(|&(full, _)| full == name)
}

fn shared_torso_motion(motion: &Motion) -> bool {
    // Some baked FK files put the same travel on sibling PMX torso/hip
    // tracks, without center motion. Recover it once at the VRM center.
    if ROOT_NAMES.iter().any(|name| {
        motion.bone_tracks.get(*name).is_some_and(|track| {
            track
                .keyframes
                .values()
                .any(|k| k.translation.length_squared() > 1e-6)
        })
    }) {
        return false;
    }
    let (Some(lower), Some(upper)) = (
        motion.bone_tracks.get("下半身"),
        motion.bone_tracks.get("上半身"),
    ) else {
        return false;
    };
    let count = lower.keyframes.len();
    if count < 2 || count != upper.keyframes.len() {
        return false;
    }
    let mut travel = 0.0;
    let mut difference = 0.0;
    for ((li, l), (ui, u)) in lower.keyframes.iter().zip(&upper.keyframes) {
        if li != ui {
            return false;
        }
        let error = l.translation.distance_squared(u.translation);
        if !error.is_finite() || error > 0.15_f32.powi(2) {
            return false;
        }
        difference += error;
        travel += l.translation.length_squared();
    }
    travel > 0.02_f32.powi(2) * count as f32 && difference < travel * 0.01
}

pub(super) fn facial_baseline(morphs: &MorphManager) -> Vec<(usize, f32)> {
    VMD_EXPRESSIONS
        .iter()
        .filter_map(|name| {
            let index = morphs.find_morph_by_name(name)?;
            Some((index, morphs.get_morph_weight(index)))
        })
        .collect()
}

pub(super) fn evaluate(
    animation: &VmdAnimation,
    frame: f32,
    additive: bool,
    rest: Option<&RestBasis>,
    bones: &mut BoneManager,
    morphs: &mut MorphManager,
) {
    let motion = animation.motion();
    let frame = frame.max(0.0);
    let index = frame.floor() as u32;
    let amount = frame.fract();
    let bind = rest.filter(|_| !additive);
    if let Some(rest) = bind {
        for &(i, neutral) in &rest.neutral {
            bones.set_bone_rotation(i, neutral);
        }
    }
    let raw = |name| motion.find_bone_transform(name, index, amount);
    for name in motion.bone_track_names() {
        let target = if let Some(primary) = primary_alias(name) {
            if motion.contains_bone_track(primary) {
                continue;
            }
            primary
        } else {
            name.as_str()
        };
        if let Some(i) = bones.find_bone_by_name(target) {
            let key = raw(name);
            let bone = bones.get_bone(i).unwrap();
            let translation = takes_translation(target).then(|| {
                if additive {
                    bone.animation_translate + key.translation
                } else {
                    key.translation
                }
            });
            let rotation = if additive {
                (bone.animation_rotate * key.orientation).normalize()
            } else if let Some(rest) = bind {
                rest.rotation(i, key.orientation)
            } else {
                key.orientation
            };
            if let Some(translation) = translation {
                bones.set_bone_translation(i, translation);
            }
            bones.set_bone_rotation(i, rotation);
        }
    }
    if !additive {
        let lower = raw("下半身");
        if let Some(spine) = bones.find_bone_by_name("上半身") {
            // PMX 下半身 and 上半身 normally share a parent. In VRM the
            // spine is a hips descendant, so cancel the extra hip rotation.
            let mut upper = lower.orientation.inverse() * raw("上半身").orientation;
            if bones.find_bone_by_name("上半身1").is_none() {
                upper *= raw("上半身1").orientation;
            }
            bones.set_bone_rotation(spine, upper.normalize());
        }
        if animation.vrm_shared_torso_motion(shared_torso_motion) {
            if let Some(center) = bones.find_bone_by_name("センター") {
                bones.set_bone_translation(center, lower.translation);
            }
        }
        if let Some(rest) = bind {
            for (target, helper) in [
                ("左腕", "左腕捩"),
                ("右腕", "右腕捩"),
                ("左ひじ", "左手捩"),
                ("右ひじ", "右手捩"),
            ] {
                if motion.contains_bone_track(helper) && bones.find_bone_by_name(helper).is_none() {
                    if let Some(i) = bones.find_bone_by_name(target) {
                        bones.set_bone_rotation(
                            i,
                            rest.rotation(i, raw(target).orientation * raw(helper).orientation),
                        );
                    }
                }
            }
        }
    }
    for name in motion
        .morph_track_names()
        .filter(|name| is_standard_vmd_expression(name))
    {
        if let Some(i) = morphs.find_morph_by_name(name) {
            let weight = motion.find_morph_weight(name, index, amount);
            let base = if additive {
                morphs.get_morph_weight(i)
            } else {
                0.0
            };
            morphs.set_morph_weight(i, (base + weight).clamp(0.0, 1.0));
        }
    }
    if !additive {
        for (name, alias) in IK_NAMES {
            let enabled = if motion.ik_tracks.contains_key(name) {
                motion.is_ik_enabled(name, index)
            } else if motion.ik_tracks.contains_key(alias) {
                motion.is_ik_enabled(alias, index)
            } else {
                // FK-only VMDs often omit IK data altogether. Don't pin
                // an animated leg to an undriven controller at bind pose.
                motion.contains_bone_track(name) || motion.contains_bone_track(alias)
            };
            bones.set_ik_enabled_by_name(name, enabled);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{animation::BoneKeyframe, skeleton::BoneLink};

    fn rig() -> BoneManager {
        let mut bones = BoneManager::new();
        for (i, (name, parent, p)) in [
            ("センター", -1, Vec3::new(0.0, 10.0, 0.0)),
            ("下半身", 0, Vec3::new(0.0, 10.0, 0.0)),
            ("上半身", 1, Vec3::new(0.0, 11.0, 0.0)),
            ("左腕", 2, Vec3::new(1.0, 12.0, 0.0)),
            ("左ひじ", 3, Vec3::new(3.0, 12.0, 0.0)),
            ("左手首", 4, Vec3::new(5.0, 12.0, 0.0)),
            ("左足", 1, Vec3::new(0.8, 9.0, 0.0)),
            ("左ひざ", 6, Vec3::new(0.8, 5.0, 0.0)),
            ("左足首", 7, Vec3::new(0.8, 1.0, 0.0)),
        ]
        .into_iter()
        .enumerate()
        {
            let mut bone = BoneLink::new(name.into());
            bone.initial_position = p;
            bone.parent_index = parent;
            bone.transform_level = i as i32;
            bones.add_bone(bone);
        }
        bones.build_hierarchy();
        bones
    }

    fn key(motion: &mut Motion, name: &str, frame: u32, translation: Vec3, rotation: Quat) {
        motion.insert_bone_keyframe(
            name,
            BoneKeyframe::with_transform(frame, translation, rotation),
        );
    }

    fn point(bones: &BoneManager, i: usize) -> Vec3 {
        bones.get_bone(i).unwrap().local_to_world.w_axis.truncate()
    }

    #[test]
    fn replacement_uses_mmd_arm_rest_and_preserves_fk_segment_lengths() {
        let mut bones = rig();
        let rest = RestBasis::from_bind(&bones);
        let mut morphs = MorphManager::new();
        let mut clip = Motion::new();
        key(
            &mut clip,
            "左腕",
            0,
            Vec3::ZERO,
            Quat::from_rotation_z(-27_f32.to_radians()),
        );
        let clip = VmdAnimation::from_motion(clip);
        evaluate(&clip, 0.0, false, Some(&rest), &mut bones, &mut morphs);
        bones.update_transforms(false);
        let arm = point(&bones, 4) - point(&bones, 3);
        let drop = (-arm.normalize().y).asin().to_degrees();
        assert!((68.0..71.0).contains(&drop), "downward arm angle: {drop}");
        let thigh = point(&bones, 7) - point(&bones, 6);
        let shin = point(&bones, 8) - point(&bones, 7);
        assert!(thigh.x.abs() < 1e-5 && shin.x.abs() < 1e-5);
        assert!(thigh.z < -0.12 && shin.z < thigh.z, "{thigh:?}, {shin:?}");
        assert!((thigh.length() - 4.0).abs() < 1e-5 && (shin.length() - 4.0).abs() < 1e-5);
        assert!(bones
            .get_bone(8)
            .unwrap()
            .local_to_world
            .x_axis
            .truncate()
            .abs_diff_eq(Vec3::X, 1e-5));

        bones.reset_all_transforms();
        evaluate(&clip, 0.0, true, Some(&rest), &mut bones, &mut morphs);
        bones.update_transforms(false);
        let additive_drop = (-(point(&bones, 4) - point(&bones, 3)).normalize().y)
            .asin()
            .to_degrees();
        assert!((additive_drop - 27.0).abs() < 1e-4);
    }

    #[test]
    fn baked_torso_travel_goes_to_center_once_and_rotations_stay_independent() {
        let mut bones = rig();
        let rest = RestBasis::from_bind(&bones);
        let mut morphs = MorphManager::new();
        let mut clip = Motion::new();
        let hip = Quat::from_rotation_y(0.75);
        let spine = Quat::from_rotation_y(0.5);
        for (frame, travel) in [(0, Vec3::ZERO), (10, Vec3::new(4.0, -0.2, 1.0))] {
            key(&mut clip, "下半身", frame, travel, hip);
            key(&mut clip, "上半身", frame, travel + Vec3::X * 0.01, spine);
        }
        key(&mut clip, "左ひじ", 0, Vec3::splat(10.0), Quat::IDENTITY);
        let mut clip = VmdAnimation::from_motion(clip);
        evaluate(&clip, 10.0, false, Some(&rest), &mut bones, &mut morphs);
        bones.update_transforms(false);
        assert!(point(&bones, 0).abs_diff_eq(Vec3::new(4.0, 9.8, 1.0), 1e-5));
        assert!(point(&bones, 2).abs_diff_eq(Vec3::new(4.0, 10.8, 1.0), 1e-5));
        let spine_x = bones.get_bone(2).unwrap().local_to_world.x_axis.truncate();
        assert!(spine_x.abs_diff_eq(spine * Vec3::X, 1e-5));
        assert_eq!(bones.get_bone(4).unwrap().animation_translate, Vec3::ZERO);

        // Explicit center travel owns the translation and invalidates the
        // cached mocap classification when tracks are edited.
        key(
            clip.motion_mut(),
            "センター",
            10,
            Vec3::X * 2.0,
            Quat::IDENTITY,
        );
        bones.reset_all_transforms();
        evaluate(&clip, 10.0, false, Some(&rest), &mut bones, &mut morphs);
        assert_eq!(
            bones.get_bone(0).unwrap().animation_translate,
            Vec3::X * 2.0
        );
    }
}
