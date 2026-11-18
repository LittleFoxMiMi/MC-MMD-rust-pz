//! glTF skin/nodes → BoneSet 骨骼系统转换（含 MMD 虚拟骨骼 + IK 骨骼注入）

use glam::{Mat4, Vec3};
use std::collections::HashMap;

use super::bone_mapping;
use super::vrm_extensions::HumanoidMapping;
use crate::skeleton::{BoneFlags, BoneLink, BoneSet, IkConfig, IkLink};

/// MMD 虚拟骨骼数量（全ての親 + センター + グルーブ）
pub(crate) const VIRTUAL_BONE_COUNT: usize = 3;

fn build_parent_map(document: &gltf::Document) -> HashMap<usize, usize> {
    let mut parent_map = HashMap::new();
    for node in document.nodes() {
        for child in node.children() {
            parent_map.insert(child.index(), node.index());
        }
    }
    parent_map
}

pub(crate) fn compute_node_world_transforms(document: &gltf::Document) -> HashMap<usize, Mat4> {
    let mut world_transforms = HashMap::new();
    let parent_map = build_parent_map(document);
    let nodes: Vec<_> = document.nodes().collect();
    for node in &nodes {
        compute_world_transform_recursive(node.index(), &nodes, &parent_map, &mut world_transforms);
    }
    world_transforms
}

fn compute_world_transform_recursive(
    node_idx: usize,
    nodes: &[gltf::Node],
    parent_map: &HashMap<usize, usize>,
    cache: &mut HashMap<usize, Mat4>,
) -> Mat4 {
    if let Some(&cached) = cache.get(&node_idx) {
        return cached;
    }
    let local = Mat4::from_cols_array_2d(&nodes[node_idx].transform().matrix());
    let world = match parent_map.get(&node_idx) {
        Some(&parent_idx) => {
            let parent_world =
                compute_world_transform_recursive(parent_idx, nodes, parent_map, cache);
            parent_world * local
        }
        None => local,
    };
    cache.insert(node_idx, world);
    world
}

/// 从完整 glTF node hierarchy 构建平移基准 BoneSet
///
/// 骨骼布局：
/// [0] 全ての親  [1] センター  [2] グルーブ
/// [3..3+N] glTF nodes（包括非蒙皮节点）
/// [3+N..3+N+4] IK 骨骼（左足ＩＫ, 右足ＩＫ, 左つま先ＩＫ, 右つま先ＩＫ）
pub(crate) fn build_skeleton(
    document: &gltf::Document,
    humanoid_mapping: &HumanoidMapping,
    basis: Mat4,
) -> crate::Result<BoneSet> {
    let parent_map = build_parent_map(document);
    let worlds = compute_node_world_transforms(document);
    let hips = humanoid_mapping.map.iter().find(|(_, n)| *n == "hips").map(|(i, _)| *i);
    let hips_pos = hips.map(|i| basis.transform_point3(worlds[&i].w_axis.truncate())).unwrap_or(Vec3::ZERO);
    let mut bone_set = BoneSet::new();
    inject_virtual_bones(&mut bone_set, hips_pos);
    let mut names = std::collections::HashSet::from(["全ての親".to_string(), "センター".to_string(), "グルーブ".to_string()]);
    for name in humanoid_mapping.map.values() {names.insert(bone_mapping::vrm_to_mmd_bone_name(name).to_string());}
    for node in document.nodes() {
        let i = node.index();
        let mut name = humanoid_mapping.map.get(&i)
            .map(|n| bone_mapping::vrm_to_mmd_bone_name(n).to_string())
            .unwrap_or_else(|| node.name().unwrap_or("node").to_string());
        if !humanoid_mapping.map.contains_key(&i) && !names.insert(name.clone()) { name = format!("{}#{}", name, i); names.insert(name.clone()); }
        let mut bone = BoneLink::new(name);
        bone.initial_position = basis.transform_point3(worlds[&i].w_axis.truncate());
        bone.parent_index = if Some(i) == hips { 2 } else {
            parent_map.get(&i).map(|p| (p + VIRTUAL_BONE_COUNT) as i32).unwrap_or(0)
        };
        let mut depth=3; let mut current=i;
        while let Some(&parent)=parent_map.get(&current) { depth+=1; current=parent; }
        bone.transform_level=depth;
        bone.flags = BoneFlags::ROTATABLE | BoneFlags::MOVABLE;
        bone_set.add_bone(bone);
    }
    inject_ik_bones(&mut bone_set);
    bone_set.build_hierarchy();
    Ok(bone_set)
}

/// 注入 MMD 足 IK 骨骼
///
/// VMD 动画通过 IK 骨骼驱动脚部运动：
/// - 左足ＩＫ: IK 链 = [左ひざ(角度限制), 左足] → 目标 = 左足首
/// - 右足ＩＫ: IK 链 = [右ひざ(角度限制), 右足] → 目标 = 右足首
/// - 左つま先ＩＫ: IK 链 = [左足首] → 目标 = 左つま先
/// - 右つま先ＩＫ: IK 链 = [右足首] → 目標 = 右つま先
fn inject_ik_bones(bone_set: &mut BoneSet) {
    let ik_defs = [
        ("左足ＩＫ", "左足首", &["左ひざ", "左足"][..], "全ての親"),
        ("右足ＩＫ", "右足首", &["右ひざ", "右足"][..], "全ての親"),
        ("左つま先ＩＫ", "左つま先", &["左足首"][..], "左足ＩＫ"),
        ("右つま先ＩＫ", "右つま先", &["右足首"][..], "右足ＩＫ"),
    ];

    for (ik_name, target_name, chain_names, parent_name) in &ik_defs {
        let target_idx = match bone_set.find_bone_by_name(target_name) {
            Some(idx) => idx,
            None => {
                log::warn!("IK 注入跳过 {}: 目标骨骼 {} 未找到", ik_name, target_name);
                continue;
            }
        };

        let target_pos = bone_set
            .get_bone(target_idx)
            .map(|b| b.initial_position)
            .unwrap_or(Vec3::ZERO);

        let parent_idx = bone_set
            .find_bone_by_name(parent_name)
            .map(|idx| idx as i32)
            .unwrap_or(0);

        let links: Vec<IkLink> = chain_names
            .iter()
            .filter_map(|&name| {
                bone_set.find_bone_by_name(name).map(|idx| {
                    let is_knee = name.contains("ひざ");
                    IkLink {
                        bone_index: idx as i32,
                        has_limits: is_knee,
                        // X+Z flip 后膝盖弯曲方向为 X 轴正方向
                        limit_min: if is_knee {
                            Vec3::new(0.008726646, 0.0, 0.0)
                        } else {
                            Vec3::ZERO
                        },
                        limit_max: if is_knee {
                            Vec3::new(std::f32::consts::PI, 0.0, 0.0)
                        } else {
                            Vec3::ZERO
                        },
                    }
                })
            })
            .collect();

        if links.is_empty() {
            log::warn!("IK 注入跳过 {}: IK 链骨骼未找到", ik_name);
            continue;
        }

        let mut ik_bone = BoneLink::new(ik_name.to_string());
        ik_bone.transform_level = 10000;
        ik_bone.initial_position = target_pos;
        ik_bone.parent_index = parent_idx;
        ik_bone.flags = BoneFlags::ROTATABLE | BoneFlags::MOVABLE | BoneFlags::IK;
        ik_bone.ik_config = Some(IkConfig {
            target_bone: target_idx as i32,
            iterations: 40,
            limit_angle: 2.0_f32.to_radians(),
            links,
        });

        // 标记 IK 链骨骼启用 IK
        for name in *chain_names {
            if let Some(idx) = bone_set.find_bone_by_name(name) {
                if let Some(bone) = bone_set.get_bone_mut(idx) {
                    bone.flags.insert(BoneFlags::IK_ENABLED);
                }
            }
        }

        bone_set.add_bone(ik_bone);
        log::info!(
            "IK 注入: {} → 目标={} 链长={}",
            ik_name,
            target_name,
            chain_names.len()
        );
    }
}

fn inject_virtual_bones(bone_set: &mut BoneSet, hips_pos: Vec3) {
    let mut master = BoneLink::new("全ての親".to_string());
    master.initial_position = Vec3::ZERO;
    master.parent_index = -1;
    master.flags = BoneFlags::ROTATABLE | BoneFlags::MOVABLE;
    bone_set.add_bone(master);

    let mut center = BoneLink::new("センター".to_string());
    center.initial_position = hips_pos;
    center.parent_index = 0;
    center.flags = BoneFlags::ROTATABLE | BoneFlags::MOVABLE;
    bone_set.add_bone(center);

    let mut groove = BoneLink::new("グルーブ".to_string());
    groove.initial_position = hips_pos;
    groove.parent_index = 1;
    groove.flags = BoneFlags::ROTATABLE | BoneFlags::MOVABLE;
    bone_set.add_bone(groove);
}
