//! glTF/MToon 材质 → MmdMaterial 转换

use glam::{Vec3, Vec4};

use super::MmdMaterial;
use crate::model::vrm_extensions::{MtoonParams, VrmExtensions};

/// 将 glTF 材质转换为 MmdMaterial，同时构建纹理路径数组
pub(crate) fn convert_materials(
    document: &gltf::Document,
    vrm_extensions: &VrmExtensions,
) -> (Vec<MmdMaterial>, Vec<String>) {
    let texture_paths: Vec<String> = document
        .images()
        .enumerate()
        .map(|(i, _)| format!(".vrm_tex_{}.png", i))
        .collect();

    let texture_to_image: Vec<usize> = document
        .textures()
        .map(|tex| tex.source().index())
        .collect();

    let materials = document
        .materials()
        .enumerate()
        .map(|(idx, mat)| convert_single_material(&mat, vrm_extensions, idx, &texture_to_image))
        .collect();

    (materials, texture_paths)
}

/// 转换单个 glTF 材质
fn convert_single_material(
    mat: &gltf::Material,
    vrm_extensions: &VrmExtensions,
    material_index: usize,
    texture_to_image: &[usize],
) -> MmdMaterial {
    let name = mat.name().unwrap_or("").to_string();
    let pbr = mat.pbr_metallic_roughness();
    let base_color = pbr.base_color_factor();

    let texture_index = resolve_texture_index(&pbr, texture_to_image);
    let mtoon = vrm_extensions
        .mtoon_materials
        .get(material_index)
        .and_then(|m| m.as_ref());

    let (diffuse, specular, specular_strength, ambient, edge_color, edge_scale) =
        if let Some(mtoon) = mtoon {
            convert_mtoon(base_color, mtoon)
        } else if mat.unlit() {
            convert_unlit(base_color)
        } else {
            convert_pbr(base_color)
        };

    let draw_flags = build_draw_flags(mat, mtoon);
    let alpha = compute_alpha(mat, base_color[3]);

    MmdMaterial {
        name,
        diffuse: Vec4::new(diffuse.x, diffuse.y, diffuse.z, alpha),
        specular,
        specular_strength,
        ambient,
        edge_color,
        edge_scale,
        texture_index,
        environment_index: -1,
        environment_blend_mode: 0,
        toon_index: -1,
        draw_flags,
    }
}

/// 解析 glTF 纹理索引 → 图片索引
fn resolve_texture_index(
    pbr: &gltf::material::PbrMetallicRoughness,
    texture_to_image: &[usize],
) -> i32 {
    pbr.base_color_texture()
        .map(|info| {
            let tex_idx = info.texture().index();
            texture_to_image
                .get(tex_idx)
                .map(|&img_idx| img_idx as i32)
                .unwrap_or(-1)
        })
        .unwrap_or(-1)
}

/// MToon 材质：shadeFactor → ambient, outlineColor → edge_color
fn convert_mtoon(base_color: [f32; 4], mtoon: &MtoonParams) -> (Vec4, Vec3, f32, Vec3, Vec4, f32) {
    let diffuse = Vec4::from(base_color);
    let specular = Vec3::ZERO;
    let specular_strength = 0.0;
    let ambient = Vec3::from(mtoon.shade_color_factor);
    let oc = mtoon.outline_color_factor;
    let edge_color = Vec4::new(oc[0], oc[1], oc[2], 1.0);
    let edge_scale = mtoon.outline_width_factor;

    (
        diffuse,
        specular,
        specular_strength,
        ambient,
        edge_color,
        edge_scale,
    )
}

/// Unlit 材质：baseColorFactor → diffuse, specular = 0
fn convert_unlit(base_color: [f32; 4]) -> (Vec4, Vec3, f32, Vec3, Vec4, f32) {
    let diffuse = Vec4::from(base_color);
    let specular = Vec3::ZERO;
    let specular_strength = 0.0;
    let ambient = Vec3::new(
        base_color[0] * 0.5,
        base_color[1] * 0.5,
        base_color[2] * 0.5,
    );
    let edge_color = Vec4::new(0.0, 0.0, 0.0, 1.0);
    let edge_scale = 1.0;

    (
        diffuse,
        specular,
        specular_strength,
        ambient,
        edge_color,
        edge_scale,
    )
}

/// PBR 材质：baseColorFactor → diffuse（忽略 metallic/roughness）
fn convert_pbr(base_color: [f32; 4]) -> (Vec4, Vec3, f32, Vec3, Vec4, f32) {
    let diffuse = Vec4::from(base_color);
    let specular = Vec3::ZERO;
    let specular_strength = 0.0;
    let ambient = Vec3::new(
        base_color[0] * 0.5,
        base_color[1] * 0.5,
        base_color[2] * 0.5,
    );
    let edge_color = Vec4::new(0.0, 0.0, 0.0, 1.0);
    let edge_scale = 1.0;

    (
        diffuse,
        specular,
        specular_strength,
        ambient,
        edge_color,
        edge_scale,
    )
}

/// 构建 draw_flags：bit 0 = 双面渲染, bit 4 = 边缘（MToon outline）
fn build_draw_flags(mat: &gltf::Material, mtoon: Option<&MtoonParams>) -> u8 {
    let mut flags: u8 = 0;
    if mat.double_sided() {
        flags |= 0x01;
    }
    if let Some(m) = mtoon {
        if m.outline_width_factor > 0.0 {
            flags |= 0x10;
        }
    }
    flags
}

// Packed JNI row: alpha mode (1 opaque, 2 mask, 3 blend), depth write,
// render queue, and the bit pattern of the effective texture-alpha cutoff.
// MASK is binary: bake baseColorFactor.a into the threshold, leaving the
// runtime material opacity available as an independent user control.
pub(crate) fn render_state(material: &serde_json::Value, vrm0: &serde_json::Value) -> [i32; 4] {
    use serde_json::Value;
    match material["alphaMode"].as_str() {
        Some("BLEND") => {
            let mtoon = &material["extensions"]["VRMC_materials_mtoon"];
            let (depth, queue) = if vrm0["shader"].as_str() == Some("VRM/MToon") {
                let floats = &vrm0["floatProperties"];
                let depth = floats["_ZWrite"].as_f64().map_or_else(
                    || floats["_BlendMode"].as_f64() == Some(3.0),
                    |value| value > 0.5,
                );
                let queue = vrm0["renderQueue"]
                    .as_i64()
                    .filter(|q| (0..=5000).contains(q))
                    .unwrap_or(if depth { 2501 } else { 3000 });
                (depth, queue)
            } else {
                let depth = mtoon["transparentWithZWrite"].as_bool().unwrap_or(false);
                let offset = mtoon["renderQueueOffsetNumber"].as_i64().unwrap_or(0);
                let queue = if depth {
                    2501 + offset.clamp(0, 9)
                } else {
                    3000 + offset.clamp(-9, 0)
                };
                (depth, queue)
            };
            [3, i32::from(depth), queue as i32, 0]
        }
        Some("MASK") => {
            let alpha = material["pbrMetallicRoughness"]["baseColorFactor"]
                .get(3)
                .and_then(Value::as_f64)
                .unwrap_or(1.0)
                .clamp(0.0, 1.0);
            let cutoff = material["alphaCutoff"].as_f64().unwrap_or(0.5).max(0.0);
            let threshold = if cutoff == 0.0 {
                0.0
            } else if alpha == 0.0 {
                2.0
            } else {
                (cutoff / alpha).min(2.0) as f32
            };
            [2, 1, 2450, threshold.to_bits() as i32]
        }
        _ => [1, 1, 2000, 0],
    }
}

fn compute_alpha(mat: &gltf::Material, base_alpha: f32) -> f32 {
    match mat.alpha_mode() {
        gltf::material::AlphaMode::Blend => base_alpha,
        _ => 1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::render_state;
    use serde_json::{json, Value};

    #[test]
    fn vrm_zero_mtoon_keeps_explicit_queue_and_depth_write() {
        let blend = json!({"alphaMode": "BLEND"});
        let source = json!({"shader":"VRM/MToon", "renderQueue":2501,
            "floatProperties":{"_ZWrite":1.0,"_BlendMode":3.0}});
        assert_eq!(render_state(&blend, &source), [3, 1, 2501, 0]);
        assert_eq!(render_state(&json!({}), &source), [1, 1, 2000, 0]);
    }

    #[test]
    fn vrm_one_separates_zwrite_blend_from_default_blend() {
        let blend = |depth, offset| {
            json!({"alphaMode":"BLEND", "extensions":{
                "VRMC_materials_mtoon":{"transparentWithZWrite":depth,"renderQueueOffsetNumber":offset}
            }})
        };
        assert_eq!(render_state(&blend(true, 4), &Value::Null), [3, 1, 2505, 0]);
        assert_eq!(
            render_state(&blend(false, -2), &Value::Null),
            [3, 0, 2998, 0]
        );
        assert_eq!(
            render_state(&json!({"alphaMode":"BLEND"}), &Value::Null),
            [3, 0, 3000, 0]
        );
    }

    #[test]
    fn alpha_mask_threshold_includes_authored_base_colour_factor() {
        let masked = json!({"alphaMode":"MASK", "alphaCutoff":0.4,
            "pbrMetallicRoughness":{"baseColorFactor":[1.0,1.0,1.0,0.5]}});
        assert_eq!(
            render_state(&masked, &Value::Null),
            [2, 1, 2450, 0.8_f32.to_bits() as i32]
        );
    }
}
