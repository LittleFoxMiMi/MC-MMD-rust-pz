//! VRM 模型加载入口

use glam::{Vec2, Vec3};
use std::path::Path;

use super::runtime::MmdModel;
use super::{vrm_extensions, vrm_material, vrm_mesh, vrm_morph, vrm_skeleton};
use crate::{MmdError, Result};

pub struct LoadedVrmModel {
    pub model: MmdModel,
    pub extensions: vrm_extensions::VrmExtensions,
}

fn validate_nodes(document:&gltf::Document, extensions:&vrm_extensions::VrmExtensions) -> Result<()> {
    let count=document.nodes().count();
    let mut parents=vec![None;count];
    for node in document.nodes() {
        if !glam::Mat4::from_cols_array_2d(&node.transform().matrix()).is_finite() {return Err(MmdError::VrmParse("Non-finite node transform".into()));}
        for child in node.children() {
            if parents[child.index()].replace(node.index()).is_some() {return Err(MmdError::VrmParse("A node has multiple parents".into()));}
        }
    }
    for i in 0..count {
        let mut node=Some(i); let mut depth=0;
        while let Some(i)=node {depth+=1;if depth>count {return Err(MmdError::VrmParse("Cyclic node hierarchy".into()));}node=parents[i];}
    }
    let mut indices:Vec<_>=extensions.humanoid.map.keys().copied().collect();
    indices.extend(extensions.first_person.bone);
    indices.extend(extensions.spring_bones.colliders.iter().map(|c|c.node));
    for spring in &extensions.spring_bones.springs { indices.extend(spring.center);indices.extend(spring.joints.iter().map(|j|j.node)); }
    for c in &extensions.node_constraints {
        indices.push(c.target);
        indices.push(match c.kind {vrm_extensions::NodeConstraintKind::Roll{source,..}|vrm_extensions::NodeConstraintKind::Aim{source,..}|vrm_extensions::NodeConstraintKind::Rotation{source,..}=>source});
    }
    if indices.iter().any(|i|*i>=count) {return Err(MmdError::VrmParse("VRM extension references a missing node".into()));}
    Ok(())
}

fn remap_extensions_to_runtime_bones(mut extensions: vrm_extensions::VrmExtensions,
    document: &gltf::Document, basis: glam::Mat4) -> vrm_extensions::VrmExtensions {
    let worlds=vrm_skeleton::compute_node_world_transforms(document);
    let count=document.nodes().count();
    let remap=|i: usize| (i<count).then_some(i+vrm_skeleton::VIRTUAL_BONE_COUNT);
    // VRM0 extension vectors use Unity's handedness; glTF nodes already
    // received the Z reflection at export. Apply that reflection exactly once.
    let extension_vector=|v:[f32;3]| {
        let v=Vec3::from_array(v);
        if extensions.version==vrm_extensions::VrmVersion::V0x {Vec3::new(v.x,v.y,-v.z)} else {v}
    };
    let offset=|node:usize, v:[f32;3]| basis.transform_vector3(worlds[&node].transform_vector3(extension_vector(v))).to_array();
    if let Some(node)=extensions.first_person.bone {
        extensions.first_person.bone_offset=offset(node,extensions.first_person.bone_offset);
        extensions.first_person.bone=remap(node);
    }
    for c in &mut extensions.spring_bones.colliders {
        let scale=(basis*worlds[&c.node]).to_scale_rotation_translation().0.abs().max_element();
        c.shape=match c.shape {
            vrm_extensions::ColliderShape::Sphere { offset:v, radius } => vrm_extensions::ColliderShape::Sphere { offset:offset(c.node,v),radius:radius*scale },
            vrm_extensions::ColliderShape::Capsule { offset:v, tail, radius } => vrm_extensions::ColliderShape::Capsule { offset:offset(c.node,v),tail:offset(c.node,tail),radius:radius*scale },
        };
        c.node=remap(c.node).unwrap();
    }
    for spring in &mut extensions.spring_bones.springs {
        spring.center=spring.center.and_then(remap);
        for joint in &mut spring.joints {
            let scale=(basis*worlds[&joint.node]).to_scale_rotation_translation().0.abs().max_element();
            joint.node=remap(joint.node).unwrap();
            joint.hit_radius*=scale;
            joint.gravity_dir=basis.transform_vector3(extension_vector(joint.gravity_dir)).normalize_or_zero().to_array();
            joint.gravity_power*=12.5;
            joint.stiffness*=12.5;
        }
        // VRM 1.0's final joint is a tail, not an independently simulated joint.
        // Runtime keeps it in the chain to resolve the preceding joint's tail.
    }
    for c in &mut extensions.node_constraints {
        c.target=remap(c.target).unwrap();
        match &mut c.kind {
            vrm_extensions::NodeConstraintKind::Roll {source,..} | vrm_extensions::NodeConstraintKind::Aim {source,..} | vrm_extensions::NodeConstraintKind::Rotation {source,..} => *source=remap(*source).unwrap(),
        }
    }
    extensions
}

/// 从 gltf::import 已解码的图片数据写入 PNG 临时文件，返回绝对路径数组
fn extract_textures(model_dir: &Path, images: &[gltf::image::Data]) -> Vec<String> {
    let mut paths = Vec::new();

    for (i, img_data) in images.iter().enumerate() {
        let filename = format!(".vrm_tex_{}.png", i);
        let full_path = model_dir.join(&filename);

        let result = (|| -> std::result::Result<(), Box<dyn std::error::Error>> {
            let (color_type, pixels) = match img_data.format {
                gltf::image::Format::R8G8B8 => (image::ColorType::Rgb8, &img_data.pixels[..]),
                gltf::image::Format::R8G8B8A8 => (image::ColorType::Rgba8, &img_data.pixels[..]),
                gltf::image::Format::R8 => (image::ColorType::L8, &img_data.pixels[..]),
                gltf::image::Format::R8G8 => (image::ColorType::La8, &img_data.pixels[..]),
                _ => {
                    return Err(format!("不支持的图片格式: {:?}", img_data.format).into());
                }
            };
            if full_path.is_file() { return Ok(()); }
            let temporary=full_path.with_file_name(format!("{}.{}.{}.png", i, std::process::id(), { static NEXT: std::sync::atomic::AtomicU64=std::sync::atomic::AtomicU64::new(0); NEXT.fetch_add(1,std::sync::atomic::Ordering::Relaxed) }));
            image::save_buffer(
                &temporary,
                pixels,
                img_data.width,
                img_data.height,
                color_type,
            )?;
            if let Err(error)=std::fs::rename(&temporary,&full_path) { if !full_path.is_file() { return Err(error.into()); } let _=std::fs::remove_file(&temporary); }
            Ok(())
        })();

        match result {
            Ok(_) => {
                log::debug!(
                    "纹理提取: {} ({}x{})",
                    filename,
                    img_data.width,
                    img_data.height
                );
                paths.push(full_path.to_string_lossy().replace('\\', "/"));
            }
            Err(e) => {
                log::warn!("纹理写入失败 {}: {}", filename, e);
                paths.push(String::new());
            }
        }
    }

    paths
}

/// 加载 VRM 模型，转换为 MmdModel 复用现有渲染管线
pub fn load_vrm<P: AsRef<Path>>(path: P) -> Result<MmdModel> {
    load_vrm_with_extensions(path).map(|loaded| loaded.model)
}

/// Loads a VRM model and returns both the runtime model and parsed VRM metadata.
pub fn load_vrm_with_extensions<P: AsRef<Path>>(path: P) -> Result<LoadedVrmModel> {
    let path = path.as_ref();
    use std::hash::{Hash, Hasher};
    let mut hash=std::collections::hash_map::DefaultHasher::new();
    path.canonicalize().unwrap_or(path.to_path_buf()).hash(&mut hash);
    if let Ok(m)=path.metadata() { m.len().hash(&mut hash); m.modified().ok().hash(&mut hash); }
    let cache_dir=std::env::temp_dir().join("pz_mmd-vrm-textures-v1").join(format!("{:016x}",hash.finish()));
    std::fs::create_dir_all(&cache_dir).map_err(|e| MmdError::VrmParse(e.to_string()))?;

    let (document, buffers, images) =
        gltf::import(path).map_err(|e| MmdError::VrmParse(format!("glTF 加载失败: {}", e)))?;

    let mut vrm_ext = vrm_extensions::parse_vrm_extensions(&document)?;
    validate_nodes(&document,&vrm_ext)?;
    let source_version=vrm_ext.version;
    let basis=glam::Mat4::from_scale(if source_version==vrm_extensions::VrmVersion::V0x { Vec3::new(-12.5,12.5,-12.5) } else { Vec3::splat(12.5) });
    let mesh = vrm_mesh::merge_meshes(&document, &buffers, basis)?;
    let bone_manager = vrm_skeleton::build_skeleton(&document, &vrm_ext.humanoid, basis)?;
    // VRM0 binds address meshes; VRM1 binds address node instances.
    for clip in vrm_ext.expressions.map.values_mut() {
        clip.morph_target_binds=clip.scoped_binds.iter().flat_map(|&(scope,target,weight)| {
            mesh.morph_targets.iter().enumerate().filter(move |(_,m)| m.target==target &&
                (if source_version==vrm_extensions::VrmVersion::V0x {m.mesh} else {m.node})==scope)
                .map(move |(i,_)| (i,weight))
        }).collect();
    }
    let vrm_ext = remap_extensions_to_runtime_bones(vrm_ext, &document, basis);
    let (mut materials, _) = vrm_material::convert_materials(&document, &vrm_ext);
    if mesh.submeshes.iter().any(|s| s.material_id as usize==materials.len()) { materials.push(super::MmdMaterial::default()); }
    let texture_paths = extract_textures(&cache_dir, &images);
    let raw=serde_json::to_value(document.as_json()).map_err(|e| MmdError::VrmParse(e.to_string()))?;
    let metadata=serde_json::json!({"format":"pz-vrm-v1", "version":format!("{:?}",source_version),
        "materials":raw["materials"], "textures":raw["textures"], "samplers":raw["samplers"],
        "extensions":raw["extensions"], "runtime":{"nodes":document.nodes().count(), "skins":document.skins().count(),
        "colliders":vrm_ext.spring_bones.colliders.len(), "springs":vrm_ext.spring_bones.springs.len(),
        "morphTargets":mesh.morph_targets.len()}, "texturePaths":texture_paths}).to_string();

    let morph_manager = vrm_morph::convert_morph_targets(
        &mesh.morph_targets,
        &vrm_ext.expressions,
        mesh.vertices.len(),
        materials.len(),
    );

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("vrm_model")
        .to_string();

    let update_positions: Vec<Vec3> = mesh.vertices.iter().map(|v| v.position).collect();
    let update_normals: Vec<Vec3> = mesh.vertices.iter().map(|v| v.normal).collect();
    let update_uvs: Vec<Vec2> = mesh.vertices.iter().map(|v| v.uv).collect();

    let mut model = MmdModel::new();
    model.name = name;
    model.vrm_metadata = metadata;
    model.vrm_material_render_states = (0..materials.len()).flat_map(|i| {
        vrm_material::render_state(&raw["materials"][i],
            &raw["extensions"]["VRM"]["materialProperties"][i])
    }).collect();
    model.set_vrm(true);
    model.vertices = mesh.vertices;
    model.indices = mesh.indices;
    model.weights = mesh.weights;
    model.materials = materials;
    model.submeshes = mesh.submeshes;
    model.texture_paths = texture_paths;
    model.rigid_bodies = Vec::new();
    model.joints = Vec::new();
    model.update_positions = update_positions;
    model.update_normals = update_normals;
    model.update_uvs = update_uvs;
    model.bone_manager = bone_manager;
    model.morph_manager = morph_manager;

    model
        .morph_manager
        .set_material_count(model.materials.len());
    model.morph_manager.set_vertex_count(model.vertices.len());
    model.init_material_visibility();
    model.initialize_animation();
    for (i,target) in mesh.morph_targets.iter().enumerate() { model.morph_manager.set_morph_weight(i,target.default_weight); }
    model.vrm_expression_morphs=vrm_ext.expressions.map.keys().filter_map(|name| model.morph_manager.find_morph_by_name(vrm_morph::vrm_expression_to_mmd(name))).collect();
    model.tick_animation(0.0);
    model.initialize_vrm_runtime(vrm_ext.clone());

    log::info!(
        "VRM 加载完成: {} (顶点:{}, 面:{}, 材质:{}, 骨骼:{}, 纹理:{})",
        model.name,
        model.vertex_count(),
        model.index_count() / 3,
        model.material_count(),
        model.bone_manager.bone_count(),
        model.texture_paths.len(),
    );

    Ok(LoadedVrmModel {
        model,
        extensions: vrm_ext,
    })
}
