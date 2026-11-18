//! Scene instances, per-skin binds and mesh-scoped morph targets.
use glam::{Mat4, Vec2, Vec3};
use std::sync::Arc;
use super::{RuntimeVertex, SubMesh, VertexWeight, vrm_skeleton};
use crate::{Result, MmdError};

#[derive(Clone, Debug)]
pub(crate) struct MorphTargetData {
    pub name: String, pub source_name: String, pub node: usize, pub mesh: usize, pub target: usize,
    pub vertex_offset: usize, pub default_weight: f32,
    pub position_offsets: Vec<Vec3>, pub normal_offsets: Vec<Vec3>,
}

pub(crate) struct MergedMesh {
    pub vertices: Vec<RuntimeVertex>, pub indices: Vec<u32>,
    pub weights: Vec<VertexWeight>, pub submeshes: Vec<SubMesh>,
    pub morph_targets: Vec<MorphTargetData>,
}

pub(crate) fn merge_meshes(document: &gltf::Document, buffers: &[gltf::buffer::Data], basis: Mat4) -> Result<MergedMesh> {
    let worlds = vrm_skeleton::compute_node_world_transforms(document);
    let mut out = MergedMesh { vertices: vec![], indices: vec![], weights: vec![], submeshes: vec![], morph_targets: vec![] };
    for node in document.nodes() {
        let Some(mesh) = node.mesh() else { continue; };
        let world = worlds[&node.index()];
        let transform = basis * world;
        if transform.determinant().abs() < 1e-10 { return Err(MmdError::VrmParse("Singular mesh transform".into())); }
        let normal_transform = transform.inverse().transpose();
        let joints: Vec<_> = node.skin().map(|s| s.joints().map(|n| n.index()).collect()).unwrap_or_default();
        let binds = if let Some(skin) = node.skin() {
            let ibms: Vec<_> = skin.reader(|b| Some(&buffers[b.index()])).read_inverse_bind_matrices()
                .map(|v| v.map(|m| Mat4::from_cols_array_2d(&m)).collect())
                .unwrap_or_else(|| vec![Mat4::IDENTITY; joints.len()]);
            if ibms.len()!=joints.len() || ibms.iter().any(|m| !m.is_finite() || m.determinant().abs()<1e-10) {return Err(MmdError::VrmParse("Invalid inverse bind matrices".into()));}
            Arc::new(joints.iter().enumerate().map(|(i, j)| {
                basis * worlds[j] * ibms[i] * transform.inverse()
            }).collect::<Vec<_>>())
        } else { Arc::new(Vec::new()) };
        let names: Vec<String> = mesh.extras().as_ref().and_then(|v| serde_json::from_str::<serde_json::Value>(v.get()).ok())
            .and_then(|v| v.get("targetNames").and_then(|a| a.as_array()).cloned())
            .map(|a| a.iter().map(|v| v.as_str().unwrap_or("morph").to_string()).collect()).unwrap_or_default();
        for primitive in mesh.primitives() {
            if primitive.mode() != gltf::mesh::Mode::Triangles { return Err(MmdError::VrmParse("Only triangle VRM primitives are supported".into())); }
            let reader = primitive.reader(|b| Some(&buffers[b.index()]));
            let raw: Vec<_> = reader.read_positions().ok_or_else(|| MmdError::VrmParse("Mesh has no positions".into()))?.collect();
            let count=raw.len(); let offset=out.vertices.len();
            let normals: Vec<_> = reader.read_normals().map(|v| v.collect()).unwrap_or_else(|| vec![[0.,1.,0.];count]);
            let uvs: Vec<_> = reader.read_tex_coords(0).map(|v| v.into_f32().collect()).unwrap_or_else(|| vec![[0.,0.];count]);
            let joint_values: Option<Vec<_>>=reader.read_joints(0).map(|v| v.into_u16().collect());
            let weight_values: Option<Vec<_>>=reader.read_weights(0).map(|v| v.into_f32().collect());
            if normals.len()!=count || uvs.len()!=count || joint_values.as_ref().map(|v|v.len()!=count).unwrap_or(false)
                || weight_values.as_ref().map(|v|v.len()!=count).unwrap_or(false)
                || (node.skin().is_some() && (joint_values.is_none() || weight_values.is_none())) {
                return Err(MmdError::VrmParse("Inconsistent vertex attributes".into()));
            }
            if reader.read_joints(1).is_some() { return Err(MmdError::VrmParse("More than four skin influences are not supported in this version".into())); }
            for i in 0..count {
                out.vertices.push(RuntimeVertex { position: transform.transform_point3(Vec3::from_array(raw[i])),
                    normal: normal_transform.transform_vector3(Vec3::from_array(normals[i])).normalize_or_zero(), uv: Vec2::new(uvs[i][0],1.-uvs[i][1]) });
                if let (Some(j),Some(w))=(&joint_values,&weight_values) {
                    let sum: f32=w[i].iter().sum();
                    if sum <= 1e-8 || !sum.is_finite() { return Err(MmdError::VrmParse("Invalid skin weights".into())); }
                    let mut bones=[0;4];
                    for k in 0..4 {
                        if j[i][k] as usize >= joints.len() { return Err(MmdError::VrmParse("Skin joint index outside this node's skin".into())); }
                        bones[k]=(joints[j[i][k] as usize]+vrm_skeleton::VIRTUAL_BONE_COUNT) as i32;
                    }
                    out.weights.push(VertexWeight::VrmBdef4 { bones, weights:w[i].map(|v| v/sum), joints:j[i], binds:binds.clone() });
                } else { out.weights.push(VertexWeight::Bdef1 { bone:(node.index()+vrm_skeleton::VIRTUAL_BONE_COUNT) as i32 }); }
            }
            let begin=out.indices.len();
            let mut ids: Vec<_>=reader.read_indices().map(|v| v.into_u32().collect()).unwrap_or_else(|| (0..count as u32).collect());
            if ids.len()%3!=0 || ids.iter().any(|i| *i as usize>=count) { return Err(MmdError::VrmParse("Invalid triangle indices".into())); }
            if transform.determinant()<0. { for tri in ids.chunks_exact_mut(3) { tri.swap(1,2); } }
            out.indices.extend(ids.iter().map(|i| i+offset as u32));
            out.submeshes.push(SubMesh::new(begin as u32,(out.indices.len()-begin) as u32,primitive.material().index().unwrap_or(document.materials().count()) as i32));
            for (target,(positions,target_normals,_)) in reader.read_morph_targets().enumerate() {
                let name=names.get(target).cloned().unwrap_or_else(|| format!("morph_{}",target));
                out.morph_targets.push(MorphTargetData { name:format!("{} [node {}, primitive {}]",name,node.index(),primitive.index()), source_name:name, node:node.index(), mesh:mesh.index(), target, vertex_offset:offset,
                    default_weight:node.weights().or(mesh.weights()).and_then(|v| v.get(target)).copied().unwrap_or(0.),
                    position_offsets:positions.map(|v| v.map(|p| transform.transform_vector3(Vec3::from_array(p))).collect()).unwrap_or_default(),
                    normal_offsets:target_normals.map(|v| v.enumerate().map(|(i,n)| {
                        let length=normal_transform.transform_vector3(Vec3::from_array(normals[i])).length();
                        normal_transform.transform_vector3(Vec3::from_array(n))/length.max(1e-8)
                    }).collect()).unwrap_or_default() });
            }
        }
    }
    Ok(out)
}

pub(crate) fn skin_vertex(position: Vec3, normal: Vec3, bones: &[i32;4], weights: &[f32;4], joints: &[u16;4], binds: &[Mat4], matrices: &[Mat4]) -> (Vec3,Vec3) {
    let mut p=Vec3::ZERO; let mut n=Vec3::ZERO;
    for i in 0..4 {
        if weights[i]==0. { continue; }
        let m=matrices.get(bones[i] as usize).copied().unwrap_or(Mat4::IDENTITY)*binds[joints[i] as usize];
        p+=m.transform_point3(position)*weights[i];
        n+=m.inverse().transpose().transform_vector3(normal)*weights[i];
    }
    (p,n.normalize_or_zero())
}
