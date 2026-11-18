use glam::{Mat4, Quat, Vec3};
use mmd_engine::model::{load_vrm_with_extensions, ColliderShape, VertexWeight};
use serde_json::{json, Value};

struct Fixture(std::path::PathBuf);
impl Drop for Fixture { fn drop(&mut self) { let _=std::fs::remove_dir_all(&self.0); } }
fn fixture(version_one:bool) -> Fixture {
    static ID:std::sync::atomic::AtomicU64=std::sync::atomic::AtomicU64::new(0);
    let dir=std::env::temp_dir().join(format!("pz-vrm-regression-{}-{}",std::process::id(),ID.fetch_add(1,std::sync::atomic::Ordering::Relaxed)));
    std::fs::create_dir_all(&dir).unwrap();
    let mut bytes=vec![]; let mut views=vec![]; let mut accessors=vec![];
    let mut add=|values:&[f32],kind:&str,count:usize,position:bool| {
        let begin=bytes.len(); for v in values { bytes.extend(v.to_le_bytes()); }
        let i=views.len(); views.push(json!({"buffer":0,"byteOffset":begin,"byteLength":bytes.len()-begin}));
        let mut a=json!({"bufferView":i,"componentType":5126,"count":count,"type":kind});
        if position {a["min"]=json!([0.,0.,0.]);a["max"]=json!([1.,1.,0.]);}
        accessors.push(a); i
    };
    let p=add(&[0.,0.,0.,1.,0.,0.,0.,1.,0.],"VEC3",3,true);
    let n=add(&[0.,0.,1.,0.,0.,1.,0.,0.,1.],"VEC3",3,false);
    let w=add(&[1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.],"VEC4",3,false);
    let morph=add(&[0.1,0.,0.,0.1,0.,0.,0.1,0.,0.],"VEC3",3,true);
    let normal_morph=add(&[1.,0.,0.,1.,0.,0.,1.,0.,0.],"VEC3",3,false);
    let ibm0=add(&Mat4::from_translation(Vec3::new(2.,-1.,0.)).to_cols_array(),"MAT4",1,false);
    let ibm1=add(&Mat4::from_translation(Vec3::new(-2.,-2.,0.)).to_cols_array(),"MAT4",1,false);
    drop(add);
    let j=views.len();let begin=bytes.len();bytes.extend([0u8;24]);
    views.push(json!({"buffer":0,"byteOffset":begin,"byteLength":24}));
    accessors.push(json!({"bufferView":j,"componentType":5123,"count":3,"type":"VEC4"}));
    let mesh=json!({"extras":{"targetNames":["Smile"]},"primitives":[{"attributes":{"POSITION":p,"NORMAL":n,"JOINTS_0":j,"WEIGHTS_0":w},"material":0,"targets":[{"POSITION":morph,"NORMAL":normal_morph}]}]});
    let extensions=if version_one { json!({
        "VRMC_vrm":{"specVersion":"1.0","humanoid":{"humanBones":{"hips":{"node":0},"head":{"node":1}}},"expressions":{"preset":{"blink":{"isBinary":true,"morphTargetBinds":[{"node":3,"index":0,"weight":1.0}]}}}},
        "VRMC_springBone":{"specVersion":"1.0","colliders":[{"node":4,"shape":{"capsule":{"offset":[0.1,0.,0.],"tail":[0.1,1.,0.],"radius":0.2}}}],"colliderGroups":[{"colliders":[0]}],"springs":[{"joints":[{"node":0},{"node":1}],"colliderGroups":[0]}]}
    }) } else { json!({"VRM":{"specVersion":"0.0","humanoid":{"humanBones":[{"bone":"hips","node":0},{"bone":"head","node":1},{"bone":"leftThumbProximal","node":2},{"bone":"leftThumbIntermediate","node":3}]},
        "blendShapeMaster":{"blendShapeGroups":[{"presetName":"blink","binds":[{"mesh":1,"index":0,"weight":100.0}]}]},
        "secondaryAnimation":{"colliderGroups":[{"node":4,"colliders":[{"offset":{"x":0.1,"y":0.,"z":0.3},"radius":0.2}]}],"boneGroups":[]},
        "materialProperties":[{"name":"test","shader":"VRM/MToon","floatProperties":{"_RimFresnelPower":3.25}}]}}) };
    let doc=json!({"asset":{"version":"2.0"},"extensionsUsed":if version_one {vec!["VRMC_vrm","VRMC_springBone"]} else {vec!["VRM"]},"extensions":extensions,
        "buffers":[{"uri":"mesh.bin","byteLength":bytes.len()}],"bufferViews":views,"accessors":accessors,
        "scenes":[{"nodes":[0,2,3]}],"scene":0,"nodes":[
            {"name":"hips","translation":[0.,1.,0.],"children":[1,4]}, {"name":"head","translation":[0.,1.,0.]},
            {"name":"meshA","translation":[2.,0.,0.],"mesh":0,"skin":0}, {"name":"meshB","translation":[-2.,0.,0.],"mesh":1,"skin":1},
            {"name":"collider_only","translation":[0.,0.5,0.]}],
        "skins":[{"joints":[0],"inverseBindMatrices":ibm0},{"joints":[1],"inverseBindMatrices":ibm1}],"meshes":[mesh.clone(),mesh],
        "materials":[{"name":"test","alphaMode":"MASK","alphaCutoff":0.9,"pbrMetallicRoughness":{"baseColorFactor":[1.,1.,1.,0.7]},"extensions":{"VRMC_materials_mtoon":{"specVersion":"1.0","rimLightingMixFactor":0.37}}}]
    });
    std::fs::write(dir.join("mesh.bin"),bytes).unwrap();
    std::fs::write(dir.join("test.vrm"),doc.to_string()).unwrap(); Fixture(dir)
}

fn check(version_one:bool) {
    let fixture=fixture(version_one);
    let mut loaded=load_vrm_with_extensions(fixture.0.join("test.vrm")).unwrap();
    let ext=&loaded.extensions;
    assert_eq!(ext.spring_bones.colliders[0].node,7,"non-skin collider node retained");
    match ext.spring_bones.colliders[0].shape {
        ColliderShape::Sphere{offset,radius}|ColliderShape::Capsule{offset,radius,..}=>{
            assert!((radius-2.5).abs()<1e-5); assert!((offset[0].abs()-1.25).abs()<1e-5);
            if !version_one {assert!((offset[2]-3.75).abs()<1e-5,"VRM0 Unity extension handedness must be converted before glTF basis");}
        }
    }
    let model=&mut loaded.model;
    assert_eq!(model.bone_manager.bone_count(),8);assert_eq!(model.vertices.len(),6);
    if !version_one {
        assert_eq!(model.bone_manager.find_bone_by_name("左親指０"),Some(5));
        assert_eq!(model.bone_manager.find_bone_by_name("左親指１"),Some(6));
    }
    match &model.weights[3] {VertexWeight::VrmBdef4{bones,..}=>assert_eq!(bones[0],4),_=>panic!("skin weight")}
    let sign=if version_one {1.}else{-1.};
    assert!((model.update_positions[0].x-25.*sign).abs()<1e-5);
    assert!((model.update_positions[3].x+25.*sign).abs()<1e-5);
    assert_eq!(model.materials[0].diffuse.w,1.0,"MASK cutoff is evaluated per texel, not as a fractional material fade");
    let metadata:Value=serde_json::from_str(&model.vrm_metadata).unwrap();
    assert!((metadata["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"][3].as_f64().unwrap()-0.7).abs()<1e-5);
    assert_eq!(metadata["materials"][0]["extensions"]["VRMC_materials_mtoon"]["rimLightingMixFactor"],0.37);
    let baseline=model.update_positions.clone();
    let expression=model.morph_manager.find_morph_by_name("まばたき").unwrap();
    model.morph_manager.set_morph_weight(expression,1.);
    model.begin_animation();model.update_morph_animation();model.update_node_animation(false);model.end_animation();model.update();
    assert!(model.update_positions[0].distance(baseline[0])<1e-5,"other mesh's target zero must stay unchanged");
    assert!((model.update_positions[3].x-baseline[3].x-1.25*sign).abs()<1e-5);
    assert!(model.update_normals[3].x.abs()>0.6,"normal target must accompany position target");
    if version_one {
        model.set_physics_enabled(false);
        model.pz_manual_morphs=vec![(expression,0.4)];model.tick_animation(0.);
        assert!(model.update_positions[3].distance(baseline[3])<1e-5,"manual binary expression respects threshold");
        model.pz_manual_morphs=vec![(expression,0.6)];model.tick_animation(0.);
        assert!((model.update_positions[3].x-baseline[3].x-1.25).abs()<1e-5);
        model.pz_manual_morphs.clear();
    }
    model.morph_manager.reset_all_weights();model.begin_animation();
    model.bone_manager.get_bone_mut(4).unwrap().animation_rotate=Quat::from_rotation_y(0.5);
    model.update_morph_animation();model.update_node_animation(false);model.end_animation();model.update();
    assert!(model.update_positions[0].distance(baseline[0])<1e-5);
    assert!(model.update_positions[3].distance(baseline[3])>1.);
}
#[test] fn vrm_zero_multi_skin_mesh_scopes_and_non_skin_colliders(){check(false);}
#[test] fn vrm_one_node_scopes_capsules_and_orientation(){check(true);}

#[test]
fn vmd_exact_raw_face_animates_both_meshes_and_preserves_custom_shape() {
    use mmd_engine::animation::{MorphKeyframe, Motion, VmdAnimation};
    use std::sync::Arc;

    let f = fixture(true);
    let path = f.0.join("test.vrm");
    let mut doc: Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
    for mesh in doc["meshes"].as_array_mut().unwrap() {
        mesh["extras"]["targetNames"] = json!(["笑い", "costume"]);
        let targets = mesh["primitives"][0]["targets"].as_array_mut().unwrap();
        targets.push(targets[0].clone());
    }
    std::fs::write(&path, doc.to_string()).unwrap();
    let mut model = load_vrm_with_extensions(path).unwrap().model;
    model.set_physics_enabled(false);
    let name = "costume [node 2, primitive 0]";
    let costume = model.morph_manager.find_morph_by_name(name).unwrap();
    model.pz_manual_morphs = vec![(costume, 0.25)];
    let mut motion = Motion::new();
    motion.insert_morph_keyframe("笑い", MorphKeyframe::new(0, 0.6));
    motion.insert_morph_keyframe(name, MorphKeyframe::new(0, 1.0));
    let base = model.update_positions.clone();
    model.pz_vmd_layers = vec![(Arc::new(VmdAnimation::from_motion(motion)), 0.0, true)];
    model.tick_animation(0.0);
    assert!((model.update_positions[0].x - base[0].x - 1.25 * 0.85).abs() < 1e-5);
    assert!((model.update_positions[3].x - base[3].x - 1.25 * 0.6).abs() < 1e-5);
}

#[test] fn malformed_extension_and_cycle_return_errors() {
    for cycle in [false,true] {
        let f=fixture(true);let path=f.0.join("test.vrm");
        let mut doc:Value=serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        if cycle {doc["nodes"][1]["children"]=json!([0]);}
        else {doc["extensions"]["VRMC_springBone"]["colliders"][0]["node"]=json!(999);}
        std::fs::write(&path,doc.to_string()).unwrap();
        assert!(load_vrm_with_extensions(path).is_err());
    }
}
