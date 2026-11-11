//! One Bullet world owned exclusively by a worker. Latest-input mailbox, no per-frame waits.
use super::{MMDPhysics, PhysicsConfig};
use glam::Mat4;
use std::collections::HashSet;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

pub struct PhysicsFrame {
    pub transforms: Vec<(usize, Mat4)>,
    pub rotation_only: HashSet<usize>,
    pub indices: HashSet<usize>,
}
struct Job { bones: Vec<Mat4>, transform: Mat4, delta: f32, config: PhysicsConfig, epoch: u64 }
#[derive(Default)]
struct Mailbox {
    pending: Option<Job>, latest: Option<Arc<PhysicsFrame>>, stop: bool, epoch: u64,
    submitted: u64, completed: u64, replaced: u64, worker_ns: u64,
    #[cfg(test)] paused: bool,
}
impl Mailbox {
    fn paused(&self) -> bool {
        #[cfg(test)] { self.paused }
        #[cfg(not(test))] { false }
    }
}
pub struct AsyncPhysics {
    shared: Arc<(Mutex<Mailbox>, Condvar)>,
    worker: Option<JoinHandle<MMDPhysics>>,
}
impl AsyncPhysics {
    pub fn new(mut physics: MMDPhysics) -> Self {
        let shared = Arc::new((Mutex::new(Mailbox::default()), Condvar::new()));
        let input = shared.clone();
        let worker = std::thread::Builder::new().name("PZ MMD physics".into()).spawn(move || {
            let mut epoch = 0;
            loop {
                let job = {
                    let (lock, wake) = &*input; let mut state = lock.lock().unwrap();
                    while !state.stop && (state.pending.is_none() || state.paused()) { state = wake.wait(state).unwrap(); }
                    if state.stop { break; }
                    state.pending.take().unwrap()
                };
                if epoch != job.epoch { physics.reset(); epoch = job.epoch; }
                let start = std::time::Instant::now();
                physics.sync_bodies_with_model_velocity(&job.bones, job.delta, job.transform, &job.config);
                physics.step_simulation(job.delta, &job.config);
                let indices = physics.get_dynamic_bone_indices().clone();
                let (transforms, rotation_only) = physics.get_dynamic_bone_transforms(job.bones.len());
                let result = Arc::new(PhysicsFrame { transforms: transforms.to_vec(), rotation_only: rotation_only.clone(), indices });
                let mut state = input.0.lock().unwrap();
                state.completed += 1; state.worker_ns = start.elapsed().as_nanos() as u64;
                if state.epoch == job.epoch { state.latest = Some(result); }
            }
            physics
        }).expect("Cannot start PZ MMD physics worker");
        Self { shared, worker: Some(worker) }
    }
    pub fn submit(&self, bones: &[Mat4], transform: Mat4, delta: f32, config: PhysicsConfig) -> Option<Arc<PhysicsFrame>> {
        let mut state = self.shared.0.lock().unwrap();
        let mut delta = if delta.is_finite() { delta.max(0.0) } else { 0.0 };
        if let Some(previous) = state.pending.take() { delta += previous.delta; state.replaced += 1; }
        // Match PZ's bounded catch-up after a stall. Replaced snapshots keep elapsed time.
        let epoch = state.epoch;
        state.pending = Some(Job { bones: bones.to_vec(), transform, delta: delta.clamp(0.0,0.1), config, epoch });
        state.submitted += 1;
        let frame = state.latest.clone();
        drop(state); self.shared.1.notify_one(); frame
    }
    pub fn reset(&self) {
        let mut state = self.shared.0.lock().unwrap();
        state.epoch += 1; state.pending = None; state.latest = None;
    }
    pub fn stats(&self) -> [i64;4] {
        let s = self.shared.0.lock().unwrap(); [s.submitted as i64,s.completed as i64,s.replaced as i64,s.worker_ns as i64]
    }
    /// Only used at a view/mode boundary. Normal rendering never joins the worker.
    pub fn finish(mut self) -> MMDPhysics {
        self.stop(); self.worker.take().unwrap().join().expect("Physics worker failed")
    }
    fn stop(&self) {
        let mut state = self.shared.0.lock().unwrap(); state.stop = true; state.pending = None;
        drop(state); self.shared.1.notify_one();
    }
}
impl Drop for AsyncPhysics {
    fn drop(&mut self) {
        // Dropping JoinHandle detaches. The worker retains all native objects until it exits.
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, Vec3};
    use mmd::pmx::rigid_body::{RigidBody, RigidBodyMode, RigidBodyShape};
    fn physics(mode: RigidBodyMode) -> MMDPhysics {
        let rigid = RigidBody { local_name: "worker oracle".into(), universal_name: String::new(),
            bone_index: 0, group: 0, un_collision_group_flag: 0, shape: RigidBodyShape::Sphere,
            size: [0.25,0.0,0.0], position: [0.0;3], rotation: [0.0;3], mass: 1.0,
            move_attenuation: 0.1, rotation_attenuation: 0.1, repulsion: 0.0, friction: 0.0, mode };
        let mut physics=MMDPhysics::new().unwrap();physics.build_physics(&[rigid],&[],&[Mat4::IDENTITY]);physics
    }
    fn wait(worker: &AsyncPhysics, completed: i64) -> Arc<PhysicsFrame> {
        let until=std::time::Instant::now()+std::time::Duration::from_secs(5);
        loop {
            let state=worker.shared.0.lock().unwrap();
            if state.completed>=completed as u64 { return state.latest.clone().expect("missing completed result"); }
            drop(state);assert!(std::time::Instant::now()<until,"worker timed out");
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    #[test]
    fn worker_matches_synchronous_bullet_for_identical_inputs() {
        for mode in [RigidBodyMode::Dynamic,RigidBodyMode::DynamicWithBonePosition] {
            let mut reference=physics(mode);let worker=AsyncPhysics::new(physics(mode));
            let config=PhysicsConfig::default();
            for i in 0..120 {
                let t=i as f32/120.0;
                let bones=[Mat4::from_rotation_translation(Quat::from_rotation_y(t),Vec3::new(t.sin(),0.0,0.0))];
                let root=Mat4::from_translation(Vec3::new(t*2.0,0.0,0.0));
                let delta=if i%17==0 {0.0} else {1.0/120.0};
                reference.sync_bodies_with_model_velocity(&bones,delta,root,&config);reference.step_simulation(delta,&config);
                worker.submit(&bones,root,delta,config.clone());let actual=wait(&worker,i+1);
                let indices=reference.get_dynamic_bone_indices().clone();
                let (expected,rotation_only)=reference.get_dynamic_bone_transforms(1);
                assert_eq!(actual.indices,indices);assert_eq!(&actual.rotation_only,rotation_only);
                assert_eq!(actual.transforms.len(),expected.len());
                for ((ai,a),(ei,e)) in actual.transforms.iter().zip(expected.iter()) {
                    assert_eq!(ai,ei);
                    assert!(a.to_cols_array().iter().zip(e.to_cols_array()).all(|(a,e)|(a-e).abs()<1e-5));
                }
            }
            let recovered=worker.finish();assert_eq!(recovered.rigid_bodies.len(),1);
        }
    }
    #[test]
    fn stalled_worker_keeps_only_latest_input_and_bounded_elapsed_time() {
        let worker=AsyncPhysics::new(physics(RigidBodyMode::Dynamic));
        worker.shared.0.lock().unwrap().paused=true;
        let started=std::time::Instant::now();
        for i in 0..1000 {assert!(worker.submit(&[Mat4::IDENTITY],Mat4::from_translation(Vec3::X*i as f32),0.016,PhysicsConfig::default()).is_none());}
        assert!(started.elapsed()<std::time::Duration::from_secs(1),"submit waited for blocked worker");
        {
            let mut state=worker.shared.0.lock().unwrap();
            assert_eq!(state.completed,0);assert_eq!(state.replaced,999);
            let job=state.pending.as_ref().unwrap();assert!((job.delta-0.1).abs()<1e-6);
            assert_eq!(job.transform.w_axis.x,999.0);state.paused=false;
        }
        worker.shared.1.notify_one();wait(&worker,1);assert_eq!(worker.stats()[1],1);
        worker.finish();
    }
    #[test]
    fn reset_drops_old_generations_and_finish_wakes_a_stalled_worker() {
        let worker=AsyncPhysics::new(physics(RigidBodyMode::Dynamic));
        worker.submit(&[Mat4::IDENTITY],Mat4::IDENTITY,0.016,PhysicsConfig::default());wait(&worker,1);
        worker.shared.0.lock().unwrap().paused=true;
        worker.submit(&[Mat4::IDENTITY],Mat4::IDENTITY,0.016,PhysicsConfig::default());worker.reset();
        {let s=worker.shared.0.lock().unwrap();assert!(s.pending.is_none()&&s.latest.is_none());assert_eq!(s.epoch,1);}
        worker.submit(&[Mat4::IDENTITY],Mat4::IDENTITY,0.0,PhysicsConfig::default());
        worker.shared.0.lock().unwrap().paused=false;worker.shared.1.notify_one();
        let actual=wait(&worker,2);let mut reference=physics(RigidBodyMode::Dynamic);
        reference.sync_bodies_with_model_velocity(&[Mat4::IDENTITY],0.0,Mat4::IDENTITY,&PhysicsConfig::default());
        reference.step_simulation(0.0,&PhysicsConfig::default());
        let (expected,_)=reference.get_dynamic_bone_transforms(1);
        assert_eq!(actual.transforms,expected);
        worker.shared.0.lock().unwrap().paused=true;worker.finish();
    }
}
