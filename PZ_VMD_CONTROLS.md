# PZ VMD controls (2026-10-15)

The PZ adapter supplies one immutable animation state per captured game frame through `SetPZAnimationState(model, animationHandles[], frames[], additive[], morphIndices[], morphWeights[])`.

The frame evaluates existing animation layers, PZ additive transforms and head input first. Manual morph values establish the model-specific baseline, then the submitted VMDs run in insertion order, before morph deformation, IK, physics and skinning. Additive VMDs add translation, postmultiply local rotation and add morph weights (clamped to 0..1). A replacement VMD clears bone transforms, IK state and morph weights before sparse evaluation. Java cancels earlier playback when starting a replacement. Missing bone or morph names are ignored by lookup; no model-specific track remapping or rejection is needed.

Java owns file discovery, global per-file options, random delay after each completed playback, wheel slots, time advancement and native animation-handle disposal. Rust receives explicit frames, so shadow cascades and render passes do not advance playback. Model replacement retains the global playback clock while constructing a fresh per-instance native animation cache. Removing the submission restores the current live PZ pose and manual morph baseline on the next update.

The JNI function validates array lengths and finite values, limits concurrent animations to 64, clones the animation `Arc`s before acquiring the model lock, and replaces both arrays atomically under that lock. It uses the existing VMD loader and interpolation. New fields default to empty, preserving upstream callers that do not submit PZ controls. The optional additive evaluator and immutable-frame interface can be proposed upstream separately from PZ file/UI policy.

Validation: real Fubuki and 少女 PMX/JNI tests cover vertex morph deformation/reset, PZ pose preservation/replacement, simultaneous additive clips, missing tracks, manual baseline restoration, fixed/random loop waits, malformed files and cross-model playback. The Rust library's 79 existing tests also pass. Full PZ gameplay remains a separate acceptance step.


## Per-action VMD and optional native turns (2026-10-16)

Java now submits the action-owned VMD before explicit menu/wheel layers. Its frame comes from the owning local PZ AnimationTrack. Replacement changes only that local track's reported duration; additive playback preserves the native duration, loops within it or holds its final frame. Shared clips and remote characters remain unchanged. Action preview permits the selected action layer while suspending global playback clocks. The existing fixed left 90-degree replacement facing correction is shared with the VMD page.

`SetPZActionTurnTransforms(model, indices[], values[])` supplies optional frame-local turn deltas, in native space, with ten floats per bone: translation XYZ, quaternion XYZW, scale XYZ. Java derives these from calibrated posed/neutral QPS, so static PMX calibration is removed from the delta. Rust validates array lengths, finite values and quaternion length, normalizes rotation and skips out-of-range bones during evaluation. Empty arrays clear the overlay; new models default to no overlay.

For a full action replacement, enabled native turning applies these deltas after the VMD layers and fixed facing correction, before morphs/IK/physics/skinning. Disabled native turning uses direct character-facing yaw in the shared body/held-item snapshot. Additive mode always keeps the complete native pose and native yaw, including native turn animation; it ignores the replacement-only turning preference and submits no extra turn layer. Explicit global replacement clears the action turn overlay.

The JAR and DLL must be deployed together and the game fully restarted. Real PMX/JNI probes cover replacement/additive clocks, loop/hold, fixed facing, turn overlay/clear, action preview, and additive turning regardless of the stored preference. The installed ZombieBuddy converter is used to verify real AnimationTrack duration advice, end events, loop behavior and shared/remote safety. Rust's 79 library tests pass; final gameplay visuals remain to be accepted in PZ.
