# PZ VMD controls (2026-10-15)

The PZ adapter supplies one immutable animation state per captured game frame through `SetPZAnimationState(model, animationHandles[], frames[], additive[], morphIndices[], morphWeights[])`.

The frame evaluates existing animation layers, PZ additive transforms and head input first. Manual morph values establish the model-specific baseline, then the submitted VMDs run in insertion order, before morph deformation, IK, physics and skinning. Additive VMDs add translation, postmultiply local rotation and add morph weights (clamped to 0..1). A replacement VMD clears bone transforms, IK state and morph weights before sparse evaluation. Java cancels earlier playback when starting a replacement. Missing bone or morph names are ignored by lookup; no model-specific track remapping or rejection is needed.

Java owns file discovery, global per-file options, random delay after each completed playback, wheel slots, time advancement and native animation-handle disposal. Rust receives explicit frames, so shadow cascades and render passes do not advance playback. Model replacement retains the global playback clock while constructing a fresh per-instance native animation cache. Removing the submission restores the current live PZ pose and manual morph baseline on the next update.

The JNI function validates array lengths and finite values, limits concurrent animations to 64, clones the animation `Arc`s before acquiring the model lock, and replaces both arrays atomically under that lock. It uses the existing VMD loader and interpolation. New fields default to empty, preserving upstream callers that do not submit PZ controls. The optional additive evaluator and immutable-frame interface can be proposed upstream separately from PZ file/UI policy.

Validation: real Fubuki and 少女 PMX/JNI tests cover vertex morph deformation/reset, PZ pose preservation/replacement, simultaneous additive clips, missing tracks, manual baseline restoration, fixed/random loop waits, malformed files and cross-model playback. The Rust library's 79 existing tests also pass. Full PZ gameplay remains a separate acceptance step.
