# PZ character window (2026-10-16)

The primary first-person CPU mesh can have its neck/head collapsed. A second UI
view therefore cannot reuse those final positions to display the complete model.

`CopyCharacterWindowGeometry(long model, ByteBuffer output)` copies vertex-count
positions followed by vertex-count normals (six native-endian f32 per vertex).
It returns the vertex count, or zero for an invalid model, non-direct/unaligned
buffer or insufficient capacity. The output buffer must hold at least 24 bytes
per vertex. This is an additive JNI API; the existing primary APIs are unchanged.

The model's original vertices plus the evaluated Group/Flip/Vertex morph weights
are skinned with `camera_skinning_matrices()` before first-person head collapse.
This preserves the same evaluated pose, view following and physics. It does not
tick animation, blink, IK or Bullet, update morph/bone state, change the primary
matrices/vertices/UVs, or load another model. Only the requested secondary CPU
skinning and buffer copy are added; the consumer is responsible for frame/thread
ownership. The existing model mutex covers the read and output construction.

This can be upstreamed as an optional secondary-view geometry API alongside the
PZ presentation helpers. It has no Minecraft dependencies. The PZ Java side
owns the UI camera, transparent target and render-thread submission.

Validation: 81 native library tests including grouped vertex morphs, primary
pose/vertex invariance and complete head; two actual PMX/JNI/GPU cases and
0/45/90/180/270-degree root-camera image comparisons. See the parent workspace's
`build/character-window`. Final PZ gameplay/UI rendering still needs validation.
