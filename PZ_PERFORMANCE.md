# PZ Debug performance API (2026-10-16)

`Java_com_shiroha_mmdskin_NativeFunc_UpdateModelProfiled(long, float, ByteBuffer)` is an optional PZ JNI extension. The caller supplies a direct buffer with at least 24 bytes. Success performs one ordinary CPU-skinning update and writes three native-order u64 nanosecond durations: animation/IK and final pose preparation, physics plus post-physics node/IK work, and CPU vertex/Morph update. Invalid buffer/handle returns false without ticking. Java falls back to UpdateModel when the extension is absent.

The ordinary UpdateModel path continues through the same evaluator with profiling disabled. No evaluation order, layer/IK policy or physics step count changes; no clocks are read when disabled. This extension can be kept as a minimal PZ fork API or offered upstream as an optional profiling entry point.

Release DLL compiled for this change. Per user request no native tests or game validation were run. Timing is CPU wall time, not GPU duration. The Java facade retains its own enclosing update measurement and must not add that parent duration to these child measurements.
