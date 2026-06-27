# Backend Decoupling

## Responsibility Boundary

`legacy_fusion_engine` is an optional compatibility path for the historical `IFusionEngine` runtime. It exists so old fusion implementations and callbacks can continue to receive legacy `NavState` and `loc::LocalizationResult` values during migration.

`pose_graph_backend` is the SLAM backend boundary. It represents graph optimization, backend result processing, map correction, and future loop closure integration through domain result types. A backend is not required to implement `IFusionEngine`.

`IMapOdomAuthority` remains the only owner of the final `map -> odom` correction state. Neither a backend implementation nor the legacy fusion compatibility layer should bypass this authority to publish its own competing final `map -> odom` semantics.

## Current Transition State

`PGOAdapter` currently exposes both `IPoseGraphBackend` and `IFusionEngine`. That is a compatibility implementation, not the required shape for new backends.

A new backend may implement only `IPoseGraphBackend`. `SystemAssembler` will keep the backend and continue without a `legacy_fusion_engine` when the backend does not expose legacy fusion compatibility.

At runtime, `TrajectoryContextImpl` now distributes data to the two paths independently:

- legacy compatibility path receives legacy dead reckoning, lidar odometry, and legacy localization when present
- pose graph backend receives domain `MotionEstimate` and `LocalizationResult` when present

The presence of `legacy_fusion_engine` no longer prevents `pose_graph_backend` from receiving backend inputs.

## Next Steps

The next backend cleanup should make the backend contract more explicit for map sessions and graph structure. Options include introducing an `ISlamBackend`, or extending `IPoseGraphBackend` with submap, constraint, and optimizer-level concepts once those semantics are stable.

Future backend components can include submap management, constraint building, loop closure detection, and pose graph optimization behind backend contracts. Core contracts should stay platform- and algorithm-neutral and should not embed concrete middleware or algorithm names.
