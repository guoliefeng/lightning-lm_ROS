# Relocalization Main Path

## Old Path

The previous trajectory keyframe path was:

```text
CloudPtr -> ILocalizer::ProcessKeyframeScan() -> ILocalizer::GetLocalizationResult()
```

`TrajectoryContextImpl` directly orchestrated the legacy localizer. This made the legacy localization interface the effective SLAM runtime center, even after the relocalization contracts existed.

## New Path

The keyframe path is now:

```text
CloudPtr
  -> domain::sensor::ScanSnapshot
  -> RelocalizationCoordinator::ProcessScan()
  -> IGlobalInitializer / ILocalTracker
  -> IMapOdomAuthority
  -> LocalizationResult event
```

`TrajectoryContextImpl` builds a `ScanSnapshot` from each keyframe scan. The snapshot carries:

- `registered_scan`
- `stamp_ns`
- `frame_id`
- `source_id`
- `odom_pose_hint` from the latest state estimate, or the latest motion estimate when no fused state is available

`RelocalizationCoordinator` owns the runtime mode transition:

- accumulate scan structure until initialization is meaningful
- run global initialization from idle/accumulating/lost states
- run local tracking after initialization succeeds
- freeze `map -> odom` when tracking is lost

## Legacy Compatibility

The direct localizer call remains only as a compatibility fallback:

```text
CloudPtr -> ILocalizer::ProcessKeyframeScan() -> LocalizationResult
```

That fallback is used when a trajectory has no `RelocalizationCoordinator`.

For the default assembler path, the legacy localizer is wrapped by `LegacyLocalizerRelocalizationAdapter`, which implements both `IGlobalInitializer` and `ILocalTracker`. This keeps the existing localization algorithm available behind the new semantic contracts without making `TrajectoryContextImpl` directly orchestrate `ILocalizer`.

## Map To Odom Authority

`map -> odom` is modeled as a single authority through `IMapOdomAuthority`.

Only the coordinator updates it from successful localization output. When relocalization enters a lost state, the coordinator freezes the authority so other runtime code cannot continue applying implicit corrections as if tracking were healthy.

The bridge may still translate localization events into legacy TF callbacks, but the correction state itself is owned by `IMapOdomAuthority`.

## Event Output

Runtime output remains event-driven:

- `RelocalizationCoordinator` emits relocalization state changes such as accumulating, global initializing, tracking, and lost.
- `TrajectoryContextImpl` stores successful coordinator results, feeds the state estimator, emits `OnLocalizationResult`, and publishes `OnCloudInWorld` for visualization.
- `LegacyRuntimeBridge` continues to adapt localization and cloud events into TF, loc_state, and Pangolin UI updates.

## Next Evolution

The next steps are to replace the legacy adapter with real contract-native implementations:

- `IGlobalInitializer` backed by a map/session-aware global alignment component.
- `ILocalTracker` backed by a local tracking backend that consumes registered scans and odom hints directly.

After that, a `MapSession` or SLAM backend can own persistent map state while the coordinator remains responsible for relocalization mode transitions and `map -> odom` authority.
