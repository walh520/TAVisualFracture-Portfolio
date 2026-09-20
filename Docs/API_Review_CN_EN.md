# API review / API 审查

## Reflected entry points

- `UTAVisualFractureComponent`: target mesh, bake settings, response settings, Landscape snapshot, impact settings, and editor preview actions.
- `UTAVisualFractureAsset`: serialized bake signature, chunk geometry, Bond data, diagnostics, and statistics.
- `UTAVisualFractureColliderComponent`: registered external Static Mesh target and manual Box configuration.
- `FTVFVisualCollisionSettings`: landed XY collision, footprint scale, correction speed, rolling, external transfer/lift, and friction decay.

## Core contracts

- `CollisionScene::Advance` retains the strict fixed-step path.
- `CollisionScene::AvoidOnGround` provides the lightweight landed-only visual path.
- `CollisionScene::SampleEnvironment` stores external Box motion samples without resetting fracture state.
- `MotionSystem::ApplyImpact` remains the single future-reusable impact data contract.

The visual path intentionally does not expose a gameplay hit entry point in this release.
