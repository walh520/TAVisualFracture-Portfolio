# Dependencies

## Engine

- Unreal Engine 5.7.4.
- Core, CoreUObject, Engine, GeometryCore, MeshDescription, StaticMeshDescription, Landscape, ProceduralMeshComponent, AssetTools, AssetRegistry, LevelEditor, PropertyEditor, Slate, SlateCore, and UnrealEd as declared by the plugin modules.

## Module boundary

- `TAVisualFractureCore` is a C++20, UObject-independent reference core. It uses Core and privately consumes GeometryCore.
- `TAVisualFractureRuntime` exposes reflected assets/components and depends on Core, Engine, and ProceduralMeshComponent.
- `TAVisualFractureEditor` owns editor-only bake, Landscape, MeshDescription, Details, preview, and temporary rendering integration.

## Test toolchain

The standalone runner uses the MSVC C++20 toolchain through `Tests/Run-CollisionStandalone.ps1`. It is independent of UBT/UHT and does not require a project asset or Editor process.

## Exclusions

This repository does not include Unreal Engine source, generated binaries, Intermediate/DerivedDataCache/Saved data, project assets, commercial content, credentials, machine paths, or media files.
