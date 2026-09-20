# TAVisualFracture Phase A+B acceptance fixtures

These fixtures are intentionally source-only in the current change. The user requested no Core, UBT/UHT, Editor, or visual compilation run.

## Normal automation fixture

`TA.VisualFracture.PhaseAB.ContractsAndCanonicalInterfaces` covers:

- serialized bake-mode values `0/1/2`;
- explicit Unreal left-handed triangle winding to Core right-handed winding conversion;
- disabled Density Focus with radius zero;
- cached `SolidTetPiece` construction;
- open Cube and Plane rejection, intersecting-shell rejection, and nested-cavity recognition;
- analytic Torus-hole and double-sided open-shell-thickness fields;
- deterministic Seed and canonical-topology output;
- fatal diagnostic absence for a 64-Seed closed cube;
- canonical Patch ownership and triangulation;
- new asset v3 defaults and source-level v2 migration marking.

## Default-parameter geometry regression (source only, not run)

`TA.VisualFracture.PhaseAB.DefaultBakeAndOrientedVolume` covers:

- an analytic clipped box of volume 5, outward tetra faces, and matching sampling-tetra volume;
- a Power bisector exactly coincident with tetra faces and grid-boundary Exterior classification;
- the actual `FTVFBakeSettings` defaults (24 Seeds / 24 grid cells), rather than separate Core defaults;
- zero fatal diagnostics and no discarded positive-volume local pieces;
- per-Chunk oriented render-surface volume matching its solid volume, and balanced surface-area vectors.

The repair uses midpoint Power planes with a canonical Seed-pair order, the same zero halfspace for classification and interpolation, outward tetra winding, and no local `minVolume` deletion. Relative validation tolerance no longer removes Power faces during pairing. These changes do not constitute a fully shared exact-construction kernel: independent Cell clipping still needs execution evidence for near-degenerate multi-plane cases. Missing-owner/coverage/volume Fatal checks remain enabled.

This fixture has **not** been compiled or executed. Passing source checks is not evidence that the reported default bake succeeds in Editor. The new `Clip2` bake signature requires rebaking after a later authorized build.

## Seed art controls (source only, not run)

`TA.VisualFracture.PhaseAB.SeedArtControls` checks fixed-seed Irregularity endpoint differences, repeat determinism, positions remaining inside the solid, independent Size Variation weights, the documented owner bound, changed baked geometry, cancellation, and work-budget termination.

Irregularity now selects between density-weighted best-candidate farthest-point sampling (0) and random volume sampling (1). Each selection batch contains 32 candidates; this is a finite sampling approximation, not a global optimum or a Power clipping neighbour limit. Intermediate values choose the random branch with the requested probability. Existing component quotas and positive-volume sampling remain in use. Work increases at low Irregularity and remains subject to the existing selection budget.

Size Variation retains `w_i = 0.2 * variation * nearestMetricDistanceSquared * randomSigned`. Zero means zero Power weights, not equal Chunk volumes. The `SeedMix1` signature invalidates older baked topology; rebuild and rebake are required. No Core or Editor tests were run in this change.

## Performance gradient

`TA.VisualFracture.PhaseAB.ScaleGradient` is tagged `PerfFilter` and must be requested explicitly. It runs the fixed `64, 100, 244, 256, 1024, 4096` gradient and reports candidate percentiles, PlaneTests, peak temporary bytes, final Chunks, and output triangles.

The gradient is validation evidence, not a promise that every input Mesh must support 4096 Seeds. A pass still requires zero unmatched interfaces, non-manifold output, unexpected empty Cells, coverage gaps, NaN/Inf, and unacceptable volume residual.

## Editor integration fixtures still requiring assets

The Core geometry versions above are present in source but have not been run. The following integration cases still require dedicated checked-in `.uasset` fixtures or an Editor-generated transient MeshDescription: Visual Union Repair of a complex source Mesh with deviation diagnostics, mode routing through the bake service, and a real serialized legacy-v2 asset reload check.
