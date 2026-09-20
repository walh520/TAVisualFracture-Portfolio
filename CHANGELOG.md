# Changelog

## 0.3.0-phase-ab — current source snapshot

- Added structured solid-domain diagnostics and explicit bake failure reporting.
- Reworked normalized geometry contracts, seed allocation, Power partition bookkeeping, and canonical topology metadata.
- Added the visual landed-XY collision path with configurable footprint scale and correction speed.
- Added moving registered-Box response: sampled boundary motion, horizontal velocity transfer, one-shot lift, contact history, and time-based landed friction decay.
- Added standalone collision fixtures and editor-side contract tests.
- Recorded the current verification boundary; no UE build or Editor restart is implied by this source snapshot.

## Pending

- UBT/UHT and ShaderCompileWorker validation.
- Editor/PIE lifecycle validation with real closed meshes and Landscape assets.
- Visual quality, real drag interaction, rotating-Box feel, and performance validation in a UE project.
