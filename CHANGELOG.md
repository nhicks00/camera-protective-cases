# Changelog

## 2026-03-02

### Mevo TPU Output Cleanup
- Removed legacy Mevo TPU cap artifacts from top-level `models/mevo_case/`:
  - `mevo_start_tpu_front_cap.step`
  - `mevo_start_tpu_rear_cap.step`
  - `mevo_start_tpu_caps_report.json`
- Archived those legacy artifacts under `models/mevo_case/archive/` with timestamped names.
- Clarified docs so active Mevo TPU output is the single-file sleeve workflow:
  - `mevo_start_tpu_sleeve.step`
  - `mevo_start_tpu_sleeve_report.json`

### Unified Wall-Stack Update (Both Cameras)
- Updated baseline defaults to:
  - ASA wall thickness: `2.5 mm`
  - TPU wall thickness: `2.0 mm`
  - TPU internal clearance to camera: `0.2 mm`
  - TPU-to-ASA radial gap target: `0.1 mm`
- Applied defaults in both Mevo and MAKI generators so reports now reflect this stack.

### TPU Sleeve Workflow Update
- Mevo TPU output switched from `mevo_start_tpu_liner.*` to `mevo_start_tpu_sleeve.*`.
- MAKI TPU output switched from `maki_live_tpu_liner.*`/unibody preference to `maki_live_tpu_sleeve.*`.
- Added thin front/rear edge-wrap geometry for TPU sleeves (perimeter hold only, not full-face TPU caps).
- Legacy TPU outputs are archived automatically and no longer shown at case-folder top level.

### Model Output Organization
- Reorganized model outputs into:
  - `models/mevo_case/`
  - `models/maki_case/`
- Moved existing Mevo and MAKI STEP/JSON outputs into their respective subfolders.
- Updated script default output paths to keep future generations organized:
  - Mevo scripts now default to `models/mevo_case/`
  - MAKI scripts now default to `models/maki_case/`
- Updated README documentation paths to match the new folder layout.
- Added automatic output archival in all generators:
  - existing files are moved into each case folder's `archive/` subfolder with timestamped filenames before new files are written.

### Mevo TPU Nesting Compatibility
- Added `scripts/generate_mevo_start_tpu_liner.py` to generate a TPU inner sleeve that is dimensioned to fit inside the existing Mevo ASA shell.
- Added auto-fit thickness budgeting:
  - reads `models/mevo_start_case_report.json`,
  - computes maximum TPU wall thickness allowed by current ASA clearance and requested assembly gap,
  - clamps to fit-safe thickness.
- Added Mevo TPU output artifacts:
  - `models/mevo_start_tpu_liner.step`
  - `models/mevo_start_tpu_liner_report.json`
- Added fit-stack reporting fields for radial/axial remaining margins and alignment of tripod/vent openings relative to ASA shell coordinates.
- Updated docs (`README.md`, `README_MEVO_CASE.md`) with the new generator and usage.

### MAKI Sleeve Vent Update
- Updated MAKI ASA sleeve vent logic to enforce a tripod-side 3-panel vent pattern:
  - `8` vents on the primary panel,
  - plus `8` vents on each directly adjacent panel (`24` total).
- Switched enforced vent pattern cutting to deep box subtraction for full wall penetration.
- Preserved tripod opening alignment logic from STEP-derived feature mapping.

### MAKI Front Cap Bezel Inversion
- Updated MAKI front cap geometry to an inverted bezel profile:
  - center panel recessed,
  - outer rim proud/raised,
  - insertion plug moved to the rear side.
- Rear cap geometry kept unchanged.
- Added tuning parameters to `generate_maki_live_caps.py`:
  - `--front-recess-depth`
  - `--front-recess-inset`

### MAKI TPU One-Piece Unibody
- Added `scripts/generate_maki_live_tpu_unibody.py` to create a single connected TPU unit (liner + front + rear fused).
- New outputs:
  - `models/maki_case/maki_live_tpu_unibody.step`
  - `models/maki_case/maki_live_tpu_unibody_report.json`
- Default behavior archives legacy separate TPU parts into `models/maki_case/archive/` so top-level shows only the current unibody TPU output.

### Mevo Rear Closure De-duplication
- Updated `scripts/generate_mevo_case.py` so `mevo_start_case_back_plate.step` is no longer exported by default.
- Added optional flag `--include-back-plate` for legacy scenarios only.
- Default Mevo output path now avoids duplicate rear closure artifacts when front/rear cap workflow is used.

### Project Baseline and Repository Packaging
- Organized the workspace for Git version control with:
  - root `README.md` (project-wide overview),
  - `CHANGELOG.md` (this file),
  - `.gitignore` tuned for CAD workflow artifacts.
- Standardized workflow around smooth B-REP generation (`build123d` / OpenCascade) and STEP exports.

### Mevo Start
- Implemented/maintained parametric Mevo two-piece case generation:
  - sleeve/body model,
  - rear closure plate,
  - report output JSON.
- Enforced symmetric capsule-style front profile logic for clean extension from corrected lens-cap reference.
- Added dedicated cap generator with profile presets:
  - ASA caps,
  - TPU caps.
- Added rear tripod-relief notch support on rear Mevo cap.
- Set rear Mevo I/O cutouts as optional/default-off pending exact measured alignment confirmation.
- Generated current Mevo outputs:
  - `mevo_start_case_body.step`
  - `mevo_start_case_back_plate.step`
  - `mevo_start_case_report.json`
  - `mevo_start_front_cap.step`
  - `mevo_start_rear_cap.step`
  - `mevo_start_caps_report.json`
  - `mevo_start_tpu_front_cap.step`
  - `mevo_start_tpu_rear_cap.step`
  - `mevo_start_tpu_caps_report.json`

### BirdDog MAKI Live
- Implemented/maintained parametric ASA sleeve generator from MAKI reference STEP/PDF dimensions.
- Increased sleeve insertion clearance per request (+2 mm over previous fit pass).
- Reworked side geometry to avoid faceted rectangle-chain approximations and preserve smooth curved panels.
- Added/mirrored extracted side features:
  - tripod mount opening alignment,
  - vent slot extraction and application across multi-panel side faces.
- Implemented TPU liner generator:
  - snug inner fit for camera body,
  - bumper zones,
  - mirrored vent/tripod openings aligned with ASA sleeve.
- Implemented MAKI cap generator (ASA + TPU profiles) with end-face cutout extraction.
- Generated current MAKI outputs:
  - `maki_live_case_sleeve.step`
  - `maki_live_case_report.json`
  - `maki_live_tpu_liner.step`
  - `maki_live_tpu_liner_report.json`
  - `maki_live_front_cap.step`
  - `maki_live_rear_cap.step`
  - `maki_live_caps_report.json`
  - `maki_live_tpu_front_cap.step`
  - `maki_live_tpu_rear_cap.step`
  - `maki_live_tpu_caps_report.json`

### Validation Summary
- Current primary outputs export as STEP and have been validated as B-REP solids.
- Mevo rear cap transient secondary-solid artifact was removed by exporting only largest resulting solid.
- Vents/cutouts for MAKI are extraction-driven from the source STEP, with report files recording detected/applied counts.

### Current Status
- Core deliverables are in place for both camera platforms:
  - hard ASA outer-shell workflow,
  - TPU companion parts,
  - cap generation pipelines.
- Remaining precision work is primarily iterative cutout alignment/tuning to exact hardware fit checks after first physical print.

### Next Planned Adjustments
- Mevo:
  - capture exact rear port/button map from measured hardware and enable tuned rear cap I/O cutouts.
- MAKI:
  - verify front/rear cap cutout exactness against real device plug-in tests,
  - tighten any cutout offsets with measured print fit feedback.
