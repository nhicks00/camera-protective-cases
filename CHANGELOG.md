# Changelog

## 2026-03-02

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
