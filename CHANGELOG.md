# Changelog

## 2026-03-02

### MAKI Combined Dual-Material Body Export
- Added `scripts/generate_maki_live_dual_material_body.py`.
- New output:
  - `models/maki_case/maki_live_body_dual_material.step` (single STEP assembly with `TPU_Sleeve` + `ASA_Shell`)
  - `models/maki_case/reports/maki_live_dual_material_report.json`
- The new generator composes existing validated MAKI ASA + TPU generators and aligns TPU into ASA automatically.
- Report includes:
  - axial alignment offset,
  - front/back axial gaps,
  - radial fit gaps,
  - warnings for any interference.
- Default fit verified in report:
  - radial gap: `~0.1 mm` each side,
  - axial front/back gap: `~2.1 mm` each end.

### MAKI Side-Trio Vent Size Correction
- Increased side `3 + 3` vent cutout dimensions (kept location and count unchanged):
  - previous side trio size was about `4.37 x 1.50 mm` (ASA) and `4.42 x 1.55 mm` (TPU),
  - updated side trio size now about `7.00 x 2.25 mm` (ASA) and `7.08 x 2.33 mm` (TPU).
- The 24-vent bottom-connected bank (`8 x 3`) was intentionally left unchanged.
- Applied in:
  - `scripts/generate_maki_live_case.py`
  - `scripts/generate_maki_live_tpu_liner.py`
- Regenerated and validated through-cuts:
  - ASA vents open: `30/30`
  - TPU vents open: `30/30`

### MAKI Side-Trio Exact-Style Sizing + Front Camera Aperture Trim
- Updated side `3 + 3` vents again to match the full vent style size from the main vent bank:
  - side trio now matches tripanel slot dimensions (ASA and TPU), rather than using reduced scaled values.
- Reduced MAKI front camera aperture opening by `2.0 mm`:
  - applied to integrated-front sleeve extraction (`generate_maki_live_case.py`)
  - applied to front-cap extraction logic (`generate_maki_live_caps.py`) for consistency.
- Regenerated:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - `models/maki_case/maki_live_body_dual_material.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - reports updated under `models/maki_case/reports/`

### Mevo Production Assembly Upgrade (Dual-Material Back Cap + TPU-Aware Fit)
- Updated `scripts/generate_mevo_dual_material_case.py` for a production-oriented two-part assembly workflow:
  - Main body remains dual-material (`TPU_Sleeve` + `ASA_Shell`).
  - Back cap is now generated as a dual-material assembly (`ASA_Back_Cap` + `TPU_Back_Gasket`).
  - Compatibility ASA-only cap export is still generated.
- Implemented TPU-aware back-cap fit tuning:
  - Back-cap lip undersize changed from `0.10 mm` to `0.28 mm` total for more reliable real-world insert/removal with TPU interfaces.
- Implemented tongue-and-groove style engagement:
  - Added a two-stage cap lip (wider tongue stage + slimmer friction stage).
  - Added matching rear groove seat in the main body opening.
- Added back-cap TPU gasket ring:
  - Default enabled, thin inner-face gasket for damped closure and reduced rattle.
- New outputs:
  - `models/mevo_case/mevo_start_back_cap_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step` (compatibility)
  - updated `models/mevo_case/reports/mevo_start_dual_material_report.json`
  - updated `models/mevo_case/reports/mevo_validation_summary.json`

### MAKI Vent Panel Orientation Correction (Bottom-Connected 24-Bank)
- Corrected MAKI vent panel placement so the `24`-vent bank sits on the tripod-side bottom-connected 3-panel region:
  - center lane `8`
  - adjacent lane left `8`
  - adjacent lane right `8`
- Fixed prior misplacement where the adjacent `16` vents were on the direct side walls.
- Side `3 + 3` vent group was also moved to the opposite end as requested.
- Applied to both:
  - `scripts/generate_maki_live_case.py`
  - `scripts/generate_maki_live_tpu_liner.py`
- Regenerated and validated:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - `models/maki_case/reports/maki_live_case_report.json`
  - `models/maki_case/reports/maki_live_tpu_sleeve_report.json`
  - `models/maki_case/reports/maki_validation_summary.json`

### MAKI Vent Pattern Correction (`24 + 6`)
- Corrected MAKI vent layout implementation to match requested pattern:
  - `24` vents above tripod area across the 3-panel bank (`8 x 3`).
  - Plus side vents: `3` on left side and `3` on right side.
- Fixed vent-slot orientation on side/corner panels (previously swapped tangential vs vertical dimensions, which visually collapsed rows into long slits).
- Applied in both generators:
  - `scripts/generate_maki_live_case.py`
  - `scripts/generate_maki_live_tpu_liner.py`
- Regenerated:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - `models/maki_case/reports/maki_live_case_report.json`
  - `models/maki_case/reports/maki_live_tpu_sleeve_report.json`
  - `models/maki_case/reports/maki_validation_summary.json`
- Validation result after regeneration:
  - ASA sleeve vents open: `30/30`
  - TPU sleeve vents open: `30/30`
  - Tripod cutout remains open through both ASA and TPU sleeves.

### MAKI Through-Cut Validation + Regeneration (STEP-Driven)
- Fixed MAKI tripod opening generation in both sleeve generators:
  - `scripts/generate_maki_live_case.py`
  - `scripts/generate_maki_live_tpu_liner.py`
- Root cause was side-hole subtraction orientation/placement on XZ/ZX planes; this caused non-through tripod cuts.
- Updated cut generation so tripod openings are true circular through-cuts in both:
  - ASA sleeve (`maki_live_case_sleeve.step`)
  - TPU sleeve (`maki_live_tpu_sleeve.step`)
- Regenerated and verified:
  - MAKI ASA sleeve vents: `24/24` open (8 center `Y` + 16 side `X`)
  - MAKI TPU sleeve vents: `24/24` open
  - MAKI ASA + TPU tripod openings: through-cut and open
  - MAKI rear cap cutouts: `5/5` center-open
- Added validation summary outputs:
  - `models/maki_case/reports/maki_validation_summary.json`
  - `models/mevo_case/reports/mevo_validation_summary.json`

### Cutout Alignment Corrections (MAKI + Mevo)
- MAKI rear-cap extraction fix:
  - `generate_maki_live_caps.py` now interrogates all STEP solids (not just largest solid) when extracting end-face cutouts.
  - Added rear tiny-hole filtering so fastener holes are not mistaken for port access cutouts.
  - Rear-cap report now contains port-sized cutouts (no longer only four corner micro-holes).
- Mevo dual-material cap/front fixes:
  - `generate_mevo_dual_material_case.py` front lens opening moved off-center using `lens_center_y_mm=16.5` to avoid centered-misaligned lens hole behavior.
  - Rear back-cap giant utility slot is now disabled by default (`include_back_utility_slot=false`).
- Regenerated:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/reports/maki_live_caps_report.json`
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`

### MAKI Port Oversize + Sleeve Vent/Tripod Alignment Pass
- Updated `generate_maki_live_caps.py`:
  - Increased default port cutout oversize to `1.5 mm` (`cutout_extra_mm=1.5`) to accommodate cable boot width, not just metal connector geometry.
  - Raised rear cutout minimum-size filter to remove corner fastener-hole artifacts from rear-cap port cutouts.
- Updated `generate_maki_live_case.py`:
  - Reworked tripanel vent clustering to lock onto the true 8-row rear vent bank from STEP geometry and avoid front outlier slots.
  - Vent panel centers now derive from STEP side-panel vent families (`x≈left/center/right`) so sleeve vent cuts align with three-panel vent layout.
  - Increased tripod sleeve cutout oversize via dedicated parameter (`tripod_cutout_extra_mm`) for better mounting access.
- Regenerated:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/reports/maki_live_case_report.json`
  - `models/maki_case/reports/maki_live_caps_report.json`

### Mevo Dual-Material Height/Depth Fit Correction
- Corrected Mevo dual-material default fit envelope to match Mevo Start dimensions:
  - camera nominal: `34.0 x 75.5 x 87.0 mm` (W x H x D)
  - TPU inner cavity (snug): `34.3 x 75.8 x 87.3 mm` (`+0.15 mm` fit clearance per side/end)
- This replaces the undersized prior dual-material defaults (`34.3 x 50.3 x 85.0 mm`) that caused the body to appear too short in height.
- Regenerated:
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`

### STEP-Interrogation Upgrade For Cutout Accuracy (MAKI)
- Upgraded MAKI feature extraction to follow a direct B-Rep interrogation strategy (build123d `import_step`) instead of relying on implicit guesses.
- `scripts/generate_maki_live_case.py`:
  - Added cylindrical-face-first tripod detection:
    - filters cylindrical faces by thread-radius target (`~3.175 mm` ± tolerance),
    - requires downward thread-axis normal (`-Z`),
    - constrains expected centerline/depth region,
    - falls back to circular-edge detection only if no valid cylinder is found.
  - Report now includes:
    - `step_side_features.tripod_source`
    - `tripod_cyl_candidate_count`
    - `tripod_edge_candidate_count`
    - `tripod_detected_raw` (raw STEP-space center/radius/normal)
    - `tripod_applied` (mapped case-space location used in cutout generation)
- `scripts/generate_maki_live_caps.py`:
  - End-cap cutout extraction now explicitly anchors to detected front/back end planes and then collects internal-loop cutouts on those planes (with tolerance and window fallback).
  - Report now includes `report.cutouts.end_planes` with front/back plane coordinates used for extraction.
- Regenerated MAKI outputs and reports:
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/reports/maki_live_case_report.json`
  - `models/maki_case/reports/maki_live_caps_report.json`

### Front-Integrated Rule Applied (Both Cameras)
- Applied the new assembly rule: front cap is integrated into the sleeve/body; only rear/back cap remains a separate part.
- Mevo dual-material generator (`generate_mevo_dual_material_case.py`):
  - Default changed to closed/front-integrated body (`open_front_ovular=false`).
  - Front lens + LED cutouts are enabled by default in closed-front mode.
  - Added `--open-front-ovular` (legacy override) and `--disable-front-lens-led-cutouts`.
- MAKI sleeve generator (`generate_maki_live_case.py`):
  - Added integrated-front mode as default (`front_integrated=true`).
  - Front-wall cutouts are extracted from source STEP end-face loops and cut through the integrated front wall.
  - Added `--open-front` (legacy override) and `--no-front-cutouts`.
- MAKI cap generator (`generate_maki_live_caps.py`):
  - Default export is now rear-cap only.
  - Front cap export is legacy optional via `--include-front-cap`.
- Regenerated and updated active outputs:
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`
  - `models/maki_case/maki_live_case_sleeve.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/reports/maki_live_case_report.json`
  - `models/maki_case/reports/maki_live_caps_report.json`
- Archived legacy Mevo separate front/rear cap artifacts from top-level outputs.

### Mevo Dual-Body Ovular Profile Regression Fix
- Updated `scripts/generate_mevo_dual_material_case.py` to remove rounded-rectangle default shell generation for the dual body.
- Dual-body defaults now use a true vertical capsule/ovular cross-section for:
  - ASA outer shell profile
  - ASA inner cavity profile
  - TPU outer profile
  - TPU inner cavity profile
- Changed dual-body default front behavior to open ovular front (`open_front_ovular=true`) to match the style of `mevo_start_case_body` and avoid the previous front circular-cutout appearance.
- Added optional CLI controls:
  - `--closed-front`
  - `--enable-front-lens-led-cutouts`
  - `--disable-capsule-profile` (fallback only)
- Regenerated:
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`

### Mevo Review-Spec Alignment (Dual-Material Bucket + Back Cap)
- Updated `scripts/generate_mevo_dual_material_case.py` to match the latest reviewed Mevo spec exactly:
  - TPU inner cavity: `34.3 x 50.3 x 85.0 mm`
  - TPU wall: `1.8 mm`
  - ASA wall: `2.2 mm`
  - TPU-to-ASA interface gap: `0.0 mm`
  - TPU inner corner radius: `4.0 mm`
  - ASA outer corner radius: `6.0 mm` (now default target, no auto-expansion)
  - Front lens cutout: `32.0 mm`
  - Front LED hole: `3.0 mm` at `+12.0 mm` Y from lens center
  - Sun-hood depth: `3.0 mm`
  - Bottom tripod hole: `20.5 mm` diameter, centered `43.2 mm` from front
  - Back-cap lip: `5.0 mm` depth, `0.1 mm` total undersize
  - Back utility slot: `15.0 mm` width with `15.0 mm` top and `10.0 mm` bottom margins
- Regenerated:
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`
- Confirmed named solids remain present in the body STEP: `TPU_Sleeve`, `ASA_Shell`.
- Note: this supersedes prior dual-material tripod defaults (`25.4 mm` and earlier `12.7 mm`) for the active Mevo reviewed workflow.

### Mevo Tripod Hole Diameter Update
- Updated Mevo bottom tripod opening default to `25.4 mm` (`1.0 in`) for:
  - `generate_mevo_case.py` (ASA case body)
  - `generate_mevo_start_tpu_liner.py` (TPU sleeve)
  - `generate_mevo_dual_material_case.py` (ASA+TPU dual-material body)
- Regenerated Mevo outputs and reports with the new diameter.

### Mevo Dual-Material Body + Pure ASA Back Cap Workflow
- Added `scripts/generate_mevo_dual_material_case.py`.
- New outputs:
  - `models/mevo_case/mevo_start_body_dual_material.step` (single STEP with named bodies `TPU_Sleeve` and `ASA_Shell`)
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - `models/mevo_case/reports/mevo_start_dual_material_report.json`
- New default stack in this workflow:
  - TPU internal clearance: `0.15 mm`
  - TPU-to-ASA interface gap: `0.0 mm`
  - Internal vertical fillet target: `3.0 mm` (capsule profile exceeds this requirement)
- Body updated to front-closed bucket architecture with `2.0 mm` lens recess.
- Back cap is separate pure ASA part with built-in USB-C and power button cutouts.

### Vent + Tripod Cutout Corrections
- Mevo sleeve:
  - Replaced bottom tripod opening with a centered circular through-hole (`12.7 mm` default diameter).
  - Disabled Y-axis body filleting by default to keep Mevo vent opening edges sharp/right-angled.
- MAKI sleeve + TPU sleeve:
  - Updated vent subtraction strategy to start cuts outside the shell face and cut inward (eliminates non-through “placeholder” vents).
  - Kept vent rows aligned between ASA and TPU via shared 3-panel/8-row pattern derivation.
  - Added vent-entry reporting (`vents_applied_entries`) for direct coordinate auditing.
- Verified with geometric ray checks:
  - MAKI ASA vents: `24/24` through
  - MAKI TPU vents: `24/24` through
  - MAKI ASA/TPU vent alignment in device frame: max delta `0.0 mm` on x and z for corresponding rows.

### Assembly Fit Hardening + 3.0 mm ASA Wall Update
- Updated default ASA wall thickness to `3.0 mm` for both Mevo and MAKI sleeve/cap workflows.
- Mevo sleeve now defaults to open-through mode for front/rear cap assembly compatibility.
- Reduced ASA cap plug depth defaults to `1.8 mm` (Mevo + MAKI) to avoid TPU interference at end interfaces.
- Updated MAKI sleeve depth to symmetric end-clearance layout (`nominal length + 2*clearance`) for balanced two-cap fit.
- Added tripod-region reinforcement:
  - Mevo: enabled external bottom armor pad around tripod opening.
  - MAKI: added local side armor boss around tripod opening.
- Added/expanded fit reporting fields for cap insertion budget and axial/radial assembly margins.
- Regenerated active models and reports with the updated stack and validated cap/sleeve/TPU fit budgets.

### Report Folder Organization
- Added `reports/` subfolders to both case output roots:
  - `models/mevo_case/reports/`
  - `models/maki_case/reports/`
- Moved current JSON reports out of top-level case folders into those `reports/` subfolders.
- Updated all active generators so future report outputs are written to `reports/` by default.
- Added backward-compatible cleanup behavior so legacy top-level report files are auto-archived when present.

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
