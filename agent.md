# Agent Context: Camera Protective Cases (Mevo Start + BirdDog MAKI Live)

This file is the persistent handoff context for future sessions.

## Scope
This repo builds parametric, 3D-printable protective case components for:
- Mevo Start
- BirdDog MAKI Live

Primary goals:
- Tight fit
- Outdoor durability
- Impact resistance
- Clean, smooth CAD geometry (not faceted mesh-style approximations)

## Core Modeling Rules
- Use `build123d` (OpenCascade B-REP).
- Export STEP files for deliverables.
- Keep curves smooth; avoid tiny rectangular facet chains for curved surfaces.
- Keep generated outputs organized by camera and archive prior versions.

## Confirmed Dimension Baseline
Use these as the current default stack unless user explicitly overrides.

Mevo Start:
- Device envelope reference: `87.0 x 75.5 x 34.0 mm` (L x H x W)
- TPU inner cavity (active fit): `34.3 x 75.8 x 87.3 mm` (W x H x depth)
- TPU wall: `1.8 mm`
- TPU inner vertical corner fillet: `4.0 mm`
- TPU-to-ASA interface gap: `0.0 mm` (coincident interface)
- ASA shell wall: `2.2 mm`
- ASA outer vertical corner fillet: `6.0 mm`
- Front bucket/sun hood depth: `3.0 mm`
- Front lens cutout: `32.0 mm`
- Front tally LED hole: `3.0 mm`, centered `12.0 mm` above lens center
- Bottom tripod hole: `20.5 mm` diameter, center `43.2 mm` from front face
- ASA back-cap plug lip: `5.0 mm` depth, `0.1 mm` total undersize
- Back utility slot: `15.0 mm` wide, top margin `15.0 mm`, bottom margin `10.0 mm`

BirdDog MAKI Live:
- Device envelope: `120.32 x 56.99 x 56.99 mm` (L x W x H)
- ASA shell wall: `3.0 mm`
- ASA radial clearance to device: `2.3 mm`
- TPU sleeve wall: `2.0 mm`
- TPU radial device clearance: `0.2 mm`
- TPU-to-ASA radial air gap: `0.1 mm`
- ASA cap plug depth: `1.8 mm`

Slicer baseline (ASA):
- 4 wall loops/perimeters minimum.

## Output Organization Policy
- Current outputs:
  - `models/mevo_case/`
  - `models/mevo_case/reports/`
  - `models/maki_case/`
  - `models/maki_case/reports/`
- Each generator archives old versions into:
  - `models/mevo_case/archive/`
  - `models/maki_case/archive/`
- Top-level in each case folder should show only latest STEP outputs for that case.
- JSON reports should live in each case's `reports/` subfolder.

## Current Project State

### MAKI Live (BirdDog)
Active hard-shell and TPU workflows:
- ASA sleeve:
  - `models/maki_case/maki_live_case_sleeve.step`
  - Generator: `scripts/generate_maki_live_case.py`
  - Front is integrated into sleeve by default (`front_integrated=True`).
  - Front cutouts are extracted from STEP and cut through the integrated front wall.
  - Back remains open for separate rear cap installation.
  - Tripod detection is STEP-interrogation-first:
    - cylindrical face detection (thread-radius target near 1/4"-20 geometry),
    - downward axis normal filter,
    - centerline/depth region filters,
    - circular-edge fallback only if cylindrical detection fails.
  - Vent behavior updated to enforced tripod-side 3-panel layout:
    - 8 vent slots on center panel
    - 8 on each adjacent panel
    - 24 total on that side
  - Through-cut depth increased so vents fully penetrate.
  - Vent row clustering now locks to the STEP-derived rear vent bank (8 rows) and ignores front outlier slots.
  - Tripod side includes local armor boss thickening around the mount opening.
  - Vent pass-through validated (`24/24` through by ray-check) and vent coordinates emitted in report under `step_side_features.vents_applied_entries`.
- ASA caps:
  - Active output: `models/maki_case/maki_live_rear_cap.step`
  - Generator: `scripts/generate_maki_live_caps.py --profile asa`
  - Default is rear-cap-only export (front cap is legacy optional via `--include-front-cap`).
  - Rear cutouts are extracted from all STEP solids with tiny-hole filtering to preserve port access cutouts over corner fastener holes.
  - Rear cap port cutouts include default oversize clearance (`cutout_extra_mm=1.5`) for cable boot/plastic strain-relief fit.
- TPU one-piece sleeve (preferred TPU output):
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - Generator: `scripts/generate_maki_live_tpu_liner.py`
  - Single connected TPU sleeve with thin front/rear edge wraps.
  - Does not use full TPU face caps.
  - Vent pass-through validated (`24/24` through by ray-check) and aligned to ASA vent rows in device frame.
  - Legacy separate TPU liner/caps and unibody files are archived automatically.

### Mevo Start
Current preferred workflow:
- Dual-material bucket-body workflow (primary):
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - Generator: `scripts/generate_mevo_dual_material_case.py`
  - Body STEP contains named solids: `TPU_Sleeve`, `ASA_Shell`.
- Geometry intent:
  - ovular/capsule-profile sleeve geometry (not rounded-rectangle profile),
  - front-integrated body by default (front wall fused to sleeve),
  - front lens/LED cutouts enabled by default in closed-front mode,
  - separate pure-ASA back cap,
  - bottom tripod hole cuts through ASA and TPU so mount contacts camera directly.
- Mevo back cap utility slot is default-disabled (`include_back_utility_slot=false`) until exact rear port map is confirmed.
- Mevo lens opening uses offset center (`lens_center_y_mm=16.5`) to avoid centered misalignment.
- Optional flags:
  - `--open-front-ovular` for legacy open-front mode,
  - `--disable-front-lens-led-cutouts` to keep front wall solid.
- Active review-spec values are recorded in `models/mevo_case/reports/mevo_start_dual_material_report.json`.
- Legacy open-through/cap workflows remain in repo for fallback only and are not the active default path.
- Note: manufacturer STEP-based auto-extraction is currently strongest for MAKI.
  Mevo workflow is dimension/spec-driven unless a full Mevo STEP is provided.

## Terminology Mapping (Important for user shorthand)
User shorthand often means:
- “Maki sleeve” = ASA outer sleeve (`maki_live_case_sleeve.step`)
- “TPU sleeve for Maki” = `maki_live_tpu_sleeve.step` (single part)
- “Maki caps” = rear cap by default (`maki_live_rear_cap.step`)
- “Mevo rear closure” = `mevo_start_back_cap_asa.step` (single rear piece)
- “Mevo case back plate” = legacy/optional only

## Key References
- MAKI references:
  - `refs/BirdDog_MAKI-Live_3D-file.step`
  - `refs/MAKI-Live_drawing.pdf`
- Mevo references:
  - `refs/Mevo_Start_lens_cover_corrected.stl`
  - Additional Mevo legacy refs are present in `refs/`.

## Common Commands
Activate environment:
```bash
source .venv311/bin/activate
```

MAKI:
```bash
python scripts/generate_maki_live_case.py
python scripts/generate_maki_live_caps.py --profile asa
python scripts/generate_maki_live_tpu_liner.py
```

Mevo:
```bash
python scripts/generate_mevo_case.py
python scripts/generate_mevo_start_caps.py --profile asa
python scripts/generate_mevo_start_caps.py --profile tpu
python scripts/generate_mevo_start_tpu_liner.py
```

Legacy Mevo back plate (only when explicitly requested):
```bash
python scripts/generate_mevo_case.py --include-back-plate
```

## Session Continuity Guidance
When the user gives brief or “random” update requests, assume they refer to this context and default behaviors unless they explicitly override:
- Keep MAKI TPU as one connected sleeve (`maki_live_tpu_sleeve.step`).
- Keep Mevo with single rear closure (rear cap path).
- Maintain archive hygiene policy.
- Preserve smooth B-REP output quality.

If the user asks for changes that affect fit/alignment:
- Regenerate relevant model(s).
- Check the JSON report in the case `reports/` subfolder.
- Verify outputs are single-solid STEP where applicable.
- Keep latest STEP outputs at top level and auto-archive previous versions.
