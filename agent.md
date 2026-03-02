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
    - plus side vents: 3 on left side + 3 on right side
    - total applied vents on sleeve: 30
  - The 24-vent bank is constrained to bottom-connected panels (not direct side walls).
  - Side 3+3 vent group is placed on the opposite end from the tripod-side vent bank.
  - Side 3+3 vent dimensions are matched to the main tripanel vent style size.
  - Main `24` vent bank and side `3+3` vents use rounded-slot cuts on both ASA and TPU (not box/square cuts).
  - Front camera aperture in integrated front wall is trimmed by `2.0 mm` vs prior extraction.
  - Through-cut depth increased so vents fully penetrate.
  - Vent row clustering now locks to the STEP-derived rear vent bank (8 rows) and ignores front outlier slots.
  - Tripod side includes local armor boss thickening around the mount opening.
  - Tripod hole cut orientation is corrected and validated as a true through-cut.
  - Dual-body tripod overlap is now numerically aligned (`dx=0.0`, `dz=0.0` in current reports).
  - Side `3 + 3` vents are rounded-slot cuts (not hard rectangular box cuts).
  - Integrated top-front lens hood/shade uses an extended curved duck-bill profile (`lens_hood_*` params in case script).
  - Vent pass-through validated (`30/30` total) and vent coordinates emitted in report under `step_side_features.vents_applied_entries`.
- ASA caps:
  - Active outputs:
    - `models/maki_case/maki_live_rear_cap_dual_material.step`
    - `models/maki_case/maki_live_rear_cap.step` (ASA-only compatibility export)
  - Generator: `scripts/generate_maki_live_rear_cap_dual_material.py`
  - Dual rear cap contains: `ASA_Back_Cap` + `TPU_Back_Gasket`.
  - TPU rear-cap body now includes:
    - full-face pad at plug-tip contact plane,
    - perimeter edge-wrap collar for back-edge/corner shock isolation,
    so the camera back contacts TPU instead of bare ASA.
  - Legacy rear-cap-only generator remains available (`scripts/generate_maki_live_caps.py --profile asa`).
  - Rear cutouts are extracted from all STEP solids with tiny-hole filtering to preserve port access cutouts over corner fastener holes.
  - Rear cap port cutouts include default oversize clearance (`cutout_extra_mm=1.5`) for cable boot/plastic strain-relief fit.
- TPU one-piece sleeve (preferred TPU output):
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - Generator: `scripts/generate_maki_live_tpu_liner.py`
  - Single connected TPU sleeve with front edge wrap enabled and rear edge wrap disabled by default.
  - Rear remains open for insertion; rear-side TPU contact is handled by rear cap TPU gasket.
  - Side `3 + 3` vents are rounded-slot cuts to match ASA style.
  - Does not use full TPU face caps.
  - Vent pass-through validated (`30/30` through by ray-check), with tripod through-cut also validated.
  - Vent rows are aligned to the ASA sleeve vent coordinates in device frame.
  - Legacy separate TPU liner/caps and unibody files are archived automatically.
- Dual-material body assembly (combined slicer object):
  - `models/maki_case/maki_live_body_dual_material.step`
  - Generator: `scripts/generate_maki_live_dual_material_body.py`
  - Contains two bodies: `TPU_Sleeve` + `ASA_Shell`.
  - TPU is auto-aligned into ASA cavity with report-verified fit stack (default ~0.1 mm radial gap each side, ~2.1 mm axial front/back gap).

### Mevo Start
Current preferred workflow:
- Dual-material bucket-body workflow (primary):
  - `models/mevo_case/mevo_start_body_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_dual_material.step`
  - `models/mevo_case/mevo_start_back_cap_asa.step`
  - Generator: `scripts/generate_mevo_dual_material_case.py`
  - Body STEP contains named solids: `TPU_Sleeve`, `ASA_Shell`.
  - Back-cap dual STEP contains named solids: `ASA_Back_Cap`, `TPU_Back_Gasket`.
- Geometry intent:
  - ovular/capsule-profile sleeve geometry (not rounded-rectangle profile),
  - front-integrated body by default (front wall fused to sleeve),
  - TPU body uses front edge wrap only (rear wrap disabled) to keep insertion path open,
  - front lens/LED cutouts enabled by default in closed-front mode,
  - lens opening defaults updated to `29.5 mm` at `Y=20.0` for improved framing,
  - integrated curved duck-bill front visor is enabled by default (`depth=16 mm`, `drop=9 mm`, `span_ratio=0.90`),
  - separate back-cap assembly with TPU gasket (ASA-only cap exported for compatibility),
  - manual two-cutout rear-cap layout is default-enabled (`include_manual_back_cutouts=true`):
    - lower slot from Mevo edge offsets (10 mm side margins, 7 mm bottom offset),
    - upper domed cutout using top offsets (3.0 mm and 28.0 mm from top, 3.0 mm side margins),
  - TPU-aware back-cap fit clearance default: `0.28 mm` total undersize,
  - two-stage tongue engagement with matching rear body groove seat,
  - bottom tripod hole cuts through ASA and TPU so mount contacts camera directly.
- Mevo back cap utility slot is default-disabled (`include_back_utility_slot=false`) until exact rear port map is confirmed.
- Mevo lens opening uses offset center (`lens_center_y_mm=20.0`) to avoid centered misalignment.
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
- “Maki caps” = dual-material rear cap by default (`maki_live_rear_cap_dual_material.step`)
- “Mevo rear closure” = `mevo_start_back_cap_dual_material.step` (ASA cap + TPU gasket)
- “Mevo ASA-only rear cap” = `mevo_start_back_cap_asa.step` (compatibility export)
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
python scripts/generate_maki_live_tpu_liner.py
python scripts/generate_maki_live_dual_material_body.py
python scripts/generate_maki_live_rear_cap_dual_material.py
```

Mevo:
```bash
python scripts/generate_mevo_dual_material_case.py
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
