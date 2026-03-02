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
- Device envelope: `87.0 x 75.5 x 34.0 mm` (L x H x W)
- ASA shell wall: `3.0 mm`
- ASA radial clearance to device: `2.3 mm`
- TPU sleeve wall: `2.0 mm`
- TPU radial device clearance: `0.2 mm`
- TPU-to-ASA radial air gap: `0.1 mm`
- ASA cap plug depth: `1.8 mm`

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
  - Vent behavior updated to enforced tripod-side 3-panel layout:
    - 8 vent slots on center panel
    - 8 on each adjacent panel
    - 24 total on that side
  - Through-cut depth increased so vents fully penetrate.
  - Tripod side includes local armor boss thickening around the mount opening.
  - Vent pass-through validated (`24/24` through by ray-check) and vent coordinates emitted in report under `step_side_features.vents_applied_entries`.
- ASA caps:
  - `models/maki_case/maki_live_front_cap.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - Generator: `scripts/generate_maki_live_caps.py --profile asa`
  - Front cap is intentionally inverted-bezel style:
    - raised outer rim
    - recessed center panel
    - plug on rear side
- TPU one-piece sleeve (preferred TPU output):
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - Generator: `scripts/generate_maki_live_tpu_liner.py`
  - Single connected TPU sleeve with thin front/rear edge wraps.
  - Does not use full TPU face caps.
  - Vent pass-through validated (`24/24` through by ray-check) and aligned to ASA vent rows in device frame.
  - Legacy separate TPU liner/caps and unibody files are archived automatically.

### Mevo Start
Current preferred workflow:
- Use open-through sleeve + front/rear cap path.
- Main outputs:
  - `models/mevo_case/mevo_start_front_cap.step`
  - `models/mevo_case/mevo_start_rear_cap.step`
  - `models/mevo_case/reports/mevo_start_caps_report.json`
- Mevo body:
  - `models/mevo_case/mevo_start_case_body.step`
  - Generator: `scripts/generate_mevo_case.py`
- Sleeve defaults to open-through mode (`open_through_sleeve=True`) for cap compatibility.
- Tripod zone includes external armor pad reinforcement on the sleeve body.
- Bottom tripod opening in sleeve body is a centered circular through-hole (`12.7 mm` default).
- Mevo vent openings default to sharp/right-angled edges (`preserve_sharp_vent_edges=True`).
- Back-plate duplication resolved:
  - `mevo_start_case_back_plate.step` is now opt-in only (`--include-back-plate`).
  - Default generation does not export back plate.

Optional/secondary Mevo TPU:
- `models/mevo_case/mevo_start_tpu_sleeve.step`
- Generator: `scripts/generate_mevo_start_tpu_liner.py`
- Auto-clamps thickness to fit existing ASA shell clearance.
- Includes thin front/rear edge wraps (perimeter-only hold on face edges).

## Terminology Mapping (Important for user shorthand)
User shorthand often means:
- “Maki sleeve” = ASA outer sleeve (`maki_live_case_sleeve.step`)
- “TPU sleeve for Maki” = `maki_live_tpu_sleeve.step` (single part)
- “Maki caps” = ASA front/rear caps
- “Mevo rear closure” = `mevo_start_rear_cap.step` (single rear piece)
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
