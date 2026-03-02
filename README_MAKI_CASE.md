# BirdDog MAKI Live Protective Case (Parametric Sleeve)

This project now includes a printable protective sleeve generated from the MAKI Live STEP model and validated against the provided drawing dimensions.

## Inputs Used
- `refs/BirdDog_MAKI-Live_3D-file.step`
- Drawing: `MAKI-Live_drawing.pdf` (nominal dimensions used)

## Nominal Constraints Applied
- Width: `56.99 mm`
- Height: `56.99 mm`
- Length: `120.32 mm`

## Generated Outputs
- `models/maki_case/maki_live_case_sleeve.step`
- `models/maki_case/reports/maki_live_case_report.json`
- `models/maki_case/maki_live_tpu_sleeve.step`
- `models/maki_case/reports/maki_live_tpu_sleeve_report.json`

Archive policy:
- Previous versions are auto-moved to `models/maki_case/archive/` before new files are written.

## Design Characteristics
- Tight, one-piece sleeve
- Front-integrated shell by default (front wall fused to sleeve, rear remains open)
- Default ASA internal clearance: `2.3 mm`
- Default ASA wall thickness: `3.0 mm`
- TPU sleeve defaults:
  - internal camera clearance: `0.2 mm`
  - TPU wall thickness: `2.0 mm`
  - TPU-to-ASA radial gap: `0.1 mm`
  - edge-wrap depth: `2.5 mm`
  - edge-wrap radial hold: `2.0 mm`
- Tripod-side access opening around detected `1/4"-20` region
- Tripod region reinforcement via local external armor boss
- Side vent slots extracted from STEP across flat + corner side panels
- Tripod location extraction is B-Rep based (cylindrical-face-first, circular-edge fallback)
- Enforced vent layout on tripod side: `3` adjacent panels x `8` slots each (`24` total), with through-cuts
- Vent coordinates are reported (`step_side_features.vents_applied_entries`) for ASA/TPU alignment audits.
- Built with `build123d` (OpenCascade B-Rep), exported as STEP only

## Generator Script
- `scripts/generate_maki_live_case.py`

## TPU Sleeve (Single Print)
- `scripts/generate_maki_live_tpu_liner.py`
- Output: `models/maki_case/maki_live_tpu_sleeve.step`
- Output report: `models/maki_case/reports/maki_live_tpu_sleeve_report.json`
- Purpose: one connected TPU sleeve with thin front/rear edge wraps (not full face caps).

Generate one-piece TPU sleeve:
```bash
source .venv311/bin/activate
python scripts/generate_maki_live_tpu_liner.py
```

Useful TPU sleeve tuning:
```bash
python scripts/generate_maki_live_tpu_liner.py \
  --clearance 0.2 \
  --thickness 2.0 \
  --edge-wrap-depth 2.5 \
  --edge-wrap-radial 2.0 \
  --end-clearance 0.2
```

Legacy note:
- `scripts/generate_maki_live_tpu_unibody.py` still exists for older full fused-cap style, but current preferred TPU workflow is `maki_live_tpu_sleeve.step`.

## Front/Rear Caps
- `scripts/generate_maki_live_caps.py`
- Default outputs:
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/reports/maki_live_caps_report.json`
- Optional legacy output:
  - `models/maki_case/maki_live_front_cap.step` (only when `--include-front-cap` is used)
- Cap cutouts are extracted from front/rear end-face loop geometry in the original STEP model.
- End-plane anchoring is reported in `report.cutouts.end_planes` to verify extraction reference planes.
- Front cap geometry uses an inverted bezel profile:
  - raised outer rim,
  - recessed center panel,
  - rear-side plug (instead of front-side center protrusion).
- ASA cap plug depth default is `1.8 mm` (tuned to clear the TPU sleeve end-stack).

Generate ASA caps:
```bash
python scripts/generate_maki_live_caps.py --profile asa
```

Generate ASA caps including legacy front cap:
```bash
python scripts/generate_maki_live_caps.py --profile asa --include-front-cap
```

Generate TPU caps (legacy separate parts):
```bash
python scripts/generate_maki_live_caps.py --profile tpu
```

TPU cap outputs:
- `models/maki_case/maki_live_tpu_front_cap.step`
- `models/maki_case/maki_live_tpu_rear_cap.step`
- `models/maki_case/reports/maki_live_tpu_caps_report.json`

Regenerate:
```bash
source .venv311/bin/activate
python scripts/generate_maki_live_case.py
```

Useful tuning flags:
```bash
python scripts/generate_maki_live_case.py \
  --clearance 2.3 \
  --wall 3.0 \
  --lens-d 30 \
  --tripod-z 48
```

## Print Notes (ASA/PETG)
- 0.2 mm layers
- 4 perimeters (wall loops) minimum
- 25-40% gyroid infill
- Print with rear opening up to keep supports low
- If fit is still too tight, increase `--clearance` to `2.9` or `3.1`
