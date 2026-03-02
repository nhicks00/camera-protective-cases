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
- `models/maki_case/maki_live_case_report.json`

Archive policy:
- Previous versions are auto-moved to `models/maki_case/archive/` before new files are written.

## Design Characteristics
- Tight, one-piece sleeve
- Fully open-through shell (no front plate and no rear plate)
- Default internal clearance: `2.65 mm`
- Default wall thickness: `3.2 mm`
- Tripod-side access opening around detected `1/4"-20` region
- Side vent slots extracted from STEP across flat + corner side panels
- Enforced vent layout on tripod side: `3` adjacent panels x `8` slots each (`24` total), with through-cuts
- Built with `build123d` (OpenCascade B-Rep), exported as STEP only

## Generator Script
- `scripts/generate_maki_live_case.py`

## TPU Liner (Second Print)
- `scripts/generate_maki_live_tpu_liner.py`
- Output: `models/maki_case/maki_live_tpu_liner.step`
- Output report: `models/maki_case/maki_live_tpu_liner_report.json`
- Purpose: snug TPU sleeve around the camera body with extra end bumper bands that fill space inside the ASA sleeve.

Generate TPU liner:
```bash
source .venv311/bin/activate
python scripts/generate_maki_live_tpu_liner.py
```

Useful TPU tuning:
```bash
python scripts/generate_maki_live_tpu_liner.py \
  --clearance 0.2 \
  --thickness 2.1 \
  --bumper-extra 0.55 \
  --band-depth 12.0 \
  --end-clearance 0.2
```

## Front/Rear Caps
- `scripts/generate_maki_live_caps.py`
- Outputs:
  - `models/maki_case/maki_live_front_cap.step`
  - `models/maki_case/maki_live_rear_cap.step`
  - `models/maki_case/maki_live_caps_report.json`
- Cap cutouts are extracted from front/rear end-face loop geometry in the original STEP model.
- Front cap geometry uses an inverted bezel profile:
  - raised outer rim,
  - recessed center panel,
  - rear-side plug (instead of front-side center protrusion).

Generate ASA caps:
```bash
python scripts/generate_maki_live_caps.py --profile asa
```

Generate TPU caps:
```bash
python scripts/generate_maki_live_caps.py --profile tpu
```

TPU cap outputs:
- `models/maki_case/maki_live_tpu_front_cap.step`
- `models/maki_case/maki_live_tpu_rear_cap.step`
- `models/maki_case/maki_live_tpu_caps_report.json`

Regenerate:
```bash
source .venv311/bin/activate
python scripts/generate_maki_live_case.py
```

Useful tuning flags:
```bash
python scripts/generate_maki_live_case.py \
  --clearance 2.65 \
  --wall 3.2 \
  --lens-d 30 \
  --tripod-z 48
```

## Print Notes (ASA/PETG)
- 0.2 mm layers
- 4-5 perimeters
- 25-40% gyroid infill
- Print with rear opening up to keep supports low
- If fit is still too tight, increase `--clearance` to `2.9` or `3.1`
