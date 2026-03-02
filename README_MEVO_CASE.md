# Mevo Start Protective Case (Parametric)

This workspace now includes a parametric two-piece, print-ready enclosure derived from your reference front-cap STL.

## Generated Outputs
- `models/mevo_case/mevo_start_case_body.step`
- `models/mevo_case/mevo_start_case_back_plate.step`
- `models/mevo_case/mevo_start_case_report.json`
- `models/mevo_case/mevo_start_front_cap.step`
- `models/mevo_case/mevo_start_rear_cap.step`
- `models/mevo_case/mevo_start_caps_report.json`
- `models/mevo_case/mevo_start_tpu_front_cap.step`
- `models/mevo_case/mevo_start_tpu_rear_cap.step`
- `models/mevo_case/mevo_start_tpu_caps_report.json`
- `models/mevo_case/mevo_start_tpu_liner.step`
- `models/mevo_case/mevo_start_tpu_liner_report.json`

Archive policy:
- Previous versions are auto-moved to `models/mevo_case/archive/` before new files are written.

## Design Summary
- Tight-fitting **two-piece** architecture (impact sleeve + rear closure plate)
- Front profile extracted from `refs/Mevo_Start_lens_cover_corrected.stl` (default)
- Dimension-locked to Mevo Start nominal envelope by default:
  - `87.0 mm` length (front-to-back)
  - `75.5 mm` front-profile major axis
  - `34.0 mm` front-profile minor axis
- Internal clearance default: `0.65 mm`
- Main wall default: `3.4 mm`
- Front wall default: `4.0 mm`
- Required access implemented:
  - Front lens opening (manual default or reference-derived)
  - Rear power button slot
  - Rear audio jack hole
  - Rear USB-C slot
  - Bottom tripod access opening
- Optional side vent slots included

For alternate cap STLs where the inner circular opening is not reliable, the generator still uses the cap for outer curvature and defaults to a manual lens cutout target.
The current default profile logic uses the cap's **inner pill loop** and enforces a symmetric capsule profile before extending to full length.
The solid modeling is done with `build123d` (OpenCascade B-Rep) and exported as STEP only.

## Regenerate
```bash
source .venv311/bin/activate
python scripts/generate_mevo_case.py
```

Generate Mevo caps (ASA profile):
```bash
python scripts/generate_mevo_start_caps.py --profile asa
```

Generate Mevo caps (TPU profile):
```bash
python scripts/generate_mevo_start_caps.py --profile tpu
```

Generate Mevo TPU inner liner matched to the current ASA shell:
```bash
python scripts/generate_mevo_start_tpu_liner.py
```

Tune TPU liner fit stack (example):
```bash
python scripts/generate_mevo_start_tpu_liner.py \
  --device-clearance 0.15 \
  --shell-gap 0.10 \
  --thickness 0.90
```

Enable rear Mevo I/O cutouts on caps (optional):
```bash
python scripts/generate_mevo_start_caps.py --profile asa --rear-io-cutouts
```

Example with explicit dimensions:
```bash
python scripts/generate_mevo_case.py --length-mm 87 --height-mm 75.5 --width-mm 34
```

Use lens opening extracted directly from cap (optional):
```bash
python scripts/generate_mevo_case.py --use-reference-lens-hole
```

The cap generator defaults to:
- Front cap lens opening enabled
- Rear Mevo I/O cutouts disabled (until exact Mevo rear port mapping is confirmed)

The TPU liner generator auto-clamps shell thickness so the liner fits inside the ASA shell.
Actual applied thickness and remaining fit margins are written to `models/mevo_case/mevo_start_tpu_liner_report.json`.

## Important Fit Note
Port and button positions are parameterized defaults in `scripts/generate_mevo_case.py` and may need small tuning to your exact camera revision and print shrink behavior (ASA + printer profile).

The key tunables are in `CaseParams`:
- `clearance_mm`
- `wall_mm`
- `device_length_mm`, `device_height_mm`, `device_width_mm`
- `power_y_mm`, `audio_y_mm`, `usb_y_mm`
- `tripod_*`

## Suggested Print Setup (ASA)
- 0.4 mm nozzle
- 0.2 mm layer height
- 5+ perimeters in high-stress zones
- 35–45% gyroid infill (body)
- 100% infill optional around rear plug region
- Anneal/cool slowly to reduce warp
