# Mevo Start Protective Case (Parametric)

This workspace now includes a parametric two-piece, print-ready enclosure derived from your reference front-cap STL.

Current focus option:
- front and rear cap workflow (`mevo_start_front_cap.step` + `mevo_start_rear_cap.step`)
- single rear closure approach: `mevo_start_rear_cap.step` (legacy `case_back_plate` export is opt-in only)

## Generated Outputs
- `models/mevo_case/mevo_start_case_body.step`
- `models/mevo_case/reports/mevo_start_case_report.json`
- `models/mevo_case/mevo_start_front_cap.step`
- `models/mevo_case/mevo_start_rear_cap.step`
- `models/mevo_case/reports/mevo_start_caps_report.json`
- `models/mevo_case/mevo_start_tpu_sleeve.step`
- `models/mevo_case/reports/mevo_start_tpu_sleeve_report.json`

Archive policy:
- Previous versions are auto-moved to `models/mevo_case/archive/` before new files are written.

## Design Summary
- Tight-fitting architecture with body + cap options
- Front profile extracted from `refs/Mevo_Start_lens_cover_corrected.stl` (default)
- Dimension-locked to Mevo Start nominal envelope by default:
  - `87.0 mm` length (front-to-back)
  - `75.5 mm` front-profile major axis
  - `34.0 mm` front-profile minor axis
- Internal ASA clearance default: `2.3 mm`
- Main ASA wall default: `3.0 mm`
- Open-through sleeve default (no fixed front wall)
- TPU sleeve defaults:
  - internal camera clearance: `0.2 mm`
  - TPU wall thickness: `2.0 mm`
  - TPU-to-ASA radial gap: `0.1 mm`
- Required access implemented:
  - Open front/rear sleeve for cap system
  - Rear power button slot
  - Rear audio jack hole
  - Rear USB-C slot
  - Bottom tripod access opening
- Optional side vent slots included
- Tripod impact zone includes an external armor pad on the sleeve body.
- Bottom tripod access uses a centered circular through-hole (`12.7 mm` default).
- Vent openings are kept sharp/right-angled by default (no Y-axis fillet pass on body).

For alternate cap STLs where the inner circular opening is not reliable, the generator still uses the cap for outer curvature and defaults to a manual lens cutout target.
The current default profile logic uses the cap's **inner pill loop** and enforces a symmetric capsule profile before extending to full length.
The solid modeling is done with `build123d` (OpenCascade B-Rep) and exported as STEP only.

## Regenerate
```bash
source .venv311/bin/activate
python scripts/generate_mevo_case.py
```

Export legacy case back plate only when explicitly needed:
```bash
python scripts/generate_mevo_case.py --include-back-plate
```

Generate legacy closed-front sleeve mode (not default):
```bash
python scripts/generate_mevo_case.py --closed-front
```

Generate Mevo caps (ASA profile):
```bash
python scripts/generate_mevo_start_caps.py --profile asa
```

Generate Mevo TPU inner sleeve matched to the current ASA shell:
```bash
python scripts/generate_mevo_start_tpu_liner.py
```

Generate legacy Mevo TPU caps (optional, archived workflow):
```bash
python scripts/generate_mevo_start_caps.py --profile tpu
```

Tune TPU sleeve fit stack (example):
```bash
python scripts/generate_mevo_start_tpu_liner.py \
  --device-clearance 0.2 \
  --shell-gap 0.10 \
  --thickness 2.0 \
  --edge-wrap-depth 2.5 \
  --edge-wrap-radial 2.0
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
- ASA cap plug depth tuned to `1.8 mm` for TPU-compatible assembly stack.

The TPU sleeve generator auto-clamps shell thickness so the sleeve fits inside the ASA shell.
Actual applied thickness and remaining fit margins are written to `models/mevo_case/reports/mevo_start_tpu_sleeve_report.json`.
Current default stack yields:
- radial TPU-to-ASA gap: `0.1 mm` per side
- cap insertion budget with TPU installed: `0.3 mm` remaining per end (with `1.8 mm` ASA cap plug depth)

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
- 4 perimeters (wall loops) minimum
- 35–45% gyroid infill (body)
- 100% infill optional around rear plug region
- Anneal/cool slowly to reduce warp
