# Camera Protective Cases (Mevo Start + BirdDog MAKI Live)

Parametric, 3D-printable protective case design workspace for two cameras:
- Mevo Start
- BirdDog MAKI Live

All geometry generation is built with `build123d` (OpenCascade B-REP) and exported as STEP for clean, smooth CAD surfaces.

## Goals
- Tight-fitting protective sleeves for outdoor sports production
- Impact resistance (volleyball strike scenarios)
- Heat/sun durable outer shell (ASA-focused)
- Optional TPU inner shock/fit components
- Precise cutout alignment with device features and ports

## Repository Structure
- `scripts/`: parametric generators for sleeves, liners, and caps
- `models/mevo_case/`: generated Mevo STEP outputs and JSON reports
- `models/maki_case/`: generated MAKI STEP outputs and JSON reports
- `refs/`: source reference CAD/STL/PDF assets used for extraction and alignment
- `README_MEVO_CASE.md`: Mevo-specific generator details
- `README_MAKI_CASE.md`: MAKI-specific generator details
- `CHANGELOG.md`: project changelog and current status

## Core Scripts
- `scripts/generate_mevo_case.py`
- `scripts/generate_mevo_start_caps.py`
- `scripts/generate_mevo_start_tpu_liner.py`
- `scripts/generate_maki_live_case.py`
- `scripts/generate_maki_live_tpu_liner.py`
- `scripts/generate_maki_live_caps.py`

## Current Outputs
Key generated outputs are in `models/`, including:
- Mevo ASA body + front/rear caps + TPU sleeve
- MAKI ASA sleeve + front/rear caps + TPU sleeve
- JSON reports for extracted/applied features and dimensions

## Regeneration
```bash
source .venv311/bin/activate

python scripts/generate_mevo_case.py
python scripts/generate_mevo_start_caps.py --profile asa
python scripts/generate_mevo_start_caps.py --profile tpu
python scripts/generate_mevo_start_tpu_liner.py

python scripts/generate_maki_live_case.py
python scripts/generate_maki_live_tpu_liner.py
python scripts/generate_maki_live_caps.py --profile asa
python scripts/generate_maki_live_caps.py --profile tpu
```

## Notes
- STEP-only workflow is used for production models.
- Temporary meshes/renders are excluded from version control.
- Output rollover policy:
  - each generator archives existing outputs into `models/mevo_case/archive/` or `models/maki_case/archive/` before writing new files,
  - top-level case folders always keep only the latest generated files.
- See `CHANGELOG.md` for latest engineering status and open alignment tasks.
