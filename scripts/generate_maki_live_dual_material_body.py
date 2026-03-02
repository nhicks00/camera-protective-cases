#!/usr/bin/env python3
"""Generate MAKI Live dual-material body assembly (ASA shell + TPU sleeve).

Outputs:
- models/maki_case/maki_live_body_dual_material.step
- models/maki_case/reports/maki_live_dual_material_report.json
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import Compound, Location, export_step
from generate_maki_live_case import MakiCaseParams, build_case
from generate_maki_live_tpu_liner import MakiTpuLinerParams, build_liner


@dataclass
class MakiDualBodyParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # ASA shell defaults
    asa_clearance_mm: float = 2.3
    asa_wall_mm: float = 3.0

    # TPU sleeve defaults
    tpu_device_clearance_mm: float = 0.2
    tpu_wall_mm: float = 2.0
    tpu_end_clearance_mm: float = 0.2


def _archive_existing(paths: list[Path], out_dir: Path) -> list[tuple[str, str]]:
    archive_dir = out_dir / "archive"
    archive_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    moved = []
    for path in paths:
        if not path.exists():
            continue
        target = archive_dir / f"{path.stem}_{stamp}{path.suffix}"
        i = 1
        while target.exists():
            target = archive_dir / f"{path.stem}_{stamp}_{i}{path.suffix}"
            i += 1
        shutil.move(str(path), str(target))
        moved.append((str(path), str(target)))
    return moved


def _largest_solid(shape):
    solids = shape.solids() if hasattr(shape, "solids") else []
    if len(solids) <= 1:
        return shape
    return max(solids, key=lambda s: s.volume)


def build_dual_body(p: MakiDualBodyParams):
    asa_p = MakiCaseParams(step_path=p.step_path)
    asa_p.clearance_mm = p.asa_clearance_mm
    asa_p.wall_mm = p.asa_wall_mm
    asa_shell, asa_report = build_case(asa_p)

    tpu_p = MakiTpuLinerParams(step_path=p.step_path)
    tpu_p.device_clearance_mm = p.tpu_device_clearance_mm
    tpu_p.shell_thickness_mm = p.tpu_wall_mm
    tpu_p.end_clearance_mm = p.tpu_end_clearance_mm
    tpu_sleeve, tpu_report = build_liner(tpu_p)

    asa_derived = asa_report["derived"]
    tpu_derived = tpu_report["derived"]
    asa_cavity_front_z = float(asa_derived["cavity_front_z_mm"])
    asa_inner_depth = float(asa_derived["inner_depth_mm"])
    tpu_depth = float(tpu_derived["shell_depth_mm"])

    # Center TPU sleeve axially inside ASA cavity.
    z_offset = asa_cavity_front_z + 0.5 * (asa_inner_depth - tpu_depth)
    tpu_aligned = tpu_sleeve.move(Location((0.0, 0.0, z_offset)))

    asa_shell = _largest_solid(asa_shell)
    tpu_aligned = _largest_solid(tpu_aligned)
    asa_shell.label = "ASA_Shell"
    tpu_aligned.label = "TPU_Sleeve"

    assembly = Compound(children=[tpu_aligned, asa_shell], label="Maki_Body_Assembly")

    asa_inner_w = float(asa_derived["inner_w_mm"])
    asa_inner_h = float(asa_derived["inner_h_mm"])
    tpu_outer_w = float(tpu_derived["dimensions_mm"]["outer_w"])
    tpu_outer_h = float(tpu_derived["dimensions_mm"]["outer_h"])

    radial_gap_w_each = 0.5 * (asa_inner_w - tpu_outer_w)
    radial_gap_h_each = 0.5 * (asa_inner_h - tpu_outer_h)
    front_gap = z_offset - asa_cavity_front_z
    back_gap = (asa_cavity_front_z + asa_inner_depth) - (z_offset + tpu_depth)

    report = {
        "named_bodies": ["TPU_Sleeve", "ASA_Shell"],
        "alignment_mm": {
            "tpu_z_offset_into_asa": float(z_offset),
            "asa_cavity_front_z": float(asa_cavity_front_z),
            "asa_inner_depth": float(asa_inner_depth),
            "tpu_depth": float(tpu_depth),
            "front_gap": float(front_gap),
            "back_gap": float(back_gap),
        },
        "fit_stack_mm": {
            "radial_gap_each_width": float(radial_gap_w_each),
            "radial_gap_each_height": float(radial_gap_h_each),
        },
        "sources": {
            "asa_case_report": asa_report,
            "tpu_sleeve_report": tpu_report,
        },
        "warnings": [],
    }
    if radial_gap_w_each < 0.0 or radial_gap_h_each < 0.0 or front_gap < 0.0 or back_gap < 0.0:
        report["warnings"].append("Detected interference/negative gap in dual-body fit stack.")

    return assembly, report


def main():
    parser = argparse.ArgumentParser(description="Generate MAKI dual-material body assembly")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--asa-clearance", type=float, default=None, help="ASA internal clearance (mm)")
    parser.add_argument("--asa-wall", type=float, default=None, help="ASA wall thickness (mm)")
    parser.add_argument("--tpu-clearance", type=float, default=None, help="TPU internal clearance (mm)")
    parser.add_argument("--tpu-wall", type=float, default=None, help="TPU wall thickness (mm)")
    parser.add_argument("--tpu-end-clearance", type=float, default=None, help="TPU end clearance (mm)")
    args = parser.parse_args()

    params = MakiDualBodyParams(step_path=args.step)
    if args.asa_clearance is not None:
        params.asa_clearance_mm = args.asa_clearance
    if args.asa_wall is not None:
        params.asa_wall_mm = args.asa_wall
    if args.tpu_clearance is not None:
        params.tpu_device_clearance_mm = args.tpu_clearance
    if args.tpu_wall is not None:
        params.tpu_wall_mm = args.tpu_wall
    if args.tpu_end_clearance is not None:
        params.tpu_end_clearance_mm = args.tpu_end_clearance

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    out_step = args.out / "maki_live_body_dual_material.step"
    out_json = reports_dir / "maki_live_dual_material_report.json"
    archived = _archive_existing([out_step, out_json, args.out / "maki_live_dual_material_report.json"], args.out)

    assembly, report = build_dual_body(params)
    export_step(assembly, str(out_step))

    payload = {"params": asdict(params), "report": report}
    payload["params"]["step_path"] = str(params.step_path)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()

