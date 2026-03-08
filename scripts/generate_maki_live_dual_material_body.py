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

from build123d import Location, export_step
from generate_maki_live_case import MakiCaseParams, build_case
from generate_maki_live_tpu_liner import MakiTpuLinerParams, build_liner


@dataclass
class MakiDualBodyParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # ASA shell defaults
    asa_clearance_mm: float = 2.3
    asa_wall_mm: float = 3.0

    # TPU sleeve defaults
    # Camera-to-TPU sidewall clearance (snug fit for stiffer TPU).
    # Set to None to auto-solve from ASA interface stack.
    tpu_device_clearance_mm: float | None = 0.15
    tpu_wall_mm: float = 2.0
    tpu_end_clearance_mm: float = 0.2
    target_interface_gap_each_mm: float = 0.0
    interface_gap_tolerance_mm: float = 0.02
    tpu_front_edge_wrap_mm: float = 2.5
    tpu_edge_wrap_radial_mm: float = 2.0
    include_tpu_front_edge_wrap: bool = False
    include_tpu_rear_edge_wrap: bool = False

    # TPU front face pad
    include_tpu_front_face_pad: bool = True
    tpu_front_face_pad_thickness_mm: float = 1.5

    # Rectangular tripod mount cutout (passed through to ASA + TPU builds)
    tripod_use_rect_cutout: bool = True
    tripod_rect_long_mm: float = 50.80
    tripod_rect_short_mm: float = 40.64
    tripod_rect_long_along_z: bool = True
    tripod_rect_z_shift_mm: float = -6.35


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
    asa_p.tripod_use_rect_cutout = p.tripod_use_rect_cutout
    asa_p.tripod_rect_long_mm = p.tripod_rect_long_mm
    asa_p.tripod_rect_short_mm = p.tripod_rect_short_mm
    asa_p.tripod_rect_long_along_z = p.tripod_rect_long_along_z
    asa_p.tripod_rect_z_shift_mm = p.tripod_rect_z_shift_mm
    asa_shell, asa_report = build_case(asa_p)

    tpu_p = MakiTpuLinerParams(step_path=p.step_path)
    # Lock TPU outer profile to the ASA inner cavity for high-confidence fusion.
    if p.tpu_device_clearance_mm is None:
        effective_tpu_clearance = max(
            p.asa_clearance_mm - p.tpu_wall_mm - p.target_interface_gap_each_mm,
            0.0,
        )
    else:
        effective_tpu_clearance = p.tpu_device_clearance_mm
    tpu_p.device_clearance_mm = effective_tpu_clearance
    tpu_p.shell_thickness_mm = p.tpu_wall_mm
    tpu_p.end_clearance_mm = p.tpu_end_clearance_mm
    tpu_p.asa_shell_clearance_mm = p.asa_clearance_mm
    tpu_p.edge_wrap_depth_mm = p.tpu_front_edge_wrap_mm
    tpu_p.edge_wrap_radial_mm = p.tpu_edge_wrap_radial_mm
    tpu_p.include_front_edge_wrap = p.include_tpu_front_edge_wrap
    tpu_p.include_rear_edge_wrap = p.include_tpu_rear_edge_wrap
    tpu_p.include_front_face_pad = p.include_tpu_front_face_pad
    tpu_p.front_face_pad_thickness_mm = p.tpu_front_face_pad_thickness_mm
    tpu_p.tripod_use_rect_cutout = p.tripod_use_rect_cutout
    tpu_p.tripod_rect_long_mm = p.tripod_rect_long_mm
    tpu_p.tripod_rect_short_mm = p.tripod_rect_short_mm
    tpu_p.tripod_rect_long_along_z = p.tripod_rect_long_along_z
    tpu_p.tripod_rect_z_shift_mm = p.tripod_rect_z_shift_mm

    # When front-flush placement is used, the TPU shifts forward by
    # (asa_clearance - tpu_end_clearance) vs centered.  Compensate all
    # Z-positioned features (vents, tripod) so they still align with ASA.
    if p.include_tpu_front_face_pad:
        front_flush_shift = p.asa_clearance_mm - p.tpu_end_clearance_mm
        tpu_p.tripanel_vent_z_shift_mm += front_flush_shift
        tpu_p.side_trio_vent_z_shift_mm += front_flush_shift
        tpu_p.tripod_rect_z_shift_mm += front_flush_shift

    tpu_sleeve, tpu_report = build_liner(tpu_p)

    asa_derived = asa_report["derived"]
    tpu_derived = tpu_report["derived"]
    asa_cavity_front_z = float(asa_derived["cavity_front_z_mm"])
    asa_inner_depth = float(asa_derived["inner_depth_mm"])
    tpu_depth = float(tpu_derived["shell_depth_mm"])

    # When front face pad is enabled, place TPU front-flush against ASA
    # inner wall so the pad sits between the ASA front wall and the camera.
    # Otherwise centre the TPU axially inside the ASA cavity.
    if p.include_tpu_front_face_pad:
        z_offset = asa_cavity_front_z
    else:
        z_offset = asa_cavity_front_z + 0.5 * (asa_inner_depth - tpu_depth)
    tpu_aligned = tpu_sleeve.move(Location((0.0, 0.0, z_offset)))

    asa_shell = _largest_solid(asa_shell)
    tpu_aligned = _largest_solid(tpu_aligned)
    asa_shell.label = "ASA_Shell"
    tpu_aligned.label = "TPU_Frame"

    asa_inner_w = float(asa_derived["inner_w_mm"])
    asa_inner_h = float(asa_derived["inner_h_mm"])
    tpu_outer_w = float(tpu_derived["dimensions_mm"]["outer_w"])
    tpu_outer_h = float(tpu_derived["dimensions_mm"]["outer_h"])

    radial_gap_w_each = 0.5 * (asa_inner_w - tpu_outer_w)
    radial_gap_h_each = 0.5 * (asa_inner_h - tpu_outer_h)
    front_gap = z_offset - asa_cavity_front_z
    back_gap = (asa_cavity_front_z + asa_inner_depth) - (z_offset + tpu_depth)
    bonded_contact_depth = max(tpu_depth, 0.0) if radial_gap_w_each >= 0.0 and radial_gap_h_each >= 0.0 else 0.0
    max_abs_radial_gap = max(abs(radial_gap_w_each), abs(radial_gap_h_each))
    bond_grade = "A" if max_abs_radial_gap <= p.interface_gap_tolerance_mm else "B"

    report = {
        "named_bodies": ["TPU_Frame", "ASA_Shell"],
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
        "bond_interface_mm": {
            "target_gap_each": float(p.target_interface_gap_each_mm),
            "tolerance_gap_each": float(p.interface_gap_tolerance_mm),
            "resolved_tpu_device_clearance": float(effective_tpu_clearance),
            "max_abs_radial_gap_each": float(max_abs_radial_gap),
            "bonded_contact_depth": float(bonded_contact_depth),
            "bond_grade": bond_grade,
        },
        "print_fusion_guidance": {
            "assemble_bodies_in_slicer": True,
            "recommended_outer_walls": 4,
            "prime_tower": True,
            "min_flush_volume_mm3": 250,
        },
        "tpu_front_face_pad": {
            "enabled": bool(p.include_tpu_front_face_pad),
            "thickness_mm": float(p.tpu_front_face_pad_thickness_mm) if p.include_tpu_front_face_pad else 0.0,
            "tpu_z_placement": "front_flush" if p.include_tpu_front_face_pad else "centered",
        },
        "tpu_edge_wrap": {
            "depth_mm": float(p.tpu_front_edge_wrap_mm),
            "radial_mm": float(p.tpu_edge_wrap_radial_mm),
            "enabled": {
                "front": bool(p.include_tpu_front_edge_wrap),
                "rear": bool(p.include_tpu_rear_edge_wrap),
            },
        },
        "sources": {
            "asa_case_report": asa_report,
            "tpu_sleeve_report": tpu_report,
        },
        "warnings": [],
    }
    if radial_gap_w_each < 0.0 or radial_gap_h_each < 0.0 or front_gap < 0.0 or back_gap < 0.0:
        report["warnings"].append("Detected interference/negative gap in dual-body fit stack.")
    if max_abs_radial_gap > p.interface_gap_tolerance_mm:
        report["warnings"].append(
            "TPU-to-ASA radial interface gap exceeds tolerance; fusion quality may be reduced."
        )

    return asa_shell, tpu_aligned, report


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
    parser.add_argument(
        "--disable-front-tpu-edge-wrap",
        action="store_true",
        help="Disable front edge wrap in TPU sleeve.",
    )
    parser.add_argument(
        "--enable-rear-tpu-edge-wrap",
        action="store_true",
        help="Enable rear edge wrap in TPU sleeve (disabled by default for insertion).",
    )
    parser.add_argument("--no-front-face-pad", action="store_true", help="Disable TPU front face pad")
    parser.add_argument("--front-face-pad-thickness", type=float, default=None, help="TPU front face pad thickness (mm)")
    parser.add_argument("--tripod-rect", action="store_true", help="Use rectangular cutout instead of circular tripod hole")
    parser.add_argument("--tripod-rect-along-width", action="store_true", help="Orient long side of rect along case width (default: along length)")
    parser.add_argument("--out-suffix", type=str, default="", help="Suffix appended to output filenames")
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
    if args.disable_front_tpu_edge_wrap:
        params.include_tpu_front_edge_wrap = False
    if args.enable_rear_tpu_edge_wrap:
        params.include_tpu_rear_edge_wrap = True
    if args.no_front_face_pad:
        params.include_tpu_front_face_pad = False
    if args.front_face_pad_thickness is not None:
        params.tpu_front_face_pad_thickness_mm = args.front_face_pad_thickness
    if args.tripod_rect:
        params.tripod_use_rect_cutout = True
    if args.tripod_rect_along_width:
        params.tripod_rect_long_along_z = False

    suffix = args.out_suffix
    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    # 2 separate output files (rear cap is already a separate script)
    shell_step = args.out / f"maki_live_asa_shell{suffix}.step"
    tpu_step = args.out / f"maki_live_tpu_frame{suffix}.step"
    out_json = reports_dir / f"maki_live_dual_material_report{suffix}.json"
    # Also archive old combined body file
    legacy_step = args.out / f"maki_live_body_dual_material{suffix}.step"
    archived = _archive_existing([shell_step, tpu_step, out_json, legacy_step], args.out)

    asa_shell, tpu_frame, report = build_dual_body(params)
    export_step(asa_shell, str(shell_step))
    export_step(tpu_frame, str(tpu_step))

    payload = {"params": asdict(params), "report": report}
    payload["params"]["step_path"] = str(params.step_path)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {shell_step}")
    print(f"Wrote {tpu_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
