#!/usr/bin/env python3
"""Generate a Mevo Start TPU sleeve that fits inside the ASA shell.

The script reads ASA shell parameters from models/mevo_case/reports/mevo_start_case_report.json by default,
then computes the maximum TPU shell thickness that still nests with a target assembly gap.

Outputs:
- mevo_start_tpu_sleeve.step
- reports/mevo_start_tpu_sleeve_report.json
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Rectangle,
    Mode,
    Plane,
    export_step,
    extrude,
    fillet,
    vertices,
)


@dataclass
class MevoTpuLinerParams:
    # Source ASA case report to inherit device and shell alignment defaults.
    asa_report_json: Path = Path("models/mevo_case/reports/mevo_start_case_report.json")

    # Device dimensions (mm)
    device_length_mm: float = 87.0
    device_height_mm: float = 75.5
    device_width_mm: float = 34.0

    # ASA shell internals this liner must fit into.
    asa_clearance_mm: float = 2.3
    asa_front_wall_mm: float = 0.0
    tripod_zone_z_ratio: float = 0.78
    tripod_hole_d_mm: float = 25.4

    # Mevo vent alignment copied from ASA shell script.
    vent_count: int = 3
    vent_h_mm: float = 2.2
    vent_len_mm: float = 12.0
    vent_pitch_mm: float = 4.8
    vent_z_ratio: float = 0.57

    # TPU fit targets
    device_clearance_mm: float = 0.20
    end_clearance_mm: float = 0.20
    shell_to_asa_gap_mm: float = 0.10
    asa_cap_plug_depth_mm: float = 1.80
    desired_shell_thickness_mm: float = 2.00
    min_printable_shell_thickness_mm: float = 1.20
    edge_wrap_depth_mm: float = 2.5
    edge_wrap_radial_mm: float = 2.0
    include_front_edge_wrap: bool = True
    include_rear_edge_wrap: bool = False

    # Optional openings in TPU for tripod/vents
    enable_tripod_opening: bool = True
    enable_side_vents: bool = True


def _safe_fillet_radius(width: float, height: float, radius: float) -> float:
    return max(0.25, min(float(radius), 0.5 * min(width, height) - 0.05))


def _add_capsule(width: float, height: float) -> None:
    Rectangle(width, height)
    fillet(vertices(), _safe_fillet_radius(width, height, 0.5 * width - 0.05))


def _largest_solid(shape):
    solids = shape.solids() if hasattr(shape, "solids") else []
    if len(solids) <= 1:
        return shape
    return max(solids, key=lambda s: s.volume)


def _apply_axis_fillet(shape, axis: Axis, radii: tuple[float, ...]):
    for r in radii:
        try:
            return fillet(shape.edges().filter_by(axis), r), float(r)
        except Exception:
            continue
    return shape, 0.0


def _apply_all_edge_fillet(shape, radii: tuple[float, ...]):
    for r in radii:
        try:
            return fillet(shape.edges(), r), float(r)
        except Exception:
            continue
    return shape, 0.0


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


def _resolve_asa_report_path(path: Path) -> Path:
    """Prefer reports/ path, but fall back to legacy top-level report location."""
    if path.exists():
        return path
    if path.parent.name == "reports":
        legacy = path.parent.parent / path.name
        if legacy.exists():
            return legacy
    return path


def _load_asa_defaults(p: MevoTpuLinerParams) -> None:
    if not p.asa_report_json.exists():
        return
    try:
        payload = json.loads(p.asa_report_json.read_text(encoding="utf-8"))
    except Exception:
        return
    params = payload.get("params", {})
    if not isinstance(params, dict):
        return

    p.device_length_mm = float(params.get("device_length_mm", p.device_length_mm))
    p.device_height_mm = float(params.get("device_height_mm", p.device_height_mm))
    p.device_width_mm = float(params.get("device_width_mm", p.device_width_mm))
    p.asa_clearance_mm = float(params.get("clearance_mm", p.asa_clearance_mm))
    open_through = bool(params.get("open_through_sleeve", False))
    if open_through:
        p.asa_front_wall_mm = 0.0
    else:
        p.asa_front_wall_mm = float(params.get("front_wall_mm", p.asa_front_wall_mm))
    p.tripod_zone_z_ratio = float(params.get("tripod_zone_z_ratio", p.tripod_zone_z_ratio))
    p.tripod_hole_d_mm = float(
        params.get(
            "tripod_hole_d_mm",
            max(params.get("tripod_slot_w_mm", p.tripod_hole_d_mm), params.get("tripod_slot_l_mm", p.tripod_hole_d_mm)),
        )
    )
    p.vent_count = int(params.get("vent_count", p.vent_count))
    p.vent_h_mm = float(params.get("vent_h_mm", p.vent_h_mm))
    p.vent_len_mm = float(params.get("vent_len_mm", p.vent_len_mm))
    p.vent_pitch_mm = float(params.get("vent_pitch_mm", p.vent_pitch_mm))
    p.vent_z_ratio = float(params.get("vent_z_ratio", p.vent_z_ratio))


def build_liner(p: MevoTpuLinerParams):
    # ASA cavity dimensions from outer shell design.
    asa_inner_w = p.device_width_mm + 2.0 * p.asa_clearance_mm
    asa_inner_h = p.device_height_mm + 2.0 * p.asa_clearance_mm
    asa_inner_depth = p.device_length_mm + 2.0 * p.asa_clearance_mm

    # TPU thickness budget to guarantee nest fit:
    # tpu_outer_half <= asa_inner_half - shell_to_asa_gap
    # => tpu_thickness <= asa_clearance - device_clearance - shell_to_asa_gap
    max_shell_t = p.asa_clearance_mm - p.device_clearance_mm - p.shell_to_asa_gap_mm
    if max_shell_t <= 0.0:
        raise ValueError(
            "No TPU thickness budget remains with current clearances. "
            "Increase ASA shell clearance or reduce device/assembly gaps."
        )
    shell_t = min(p.desired_shell_thickness_mm, max_shell_t)

    inner_w = p.device_width_mm + 2.0 * p.device_clearance_mm
    inner_h = p.device_height_mm + 2.0 * p.device_clearance_mm
    outer_w = inner_w + 2.0 * shell_t
    outer_h = inner_h + 2.0 * shell_t

    liner_depth = p.device_length_mm + 2.0 * p.end_clearance_mm

    # Keep liner centered in shell cavity and maintain assembly headroom.
    radial_margin_w = 0.5 * (asa_inner_w - outer_w)
    radial_margin_h = 0.5 * (asa_inner_h - outer_h)
    axial_margin = asa_inner_depth - liner_depth
    axial_margin_each_end = 0.5 * axial_margin

    # Alignment of openings to the existing ASA shell coordinate frame.
    tripod_z = (p.asa_front_wall_mm + asa_inner_depth) * p.tripod_zone_z_ratio - p.asa_front_wall_mm
    vent_z = p.asa_front_wall_mm + asa_inner_depth * p.vent_z_ratio - p.asa_front_wall_mm

    min_y = -0.5 * outer_h
    max_x = 0.5 * outer_w

    edge_wrap_depth = max(min(p.edge_wrap_depth_mm, 0.45 * liner_depth), 0.6)
    edge_wrap_radial = max(p.edge_wrap_radial_mm, 0.6)
    wrap_inner_w = max(inner_w - 2.0 * edge_wrap_radial, 2.0)
    wrap_inner_h = max(inner_h - 2.0 * edge_wrap_radial, wrap_inner_w + 0.2)

    with BuildPart() as liner_bp:
        # Open-through capsule tube (single TPU sleeve).
        with BuildSketch(Plane.XY):
            _add_capsule(outer_w, outer_h)
        extrude(amount=liner_depth)

        with BuildSketch(Plane.XY.offset(-0.2)):
            _add_capsule(inner_w, inner_h)
        extrude(amount=liner_depth + 0.4, mode=Mode.SUBTRACT)

        # Thin edge wraps around selected face perimeter(s) only.
        # Default is front-only so the camera can slide in from the open rear.
        if p.include_front_edge_wrap:
            with BuildSketch(Plane.XY):
                _add_capsule(inner_w, inner_h)
            extrude(amount=edge_wrap_depth)
            with BuildSketch(Plane.XY.offset(-0.1)):
                _add_capsule(wrap_inner_w, wrap_inner_h)
            extrude(amount=edge_wrap_depth + 0.2, mode=Mode.SUBTRACT)

        if p.include_rear_edge_wrap:
            rear_start = liner_depth - edge_wrap_depth
            with BuildSketch(Plane.XY.offset(rear_start)):
                _add_capsule(inner_w, inner_h)
            extrude(amount=edge_wrap_depth)
            with BuildSketch(Plane.XY.offset(rear_start - 0.1)):
                _add_capsule(wrap_inner_w, wrap_inner_h)
            extrude(amount=edge_wrap_depth + 0.2, mode=Mode.SUBTRACT)

        if p.enable_side_vents:
            for side in (-1.0, 1.0):
                x = side * (max_x - shell_t * 0.55)
                for i in range(p.vent_count):
                    y = (i - (p.vent_count - 1) * 0.5) * p.vent_pitch_mm
                    with Locations((x, y, vent_z)):
                        Box(shell_t * 2.4, p.vent_h_mm, p.vent_len_mm, mode=Mode.SUBTRACT)

        if p.enable_tripod_opening:
            hole_depth = shell_t + 1.8
            with BuildSketch(Plane.XZ.offset(min_y + 0.12)):
                with Locations((0.0, tripod_z)):
                    Circle(0.5 * p.tripod_hole_d_mm)
            extrude(amount=-hole_depth, mode=Mode.SUBTRACT)

    liner = _largest_solid(liner_bp.part)
    liner, axis_y_fillet = _apply_axis_fillet(liner, Axis.Y, (0.7, 0.55, 0.4, 0.3, 0.2))
    all_edge_fillet = 0.0
    if axis_y_fillet <= 0.0:
        liner, all_edge_fillet = _apply_all_edge_fillet(liner, (0.25, 0.2, 0.15))

    report = {
        "derived": {
            "device_mm": {
                "length": float(p.device_length_mm),
                "height": float(p.device_height_mm),
                "width": float(p.device_width_mm),
            },
            "asa_inner_mm": {
                "depth": float(asa_inner_depth),
                "height": float(asa_inner_h),
                "width": float(asa_inner_w),
            },
            "tpu_mm": {
                "shell_thickness_max_by_fit": float(max_shell_t),
                "shell_thickness_applied": float(shell_t),
                "inner_depth": float(liner_depth),
                "inner_height": float(inner_h),
                "inner_width": float(inner_w),
                "outer_height": float(outer_h),
                "outer_width": float(outer_w),
                "edge_wrap_depth": float(edge_wrap_depth),
                "edge_wrap_radial": float(edge_wrap_radial),
                "edge_wrap_enabled": {
                    "front": bool(p.include_front_edge_wrap),
                    "rear": bool(p.include_rear_edge_wrap),
                },
            },
            "fit_margins_mm": {
                "radial_w_each_side": float(radial_margin_w),
                "radial_h_each_side": float(radial_margin_h),
                "axial_total": float(axial_margin),
                "axial_each_end": float(axial_margin_each_end),
                "axial_cap_budget_each_end": float(axial_margin_each_end - p.asa_cap_plug_depth_mm),
                "axial_cap_budget_total_two_caps": float(axial_margin - 2.0 * p.asa_cap_plug_depth_mm),
            },
            "openings_alignment_mm": {
                "tripod_hole_center_z": float(tripod_z),
                "tripod_hole_diameter": float(p.tripod_hole_d_mm),
                "vent_center_z": float(vent_z),
            },
            "machined_finish_mm": {
                "liner_axis_y_fillet": float(axis_y_fillet),
                "liner_all_edge_fillet": float(all_edge_fillet),
            },
            "warnings": {
                "thin_shell_for_existing_asa": shell_t < p.min_printable_shell_thickness_mm,
                "recommended_asa_clearance_for_1p2mm_tpu": float(
                    p.device_clearance_mm + p.shell_to_asa_gap_mm + 1.2
                ),
            },
        }
    }

    return liner, report


def main():
    parser = argparse.ArgumentParser(description="Generate Mevo Start TPU sleeve")
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument(
        "--asa-report",
        type=Path,
        default=Path("models/mevo_case/reports/mevo_start_case_report.json"),
        help="ASA case report JSON used for alignment and fit budget",
    )
    parser.add_argument("--device-clearance", type=float, default=None, help="TPU-to-device radial clearance (mm)")
    parser.add_argument("--end-clearance", type=float, default=None, help="TPU front/rear end clearance (mm)")
    parser.add_argument("--shell-gap", type=float, default=None, help="Target radial gap between TPU and ASA shell (mm)")
    parser.add_argument("--thickness", type=float, default=None, help="Desired TPU shell thickness (mm)")
    parser.add_argument("--edge-wrap-depth", type=float, default=None, help="Front/rear edge wrap depth (mm)")
    parser.add_argument("--edge-wrap-radial", type=float, default=None, help="How far edge wrap extends onto face from perimeter (mm)")
    parser.add_argument(
        "--disable-front-edge-wrap",
        action="store_true",
        help="Disable TPU front perimeter edge wrap.",
    )
    parser.add_argument(
        "--enable-rear-edge-wrap",
        action="store_true",
        help="Enable TPU rear perimeter edge wrap (disabled by default for insertion).",
    )
    parser.add_argument("--no-tripod-opening", action="store_true", help="Disable TPU tripod opening")
    parser.add_argument("--no-side-vents", action="store_true", help="Disable TPU side vent openings")
    args = parser.parse_args()

    params = MevoTpuLinerParams(asa_report_json=_resolve_asa_report_path(args.asa_report))
    _load_asa_defaults(params)
    if args.device_clearance is not None:
        params.device_clearance_mm = args.device_clearance
    if args.end_clearance is not None:
        params.end_clearance_mm = args.end_clearance
    if args.shell_gap is not None:
        params.shell_to_asa_gap_mm = args.shell_gap
    if args.thickness is not None:
        params.desired_shell_thickness_mm = args.thickness
    if args.edge_wrap_depth is not None:
        params.edge_wrap_depth_mm = args.edge_wrap_depth
    if args.edge_wrap_radial is not None:
        params.edge_wrap_radial_mm = args.edge_wrap_radial
    if args.disable_front_edge_wrap:
        params.include_front_edge_wrap = False
    if args.enable_rear_edge_wrap:
        params.include_rear_edge_wrap = True
    if args.no_tripod_opening:
        params.enable_tripod_opening = False
    if args.no_side_vents:
        params.enable_side_vents = False

    liner, report = build_liner(params)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    out_step = args.out / "mevo_start_tpu_sleeve.step"
    out_json = reports_dir / "mevo_start_tpu_sleeve_report.json"
    archived = _archive_existing(
        [
            out_step,
            out_json,
            args.out / "mevo_start_tpu_sleeve_report.json",  # legacy top-level location
            args.out / "mevo_start_tpu_liner.step",
            args.out / "mevo_start_tpu_liner_report.json",
        ],
        args.out,
    )

    export_step(liner, str(out_step))
    payload = {"params": asdict(params), "report": report}
    payload["params"]["asa_report_json"] = str(params.asa_report_json)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
