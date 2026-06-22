#!/usr/bin/env python3
"""Generate an ASA-only AVKANS Go protective case shell.

Primary output:
- models/avkans_go_case/avkans_go_asa_shell.step

Design notes:
- Clean B-REP rebuild based on the downloaded AVKANS_GO_RAIN_CASE.stl.
- Combines the original bottom and slide-on hood idea into one fused shell.
- Rear remains open so the camera can slide in.
- Internal divider/partition walls from the source STL are intentionally omitted.
- Front face has a 0.25 inch vertical vent slot through the lens wall.
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    Align,
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Mode,
    Plane,
    Rectangle,
    export_step,
    extrude,
    fillet,
    vertices,
)


@dataclass
class AvkansGoParams:
    # Reference dimensions extracted from /Users/nathanhicks/Downloads/AVKANS_GO_RAIN_CASE.stl.
    # The original STL has two separate mesh bodies: bottom body and slide-on hood.
    source_bottom_outer_w_mm: float = 57.764
    source_bottom_outer_h_mm: float = 77.506
    source_bottom_depth_mm: float = 139.498
    source_hood_outer_w_mm: float = 63.866
    source_hood_outer_h_mm: float = 81.928
    source_hood_depth_mm: float = 159.308

    # One-piece ASA shell.
    asa_wall_mm: float = 3.2
    rear_slide_clearance_mm: float = 0.6
    outer_corner_r_mm: float = 4.0
    inner_corner_r_mm: float = 2.0
    front_wall_mm: float = 3.2

    # Camera/lens opening on the front face.
    # Source lower body circular opening measured about 57.9 mm in projection.
    lens_cutout_d_mm: float = 58.0
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 0.0

    # User-requested front vertical vent: 0.25 inch.
    front_vent_width_mm: float = 6.35
    front_vent_top_bottom_margin_mm: float = 4.0
    front_vent_corner_r_mm: float = 1.5

    # Rear slide-in opening cleanup.
    rear_opening_lead_in_mm: float = 0.6


def _archive_existing(paths: list[Path], out_dir: Path) -> list[tuple[str, str]]:
    archive_dir = out_dir / "archive"
    archive_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    moved: list[tuple[str, str]] = []
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


def _rounded_rect(width: float, height: float, radius: float) -> None:
    Rectangle(width, height)
    r = min(max(radius, 0.0), 0.5 * min(width, height) - 0.1)
    if r > 0.0:
        fillet(vertices(), r)


def build_asa_shell(p: AvkansGoParams):
    outer_w = p.source_hood_outer_w_mm
    outer_h = p.source_hood_outer_h_mm
    outer_depth = p.source_hood_depth_mm

    inner_w = outer_w - 2.0 * p.asa_wall_mm + p.rear_slide_clearance_mm
    inner_h = outer_h - 2.0 * p.asa_wall_mm + p.rear_slide_clearance_mm
    inner_depth = outer_depth - p.front_wall_mm + 0.4
    rear_opening_w = inner_w
    rear_opening_h = inner_h

    vent_h = max(outer_h - 2.0 * p.front_vent_top_bottom_margin_mm, p.front_vent_width_mm)
    vent_r = min(p.front_vent_corner_r_mm, 0.5 * p.front_vent_width_mm - 0.1)

    with BuildPart() as shell_bp:
        with BuildSketch(Plane.XY.offset(0.0)):
            _rounded_rect(outer_w, outer_h, p.outer_corner_r_mm)
        extrude(amount=outer_depth)

        # Rear-load camera cavity. This deliberately removes the separate
        # rear/front compartment divider concept from the source STL.
        with BuildSketch(Plane.XY.offset(p.front_wall_mm)):
            _rounded_rect(inner_w, inner_h, p.inner_corner_r_mm)
        extrude(amount=inner_depth, mode=Mode.SUBTRACT)

        # Slight rear lead-in so the open back is not a sharp scraping edge.
        if p.rear_opening_lead_in_mm > 0.0:
            lead_w = min(inner_w + 2.0 * p.rear_opening_lead_in_mm, outer_w - 0.4)
            lead_h = min(inner_h + 2.0 * p.rear_opening_lead_in_mm, outer_h - 0.4)
            with BuildSketch(Plane.XY.offset(outer_depth - p.rear_opening_lead_in_mm)):
                _rounded_rect(lead_w, lead_h, p.inner_corner_r_mm + p.rear_opening_lead_in_mm)
            extrude(amount=p.rear_opening_lead_in_mm + 0.3, mode=Mode.SUBTRACT)

        # Front camera opening.
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                Circle(
                    0.5 * p.lens_cutout_d_mm,
                    align=(Align.CENTER, Align.CENTER),
                )
        extrude(amount=p.front_wall_mm + 0.6, mode=Mode.SUBTRACT)

        # 0.25 inch front vertical vent through the front face. It is centered
        # and intentionally intersects the circular opening, making one
        # continuous ventilation/keyhole-style front opening.
        with BuildSketch(Plane.XY.offset(-0.2)):
            Rectangle(p.front_vent_width_mm, vent_h)
            if vent_r > 0.0:
                fillet(vertices(), vent_r)
        extrude(amount=p.front_wall_mm + 0.6, mode=Mode.SUBTRACT)

    shell = _largest_solid(shell_bp.part)
    shell.label = "AVKANS_GO_ASA_Shell"

    report = {
        "params": asdict(p),
        "source_reference": {
            "file": "refs/AVKANS_GO_RAIN_CASE.stl",
            "source_kind": "two-body STL reference rebuilt as clean B-REP",
            "source_bottom_outer_mm": {
                "w": float(p.source_bottom_outer_w_mm),
                "h": float(p.source_bottom_outer_h_mm),
                "depth": float(p.source_bottom_depth_mm),
            },
            "source_hood_outer_mm": {
                "w": float(p.source_hood_outer_w_mm),
                "h": float(p.source_hood_outer_h_mm),
                "depth": float(p.source_hood_depth_mm),
            },
        },
        "asa_shell_mm": {
            "outer_w": float(outer_w),
            "outer_h": float(outer_h),
            "outer_depth": float(outer_depth),
            "inner_w": float(inner_w),
            "inner_h": float(inner_h),
            "front_wall": float(p.front_wall_mm),
            "rear_opening_w": float(rear_opening_w),
            "rear_opening_h": float(rear_opening_h),
            "wall": float(p.asa_wall_mm),
            "single_fused_shell": True,
            "rear_sliding_insertion": True,
            "internal_partition_walls_removed": True,
        },
        "features_mm": {
            "front_lens_opening": {
                "diameter": float(p.lens_cutout_d_mm),
                "center_x": float(p.lens_center_x_mm),
                "center_y": float(p.lens_center_y_mm),
            },
            "front_vertical_vent": {
                "width": float(p.front_vent_width_mm),
                "height": float(vent_h),
                "width_source": "0.25 inch",
                "centered": True,
                "intersects_lens_opening": True,
                "corner_r": float(vent_r),
            },
        },
        "named_bodies": ["AVKANS_GO_ASA_Shell"],
    }

    return shell, report


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate AVKANS Go ASA shell")
    parser.add_argument("--out", type=Path, default=Path("models/avkans_go_case"))
    args = parser.parse_args()

    out_dir = args.out
    reports_dir = out_dir / "reports"
    out_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)

    shell_step = out_dir / "avkans_go_asa_shell.step"
    report_json = reports_dir / "avkans_go_report.json"
    archived = _archive_existing([shell_step, report_json], out_dir)

    params = AvkansGoParams()
    shell, report = build_asa_shell(params)
    report["archived_previous_outputs"] = archived

    export_step(shell, str(shell_step))
    report_json.write_text(json.dumps(report, indent=2) + "\n")

    print(f"Wrote {shell_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
