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
import math
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Mode,
    Plane,
    Polygon,
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
    front_wall_mm: float = 3.2
    hood_arch_spring_y_mm: float = 14.0
    lower_body_shoulder_y_mm: float = -22.0
    arch_segments: int = 18

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

    # Source-inspired side/top shell relief and rail features.
    include_side_relief_cutouts: bool = True
    side_relief_slot_h_mm: float = 5.0
    side_relief_slot_len_mm: float = 26.0
    side_relief_center_y_mm: float = 18.0
    side_relief_z_centers_mm: tuple[float, ...] = (42.0, 80.0, 118.0)
    side_relief_corner_r_mm: float = 1.2
    include_top_relief_cutouts: bool = True
    top_relief_slot_w_mm: float = 24.0
    top_relief_slot_len_mm: float = 24.0
    top_relief_z_centers_mm: tuple[float, ...] = (42.0, 80.0, 118.0)
    top_relief_corner_r_mm: float = 1.5
    include_side_rails: bool = True
    side_rail_width_mm: float = 1.6
    side_rail_height_mm: float = 8.0
    side_rail_z_margin_mm: float = 12.0


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


def _arched_profile_points(
    lower_w: float,
    hood_w: float,
    height: float,
    shoulder_y: float,
    spring_y: float,
    segments: int,
) -> list[tuple[float, float]]:
    half_lower = 0.5 * lower_w
    half_hood = 0.5 * hood_w
    bottom_y = -0.5 * height
    top_y = 0.5 * height
    arch_h = max(top_y - spring_y, 1.0)
    segments = max(int(segments), 6)

    points: list[tuple[float, float]] = [
        (-half_lower, bottom_y),
        (half_lower, bottom_y),
        (half_lower, shoulder_y),
        (half_hood, shoulder_y),
        (half_hood, spring_y),
    ]
    for i in range(1, segments):
        t = i / segments
        theta = t * math.pi
        x = half_hood * math.cos(theta)
        y = spring_y + arch_h * math.sin(theta)
        points.append((x, y))
    points.extend([
        (-half_hood, spring_y),
        (-half_hood, shoulder_y),
        (-half_lower, shoulder_y),
    ])
    return points


def _arched_profile(
    lower_w: float,
    hood_w: float,
    height: float,
    shoulder_y: float,
    spring_y: float,
    segments: int,
) -> None:
    Polygon(
        *_arched_profile_points(
            lower_w=lower_w,
            hood_w=hood_w,
            height=height,
            shoulder_y=shoulder_y,
            spring_y=spring_y,
            segments=segments,
        )
    )


def _rounded_rect_slot(width: float, height: float, radius: float) -> None:
    Rectangle(width, height)
    r = min(max(radius, 0.0), 0.5 * min(width, height) - 0.1)
    if r > 0.0:
        fillet(vertices(), r)


def build_asa_shell(p: AvkansGoParams):
    lower_w = p.source_bottom_outer_w_mm
    outer_w = p.source_hood_outer_w_mm
    outer_h = p.source_hood_outer_h_mm
    outer_depth = p.source_hood_depth_mm

    cavity_offset = p.asa_wall_mm - 0.5 * p.rear_slide_clearance_mm
    inner_lower_w = lower_w - 2.0 * cavity_offset
    inner_hood_w = outer_w - 2.0 * cavity_offset
    inner_h = outer_h - 2.0 * cavity_offset
    inner_depth = outer_depth - p.front_wall_mm + 0.4
    rear_opening_w = inner_hood_w
    rear_opening_h = inner_h

    vent_h = max(outer_h - 2.0 * p.front_vent_top_bottom_margin_mm, p.front_vent_width_mm)
    vent_r = min(p.front_vent_corner_r_mm, 0.5 * p.front_vent_width_mm - 0.1)

    with BuildPart() as shell_bp:
        with BuildSketch(Plane.XY.offset(0.0)):
            _arched_profile(
                lower_w,
                outer_w,
                outer_h,
                p.lower_body_shoulder_y_mm,
                p.hood_arch_spring_y_mm,
                p.arch_segments,
            )
        extrude(amount=outer_depth)

        # Rear-load camera cavity. This deliberately removes the separate
        # rear/front compartment divider concept from the source STL.
        with BuildSketch(Plane.XY.offset(p.front_wall_mm)):
            _arched_profile(
                inner_lower_w,
                inner_hood_w,
                inner_h,
                p.lower_body_shoulder_y_mm + cavity_offset,
                p.hood_arch_spring_y_mm - cavity_offset,
                p.arch_segments,
            )
        extrude(amount=inner_depth, mode=Mode.SUBTRACT)

        # Slight rear lead-in so the open back is not a sharp scraping edge.
        if p.rear_opening_lead_in_mm > 0.0:
            lead_lower_w = min(inner_lower_w + 2.0 * p.rear_opening_lead_in_mm, lower_w - 0.4)
            lead_hood_w = min(inner_hood_w + 2.0 * p.rear_opening_lead_in_mm, outer_w - 0.4)
            lead_h = min(inner_h + 2.0 * p.rear_opening_lead_in_mm, outer_h - 0.4)
            with BuildSketch(Plane.XY.offset(outer_depth - p.rear_opening_lead_in_mm)):
                _arched_profile(
                    lead_lower_w,
                    lead_hood_w,
                    lead_h,
                    p.lower_body_shoulder_y_mm + cavity_offset,
                    p.hood_arch_spring_y_mm - cavity_offset,
                    p.arch_segments,
                )
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

        if p.include_side_relief_cutouts:
            side_cut_depth = p.asa_wall_mm + p.side_rail_width_mm + 1.0
            slot_r = min(
                p.side_relief_corner_r_mm,
                0.5 * min(p.side_relief_slot_h_mm, p.side_relief_slot_len_mm) - 0.1,
            )
            for sx in (-1.0, 1.0):
                x_face = sx * (0.5 * outer_w + 0.1)
                for zc in p.side_relief_z_centers_mm:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_relief_center_y_mm, zc)):
                            _rounded_rect_slot(
                                p.side_relief_slot_h_mm,
                                p.side_relief_slot_len_mm,
                                slot_r,
                            )
                    extrude(
                        amount=-sx * side_cut_depth,
                        mode=Mode.SUBTRACT,
                    )

        if p.include_top_relief_cutouts:
            top_cut_depth = p.asa_wall_mm + 1.0
            slot_r = min(
                p.top_relief_corner_r_mm,
                0.5 * min(p.top_relief_slot_w_mm, p.top_relief_slot_len_mm) - 0.1,
            )
            top_y = 0.5 * outer_h + 0.1
            for zc in p.top_relief_z_centers_mm:
                with BuildSketch(Plane.XZ.offset(top_y)):
                    with Locations((0.0, zc)):
                        _rounded_rect_slot(
                            p.top_relief_slot_w_mm,
                            p.top_relief_slot_len_mm,
                            slot_r,
                        )
                extrude(amount=-top_cut_depth, mode=Mode.SUBTRACT)

        if p.include_side_rails:
            rail_depth = max(outer_depth - 2.0 * p.side_rail_z_margin_mm, 1.0)
            rail_z = p.side_rail_z_margin_mm + 0.5 * rail_depth
            rail_y = p.lower_body_shoulder_y_mm - 0.5 * p.side_rail_height_mm
            for sx in (-1.0, 1.0):
                rail_x = sx * (0.5 * lower_w + 0.5 * p.side_rail_width_mm)
                with Locations((rail_x, rail_y, rail_z)):
                    Box(
                        p.side_rail_width_mm,
                        p.side_rail_height_mm,
                        rail_depth,
                        mode=Mode.ADD,
                    )

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
            "outer_profile": "arched hood with narrower lower body side skirts",
            "lower_body_w": float(lower_w),
            "inner_lower_w": float(inner_lower_w),
            "inner_hood_w": float(inner_hood_w),
            "inner_h": float(inner_h),
            "front_wall": float(p.front_wall_mm),
            "rear_opening_w": float(rear_opening_w),
            "rear_opening_h": float(rear_opening_h),
            "wall": float(p.asa_wall_mm),
            "effective_cavity_offset": float(cavity_offset),
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
            "source_inspired_contours": {
                "arched_hood_profile": True,
                "narrower_lower_body": True,
                "side_relief_cutouts": bool(p.include_side_relief_cutouts),
                "top_relief_cutouts": bool(p.include_top_relief_cutouts),
                "side_rails": bool(p.include_side_rails),
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
