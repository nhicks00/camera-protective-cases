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

    # One-piece ASA shell in the same convention as the other project generators:
    # X = width, Y = vertical height, Z = front/back depth.
    asa_wall_mm: float = 3.2
    rear_slide_clearance_mm: float = 0.6
    front_wall_depth_mm: float = 3.2
    hood_arch_spring_y_mm: float = 143.0
    lower_body_shoulder_y_mm: float = 118.0
    arch_segments: int = 18
    source_bottom_inner_w_mm: float = 41.0
    source_bottom_inner_depth_mm: float = 71.1
    source_hood_inner_w_mm: float = 55.0

    # Camera/lens opening on the front face.
    # Source lower body front cutouts extracted from the bottom STL front-face loops.
    lens_cutout_d_mm: float = 33.0
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 50.54
    front_source_slot_w_mm: float = 22.62
    front_upper_source_slot_h_mm: float = 17.60
    front_upper_source_slot_center_y_mm: float = 119.50
    front_lower_source_slot_h_mm: float = 7.89
    front_lower_source_slot_center_y_mm: float = 18.25
    front_source_slot_corner_r_mm: float = 1.2

    # User-requested front vertical vent: 0.25 inch.
    front_vent_width_mm: float = 6.35
    front_vent_top_bottom_margin_mm: float = 4.0
    front_vent_corner_r_mm: float = 1.5

    # Rear slide-in opening cleanup.
    rear_opening_lead_in_mm: float = 0.6

    # Source-inspired side/top shell relief and rail features.
    include_side_relief_cutouts: bool = True
    side_relief_slot_depth_mm: float = 5.0
    side_relief_slot_h_mm: float = 22.0
    side_relief_center_z_mm: float = 41.0
    side_relief_y_centers_mm: tuple[float, ...] = (38.0, 78.0, 118.0)
    side_relief_corner_r_mm: float = 1.2
    include_top_relief_cutouts: bool = True
    top_relief_slot_w_mm: float = 24.0
    top_relief_slot_depth_mm: float = 24.0
    top_relief_z_centers_mm: tuple[float, ...] = (22.0, 41.0, 60.0)
    top_relief_corner_r_mm: float = 1.5
    include_side_rails: bool = True
    side_rail_width_mm: float = 1.6
    side_rail_depth_mm: float = 8.0
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
    z_min: float,
    z_max: float,
    shoulder_z: float,
    spring_z: float,
    segments: int,
) -> list[tuple[float, float]]:
    half_lower = 0.5 * lower_w
    half_hood = 0.5 * hood_w
    arch_h = max(z_max - spring_z, 1.0)
    segments = max(int(segments), 6)

    points: list[tuple[float, float]] = [
        (-half_lower, z_min),
        (half_lower, z_min),
        (half_lower, shoulder_z),
        (half_hood, shoulder_z),
        (half_hood, spring_z),
    ]
    for i in range(1, segments):
        t = i / segments
        theta = t * math.pi
        x = half_hood * math.cos(theta)
        z = spring_z + arch_h * math.sin(theta)
        points.append((x, z))
    points.extend([
        (-half_hood, spring_z),
        (-half_hood, shoulder_z),
        (-half_lower, shoulder_z),
    ])
    return points


def _arched_profile(
    lower_w: float,
    hood_w: float,
    z_min: float,
    z_max: float,
    shoulder_z: float,
    spring_z: float,
    segments: int,
) -> None:
    Polygon(
        *_arched_profile_points(
            lower_w=lower_w,
            hood_w=hood_w,
            z_min=z_min,
            z_max=z_max,
            shoulder_z=shoulder_z,
            spring_z=spring_z,
            segments=segments,
        ),
        align=(Align.NONE, Align.NONE),
    )


def _rounded_rect_slot(width: float, height: float, radius: float) -> None:
    Rectangle(width, height)
    r = min(max(radius, 0.0), 0.5 * min(width, height) - 0.1)
    if r > 0.0:
        fillet(vertices(), r)


def build_asa_shell(p: AvkansGoParams):
    lower_w = p.source_bottom_outer_w_mm
    outer_w = p.source_hood_outer_w_mm
    lower_depth = p.source_bottom_outer_h_mm
    lower_height = p.source_bottom_depth_mm
    outer_depth = p.source_hood_outer_h_mm
    outer_height = p.source_hood_depth_mm

    cavity_offset = p.asa_wall_mm - 0.5 * p.rear_slide_clearance_mm
    inner_lower_w = p.source_bottom_inner_w_mm
    inner_hood_w = p.source_hood_inner_w_mm
    inner_depth = p.source_bottom_inner_depth_mm
    inner_y_min = cavity_offset
    inner_y_max = outer_height - cavity_offset
    rear_opening_w = inner_hood_w
    rear_opening_h = inner_y_max - inner_y_min

    front_z = 0.0
    rear_z = outer_depth
    cavity_front_z = rear_z - inner_depth
    effective_front_wall_depth = cavity_front_z - front_z
    vent_h = max(lower_height - 2.0 * p.front_vent_top_bottom_margin_mm, p.front_vent_width_mm)
    vent_r = min(p.front_vent_corner_r_mm, 0.5 * p.front_vent_width_mm - 0.1)

    with BuildPart() as shell_bp:
        with BuildSketch(Plane.XY):
            _arched_profile(
                lower_w,
                outer_w,
                0.0,
                outer_height,
                p.lower_body_shoulder_y_mm,
                p.hood_arch_spring_y_mm,
                p.arch_segments,
            )
        extrude(amount=outer_depth)

        # Rear-load camera cavity. This deliberately removes the separate
        # rear/front compartment divider concept from the source STL.
        with BuildSketch(Plane.XY.offset(cavity_front_z)):
            _arched_profile(
                inner_lower_w,
                inner_hood_w,
                inner_y_min,
                inner_y_max,
                p.lower_body_shoulder_y_mm - cavity_offset,
                p.hood_arch_spring_y_mm - cavity_offset,
                p.arch_segments,
            )
        extrude(amount=(rear_z - cavity_front_z + 0.6), mode=Mode.SUBTRACT)

        # Slight rear lead-in so the open back is not a sharp scraping edge.
        if p.rear_opening_lead_in_mm > 0.0:
            lead_lower_w = min(inner_lower_w + 2.0 * p.rear_opening_lead_in_mm, lower_w - 0.4)
            lead_hood_w = min(inner_hood_w + 2.0 * p.rear_opening_lead_in_mm, outer_w - 0.4)
            lead_y_min = max(inner_y_min - p.rear_opening_lead_in_mm, 0.4)
            lead_y_max = min(inner_y_max + p.rear_opening_lead_in_mm, outer_height - 0.4)
            lead_start_z = rear_z - p.rear_opening_lead_in_mm
            with BuildSketch(Plane.XY.offset(lead_start_z)):
                _arched_profile(
                    lead_lower_w,
                    lead_hood_w,
                    lead_y_min,
                    lead_y_max,
                    p.lower_body_shoulder_y_mm - cavity_offset,
                    p.hood_arch_spring_y_mm - cavity_offset,
                    p.arch_segments,
                )
            extrude(amount=(p.rear_opening_lead_in_mm + 0.3), mode=Mode.SUBTRACT)

        # Front camera opening.
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                Circle(
                    0.5 * p.lens_cutout_d_mm,
                    align=(Align.CENTER, Align.CENTER),
                )
        extrude(amount=(effective_front_wall_depth + 0.6), mode=Mode.SUBTRACT)

        # Preserve the two smaller source front-face slots, then connect the
        # face vertically with the requested narrow vent cut.
        for slot_h, slot_y in (
            (p.front_upper_source_slot_h_mm, p.front_upper_source_slot_center_y_mm),
            (p.front_lower_source_slot_h_mm, p.front_lower_source_slot_center_y_mm),
        ):
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((0.0, slot_y)):
                    _rounded_rect_slot(
                        p.front_source_slot_w_mm,
                        slot_h,
                        p.front_source_slot_corner_r_mm,
                    )
            extrude(amount=(effective_front_wall_depth + 0.6), mode=Mode.SUBTRACT)

        # 0.25 inch front vertical vent through the front lower body face. It
        # remains centered and intersects the circular lens opening.
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((0.0, 0.5 * lower_height)):
                Rectangle(p.front_vent_width_mm, vent_h)
                if vent_r > 0.0:
                    fillet(vertices(), vent_r)
        extrude(amount=(effective_front_wall_depth + 0.6), mode=Mode.SUBTRACT)

        if p.include_side_relief_cutouts:
            side_cut_depth = p.asa_wall_mm + p.side_rail_width_mm + 1.0
            slot_r = min(
                p.side_relief_corner_r_mm,
                0.5 * min(p.side_relief_slot_depth_mm, p.side_relief_slot_h_mm) - 0.1,
            )
            for sx in (-1.0, 1.0):
                x_face = sx * (0.5 * outer_w + 0.1)
                for yc in p.side_relief_y_centers_mm:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((yc, p.side_relief_center_z_mm)):
                            _rounded_rect_slot(
                                p.side_relief_slot_h_mm,
                                p.side_relief_slot_depth_mm,
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
                0.5 * min(p.top_relief_slot_w_mm, p.top_relief_slot_depth_mm) - 0.1,
            )
            top_y = outer_height + 0.1
            for zc in p.top_relief_z_centers_mm:
                with BuildSketch(Plane.XZ.offset(-top_y)):
                    with Locations((0.0, zc)):
                        _rounded_rect_slot(
                            p.top_relief_slot_w_mm,
                            p.top_relief_slot_depth_mm,
                            slot_r,
                        )
                extrude(amount=top_cut_depth, mode=Mode.SUBTRACT)

        if p.include_side_rails:
            rail_height = max(lower_height - 2.0 * p.side_rail_z_margin_mm, 1.0)
            rail_y = p.side_rail_z_margin_mm + 0.5 * rail_height
            rail_z = 0.5 * p.side_rail_depth_mm
            for sx in (-1.0, 1.0):
                rail_x = sx * (0.5 * lower_w + 0.5 * p.side_rail_width_mm)
                with Locations((rail_x, rail_y, rail_z)):
                    Box(
                        p.side_rail_width_mm,
                        rail_height,
                        p.side_rail_depth_mm,
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
            "outer_depth": float(outer_depth),
            "outer_height": float(outer_height),
            "outer_profile": "assembled vertical arched hood with narrower lower body side skirts",
            "lower_body_w": float(lower_w),
            "lower_body_depth": float(lower_depth),
            "lower_body_height": float(lower_height),
            "inner_lower_w": float(inner_lower_w),
            "inner_hood_w": float(inner_hood_w),
            "inner_depth": float(inner_depth),
            "inner_y_min": float(inner_y_min),
            "inner_y_max": float(inner_y_max),
            "front_wall_depth": float(effective_front_wall_depth),
            "requested_min_front_wall_depth": float(p.front_wall_depth_mm),
            "rear_opening_w": float(rear_opening_w),
            "rear_opening_h": float(rear_opening_h),
            "wall": float(p.asa_wall_mm),
            "effective_cavity_offset": float(cavity_offset),
            "coordinate_system": "X width, Y vertical height, Z front/back depth",
            "single_fused_shell": True,
            "rear_sliding_insertion": True,
            "internal_partition_walls_removed": True,
        },
        "features_mm": {
            "front_lens_opening": {
                "diameter": float(p.lens_cutout_d_mm),
                "center_x": float(p.lens_center_x_mm),
                "center_y": float(p.lens_center_y_mm),
                "face": "front Z- face",
                "source": "bottom STL front-face circular loop",
            },
            "front_source_slots": {
                "width": float(p.front_source_slot_w_mm),
                "upper_height": float(p.front_upper_source_slot_h_mm),
                "upper_center_y": float(p.front_upper_source_slot_center_y_mm),
                "lower_height": float(p.front_lower_source_slot_h_mm),
                "lower_center_y": float(p.front_lower_source_slot_center_y_mm),
                "corner_r": float(p.front_source_slot_corner_r_mm),
                "source": "bottom STL front-face slot loops",
            },
            "front_vertical_vent": {
                "width": float(p.front_vent_width_mm),
                "height": float(vent_h),
                "width_source": "0.25 inch",
                "centered": True,
                "intersects_lens_opening": True,
                "face": "front Z- lower body face",
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
