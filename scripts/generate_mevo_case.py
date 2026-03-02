#!/usr/bin/env python3
"""Generate a parametric two-piece protective case for Mevo Start.

Outputs:
- mevo_start_case_body.step: main impact sleeve (open rear)
- mevo_start_case_back_plate.step: friction-fit rear plate with I/O cutouts

The front profile and lens location are extracted from refs/Mevo_Start_lens_cover_corrected.stl.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np
import trimesh
from build123d import (
    Align,
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Mode,
    Plane,
    Rectangle,
    SlotOverall,
    export_step,
    extrude,
    fillet,
    vertices,
)
from scipy.spatial import ConvexHull
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon


@dataclass
class CaseParams:
    # Inputs
    reference_stl: Path = Path("refs/Mevo_Start_lens_cover_corrected.stl")
    profile_slice_y_mm: float = 5.0
    lens_slice_y_mm: float = 0.1

    # Fit and structure
    # User-provided Mevo Start envelope dimensions.
    # Mapping assumption:
    # - length_mm: sleeve depth (front-to-back extrusion)
    # - height_mm: major axis of front profile
    # - width_mm: minor axis of front profile
    device_length_mm: float = 87.0
    device_height_mm: float = 75.5
    device_width_mm: float = 34.0
    enforce_nominal_dimensions: bool = True
    strict_symmetric_profile: bool = True
    clearance_mm: float = 0.65
    wall_mm: float = 3.4
    front_wall_mm: float = 4.0
    use_inner_pill_profile: bool = True
    enforce_capsule_profile: bool = True

    # Lens opening and front bezel
    use_reference_lens_hole: bool = False
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 16.5
    lens_cut_radius_mm: float = 19.4
    lens_extra_clearance_mm: float = 0.9
    front_bezel_extra_mm: float = 1.2
    front_bezel_height_mm: float = 1.1

    # Tripod mount protection zone
    tripod_slot_w_mm: float = 14.0
    tripod_slot_l_mm: float = 12.0
    tripod_pad_w_mm: float = 28.0
    tripod_pad_l_mm: float = 18.0
    tripod_pad_h_mm: float = 2.2
    tripod_zone_z_ratio: float = 0.78

    # Rear plate (friction fit)
    rear_plate_t_mm: float = 3.0
    rear_plug_depth_mm: float = 4.6
    rear_plug_clearance_mm: float = 0.25

    # Rear I/O cutouts (centered on X axis, tunable)
    power_y_mm: float = 15.0
    power_w_mm: float = 9.0
    power_h_mm: float = 4.0

    audio_y_mm: float = 6.8
    audio_d_mm: float = 6.7

    usb_y_mm: float = -2.8
    usb_w_mm: float = 12.8
    usb_h_mm: float = 6.4

    # Venting
    vent_count: int = 3
    vent_h_mm: float = 2.2
    vent_len_mm: float = 12.0
    vent_pitch_mm: float = 4.8
    vent_z_ratio: float = 0.57

    # Polygon processing
    profile_simplify_tol_mm: float = 0.15
    offset_resolution: int = 48


def _largest_polygon(polys: Iterable[Polygon]) -> Polygon:
    polys = list(polys)
    if not polys:
        raise ValueError("No polygons found in section")
    return max(polys, key=lambda p: abs(p.area))


def _fit_circle(points: np.ndarray) -> Tuple[float, float, float]:
    """Least-squares circle fit. Returns (xc, yc, r)."""
    a = np.column_stack([2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points))])
    b = points[:, 0] ** 2 + points[:, 1] ** 2
    xc, yc, c = np.linalg.lstsq(a, b, rcond=None)[0]
    r = float(np.sqrt(c + xc**2 + yc**2))
    return float(xc), float(yc), r


def _to_single_poly(geom) -> Polygon:
    if geom.geom_type == "Polygon":
        return geom
    if geom.geom_type == "MultiPolygon":
        return max(list(geom.geoms), key=lambda p: p.area)
    raise ValueError(f"Unexpected geometry type: {geom.geom_type}")


def _estimate_corner_radius(profile: Polygon) -> float:
    """Estimate rounded-rectangle corner radius from a polygon profile."""
    min_x, min_y, max_x, max_y = profile.bounds
    w = max_x - min_x
    h = max_y - min_y
    corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]
    r_from_corners = []
    for c in corners:
        d = Point(c).distance(profile.exterior)
        r_from_corners.append(d / (math.sqrt(2.0) - 1.0))

    area_term = max((w * h - profile.area) / (4.0 - math.pi), 0.0)
    r_area = math.sqrt(area_term)
    r_raw = float(np.median(r_from_corners + [r_area]))
    r_max = 0.5 * min(w, h) - 0.2
    return max(0.6, min(r_raw, r_max))


def _safe_fillet_radius(width: float, height: float, radius: float) -> float:
    return max(0.3, min(float(radius), 0.5 * min(width, height) - 0.05))


def _add_rounded_rectangle(width: float, height: float, radius: float) -> None:
    Rectangle(width, height)
    fillet(vertices(), _safe_fillet_radius(width, height, radius))


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


def _make_capsule_profile(minor_mm: float, major_mm: float, resolution: int) -> Polygon:
    """Create a perfectly symmetric vertical capsule profile centered at origin."""
    r = minor_mm * 0.5
    straight = max(major_mm - 2.0 * r, 0.0)
    return _to_single_poly(
        LineString([(0.0, -straight * 0.5), (0.0, straight * 0.5)]).buffer(
            r, cap_style=1, join_style=1, resolution=resolution
        )
    )


def _make_profile(reference_stl: Path, p: CaseParams):
    mesh = trimesh.load(reference_stl, force="mesh")
    # Auto-detect cap thickness axis and slice through its mid-plane.
    # This makes extraction robust across different STL orientations.
    thickness_axis = int(np.argmin(mesh.extents))
    mids = (mesh.bounds[0] + mesh.bounds[1]) * 0.5
    plane_origin = np.zeros(3)
    plane_normal = np.zeros(3)
    plane_origin[thickness_axis] = mids[thickness_axis]
    plane_normal[thickness_axis] = 1.0

    sec = mesh.section(plane_origin=plane_origin, plane_normal=plane_normal)
    if sec is None:
        raise ValueError("Could not extract mid-plane section from reference STL")

    p2d, to_3d = sec.to_2D()
    poly = _largest_polygon(p2d.polygons_full)
    profile_axes = [i for i in range(3) if i != thickness_axis]

    def to_profile_2d(coords_2d: np.ndarray) -> np.ndarray:
        pts3 = trimesh.transformations.transform_points(
            np.column_stack([coords_2d, np.zeros(len(coords_2d))]), to_3d
        )
        return pts3[:, profile_axes]

    exterior = to_profile_2d(np.array(poly.exterior.coords[:-1]))
    holes = [to_profile_2d(np.array(r.coords[:-1])) for r in poly.interiors]

    # Pick profile source:
    # - preferred: largest interior "pill" loop from the cap
    # - fallback: outer loop convex hull
    profile_source = exterior
    if p.use_inner_pill_profile and holes:
        profile_source = max(holes, key=lambda loop: np.prod(loop.max(axis=0) - loop.min(axis=0)))
    elif not holes:
        hull = ConvexHull(exterior)
        profile_source = exterior[hull.vertices]

    # Center profile for stable CAD operations.
    cx = float((profile_source[:, 0].min() + profile_source[:, 0].max()) * 0.5)
    cy = float((profile_source[:, 1].min() + profile_source[:, 1].max()) * 0.5)
    prof_centered = profile_source - np.array([cx, cy])

    if p.enforce_capsule_profile:
        span = prof_centered.max(axis=0) - prof_centered.min(axis=0)
        if span[1] >= span[0]:
            minor = span[0]
            major = span[1]
            base_poly = _make_capsule_profile(minor, major, p.offset_resolution)
        else:
            minor = span[1]
            major = span[0]
            base_poly = affinity.rotate(
                _make_capsule_profile(minor, major, p.offset_resolution), 90.0, origin=(0.0, 0.0)
            )
    else:
        base_poly = Polygon(prof_centered)
        base_poly = _to_single_poly(base_poly.simplify(p.profile_simplify_tol_mm, preserve_topology=True))

    # Lens loop extraction is used only when --use-reference-lens-hole is enabled.
    # On corrected caps, the large inner pill should not be treated as lens geometry.
    def circular_score(loop: np.ndarray) -> tuple[float, float]:
        ext = loop.max(axis=0) - loop.min(axis=0)
        ratio_err = abs((ext[0] / ext[1]) - 1.0)
        area_proxy = ext[0] * ext[1]
        return ratio_err, -area_proxy

    lens_cx = 0.0
    lens_cy = 0.0
    lens_r = min(base_poly.bounds[2] - base_poly.bounds[0], base_poly.bounds[3] - base_poly.bounds[1]) * 0.35
    lens_found = False

    for loop in sorted(holes, key=circular_score):
        ext = loop.max(axis=0) - loop.min(axis=0)
        ratio_err = abs((ext[0] / ext[1]) - 1.0)
        if ratio_err > 0.20:
            continue
        lc_x, lc_y, lr = _fit_circle(loop)
        lens_cx, lens_cy, lens_r = lc_x, lc_y, lr
        lens_found = True
        break

    if lens_found:
        lens_cx -= cx
        lens_cy -= cy
    return base_poly, (lens_cx, lens_cy, lens_r)


def build_case(p: CaseParams):
    base_poly, (lens_x_ref, lens_y_ref, lens_r_ref) = _make_profile(p.reference_stl, p)

    raw_min_x, raw_min_y, raw_max_x, raw_max_y = base_poly.bounds
    raw_profile_w = raw_max_x - raw_min_x
    raw_profile_h = raw_max_y - raw_min_y
    rotated_profile = False

    # Normalize orientation so X is the minor axis and Y is the major axis.
    if raw_profile_w > raw_profile_h:
        base_poly = affinity.rotate(base_poly, 90.0, origin=(0.0, 0.0))
        lens_x_ref, lens_y_ref = -lens_y_ref, lens_x_ref
        raw_min_x, raw_min_y, raw_max_x, raw_max_y = base_poly.bounds
        raw_profile_w = raw_max_x - raw_min_x
        raw_profile_h = raw_max_y - raw_min_y
        rotated_profile = True

    scale_x = 1.0
    scale_y = 1.0

    if p.enforce_nominal_dimensions:
        scale_x = p.device_width_mm / raw_profile_w
        scale_y = p.device_height_mm / raw_profile_h
        base_poly = affinity.scale(base_poly, xfact=scale_x, yfact=scale_y, origin=(0.0, 0.0))
        lens_x_ref *= scale_x
        lens_y_ref *= scale_y
        # Keep lens opening circular; scale by mean profile scale factor.
        lens_r_ref *= (scale_x + scale_y) * 0.5

    if p.strict_symmetric_profile:
        target_minor = p.device_width_mm if p.enforce_nominal_dimensions else (base_poly.bounds[2] - base_poly.bounds[0])
        target_major = p.device_height_mm if p.enforce_nominal_dimensions else (base_poly.bounds[3] - base_poly.bounds[1])
        base_poly = _make_capsule_profile(target_minor, target_major, p.offset_resolution)

    inner_poly = _to_single_poly(
        base_poly.buffer(p.clearance_mm, join_style=1, resolution=p.offset_resolution)
    )
    outer_poly = _to_single_poly(
        inner_poly.buffer(p.wall_mm, join_style=1, resolution=p.offset_resolution)
    )
    plug_poly = _to_single_poly(
        inner_poly.buffer(-p.rear_plug_clearance_mm, join_style=1, resolution=p.offset_resolution)
    )

    inner_depth = p.device_length_mm + 2.0 * p.clearance_mm
    shell_depth = p.front_wall_mm + inner_depth

    out_min_x, out_min_y, out_max_x, out_max_y = outer_poly.bounds
    in_min_x, in_min_y, in_max_x, in_max_y = inner_poly.bounds
    plug_min_x, plug_min_y, plug_max_x, plug_max_y = plug_poly.bounds

    outer_w = out_max_x - out_min_x
    outer_h = out_max_y - out_min_y
    inner_w = in_max_x - in_min_x
    inner_h = in_max_y - in_min_y
    plug_w = plug_max_x - plug_min_x
    plug_h = plug_max_y - plug_min_y

    outer_corner_r = _estimate_corner_radius(outer_poly)
    inner_corner_r = _estimate_corner_radius(inner_poly)
    plug_corner_r = _estimate_corner_radius(plug_poly)

    # Lens opening + raised bezel.
    if p.use_reference_lens_hole:
        lens_x, lens_y = lens_x_ref, lens_y_ref
        lens_r = lens_r_ref + p.lens_extra_clearance_mm
    else:
        lens_x, lens_y = p.lens_center_x_mm, p.lens_center_y_mm
        lens_r = p.lens_cut_radius_mm

    # Optional side vents near rear half.
    min_x = -0.5 * outer_w
    max_x = 0.5 * outer_w
    min_y = -0.5 * outer_h
    max_y = 0.5 * outer_h
    vent_z = p.front_wall_mm + inner_depth * p.vent_z_ratio
    with BuildPart() as body_bp:
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(outer_w, outer_h, outer_corner_r)
        extrude(amount=shell_depth)

        with BuildSketch(Plane.XY.offset(p.front_wall_mm)):
            _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
        extrude(amount=inner_depth + 0.2, mode=Mode.SUBTRACT)

        with BuildSketch(Plane.XY.offset(-1.0)):
            with Locations((lens_x, lens_y)):
                Circle(lens_r)
        extrude(amount=p.front_wall_mm + 2.0, mode=Mode.SUBTRACT)

        with BuildSketch(Plane.XY):
            with Locations((lens_x, lens_y)):
                Circle(lens_r + p.front_bezel_extra_mm)
                Circle(lens_r, mode=Mode.SUBTRACT)
        extrude(amount=p.front_bezel_height_mm)

        for side in (-1.0, 1.0):
            x = side * (max_x - p.wall_mm * 0.55)
            for i in range(p.vent_count):
                y = (i - (p.vent_count - 1) * 0.5) * p.vent_pitch_mm
                with Locations((x, y, vent_z)):
                    Box(p.wall_mm * 2.5, p.vent_h_mm, p.vent_len_mm, mode=Mode.SUBTRACT)

        # Bottom tripod opening on rear zone.
        tripod_z = shell_depth * p.tripod_zone_z_ratio
        slot_depth = p.wall_mm + 2.0
        with Locations((0.0, min_y + 0.2, tripod_z)):
            Box(
                p.tripod_slot_w_mm,
                slot_depth,
                p.tripod_slot_l_mm,
                align=(Align.CENTER, Align.MIN, Align.CENTER),
                mode=Mode.SUBTRACT,
            )

    body = _largest_solid(body_bp.part)
    body, body_fillet_x = _apply_axis_fillet(body, Axis.X, (0.8, 0.6, 0.45, 0.3, 0.2))
    body, body_fillet_y = _apply_axis_fillet(body, Axis.Y, (0.4, 0.3, 0.2, 0.15))

    with BuildPart() as rear_plate_bp:
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(outer_w, outer_h, outer_corner_r)
        extrude(amount=p.rear_plate_t_mm)

        with BuildSketch(Plane.XY.offset(p.rear_plate_t_mm)):
            _add_rounded_rectangle(plug_w, plug_h, plug_corner_r)
        extrude(amount=p.rear_plug_depth_mm)

        cut_depth = p.rear_plate_t_mm + p.rear_plug_depth_mm + 2.0
        with BuildSketch(Plane.XY.offset(-1.0)):
            with Locations((0.0, p.power_y_mm)):
                SlotOverall(p.power_w_mm, p.power_h_mm)
            with Locations((0.0, p.audio_y_mm)):
                Circle(p.audio_d_mm * 0.5)
            with Locations((0.0, p.usb_y_mm)):
                SlotOverall(p.usb_w_mm, p.usb_h_mm)
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

        # Bottom rear notch for tripod extension access through rear plate.
        notch_center_y = min_y + 5.2
        with Locations((0.0, notch_center_y, (cut_depth * 0.5) - 1.0)):
            Box(p.tripod_slot_w_mm + 2.0, 9.5, cut_depth, mode=Mode.SUBTRACT)

    rear_plate = _largest_solid(rear_plate_bp.part)
    rear_plate, rear_plate_fillet = _apply_all_edge_fillet(rear_plate, (0.8, 0.6, 0.45, 0.3, 0.2))
    rear_plate = _largest_solid(rear_plate)

    return body, rear_plate, {
        "raw_profile_mm": {
            "width": float(raw_profile_w),
            "height": float(raw_profile_h),
        },
        "orientation": {"rotated_90_deg_for_normalization": rotated_profile},
        "profile_scaling": {
            "scale_x": float(scale_x),
            "scale_y": float(scale_y),
        },
        "profile_bounds_mm": {
            "min_x": float(min_x),
            "max_x": float(max_x),
            "min_y": float(min_y),
            "max_y": float(max_y),
        },
        "derived": {
            "inner_depth_mm": inner_depth,
            "shell_depth_mm": shell_depth,
            "corner_radius_mm": {
                "inner": float(inner_corner_r),
                "outer": float(outer_corner_r),
                "rear_plug": float(plug_corner_r),
            },
            "machined_finish_mm": {
                "body_axis_x_fillet": body_fillet_x,
                "body_axis_y_fillet": body_fillet_y,
                "rear_plate_global_fillet": rear_plate_fillet,
            },
            "lens_reference_center_mm": [lens_x_ref, lens_y_ref],
            "lens_reference_radius_mm": lens_r_ref,
            "lens_mode": "reference" if p.use_reference_lens_hole else "manual",
            "lens_center_mm": [lens_x, lens_y],
            "lens_cut_radius_mm": lens_r,
        },
    }


def main():
    parser = argparse.ArgumentParser(description="Generate Mevo Start protective case")
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument(
        "--reference",
        type=Path,
        default=Path("refs/Mevo_Start_lens_cover_corrected.stl"),
        help="Front profile reference STL",
    )
    parser.add_argument("--clearance", type=float, default=None, help="Internal clearance (mm)")
    parser.add_argument("--wall", type=float, default=None, help="Main wall thickness (mm)")
    parser.add_argument("--device-depth", type=float, default=None, help="Legacy alias for camera length/depth (mm)")
    parser.add_argument("--length-mm", type=float, default=None, help="Camera length (mm)")
    parser.add_argument("--height-mm", type=float, default=None, help="Camera front-profile major axis (mm)")
    parser.add_argument("--width-mm", type=float, default=None, help="Camera front-profile minor axis (mm)")
    parser.add_argument(
        "--use-reference-lens-hole",
        action="store_true",
        help="Use lens cutout extracted from reference STL opening",
    )
    parser.add_argument("--lens-center-y", type=float, default=None, help="Manual lens center Y (mm)")
    parser.add_argument("--lens-radius", type=float, default=None, help="Manual lens cut radius (mm)")
    parser.add_argument("--power-y", type=float, default=None, help="Power slot center Y (mm)")
    parser.add_argument("--audio-y", type=float, default=None, help="Audio hole center Y (mm)")
    parser.add_argument("--usb-y", type=float, default=None, help="USB-C slot center Y (mm)")
    args = parser.parse_args()

    params = CaseParams(reference_stl=args.reference)
    if args.clearance is not None:
        params.clearance_mm = args.clearance
    if args.wall is not None:
        params.wall_mm = args.wall
    if args.length_mm is not None:
        params.device_length_mm = args.length_mm
    if args.height_mm is not None:
        params.device_height_mm = args.height_mm
    if args.width_mm is not None:
        params.device_width_mm = args.width_mm
    if args.device_depth is not None:
        params.device_length_mm = args.device_depth
    if args.use_reference_lens_hole:
        params.use_reference_lens_hole = True
    if args.lens_center_y is not None:
        params.lens_center_y_mm = args.lens_center_y
    if args.lens_radius is not None:
        params.lens_cut_radius_mm = args.lens_radius
    if args.power_y is not None:
        params.power_y_mm = args.power_y
    if args.audio_y is not None:
        params.audio_y_mm = args.audio_y
    if args.usb_y is not None:
        params.usb_y_mm = args.usb_y
    body, rear_plate, report = build_case(params)

    args.out.mkdir(parents=True, exist_ok=True)

    body_step = args.out / "mevo_start_case_body.step"
    plate_step = args.out / "mevo_start_case_back_plate.step"
    report_json = args.out / "mevo_start_case_report.json"
    archived = _archive_existing([body_step, plate_step, report_json], args.out)

    export_step(body, str(body_step))
    export_step(rear_plate, str(plate_step))

    report_payload = {"params": asdict(params), "extraction": report}
    report_payload["params"]["reference_stl"] = str(params.reference_stl)
    report_json.write_text(json.dumps(report_payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {body_step}")
    print(f"Wrote {plate_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
