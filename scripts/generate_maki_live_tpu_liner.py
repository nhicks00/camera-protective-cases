#!/usr/bin/env python3
"""Generate a one-piece TPU sleeve for BirdDog MAKI Live to nest inside the ASA sleeve.

Outputs:
- maki_live_tpu_sleeve.step
- reports/maki_live_tpu_sleeve_report.json
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from build123d import (
    Align,
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    GeomType,
    Locations,
    Mode,
    Plane,
    Rectangle,
    export_step,
    export_stl,
    extrude,
    fillet,
    import_step,
    vertices,
)
from scipy.spatial import ConvexHull
from shapely import affinity
from shapely.geometry import Point, Polygon


@dataclass
class MakiTpuLinerParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # Device nominal envelope from drawing (mm)
    nominal_width_mm: float = 56.99
    nominal_height_mm: float = 56.99
    nominal_length_mm: float = 120.32

    # Fit against camera body (snug)
    device_clearance_mm: float = 0.2
    end_clearance_mm: float = 0.2
    asa_shell_clearance_mm: float = 2.3
    asa_cap_plug_depth_mm: float = 1.8

    # TPU shell geometry
    shell_thickness_mm: float = 2.0
    edge_wrap_depth_mm: float = 2.5
    edge_wrap_radial_mm: float = 2.0

    # Keep side vent/tripod regions open through TPU
    use_step_side_features: bool = True
    side_feature_clearance_mm: float = 0.35

    # Fallback openings if extraction fails
    vent_count: int = 8
    vent_slot_w_mm: float = 15.5
    vent_slot_h_mm: float = 2.6
    vent_pitch_mm: float = 5.8
    vent_start_from_front_mm: float = 66.0
    vent_rows_per_panel: int = 8
    enforce_tripanel_vent_layout: bool = True
    tripanel_fallback_x_offset_mm: float = 16.0
    vent_cut_depth_mm: float = 8.0
    tripod_center_from_front_mm: float = 48.0
    tripod_hole_diameter_mm: float = 10.2

    # Processing
    section_z_ratio: float = 0.50
    profile_simplify_tol_mm: float = 0.08
    offset_resolution: int = 64


def _largest_polygon(polys: Iterable[Polygon]) -> Polygon:
    polys = list(polys)
    if not polys:
        raise ValueError("No polygons produced by section")
    return max(polys, key=lambda p: abs(p.area))


def _to_single_poly(geom) -> Polygon:
    if geom.geom_type == "Polygon":
        return geom
    if geom.geom_type == "MultiPolygon":
        return max(list(geom.geoms), key=lambda p: p.area)
    raise ValueError(f"Unexpected geometry: {geom.geom_type}")


def _estimate_corner_radius(profile: Polygon) -> float:
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


def _load_step_as_mesh(step_path: Path, tmp_stl: Path):
    shape = import_step(str(step_path))
    housing = max(shape.solids(), key=lambda s: s.volume)
    step_features = _extract_step_side_features(housing)

    tmp_stl.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(tmp_stl))
    mesh = trimesh.load(str(tmp_stl), force="mesh")
    try:
        tmp_stl.unlink(missing_ok=True)
    except Exception:
        pass
    return mesh, step_features


def _extract_step_side_features(housing):
    vents = []
    seen = set()

    for f in housing.faces():
        wires = f.wires()
        if len(wires) <= 1:
            continue
        try:
            n = f.normal_at()
        except Exception:
            n = None
        if n is None:
            continue
        if abs(n.Z) > 0.70:
            continue

        c = f.center()
        for w in wires[1:]:
            bb = w.bounding_box()
            sx = float(bb.size.X)
            sy = float(bb.size.Y)
            sz = float(bb.size.Z)
            dims = sorted([sx, sy, sz], reverse=True)
            d_long, d_mid, d_small = dims
            if not (3.0 <= d_long <= 20.0 and 0.8 <= d_mid <= 3.5 and d_small <= 1.4):
                continue
            if d_long / max(d_mid, 1e-6) < 2.0:
                continue

            if abs(n.Y) >= abs(n.X):
                axis = "y"
                side = "neg" if c.Y < 0 else "pos"
                t_span = sx
                z_span = sz
            else:
                axis = "x"
                side = "neg" if c.X < 0 else "pos"
                t_span = sy
                z_span = sz

            x_mid = (bb.min.X + bb.max.X) * 0.5
            y_mid = (bb.min.Y + bb.max.Y) * 0.5
            z_mid = (bb.min.Z + bb.max.Z) * 0.5
            key = (
                axis,
                side,
                round(x_mid, 2),
                round(y_mid, 2),
                round(z_mid, 2),
                round(t_span, 2),
                round(z_span, 2),
            )
            if key in seen:
                continue
            seen.add(key)
            vents.append(
                {
                    "axis": axis,
                    "side": side,
                    "x": x_mid,
                    "y": y_mid,
                    "z": z_mid,
                    "slot_t": t_span,
                    "slot_z": z_span,
                }
            )

    tripod_candidates = []
    for e in housing.edges():
        if e.geom_type != GeomType.CIRCLE:
            continue
        c = e.center()
        r = e.radius
        bb = e.bounding_box()
        if not (2.5 <= r <= 8.0):
            continue
        if not (-90.0 <= c.Z <= -20.0):
            continue
        if abs(c.X) > 2.0:
            continue
        side = "neg" if c.Y < 0 else "pos"
        circular_in_xz = abs(bb.size.X - bb.size.Z) < 0.25 and bb.size.Y < 0.25
        if not circular_in_xz:
            continue
        tripod_candidates.append({"side": side, "x": c.X, "z": c.Z, "r": r})

    tripod = None
    if tripod_candidates:
        neg = [c for c in tripod_candidates if c["side"] == "neg"]
        pool = neg if neg else tripod_candidates
        tripod = max(pool, key=lambda c: c["r"])

    return {"vents": vents, "tripod": tripod}


def _collapse_close(values: list[float], tol: float) -> list[float]:
    if not values:
        return []
    values = sorted(values)
    out = [values[0]]
    for v in values[1:]:
        if abs(v - out[-1]) > tol:
            out.append(v)
        else:
            out[-1] = 0.5 * (out[-1] + v)
    return out


def _derive_tripanel_vents(step_vents, map_x, map_z, sx: float, sz: float, outer_w: float, p: MakiTpuLinerParams):
    """Derive a 3-panel x/z vent pattern (8 rows each) from STEP side vents."""
    neg_side = []
    for v in step_vents:
        if v["axis"] != "y" or v["side"] != "neg":
            continue
        t_span = v["slot_t"] * sx
        z_span = v["slot_z"] * sz
        slot_w = max(t_span, z_span) + p.side_feature_clearance_mm
        slot_h = max(min(t_span, z_span) + p.side_feature_clearance_mm, 0.8)
        neg_side.append(
            {
                "x": float(map_x(v["x"])),
                "z": float(map_z(v["z"])),
                "slot_w": float(slot_w),
                "slot_h": float(slot_h),
            }
        )

    if not neg_side:
        z_centers = [
            p.end_clearance_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm
            for i in range(p.vent_rows_per_panel)
        ]
        x_off = p.tripanel_fallback_x_offset_mm
        return {
            "x_centers": [-x_off, 0.0, x_off],
            "z_centers": z_centers,
            "slot_w": p.vent_slot_w_mm,
            "slot_h": p.vent_slot_h_mm,
            "source": "fallback",
        }

    slot_w = float(np.median([v["slot_w"] for v in neg_side]))
    slot_h = float(np.median([v["slot_h"] for v in neg_side]))

    z_vals = _collapse_close([v["z"] for v in neg_side], tol=0.9)
    if len(z_vals) >= p.vent_rows_per_panel:
        z_centers = z_vals[: p.vent_rows_per_panel]
    else:
        if len(z_vals) >= 2:
            pitch = float(np.median(np.diff(sorted(z_vals))))
        else:
            pitch = p.vent_pitch_mm
        z0 = z_vals[0] if z_vals else (p.end_clearance_mm + p.vent_start_from_front_mm)
        z_centers = [z0 + i * pitch for i in range(p.vent_rows_per_panel)]

    x_vals = _collapse_close([v["x"] for v in neg_side], tol=2.0)
    if len(x_vals) >= 3:
        mid = min(x_vals, key=lambda x: abs(x))
        left = min([x for x in x_vals if x < mid], default=mid - p.tripanel_fallback_x_offset_mm)
        right = max([x for x in x_vals if x > mid], default=mid + p.tripanel_fallback_x_offset_mm)
        x_centers = [left, mid, right]
    elif len(x_vals) == 2:
        mid = 0.5 * (x_vals[0] + x_vals[1])
        x_off = max(abs(x_vals[0] - mid), p.tripanel_fallback_x_offset_mm * 0.7)
        x_centers = [mid - x_off, mid, mid + x_off]
    else:
        x_off = max(0.23 * outer_w, p.tripanel_fallback_x_offset_mm)
        x_centers = [x_vals[0] - x_off, x_vals[0], x_vals[0] + x_off]

    return {
        "x_centers": [float(x) for x in x_centers],
        "z_centers": [float(z) for z in z_centers],
        "slot_w": slot_w,
        "slot_h": slot_h,
        "source": "step_neg_y",
    }


def _extract_profile_xy(mesh: trimesh.Trimesh, z_mm: float) -> Polygon:
    sec = mesh.section(plane_origin=[0.0, 0.0, z_mm], plane_normal=[0.0, 0.0, 1.0])
    if sec is None:
        raise ValueError(f"No section at z={z_mm}")

    p2d, to_3d = sec.to_2D()
    poly = _largest_polygon(p2d.polygons_full)

    ext2 = np.array(poly.exterior.coords[:-1])
    ext3 = trimesh.transformations.transform_points(
        np.column_stack([ext2, np.zeros(len(ext2))]), to_3d
    )
    xy = ext3[:, :2]

    hull = ConvexHull(xy)
    hull_xy = xy[hull.vertices]

    cx = float((hull_xy[:, 0].min() + hull_xy[:, 0].max()) * 0.5)
    cy = float((hull_xy[:, 1].min() + hull_xy[:, 1].max()) * 0.5)
    hull_xy -= np.array([cx, cy])

    base = Polygon(hull_xy)
    return _to_single_poly(base.simplify(0.05, preserve_topology=True))


def build_liner(p: MakiTpuLinerParams):
    tmp_stl = Path("tmp/maki_device_from_step_tpu_ref.stl")
    mesh, step_features = _load_step_as_mesh(p.step_path, tmp_stl)

    zmin, zmax = mesh.bounds[:, 2]
    z_section = zmin + (zmax - zmin) * p.section_z_ratio
    base_profile = _extract_profile_xy(mesh, z_section)

    raw_min_x, raw_min_y, raw_max_x, raw_max_y = base_profile.bounds
    raw_w = raw_max_x - raw_min_x
    raw_h = raw_max_y - raw_min_y

    sx = p.nominal_width_mm / raw_w
    sy = p.nominal_height_mm / raw_h
    base_profile = affinity.scale(base_profile, xfact=sx, yfact=sy, origin=(0.0, 0.0))
    sz = p.nominal_length_mm / (zmax - zmin)

    base_corner_r = _estimate_corner_radius(base_profile)
    inner_w = p.nominal_width_mm + 2.0 * p.device_clearance_mm
    inner_h = p.nominal_height_mm + 2.0 * p.device_clearance_mm
    outer_w = inner_w + 2.0 * p.shell_thickness_mm
    outer_h = inner_h + 2.0 * p.shell_thickness_mm

    inner_corner_r = min(base_corner_r + p.device_clearance_mm, 0.5 * min(inner_w, inner_h) - 0.2)
    outer_corner_r = min(inner_corner_r + p.shell_thickness_mm, 0.5 * min(outer_w, outer_h) - 0.2)
    inner_depth = p.nominal_length_mm + 2.0 * p.end_clearance_mm
    shell_depth = inner_depth
    edge_wrap_depth = max(min(p.edge_wrap_depth_mm, 0.35 * shell_depth), 0.6)
    edge_wrap_radial = max(p.edge_wrap_radial_mm, 0.6)
    wrap_inner_w = max(inner_w - 2.0 * edge_wrap_radial, 2.0)
    wrap_inner_h = max(inner_h - 2.0 * edge_wrap_radial, wrap_inner_w + 0.2)

    min_y = -0.5 * outer_h
    max_y = 0.5 * outer_h

    def map_x(x_dev: float) -> float:
        return float(x_dev * sx)

    def map_y(y_dev: float) -> float:
        return float(y_dev * sy)

    def map_z(z_dev: float) -> float:
        # Match ASA sleeve datum with end clearance offset from front edge.
        return float((zmax - z_dev) * sz + p.end_clearance_mm)

    vents_used = []
    tripod_used = None

    with BuildPart() as liner_bp:
        # Base open-through tube (single TPU sleeve)
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(outer_w, outer_h, outer_corner_r)
        extrude(amount=shell_depth)

        with BuildSketch(Plane.XY.offset(-0.2)):
            _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
        extrude(amount=shell_depth + 0.4, mode=Mode.SUBTRACT)

        # Thin front edge-wrap (perimeter only, no full-face cap).
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
        extrude(amount=edge_wrap_depth)
        with BuildSketch(Plane.XY.offset(-0.2)):
            _add_rounded_rectangle(
                wrap_inner_w,
                wrap_inner_h,
                max(inner_corner_r - edge_wrap_radial, 0.6),
            )
        extrude(amount=edge_wrap_depth + 0.4, mode=Mode.SUBTRACT)

        # Thin rear edge-wrap.
        rear_start = shell_depth - edge_wrap_depth
        with BuildSketch(Plane.XY.offset(rear_start)):
            _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
        extrude(amount=edge_wrap_depth)
        with BuildSketch(Plane.XY.offset(rear_start - 0.2)):
            _add_rounded_rectangle(
                wrap_inner_w,
                wrap_inner_h,
                max(inner_corner_r - edge_wrap_radial, 0.6),
            )
        extrude(amount=edge_wrap_depth + 0.4, mode=Mode.SUBTRACT)

        if p.use_step_side_features:
            if p.enforce_tripanel_vent_layout:
                vent_pattern = _derive_tripanel_vents(step_features["vents"], map_x, map_z, sx, sz, outer_w, p)
                y_face = min_y - 0.2
                cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                for panel_idx, x_c in enumerate(vent_pattern["x_centers"]):
                    for z_c in vent_pattern["z_centers"]:
                        with Locations((x_c, y_face, z_c)):
                            Box(
                                vent_pattern["slot_h"],
                                cut_depth,
                                vent_pattern["slot_w"],
                                align=(Align.CENTER, Align.MIN, Align.CENTER),
                                mode=Mode.SUBTRACT,
                            )
                        vents_used.append(
                            {
                                "axis": "y",
                                "side": "neg",
                                "x": float(x_c),
                                "y": float(y_face),
                                "z": float(z_c),
                                "slot_w": float(vent_pattern["slot_w"]),
                                "slot_h": float(vent_pattern["slot_h"]),
                                "panel_index": panel_idx,
                                "pattern_source": vent_pattern["source"],
                            }
                        )
            else:
                for v in step_features["vents"]:
                    z_c = map_z(v["z"])
                    if v["axis"] == "y":
                        x_c = map_x(v["x"])
                        t_span = v["slot_t"] * sx
                        z_span = v["slot_z"] * sz
                        slot_w = max(max(t_span, z_span) + p.side_feature_clearance_mm, 0.8)
                        slot_h = max(min(t_span, z_span) + p.side_feature_clearance_mm, 0.8)
                        on_neg = v["side"] == "neg"
                        y_face = min_y - 0.2 if on_neg else max_y + 0.2
                        cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                        with Locations((x_c, y_face, z_c)):
                            Box(
                                slot_h,
                                cut_depth,
                                slot_w,
                                align=(Align.CENTER, Align.MIN, Align.CENTER)
                                if on_neg
                                else (Align.CENTER, Align.MAX, Align.CENTER),
                                mode=Mode.SUBTRACT,
                            )
                    else:
                        y_c = map_y(v["y"])
                        t_span = v["slot_t"] * sy
                        z_span = v["slot_z"] * sz
                        slot_w = max(max(t_span, z_span) + p.side_feature_clearance_mm, 0.8)
                        slot_h = max(min(t_span, z_span) + p.side_feature_clearance_mm, 0.8)
                        on_neg = v["side"] == "neg"
                        x_face = min_x - 0.2 if on_neg else max_x + 0.2
                        cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                        with Locations((x_face, y_c, z_c)):
                            Box(
                                cut_depth,
                                slot_h,
                                slot_w,
                                align=(Align.MIN, Align.CENTER, Align.CENTER)
                                if on_neg
                                else (Align.MAX, Align.CENTER, Align.CENTER),
                                mode=Mode.SUBTRACT,
                            )
                    vents_used.append(
                        {
                            "axis": v["axis"],
                            "side": v["side"],
                            "x": map_x(v["x"]),
                            "y": map_y(v["y"]),
                            "z": z_c,
                            "slot_w": slot_w,
                            "slot_h": slot_h,
                        }
                    )

            t = step_features["tripod"]
            if t is not None:
                x_c = map_x(t["x"])
                z_c = map_z(t["z"])
                d = max(2.0 * t["r"] * sx + p.side_feature_clearance_mm, 2.0)
                on_neg = t["side"] == "neg"
                y_face = min_y - 0.2 if on_neg else max_y + 0.2
                cut_depth = p.shell_thickness_mm + 2.5
                if on_neg:
                    cut_plane = Plane.ZX.offset(y_face)
                    cut_loc = (z_c, x_c)
                else:
                    cut_plane = Plane.XZ.offset(y_face)
                    cut_loc = (x_c, z_c)
                with BuildSketch(cut_plane):
                    with Locations(cut_loc):
                        Circle(d * 0.5)
                extrude(amount=cut_depth, mode=Mode.SUBTRACT)
                tripod_used = {"side": t["side"], "x": x_c, "z": z_c, "diameter": d}

        if not vents_used:
            y_face = min_y - 0.2
            cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
            for i in range(p.vent_count):
                z = p.end_clearance_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm
                with Locations((0.0, y_face, z)):
                    Box(
                        p.vent_slot_h_mm,
                        cut_depth,
                        p.vent_slot_w_mm,
                        align=(Align.CENTER, Align.MIN, Align.CENTER),
                        mode=Mode.SUBTRACT,
                    )

        if tripod_used is None:
            with BuildSketch(Plane.ZX.offset(min_y - 0.2)):
                with Locations((p.end_clearance_mm + p.tripod_center_from_front_mm, 0.0)):
                    Circle(p.tripod_hole_diameter_mm * 0.5)
            extrude(amount=(p.shell_thickness_mm + 2.5), mode=Mode.SUBTRACT)
            tripod_used = {
                "side": "neg",
                "x": 0.0,
                "z": p.end_clearance_mm + p.tripod_center_from_front_mm,
                "diameter": p.tripod_hole_diameter_mm,
            }

    liner = _largest_solid(liner_bp.part)
    liner, liner_fillet_y = _apply_axis_fillet(liner, Axis.Y, (1.0, 0.8, 0.6, 0.45, 0.3))

    report = {
        "mesh_bounds_mm": {
            "min": [float(v) for v in mesh.bounds[0]],
            "max": [float(v) for v in mesh.bounds[1]],
            "extents": [float(v) for v in mesh.extents],
        },
        "profile_raw_mm": {"width": float(raw_w), "height": float(raw_h)},
        "profile_scale": {"sx": float(sx), "sy": float(sy), "sz": float(sz)},
        "step_side_features": {
            "vents_detected": len(step_features["vents"]),
            "vents_applied": len(vents_used),
            "vents_applied_entries": vents_used,
            "tripod_detected": step_features["tripod"] is not None,
            "tripod_applied": tripod_used,
        },
        "derived": {
            "open_sleeve": True,
            "inner_depth_mm": float(inner_depth),
            "shell_depth_mm": float(shell_depth),
            "nominal_asa_fit_mm": {
                "asa_clearance_each_side": float(p.asa_shell_clearance_mm),
                "expected_radial_gap_each_side": float(
                    p.asa_shell_clearance_mm - p.device_clearance_mm - p.shell_thickness_mm
                ),
                "expected_axial_gap_each_end": float(p.asa_shell_clearance_mm - p.end_clearance_mm),
                "expected_cap_budget_each_end": float(
                    p.asa_shell_clearance_mm - p.end_clearance_mm - p.asa_cap_plug_depth_mm
                ),
                "expected_cap_budget_total_two_caps": float(
                    2.0 * (p.asa_shell_clearance_mm - p.end_clearance_mm - p.asa_cap_plug_depth_mm)
                ),
            },
            "edge_wrap_depth_mm": float(edge_wrap_depth),
            "edge_wrap_radial_mm": float(edge_wrap_radial),
            "corner_radius_mm": {
                "base_section": float(base_corner_r),
                "inner": float(inner_corner_r),
                "outer": float(outer_corner_r),
            },
            "dimensions_mm": {
                "inner_w": float(inner_w),
                "inner_h": float(inner_h),
                "outer_w": float(outer_w),
                "outer_h": float(outer_h),
                "edge_wrap_inner_opening_w": float(wrap_inner_w),
                "edge_wrap_inner_opening_h": float(wrap_inner_h),
            },
            "machined_finish_mm": {
                "liner_axis_y_fillet": liner_fillet_y,
            },
        },
    }

    return liner, report


def main():
    parser = argparse.ArgumentParser(description="Generate MAKI Live TPU sleeve")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--clearance", type=float, default=None, help="Inner clearance to camera body (mm)")
    parser.add_argument("--thickness", type=float, default=None, help="Base TPU shell thickness (mm)")
    parser.add_argument("--edge-wrap-depth", type=float, default=None, help="Front/rear edge wrap depth (mm)")
    parser.add_argument("--edge-wrap-radial", type=float, default=None, help="Edge wrap radial hold on face perimeter (mm)")
    parser.add_argument("--end-clearance", type=float, default=None, help="Front/rear clearance to camera (mm each end)")
    parser.add_argument("--no-step-side-features", action="store_true", help="Disable STEP-derived side vents/tripod")
    args = parser.parse_args()

    params = MakiTpuLinerParams(step_path=args.step)
    if args.clearance is not None:
        params.device_clearance_mm = args.clearance
    if args.thickness is not None:
        params.shell_thickness_mm = args.thickness
    if args.edge_wrap_depth is not None:
        params.edge_wrap_depth_mm = args.edge_wrap_depth
    if args.edge_wrap_radial is not None:
        params.edge_wrap_radial_mm = args.edge_wrap_radial
    if args.end_clearance is not None:
        params.end_clearance_mm = args.end_clearance
    if args.no_step_side_features:
        params.use_step_side_features = False

    liner, report = build_liner(params)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    out_step = args.out / "maki_live_tpu_sleeve.step"
    out_json = reports_dir / "maki_live_tpu_sleeve_report.json"
    archived = _archive_existing(
        [
            out_step,
            out_json,
            args.out / "maki_live_tpu_sleeve_report.json",  # legacy top-level location
            args.out / "maki_live_tpu_unibody.step",
            args.out / "maki_live_tpu_unibody_report.json",
            args.out / "maki_live_tpu_liner.step",
            args.out / "maki_live_tpu_liner_report.json",
            args.out / "maki_live_tpu_front_cap.step",
            args.out / "maki_live_tpu_rear_cap.step",
            args.out / "maki_live_tpu_caps_report.json",
        ],
        args.out,
    )

    export_step(liner, str(out_step))

    payload = {"params": asdict(params), "report": report}
    payload["params"]["step_path"] = str(params.step_path)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
