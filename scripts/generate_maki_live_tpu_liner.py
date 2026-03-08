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
    SlotOverall,
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
    device_clearance_mm: float = 0.15
    end_clearance_mm: float = 0.2
    asa_shell_clearance_mm: float = 2.3
    asa_cap_plug_depth_mm: float = 1.8

    # TPU shell geometry
    shell_thickness_mm: float = 2.0
    edge_wrap_depth_mm: float = 2.5
    edge_wrap_radial_mm: float = 2.0
    include_front_edge_wrap: bool = False
    include_rear_edge_wrap: bool = False

    # Front face pad (shock absorption between ASA front wall and camera face)
    include_front_face_pad: bool = True
    front_face_pad_thickness_mm: float = 1.5

    # Keep side vent/tripod regions open through TPU
    use_step_side_features: bool = True
    side_feature_clearance_mm: float = 0.30
    tripod_cutout_extra_mm: float = 1.5

    # Fallback openings if extraction fails
    vent_count: int = 8
    vent_slot_w_mm: float = 15.5
    vent_slot_h_mm: float = 2.6
    vent_pitch_mm: float = 5.8
    vent_start_from_front_mm: float = 66.0
    vent_rows_per_panel: int = 8
    enforce_tripanel_vent_layout: bool = True
    tripanel_fallback_x_offset_mm: float = 16.0
    # Pull outer 24-grid columns inward toward center (center column stays fixed).
    tripanel_outer_column_inset_mm: float = 2.6
    # Z shift for vent arrays (0 = place at STEP-derived positions).
    tripanel_vent_z_shift_mm: float = 0.0
    vent_cut_depth_mm: float = 8.0
    include_side_trio_vents: bool = True
    side_trio_per_side: int = 3
    side_trio_z_threshold_mm: float = -60.0
    side_trio_select_rear: bool = True
    side_trio_flip_end: bool = False
    side_trio_vent_z_shift_mm: float = 0.0
    side_trio_scale_t: float = 1.6
    side_trio_scale_z: float = 1.5
    side_trio_min_t_mm: float = 6.4
    side_trio_min_z_mm: float = 2.2
    side_trio_match_tripanel_size: bool = True
    tripod_center_from_front_mm: float = 48.0
    tripod_hole_diameter_mm: float = 10.2
    tripod_thread_radius_mm: float = 3.175
    tripod_radius_tolerance_mm: float = 1.0
    tripod_face_normal_min_abs_z: float = 0.92
    tripod_centerline_x_max_mm: float = 6.0
    tripod_z_min_mm: float = -110.0
    tripod_z_max_mm: float = -20.0
    tripod_expected_side: str = "neg"

    # Rectangular tripod mount cutout (replaces circular hole when enabled)
    tripod_use_rect_cutout: bool = True
    tripod_rect_long_mm: float = 50.80   # 2.0 inches
    tripod_rect_short_mm: float = 40.64  # 1.6 inches
    tripod_rect_long_along_z: bool = True  # True = long side along case length
    tripod_rect_z_shift_mm: float = -6.35  # negative = toward front/lens (1/4")

    # Skeleton TPU frame (corner bumpers + edge rails instead of solid walls)
    skeleton_frame: bool = True
    skeleton_corner_bumper_w_mm: float = 12.0   # width of each corner bumper along wall
    skeleton_edge_rail_w_mm: float = 4.0        # width of connecting rails along each edge

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


def _load_step_as_mesh(step_path: Path, tmp_stl: Path, p: MakiTpuLinerParams):
    shape = import_step(str(step_path))
    housing = max(shape.solids(), key=lambda s: s.volume)
    step_features = _extract_step_side_features(housing, p)

    tmp_stl.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(tmp_stl))
    mesh = trimesh.load(str(tmp_stl), force="mesh")
    try:
        tmp_stl.unlink(missing_ok=True)
    except Exception:
        pass
    return mesh, housing, step_features


def _extract_tripod_from_cylindrical_faces(housing, p: MakiTpuLinerParams):
    candidates = []
    for f in housing.faces():
        if f.geom_type != GeomType.CYLINDER:
            continue
        r = getattr(f, "radius", None)
        if r is None:
            continue
        if abs(r - p.tripod_thread_radius_mm) > p.tripod_radius_tolerance_mm:
            continue
        try:
            c = f.center()
            n = f.normal_at()
        except Exception:
            continue
        if abs(float(c.X)) > p.tripod_centerline_x_max_mm:
            continue
        if not (p.tripod_z_min_mm <= float(c.Z) <= p.tripod_z_max_mm):
            continue
        if abs(float(n.Z)) < p.tripod_face_normal_min_abs_z or float(n.Z) >= 0.0:
            continue
        bb = f.bounding_box()
        major_span = max(float(bb.size.X), float(bb.size.Y), float(bb.size.Z))
        if major_span < 4.0:
            continue
        # Use bounding-box midpoint for X and Z (the cross-section plane
        # coordinates) because face.center() on cylindrical faces can be
        # offset by one radius from the true axis centre.
        bb_cx = float((bb.min.X + bb.max.X) * 0.5)
        bb_cz = float((bb.min.Z + bb.max.Z) * 0.5)
        side = "neg" if c.Y < 0 else "pos"
        score = (
            0.0 if side == p.tripod_expected_side else 1.0,
            abs(float(r) - p.tripod_thread_radius_mm),
            abs(float(n.Z) + 1.0),
            abs(bb_cx),
            abs(bb_cz + 45.0),
        )
        candidates.append({"side": side, "x": bb_cx, "y": float(c.Y), "z": bb_cz, "r": float(r), "score": score})
    if not candidates:
        return None, 0
    best = min(candidates, key=lambda c: c["score"])
    best.pop("score", None)
    return best, len(candidates)


def _extract_tripod_from_circular_edges(housing):
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
        tripod_candidates.append({"side": side, "x": float(c.X), "z": float(c.Z), "r": float(r), "y": float(c.Y)})
    if not tripod_candidates:
        return None, 0
    neg = [c for c in tripod_candidates if c["side"] == "neg"]
    pool = neg if neg else tripod_candidates
    return max(pool, key=lambda c: c["r"]), len(tripod_candidates)


def _extract_step_side_features(housing, p: MakiTpuLinerParams):
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

    tripod, cyl_count = _extract_tripod_from_cylindrical_faces(housing, p)
    tripod_source = "cylindrical_face"
    edge_count = 0
    if tripod is None:
        tripod, edge_count = _extract_tripod_from_circular_edges(housing)
        tripod_source = "circle_edge" if tripod is not None else "fallback_param"

    return {
        "vents": vents,
        "tripod": tripod,
        "tripod_source": tripod_source,
        "tripod_cyl_candidate_count": int(cyl_count),
        "tripod_edge_candidate_count": int(edge_count),
    }


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


def _resolve_tripanel_side(step_vents, fallback_side: str) -> str:
    fallback = fallback_side if fallback_side in ("neg", "pos") else "neg"
    counts = {"neg": 0, "pos": 0}
    for v in step_vents:
        if v.get("axis") != "y":
            continue
        if float(v.get("z", 0.0)) > -20.0:
            continue
        if float(v.get("slot_t", 0.0)) < 8.0 or float(v.get("slot_z", 0.0)) < 1.6:
            continue
        side = v.get("side")
        if side in counts:
            counts[side] += 1
    if counts["neg"] == counts["pos"]:
        return fallback
    return "neg" if counts["neg"] > counts["pos"] else "pos"


def _derive_tripanel_vents(
    step_vents,
    map_x,
    map_y,
    map_z,
    sx: float,
    sz: float,
    outer_w: float,
    p: MakiTpuLinerParams,
    target_side: str | None = None,
):
    """Derive a 3-panel vent pattern (8 rows each) from STEP side vents."""
    if target_side not in ("neg", "pos"):
        target_side = _resolve_tripanel_side(step_vents, p.tripod_expected_side)
    side_primary = []
    for v in step_vents:
        if v["axis"] != "y" or v["side"] != target_side:
            continue
        if v["z"] > -20.0:
            continue
        if v["slot_t"] < 8.0 or v["slot_z"] < 1.6:
            continue
        side_primary.append(v)

    if not side_primary:
        z_centers = [
            p.end_clearance_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm
            for i in range(p.vent_rows_per_panel)
        ]
        x_off = max(p.tripanel_fallback_x_offset_mm - p.tripanel_outer_column_inset_mm, 6.0)
        panels = [
            {"axis": "y", "side": target_side, "x": -x_off, "y": 0.0},
            {"axis": "y", "side": target_side, "x": 0.0, "y": 0.0},
            {"axis": "y", "side": target_side, "x": x_off, "y": 0.0},
        ]
        return {
            "panels": panels,
            "z_centers": z_centers,
            "slot_t": p.vent_slot_w_mm,
            "slot_z": p.vent_slot_h_mm,
            "panel_side": target_side,
            "source": "fallback",
        }

    slot_t = float(np.median([v["slot_t"] * sx + p.side_feature_clearance_mm for v in side_primary]))
    slot_z = float(np.median([max(v["slot_z"] * sz + p.side_feature_clearance_mm, 0.8) for v in side_primary]))

    z_vals = sorted(_collapse_close([v["z"] for v in side_primary], tol=0.7))
    if len(z_vals) >= p.vent_rows_per_panel:
        best = None
        for i in range(0, len(z_vals) - p.vent_rows_per_panel + 1):
            chunk = z_vals[i : i + p.vent_rows_per_panel]
            diffs = np.diff(chunk)
            pitch = float(np.median(diffs))
            spread = float(np.max(np.abs(diffs - pitch))) if len(diffs) else 0.0
            score = (spread, abs(pitch - p.vent_pitch_mm), -chunk[0])
            if best is None or score < best[0]:
                best = (score, chunk)
        z_centers = [float(map_z(z)) for z in best[1]]
    else:
        pitch = float(np.median(np.diff(sorted(z_vals)))) if len(z_vals) >= 2 else p.vent_pitch_mm
        z0 = z_vals[0] if z_vals else (p.end_clearance_mm + p.vent_start_from_front_mm)
        z_centers = [float(map_z(z0 + i * pitch)) for i in range(p.vent_rows_per_panel)]

    center_x = float(np.median([map_x(v["x"]) for v in side_primary]))
    x_off = min(max(p.tripanel_fallback_x_offset_mm - p.tripanel_outer_column_inset_mm, 6.0), 0.36 * outer_w)
    panels = [
        {"axis": "y", "side": target_side, "x": center_x - x_off, "y": 0.0},
        {"axis": "y", "side": target_side, "x": center_x, "y": 0.0},
        {"axis": "y", "side": target_side, "x": center_x + x_off, "y": 0.0},
    ]
    return {
        "panels": panels,
        "z_centers": [float(z) for z in z_centers],
        "slot_t": slot_t,
        "slot_z": slot_z,
        "panel_side": target_side,
        "source": "step_tripanel_cluster",
    }


def _derive_side_trio_vents(
    step_vents,
    map_y,
    map_z,
    sy: float,
    sz: float,
    p: MakiTpuLinerParams,
    size_override: tuple[float, float] | None = None,
):
    family = []
    for v in step_vents:
        if v["axis"] != "x":
            continue
        if p.side_trio_select_rear:
            if v["z"] > p.side_trio_z_threshold_mm:
                continue
        else:
            if v["z"] <= p.side_trio_z_threshold_mm:
                continue
        if not (1.5 <= v["slot_t"] <= 12.0 and 0.5 <= v["slot_z"] <= 3.5):
            continue
        family.append(v)

    if not family:
        return {
            "y_centers": [-12.0, 0.0, 12.0],
            "z_center": p.end_clearance_mm + 12.0,
            "slot_t": 4.2,
            "slot_z": 1.6,
            "source": "fallback",
        }

    z_center = float(np.median([map_z(v["z"]) for v in family]))
    y_abs = [abs(map_y(v["y"])) for v in family if abs(v["y"]) > 0.25]
    y_off = float(np.median(y_abs)) if y_abs else 12.0
    slot_t_raw = float(np.median([max(v["slot_t"] * sy + p.side_feature_clearance_mm, 1.2) for v in family]))
    slot_z_raw = float(np.median([max(v["slot_z"] * sz + p.side_feature_clearance_mm, 0.8) for v in family]))
    slot_t = max(slot_t_raw * p.side_trio_scale_t, p.side_trio_min_t_mm)
    slot_z = max(slot_z_raw * p.side_trio_scale_z, p.side_trio_min_z_mm)
    if size_override is not None and p.side_trio_match_tripanel_size:
        slot_t = max(slot_t, float(size_override[0]))
        slot_z = max(slot_z, float(size_override[1]))
    y_centers = [-y_off, 0.0, y_off] if p.side_trio_per_side >= 3 else ([-y_off, y_off] if p.side_trio_per_side == 2 else [0.0])

    return {
        "y_centers": [float(y) for y in y_centers],
        "z_center": z_center,
        "slot_t": slot_t,
        "slot_z": slot_z,
        "source": "step_side_trio",
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


def _extract_front_cutouts_tpu(housing, p: MakiTpuLinerParams, sx: float, sy: float, zmax: float):
    """Extract front-face openings from STEP housing (lens, LED, etc.) for TPU pad cutouts."""
    front_window_mm = 16.0
    cutout_extra_mm = 0.25
    min_cutout_dim_mm = 3.0
    min_cutout_area_mm2 = 8.0
    max_cutout_ratio_xy = 0.80
    aperture_shrink_mm = 2.0

    cutouts = []
    for f in housing.faces():
        wires = f.wires()
        if len(wires) <= 1:
            continue
        try:
            n = f.normal_at()
        except Exception:
            continue
        if n.Z <= 0 or abs(n.Z) < 0.92:
            continue
        for w in wires[1:]:
            bb = w.bounding_box()
            xlen = float(bb.size.X)
            ylen = float(bb.size.Y)
            zmid = float((bb.min.Z + bb.max.Z) * 0.5)
            if zmid <= (zmax - front_window_mm):
                continue

            xmid = float((bb.min.X + bb.max.X) * 0.5)
            ymid = float((bb.min.Y + bb.max.Y) * 0.5)

            too_large = (
                xlen > p.nominal_width_mm * max_cutout_ratio_xy
                and ylen > p.nominal_height_mm * max_cutout_ratio_xy
            )

            if too_large:
                d_maj = (
                    (xlen + ylen) * 0.5 * (sx + sy) * 0.5
                    + cutout_extra_mm
                    - aperture_shrink_mm
                )
                d_maj = max(d_maj, min_cutout_dim_mm)
                cutouts.append({"x": xmid * sx, "y": ymid * sy, "shape": "circle", "d": d_maj})
                continue

            d_max = max(xlen, ylen)
            d_min = min(xlen, ylen)
            if d_min < 0.6:
                continue
            ratio = d_max / max(d_min, 1e-6)
            if ratio <= 1.18:
                shape_info = {"shape": "circle", "d": (xlen + ylen) * 0.5}
            elif ratio <= 3.6:
                shape_info = {"shape": "slot", "w": d_max, "h": d_min}
            else:
                shape_info = {"shape": "rect", "w": xlen, "h": ylen}

            entry = {"x": xmid * sx, "y": ymid * sy, "shape": shape_info["shape"]}
            if shape_info["shape"] == "circle":
                entry["d"] = shape_info["d"] * (sx + sy) * 0.5 + cutout_extra_mm
                max_dim = entry["d"]
                area = math.pi * (0.5 * entry["d"]) ** 2
            else:
                entry["w"] = max(shape_info.get("w", 1.0) * sx + cutout_extra_mm, 0.8)
                entry["h"] = max(shape_info.get("h", 1.0) * sy + cutout_extra_mm, 0.8)
                max_dim = max(entry["w"], entry["h"])
                area = entry["w"] * entry["h"]
            if max_dim < min_cutout_dim_mm or area < min_cutout_area_mm2:
                continue
            cutouts.append(entry)

    out = []
    seen = set()
    for c in cutouts:
        if c["shape"] == "circle":
            key = (c["shape"], round(c["x"], 2), round(c["y"], 2), round(c["d"], 2))
        else:
            key = (c["shape"], round(c["x"], 2), round(c["y"], 2), round(c.get("w", 0), 2), round(c.get("h", 0), 2))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
    return out


def build_liner(p: MakiTpuLinerParams):
    tmp_stl = Path("tmp/maki_device_from_step_tpu_ref.stl")
    mesh, housing, step_features = _load_step_as_mesh(p.step_path, tmp_stl, p)

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
    min_x = -0.5 * outer_w
    max_x = 0.5 * outer_w

    def map_x(x_dev: float) -> float:
        return float(x_dev * sx)

    def map_y(y_dev: float) -> float:
        return float(y_dev * sy)

    def map_z(z_dev: float) -> float:
        # Match ASA sleeve datum with end clearance offset from front edge.
        return float((zmax - z_dev) * sz + p.end_clearance_mm)

    resolved_tripod_side = _resolve_tripanel_side(step_features["vents"], p.tripod_expected_side)
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

        # Skeleton frame: cut rectangular windows from each flat wall face,
        # leaving corner bumpers at each vertical edge and edge rails at top/bottom.
        if p.skeleton_frame:
            bumper_w = p.skeleton_corner_bumper_w_mm
            rail_w = p.skeleton_edge_rail_w_mm
            skel_cut_depth = p.shell_thickness_mm + 1.0

            # X walls (left/right): wall runs along Y axis
            x_wall_clear_h = max(outer_h - 2.0 * bumper_w, 0.0)
            x_wall_clear_z = max(shell_depth - 2.0 * rail_w, 0.0)
            if x_wall_clear_h > 1.0 and x_wall_clear_z > 1.0:
                x_cut_z = 0.5 * shell_depth
                for side in (-1.0, 1.0):
                    x_face = side * (0.5 * outer_w + 0.2)
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((0.0, x_cut_z)):
                            Rectangle(x_wall_clear_h, x_wall_clear_z)
                    extrude(
                        amount=skel_cut_depth if side < 0 else -skel_cut_depth,
                        mode=Mode.SUBTRACT,
                    )

            # Y walls (top/bottom): wall runs along X axis
            y_wall_clear_w = max(outer_w - 2.0 * bumper_w, 0.0)
            y_wall_clear_z = max(shell_depth - 2.0 * rail_w, 0.0)
            if y_wall_clear_w > 1.0 and y_wall_clear_z > 1.0:
                y_cut_z = 0.5 * shell_depth
                for side in (-1.0, 1.0):
                    y_face = side * (0.5 * outer_h + 0.2)
                    with BuildSketch(Plane.XZ.offset(-y_face)):
                        with Locations((0.0, y_cut_z)):
                            Rectangle(y_wall_clear_w, y_wall_clear_z)
                    extrude(
                        amount=skel_cut_depth if side > 0 else -skel_cut_depth,
                        mode=Mode.SUBTRACT,
                    )

        # Front face pad — full-face TPU between ASA front wall and camera.
        front_pad_cutouts_used = []
        if p.include_front_face_pad:
            pad_thick = p.front_face_pad_thickness_mm
            with BuildSketch(Plane.XY):
                _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
            extrude(amount=pad_thick)

            # Cut through lens / LED / port openings extracted from STEP
            front_cutouts = _extract_front_cutouts_tpu(housing, p, sx, sy, zmax)
            for co in front_cutouts:
                cx, cy = co["x"], co["y"]
                if co["shape"] == "circle":
                    with BuildSketch(Plane.XY.offset(-0.1)):
                        with Locations((cx, cy)):
                            Circle(co["d"] * 0.5)
                    extrude(amount=pad_thick + 0.2, mode=Mode.SUBTRACT)
                else:
                    cw = co.get("w", 6.0)
                    ch = co.get("h", 3.0)
                    with BuildSketch(Plane.XY.offset(-0.1)):
                        with Locations((cx, cy)):
                            Rectangle(cw, ch)
                    extrude(amount=pad_thick + 0.2, mode=Mode.SUBTRACT)
                front_pad_cutouts_used.append(co)

        # Front edge-wrap (perimeter only, no full-face cap).
        if p.include_front_edge_wrap:
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

        # Rear edge-wrap is optional. Default disabled to keep insertion side open.
        if p.include_rear_edge_wrap:
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
                vent_pattern = _derive_tripanel_vents(
                    step_features["vents"],
                    map_x,
                    map_y,
                    map_z,
                    sx,
                    sz,
                    outer_w,
                    p,
                    target_side=resolved_tripod_side,
                )
                resolved_tripod_side = vent_pattern.get("panel_side", resolved_tripod_side)
                cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                for panel_idx, panel in enumerate(vent_pattern["panels"]):
                    for z_c in vent_pattern["z_centers"]:
                        z_shifted = min(max(z_c + p.tripanel_vent_z_shift_mm, 1.0), shell_depth - 1.0)
                        if panel["axis"] == "y":
                            on_neg = panel["side"] == "neg"
                            y_face = min_y - 0.2 if on_neg else max_y + 0.2
                            with BuildSketch(Plane.XZ.offset(y_face)):
                                with Locations((panel["x"], z_shifted)):
                                    SlotOverall(vent_pattern["slot_t"], vent_pattern["slot_z"])
                            extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                            vents_used.append(
                                {
                                    "axis": "y",
                                    "side": panel["side"],
                                    "x": float(panel["x"]),
                                    "y": float(y_face),
                                    "z": float(z_shifted),
                                    "slot_t": float(vent_pattern["slot_t"]),
                                    "slot_z": float(vent_pattern["slot_z"]),
                                    "slot_w": float(vent_pattern["slot_t"]),
                                    "slot_h": float(vent_pattern["slot_z"]),
                                    "panel_index": panel_idx,
                                    "pattern_source": vent_pattern["source"],
                                }
                            )
                        else:
                            on_neg = panel["side"] == "neg"
                            x_face = panel["x"] - 0.2 if on_neg else panel["x"] + 0.2
                            with BuildSketch(Plane.YZ.offset(x_face)):
                                with Locations((panel["y"], z_shifted)):
                                    SlotOverall(vent_pattern["slot_t"], vent_pattern["slot_z"])
                            extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                            vents_used.append(
                                {
                                    "axis": "x",
                                    "side": panel["side"],
                                    "x": float(panel["x"]),
                                    "y": float(panel["y"]),
                                    "z": float(z_shifted),
                                    "slot_t": float(vent_pattern["slot_t"]),
                                    "slot_z": float(vent_pattern["slot_z"]),
                                    "slot_w": float(vent_pattern["slot_t"]),
                                    "slot_h": float(vent_pattern["slot_z"]),
                                    "panel_index": panel_idx,
                                    "pattern_source": vent_pattern["source"],
                                }
                            )
                if p.include_side_trio_vents:
                    trio = _derive_side_trio_vents(
                        step_features["vents"],
                        map_y,
                        map_z,
                        sy,
                        sz,
                        p,
                        size_override=(vent_pattern["slot_t"], vent_pattern["slot_z"]),
                    )
                    side_trio_z = shell_depth - trio["z_center"] if p.side_trio_flip_end else trio["z_center"]
                    side_trio_z = min(max(side_trio_z + p.side_trio_vent_z_shift_mm, 1.0), shell_depth - 1.0)
                    for side in ("neg", "pos"):
                        x_face = min_x - 0.2 if side == "neg" else max_x + 0.2
                        for y_c in trio["y_centers"]:
                            with BuildSketch(Plane.YZ.offset(x_face)):
                                with Locations((y_c, side_trio_z)):
                                    SlotOverall(trio["slot_t"], trio["slot_z"])
                            extrude(amount=cut_depth if side == "neg" else -cut_depth, mode=Mode.SUBTRACT)
                            vents_used.append(
                                {
                                    "axis": "x",
                                    "side": side,
                                    "x": float(x_face),
                                    "y": float(y_c),
                                    "z": float(side_trio_z),
                                    "slot_t": float(trio["slot_t"]),
                                    "slot_z": float(trio["slot_z"]),
                                    "slot_w": float(trio["slot_t"]),
                                    "slot_h": float(trio["slot_z"]),
                                    "pattern_source": trio["source"],
                                    "vent_family": "side_trio",
                                }
                            )
            else:
                for v in step_features["vents"]:
                    z_c = map_z(v["z"])
                    if v["axis"] == "y":
                        x_c = map_x(v["x"])
                        t_span = v["slot_t"] * sx
                        z_span = v["slot_z"] * sz
                        slot_t = max(t_span + p.side_feature_clearance_mm, 0.8)
                        slot_z = max(z_span + p.side_feature_clearance_mm, 0.8)
                        on_neg = v["side"] == "neg"
                        y_face = min_y - 0.2 if on_neg else max_y + 0.2
                        cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                        with Locations((x_c, y_face, z_c)):
                            Box(
                                slot_t,
                                cut_depth,
                                slot_z,
                                align=(Align.CENTER, Align.MIN, Align.CENTER)
                                if on_neg
                                else (Align.CENTER, Align.MAX, Align.CENTER),
                                mode=Mode.SUBTRACT,
                            )
                    else:
                        y_c = map_y(v["y"])
                        t_span = v["slot_t"] * sy
                        z_span = v["slot_z"] * sz
                        slot_t = max(t_span + p.side_feature_clearance_mm, 0.8)
                        slot_z = max(z_span + p.side_feature_clearance_mm, 0.8)
                        on_neg = v["side"] == "neg"
                        x_face = min_x - 0.2 if on_neg else max_x + 0.2
                        cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
                        with Locations((x_face, y_c, z_c)):
                            Box(
                                cut_depth,
                                slot_t,
                                slot_z,
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
                            "slot_t": slot_t,
                            "slot_z": slot_z,
                            "slot_w": slot_t,
                            "slot_h": slot_z,
                        }
                    )

            t = step_features["tripod"]
            if t is not None:
                x_c = map_x(t["x"])
                z_c = map_z(t["z"])
                d = max(2.0 * t["r"] * sx + p.tripod_cutout_extra_mm, 2.0)
                detected_on_neg = t["side"] == "neg"
                expected_on_neg = resolved_tripod_side == "neg"
                on_neg = expected_on_neg
                y_face = min_y - 0.2 if on_neg else max_y + 0.2
                cut_depth = p.shell_thickness_mm + 2.5
                if p.tripod_use_rect_cutout:
                    z_c += p.tripod_rect_z_shift_mm
                with BuildSketch(Plane.XZ.offset(y_face)):
                    with Locations((x_c, z_c)):
                        if p.tripod_use_rect_cutout:
                            rect_x = p.tripod_rect_short_mm if p.tripod_rect_long_along_z else p.tripod_rect_long_mm
                            rect_z = p.tripod_rect_long_mm if p.tripod_rect_long_along_z else p.tripod_rect_short_mm
                            Rectangle(rect_x, rect_z)
                        else:
                            Circle(d * 0.5)
                extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                tripod_used = {
                    "side": "neg" if on_neg else "pos",
                    "detected_side": "neg" if detected_on_neg else "pos",
                    "x": x_c,
                    "z": z_c,
                    "cutout_shape": "rect" if p.tripod_use_rect_cutout else "circle",
                    "diameter": d,
                }

        if not vents_used:
            fallback_on_neg = resolved_tripod_side == "neg"
            y_face = min_y - 0.2 if fallback_on_neg else max_y + 0.2
            cut_depth = max(p.vent_cut_depth_mm, p.shell_thickness_mm + 3.0)
            for i in range(p.vent_count):
                z = min(
                    max(
                        p.end_clearance_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm + p.tripanel_vent_z_shift_mm,
                        1.0,
                    ),
                    shell_depth - 1.0,
                )
                with BuildSketch(Plane.XZ.offset(y_face)):
                    with Locations((0.0, z)):
                        SlotOverall(p.vent_slot_w_mm, p.vent_slot_h_mm)
                extrude(amount=cut_depth if fallback_on_neg else -cut_depth, mode=Mode.SUBTRACT)

        if tripod_used is None:
            fallback_on_neg = resolved_tripod_side == "neg"
            fallback_y = min_y - 0.2 if fallback_on_neg else max_y + 0.2
            fallback_depth = p.shell_thickness_mm + 2.5
            fallback_tripod_z = p.end_clearance_mm + p.tripod_center_from_front_mm
            if p.tripod_use_rect_cutout:
                fallback_tripod_z += p.tripod_rect_z_shift_mm
            with BuildSketch(Plane.XZ.offset(fallback_y)):
                with Locations((0.0, fallback_tripod_z)):
                    if p.tripod_use_rect_cutout:
                        rect_x = p.tripod_rect_short_mm if p.tripod_rect_long_along_z else p.tripod_rect_long_mm
                        rect_z = p.tripod_rect_long_mm if p.tripod_rect_long_along_z else p.tripod_rect_short_mm
                        Rectangle(rect_x, rect_z)
                    else:
                        Circle(p.tripod_hole_diameter_mm * 0.5)
            extrude(amount=fallback_depth if fallback_on_neg else -fallback_depth, mode=Mode.SUBTRACT)
            tripod_used = {
                "side": "neg" if fallback_on_neg else "pos",
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
            "resolved_tripod_side": resolved_tripod_side,
            "vents_detected": len(step_features["vents"]),
            "vents_applied": len(vents_used),
            "vents_applied_entries": vents_used,
            "tripod_detected": step_features["tripod"] is not None,
            "tripod_source": step_features.get("tripod_source", "unknown"),
            "tripod_cyl_candidate_count": step_features.get("tripod_cyl_candidate_count", 0),
            "tripod_edge_candidate_count": step_features.get("tripod_edge_candidate_count", 0),
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
            "edge_wrap_enabled": {
                "front": bool(p.include_front_edge_wrap),
                "rear": bool(p.include_rear_edge_wrap),
            },
            "front_face_pad": {
                "enabled": bool(p.include_front_face_pad),
                "thickness_mm": float(p.front_face_pad_thickness_mm) if p.include_front_face_pad else 0.0,
                "cutouts_applied": len(front_pad_cutouts_used),
                "cutouts": front_pad_cutouts_used,
            },
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
    parser.add_argument("--tripod-rect", action="store_true", help="Use rectangular cutout instead of circular tripod hole")
    parser.add_argument("--tripod-rect-along-width", action="store_true", help="Orient long side of rect along case width (default: along length)")
    parser.add_argument("--no-front-face-pad", action="store_true", help="Disable front face TPU pad")
    parser.add_argument("--front-face-pad-thickness", type=float, default=None, help="Front face pad thickness (mm)")
    parser.add_argument("--out-suffix", type=str, default="", help="Suffix appended to output filenames")
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
    if args.tripod_rect:
        params.tripod_use_rect_cutout = True
    if args.tripod_rect_along_width:
        params.tripod_rect_long_along_z = False
    if args.no_front_face_pad:
        params.include_front_face_pad = False
    if args.front_face_pad_thickness is not None:
        params.front_face_pad_thickness_mm = args.front_face_pad_thickness

    liner, report = build_liner(params)

    suffix = args.out_suffix
    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    out_step = args.out / f"maki_live_tpu_sleeve{suffix}.step"
    out_json = reports_dir / f"maki_live_tpu_sleeve_report{suffix}.json"
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
