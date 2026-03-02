#!/usr/bin/env python3
"""Generate a printable protective sleeve for BirdDog MAKI Live.

Inputs:
- STEP model: full MAKI device geometry
- PDF dimensions (applied as nominal constraints)

Outputs:
- maki_live_case_sleeve.step
- reports/maki_live_case_report.json
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
class MakiCaseParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # Nominal envelope from MAKI drawing (mm)
    nominal_width_mm: float = 56.99
    nominal_height_mm: float = 56.99
    nominal_length_mm: float = 120.32

    # Fit and shell
    clearance_mm: float = 2.3
    wall_mm: float = 3.0
    front_wall_mm: float = 3.0
    front_integrated: bool = True
    include_front_cutouts: bool = True
    front_window_mm: float = 16.0
    cutout_extra_mm: float = 0.25
    max_cutout_ratio_xy: float = 0.80
    include_major_front_aperture: bool = True

    # Front optics opening
    lens_center_y_mm: float = 4.5
    lens_diameter_mm: float = 30.0
    front_bezel_extra_mm: float = 1.0
    front_bezel_height_mm: float = 1.1

    # Side tripod opening (derived from STEP feature location)
    tripod_center_from_front_mm: float = 48.0
    tripod_open_w_mm: float = 20.0
    tripod_open_h_mm: float = 18.0
    tripod_hole_diameter_mm: float = 10.0
    tripod_armor_extra_mm: float = 1.4
    tripod_armor_margin_mm: float = 4.5
    use_step_side_features: bool = True

    # Vent slots (optional)
    vent_count: int = 10
    vent_slot_w_mm: float = 16.0
    vent_slot_h_mm: float = 2.6
    vent_pitch_mm: float = 5.8
    vent_start_from_front_mm: float = 66.0
    vent_rows_per_panel: int = 8
    enforce_tripanel_vent_layout: bool = True
    tripanel_fallback_x_offset_mm: float = 16.0
    vent_cut_depth_mm: float = 12.0

    # Shape processing
    section_z_ratio: float = 0.50
    profile_simplify_tol_mm: float = 0.08
    offset_resolution: int = 64
    side_feature_clearance_mm: float = 0.3


# ---------- geometry helpers ----------

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
    """Estimate an equivalent rounded-rectangle corner radius from a section profile."""
    min_x, min_y, max_x, max_y = profile.bounds
    w = max_x - min_x
    h = max_y - min_y
    corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]
    r_from_corners = []
    for c in corners:
        d = Point(c).distance(profile.exterior)
        r = d / (math.sqrt(2.0) - 1.0)
        r_from_corners.append(r)

    # Area-derived estimate for rounded-rectangle profile:
    # area = w*h - (4-pi)*r^2  -> r = sqrt((w*h-area)/(4-pi))
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
    # Extract side features before STL export. Export can alter queried wire bounds.
    step_features = _extract_step_side_features(housing)

    tmp_stl.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(tmp_stl))

    mesh = trimesh.load(str(tmp_stl), force="mesh")
    try:
        tmp_stl.unlink(missing_ok=True)
    except Exception:
        pass
    return mesh, housing, step_features


def _extract_step_side_features(housing):
    """Extract side vent slots + tripod hole from the original STEP housing.

    Vents are gathered across flat and corner side panels so multi-panel slot rows
    are preserved (not just the single flat wall).
    """
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
        # Skip front/back end-cap style faces; keep side panel families.
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

            # Slot-like vent filter: elongated opening with small thickness.
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
        # Circular edges that lie on side walls and describe the tripod feature.
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
        tripod_candidates.append(
            {"side": side, "x": c.X, "z": c.Z, "r": r, "y": c.Y}
        )

    tripod = None
    if tripod_candidates:
        # Prefer the largest clear circular edge on the negative-Y tripod side.
        neg = [c for c in tripod_candidates if c["side"] == "neg"]
        pool = neg if neg else tripod_candidates
        tripod = max(pool, key=lambda c: c["r"])

    return {"vents": vents, "tripod": tripod}


def _classify_cutout(xlen: float, ylen: float) -> dict | None:
    d_max = max(xlen, ylen)
    d_min = min(xlen, ylen)
    if d_min < 0.6:
        return None
    ratio = d_max / max(d_min, 1e-6)
    if ratio <= 1.18:
        return {"shape": "circle", "d": (xlen + ylen) * 0.5}
    if ratio <= 3.6:
        return {"shape": "slot", "w": d_max, "h": d_min}
    return {"shape": "rect", "w": xlen, "h": ylen}


def _extract_front_cutouts(housing, p: MakiCaseParams, sx: float, sy: float, zmax: float):
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
            if zmid <= (zmax - p.front_window_mm):
                continue

            xmid = float((bb.min.X + bb.max.X) * 0.5)
            ymid = float((bb.min.Y + bb.max.Y) * 0.5)

            too_large = (
                xlen > p.nominal_width_mm * p.max_cutout_ratio_xy
                and ylen > p.nominal_height_mm * p.max_cutout_ratio_xy
            )
            if too_large and not p.include_major_front_aperture:
                continue

            if too_large:
                d_maj = (xlen + ylen) * 0.5 * (sx + sy) * 0.5 + p.cutout_extra_mm
                cutouts.append({"x": xmid * sx, "y": ymid * sy, "shape": "circle", "d": d_maj})
                continue

            shape = _classify_cutout(xlen, ylen)
            if shape is None:
                continue
            entry = {"x": xmid * sx, "y": ymid * sy, "shape": shape["shape"]}
            if shape["shape"] == "circle":
                entry["d"] = shape["d"] * (sx + sy) * 0.5 + p.cutout_extra_mm
            else:
                entry["w"] = max(shape["w"] * sx + p.cutout_extra_mm, 0.8)
                entry["h"] = max(shape["h"] * sy + p.cutout_extra_mm, 0.8)
            cutouts.append(entry)

    out = []
    seen = set()
    for c in cutouts:
        if c["shape"] == "circle":
            key = (c["shape"], round(c["x"], 2), round(c["y"], 2), round(c["d"], 2))
        else:
            key = (c["shape"], round(c["x"], 2), round(c["y"], 2), round(c["w"], 2), round(c["h"], 2))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
    return out


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


def _derive_tripanel_vents(step_vents, map_x, map_z, sx: float, sz: float, outer_w: float, p: MakiCaseParams):
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

    # Fallback pattern if extraction is sparse.
    if not neg_side:
        z_centers = [
            p.clearance_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm
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

    # Use median slot size for consistency across the 3 panels.
    slot_w = float(np.median([v["slot_w"] for v in neg_side]))
    slot_h = float(np.median([v["slot_h"] for v in neg_side]))

    # Build z row centers (8 vents per panel).
    z_vals = _collapse_close([v["z"] for v in neg_side], tol=0.9)
    if len(z_vals) >= p.vent_rows_per_panel:
        z_centers = z_vals[: p.vent_rows_per_panel]
    else:
        if len(z_vals) >= 2:
            pitch = float(np.median(np.diff(sorted(z_vals))))
        else:
            pitch = p.vent_pitch_mm
        z0 = z_vals[0] if z_vals else (p.clearance_mm + p.vent_start_from_front_mm)
        z_centers = [z0 + i * pitch for i in range(p.vent_rows_per_panel)]

    # Build 3 panel x centers: center + 2 adjacent panels.
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

    # Convex hull gives a robust outer envelope from noisy section topology.
    hull = ConvexHull(xy)
    hull_xy = xy[hull.vertices]

    cx = float((hull_xy[:, 0].min() + hull_xy[:, 0].max()) * 0.5)
    cy = float((hull_xy[:, 1].min() + hull_xy[:, 1].max()) * 0.5)
    hull_xy -= np.array([cx, cy])

    base = Polygon(hull_xy)
    base = _to_single_poly(base.simplify(0.05, preserve_topology=True))
    return base


def build_case(p: MakiCaseParams):
    tmp_stl = Path("tmp/maki_device_from_step.stl")
    mesh, housing, step_features = _load_step_as_mesh(p.step_path, tmp_stl)

    zmin, zmax = mesh.bounds[:, 2]
    z_section = zmin + (zmax - zmin) * p.section_z_ratio

    base_profile = _extract_profile_xy(mesh, z_section)

    raw_min_x, raw_min_y, raw_max_x, raw_max_y = base_profile.bounds
    raw_w = raw_max_x - raw_min_x
    raw_h = raw_max_y - raw_min_y

    # Lock profile to drawing dimensions.
    sx = p.nominal_width_mm / raw_w
    sy = p.nominal_height_mm / raw_h
    base_profile = affinity.scale(base_profile, xfact=sx, yfact=sy, origin=(0.0, 0.0))
    sz = p.nominal_length_mm / (zmax - zmin)

    base_corner_r = _estimate_corner_radius(base_profile)
    inner_w = p.nominal_width_mm + 2.0 * p.clearance_mm
    inner_h = p.nominal_height_mm + 2.0 * p.clearance_mm
    outer_w = inner_w + 2.0 * p.wall_mm
    outer_h = inner_h + 2.0 * p.wall_mm
    inner_corner_r = min(base_corner_r + p.clearance_mm, 0.5 * min(inner_w, inner_h) - 0.2)
    outer_corner_r = min(inner_corner_r + p.wall_mm, 0.5 * min(outer_w, outer_h) - 0.2)

    inner_depth = p.nominal_length_mm + 2.0 * p.clearance_mm
    if p.front_integrated:
        cavity_front_z = p.front_wall_mm
        shell_depth = p.front_wall_mm + inner_depth
    else:
        cavity_front_z = 0.0
        shell_depth = inner_depth

    min_x = -0.5 * outer_w
    max_x = 0.5 * outer_w
    min_y = -0.5 * outer_h
    max_y = 0.5 * outer_h
    def map_x(x_dev: float) -> float:
        return float(x_dev * sx)

    def map_y(y_dev: float) -> float:
        return float(y_dev * sy)

    def map_z(z_dev: float) -> float:
        # Device front is near zmax in source STEP; map into sleeve cavity space.
        return float(cavity_front_z + (zmax - z_dev) * sz + p.clearance_mm)

    vents_used = []
    tripod_used = None
    front_cutouts_applied = []
    front_cutouts_detected = []

    with BuildPart() as sleeve_bp:
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(outer_w, outer_h, outer_corner_r)
        extrude(amount=shell_depth)

        # Hollow sleeve cavity; front wall remains when front-integrated mode is enabled.
        inner_cut_z = cavity_front_z if p.front_integrated else -0.2
        inner_cut_depth = inner_depth + 0.2 if p.front_integrated else shell_depth + 0.4
        with BuildSketch(Plane.XY.offset(inner_cut_z)):
            _add_rounded_rectangle(inner_w, inner_h, inner_corner_r)
        extrude(amount=inner_cut_depth, mode=Mode.SUBTRACT)

        # Front cutouts in integrated front wall.
        if p.front_integrated and p.include_front_cutouts:
            front_cutouts_detected = _extract_front_cutouts(housing, p, sx, sy, zmax)
            if not front_cutouts_detected:
                front_cutouts_detected = [
                    {"x": 0.0, "y": p.lens_center_y_mm, "shape": "circle", "d": p.lens_diameter_mm}
                ]
            with BuildSketch(Plane.XY.offset(-0.2)):
                for c in front_cutouts_detected:
                    with Locations((c["x"], c["y"])):
                        if c["shape"] == "circle":
                            Circle(c["d"] * 0.5)
                        elif c["shape"] == "slot":
                            SlotOverall(c["w"], c["h"])
                        else:
                            Rectangle(c["w"], c["h"])
            extrude(amount=p.front_wall_mm + 0.6, mode=Mode.SUBTRACT)
            front_cutouts_applied = list(front_cutouts_detected)

        if p.use_step_side_features:
            if p.enforce_tripanel_vent_layout:
                vent_pattern = _derive_tripanel_vents(step_features["vents"], map_x, map_z, sx, sz, outer_w, p)
                y_face = min_y - 0.2
                cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + p.tripod_armor_extra_mm + 3.0)
                for panel_idx, x_c in enumerate(vent_pattern["x_centers"]):
                    for z_c in vent_pattern["z_centers"]:
                        # Start from outer face and cut inward to guarantee through-wall opening.
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
                        cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + p.tripod_armor_extra_mm + 3.0)
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
                        cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + p.tripod_armor_extra_mm + 3.0)
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
                y_face = min_y + 0.15 if on_neg else max_y - 0.15

                # Local armor boss to distribute impact around tripod region.
                if on_neg:
                    with Locations((x_c, min_y, z_c)):
                        Box(
                            d + 2.0 * p.tripod_armor_margin_mm,
                            p.tripod_armor_extra_mm,
                            d + 2.0 * p.tripod_armor_margin_mm,
                            align=(Align.CENTER, Align.MAX, Align.CENTER),
                        )
                else:
                    with Locations((x_c, max_y, z_c)):
                        Box(
                            d + 2.0 * p.tripod_armor_margin_mm,
                            p.tripod_armor_extra_mm,
                            d + 2.0 * p.tripod_armor_margin_mm,
                            align=(Align.CENTER, Align.MIN, Align.CENTER),
                        )

                cut_depth = p.wall_mm + 3.0
                with BuildSketch(Plane.XZ.offset(y_face)):
                    with Locations((x_c, z_c)):
                        Circle(d * 0.5)
                extrude(amount=(-cut_depth if on_neg else cut_depth), mode=Mode.SUBTRACT)
                tripod_used = {"side": t["side"], "x": x_c, "z": z_c, "diameter": d}

        # Fallback if STEP-derived features were unavailable.
        if not vents_used:
            vent_z0 = cavity_front_z + p.clearance_mm + p.vent_start_from_front_mm
            y_face = min_y - 0.2
            cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + p.tripod_armor_extra_mm + 3.0)
            for i in range(p.vent_count):
                z = vent_z0 + i * p.vent_pitch_mm
                with Locations((0.0, y_face, z)):
                    Box(
                        p.vent_slot_h_mm,
                        cut_depth,
                        p.vent_slot_w_mm,
                        align=(Align.CENTER, Align.MIN, Align.CENTER),
                        mode=Mode.SUBTRACT,
                    )

        if tripod_used is None:
            tripod_z = cavity_front_z + p.clearance_mm + p.tripod_center_from_front_mm

            with Locations((0.0, min_y, tripod_z)):
                Box(
                    p.tripod_hole_diameter_mm + 2.0 * p.tripod_armor_margin_mm,
                    p.tripod_armor_extra_mm,
                    p.tripod_hole_diameter_mm + 2.0 * p.tripod_armor_margin_mm,
                    align=(Align.CENTER, Align.MAX, Align.CENTER),
                )

            with BuildSketch(Plane.XZ.offset(min_y + 0.2)):
                with Locations((0.0, tripod_z)):
                    Circle(p.tripod_hole_diameter_mm * 0.5)
            extrude(amount=-(p.wall_mm + 3.0), mode=Mode.SUBTRACT)
            tripod_used = {"side": "neg", "x": 0.0, "z": tripod_z, "diameter": p.tripod_hole_diameter_mm}

    sleeve = _largest_solid(sleeve_bp.part)
    sleeve, sleeve_fillet_y = _apply_axis_fillet(sleeve, Axis.Y, (0.8, 0.6, 0.45, 0.3, 0.2))

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
            "tripanel_vent_layout_enforced": p.enforce_tripanel_vent_layout,
            "tripod_detected": step_features["tripod"] is not None,
            "tripod_applied": tripod_used,
        },
        "derived": {
            "inner_depth_mm": float(inner_depth),
            "shell_depth_mm": float(shell_depth),
            "open_sleeve": not bool(p.front_integrated),
            "front_integrated": bool(p.front_integrated),
            "cavity_front_z_mm": float(cavity_front_z),
            "inner_w_mm": float(inner_w),
            "inner_h_mm": float(inner_h),
            "outer_w_mm": float(outer_w),
            "outer_h_mm": float(outer_h),
            "end_clearance_each_mm": float(p.clearance_mm),
            "front_cutouts": {
                "enabled": bool(p.include_front_cutouts),
                "detected": len(front_cutouts_detected),
                "applied": len(front_cutouts_applied),
                "entries": front_cutouts_applied,
            },
            "tripod_armor_mm": {
                "extra_thickness": float(p.tripod_armor_extra_mm),
                "margin": float(p.tripod_armor_margin_mm),
            },
            "corner_radius_mm": {
                "base_section": float(base_corner_r),
                "inner": float(inner_corner_r),
                "outer": float(outer_corner_r),
            },
            "machined_finish_mm": {
                "sleeve_axis_y_fillet": sleeve_fillet_y,
            },
            "outer_bounds_xy_mm": {
                "min_x": float(min_x),
                "max_x": float(max_x),
                "min_y": float(min_y),
                "max_y": float(max_y),
            },
        },
    }

    return sleeve, report


def main():
    parser = argparse.ArgumentParser(description="Generate MAKI Live protective sleeve")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--clearance", type=float, default=None, help="Internal clearance (mm)")
    parser.add_argument("--wall", type=float, default=None, help="Wall thickness (mm)")
    parser.add_argument("--open-front", action="store_true", help="Legacy mode: keep front end fully open.")
    parser.add_argument("--no-front-cutouts", action="store_true", help="Disable front-wall cutouts in integrated-front mode.")
    parser.add_argument("--lens-d", type=float, default=None, help="Front lens opening diameter (mm)")
    parser.add_argument("--tripod-z", type=float, default=None, help="Fallback tripod opening center from front (mm)")
    parser.add_argument("--no-step-side-features", action="store_true", help="Disable STEP-derived side vents/tripod hole")
    args = parser.parse_args()

    params = MakiCaseParams(step_path=args.step)
    if args.clearance is not None:
        params.clearance_mm = args.clearance
    if args.wall is not None:
        params.wall_mm = args.wall
    if args.open_front:
        params.front_integrated = False
    if args.no_front_cutouts:
        params.include_front_cutouts = False
    if args.lens_d is not None:
        params.lens_diameter_mm = args.lens_d
    if args.tripod_z is not None:
        params.tripod_center_from_front_mm = args.tripod_z
    if args.no_step_side_features:
        params.use_step_side_features = False

    sleeve, report = build_case(params)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    out_step = args.out / "maki_live_case_sleeve.step"
    out_json = reports_dir / "maki_live_case_report.json"
    archived = _archive_existing(
        [
            out_step,
            out_json,
            args.out / "maki_live_case_report.json",  # legacy top-level location
        ],
        args.out,
    )

    export_step(sleeve, str(out_step))

    payload = {"params": asdict(params), "report": report}
    payload["params"]["step_path"] = str(params.step_path)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
