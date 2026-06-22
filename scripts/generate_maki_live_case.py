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
import tempfile
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
    rear_extension_mm: float = 6.0
    wall_mm: float = 3.0
    front_wall_mm: float = 3.0
    front_integrated: bool = True
    include_front_cutouts: bool = True
    front_single_circle_cutout_only: bool = True
    front_window_mm: float = 16.0
    cutout_extra_mm: float = 0.25
    front_min_cutout_dim_mm: float = 3.0
    front_min_cutout_area_mm2: float = 8.0
    max_cutout_ratio_xy: float = 0.80
    include_major_front_aperture: bool = True
    front_major_aperture_shrink_mm: float = 2.0

    # Front optics opening
    lens_center_y_mm: float = 0.0
    lens_diameter_mm: float = 46.7        # slightly enlarged again from prior 45.7 mm
    front_bezel_extra_mm: float = 1.0
    front_bezel_height_mm: float = 1.1
    lens_hood_enabled: bool = False
    lens_hood_depth_mm: float = 10.0
    lens_hood_wall_mm: float = 2.5
    lens_hood_clearance_mm: float = 1.0
    lens_hood_perimeter_margin_mm: float = 1.5
    lens_hood_base_flare_mm: float = 4.0
    lens_hood_base_depth_mm: float = 5.0

    # Cold shoe mount (ISO 518 female receptor)
    cold_shoe_enabled: bool = True
    cold_shoe_boss_height_mm: float = 4.0      # Boss protrusion above outer shell surface
    cold_shoe_boss_length_mm: float = 22.0     # Along Z (slightly longer than slot for walls)
    cold_shoe_boss_width_mm: float = 22.0      # Along X (slightly wider than slot for walls)
    cold_shoe_slot_width_mm: float = 18.8      # +0.4mm/side clearance for foot flange
    cold_shoe_rail_overhang_mm: float = 2.65   # reduced for looser stem fit
    cold_shoe_rail_thickness_mm: float = 1.8   # Vertical rail height
    cold_shoe_slot_depth_mm: float = 2.5       # Below rail to floor (deeper C-channel)
    cold_shoe_z_from_rear_mm: float = 20.0     # Center distance from rear edge
    cold_shoe_fillet_mm: float = 0.8           # Edge fillet on boss

    # Floating sun shade canopy (top + partial sides)
    include_sun_shade: bool = True
    sun_shade_standoff_mm: float = 6.0
    sun_shade_wall_mm: float = 2.0
    sun_shade_post_width_mm: float = 4.0
    sun_shade_side_drop_ratio: float = 0.72
    sun_shade_side_support_height_mm: float = 0.0
    sun_shade_vent_relief_mm: float = 0.8
    sun_shade_side_skirt_corner_r_mm: float = 2.0
    sun_shade_side_skirt_inner_edge_fillet_mm: float = 0.75

    # Snap-latch flexure clips for rear cap retention
    include_snap_clips: bool = False
    snap_clip_beam_length_mm: float = 7.0
    snap_clip_beam_width_mm: float = 5.0
    snap_clip_beam_thickness_mm: float = 1.5
    snap_clip_catch_height_mm: float = 1.0
    snap_clip_catch_depth_mm: float = 1.0
    snap_clip_setback_mm: float = 3.0
    snap_clip_y_position_mm: float = 0.0

    # Continuous friction ridge around plug perimeter
    include_friction_ridge: bool = True
    friction_ridge_height_mm: float = 0.6
    friction_ridge_width_mm: float = 1.5
    friction_ridge_setback_mm: float = 3.0

    # Side tripod opening (derived from STEP feature location)
    tripod_center_from_front_mm: float = 48.0
    tripod_open_w_mm: float = 20.0
    tripod_open_h_mm: float = 18.0
    tripod_hole_diameter_mm: float = 10.0
    tripod_cutout_extra_mm: float = 1.5
    tripod_armor_extra_mm: float = 0.0  # Armor boss disabled
    tripod_armor_margin_mm: float = 0.0  # Armor boss disabled
    use_step_side_features: bool = True
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

    # Vent slots (optional)
    vent_count: int = 10
    vent_slot_w_mm: float = 16.0
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
    vent_cut_depth_mm: float = 12.0
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
    merged_tripanel_cutout_margin_mm: float = 1.0
    merged_tripanel_tripod_bridge_mm: float = 2.0
    include_large_other_side_cutouts: bool = True
    large_other_side_cutout_front_margin_mm: float = 8.0
    large_other_side_cutout_rear_margin_mm: float = 14.0
    large_other_side_cutout_corner_margin_mm: float = 2.0

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


def _load_step_as_mesh(step_path: Path, tmp_stl: Path, p: MakiCaseParams):
    shape = import_step(str(step_path))
    housing = max(shape.solids(), key=lambda s: s.volume)
    # Extract side features before STL export. Export can alter queried wire bounds.
    step_features = _extract_step_side_features(housing, p)

    tmp_stl.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(tmp_stl))

    mesh = trimesh.load(str(tmp_stl), force="mesh")
    try:
        tmp_stl.unlink(missing_ok=True)
    except Exception:
        pass
    return mesh, housing, step_features


def _extract_tripod_from_cylindrical_faces(housing, p: MakiCaseParams):
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
        # Thread axis expected along -Z for this MAKI STEP orientation.
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
        candidates.append(
            {
                "side": side,
                "x": bb_cx,
                "y": float(c.Y),
                "z": bb_cz,
                "r": float(r),
                "normal": [float(n.X), float(n.Y), float(n.Z)],
                "score": score,
            }
        )
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
            {"side": side, "x": float(c.X), "z": float(c.Z), "r": float(r), "y": float(c.Y)}
        )
    if not tripod_candidates:
        return None, 0
    neg = [c for c in tripod_candidates if c["side"] == "neg"]
    pool = neg if neg else tripod_candidates
    return max(pool, key=lambda c: c["r"]), len(tripod_candidates)


def _extract_step_side_features(housing, p: MakiCaseParams):
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


def _estimate_wire_corner_radius_mm(wire, sx: float, sy: float, fallback_mm: float) -> float:
    arc_radii = []
    scale = 0.5 * (sx + sy)
    for edge in wire.edges():
        try:
            if edge.geom_type != GeomType.CIRCLE:
                continue
            arc_radii.append(float(edge.radius) * scale)
        except Exception:
            continue
    usable = [r for r in arc_radii if r > 1.5]
    if usable:
        return float(np.median(usable))
    return float(max(fallback_mm, 0.6))


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
                edge_delta = 0.5 * (p.cutout_extra_mm - p.front_major_aperture_shrink_mm)
                w_maj = max(xlen * sx + p.cutout_extra_mm - p.front_major_aperture_shrink_mm, p.front_min_cutout_dim_mm)
                h_maj = max(ylen * sy + p.cutout_extra_mm - p.front_major_aperture_shrink_mm, p.front_min_cutout_dim_mm)
                fallback_r = 0.28 * min(w_maj, h_maj)
                r_maj = max(
                    _estimate_wire_corner_radius_mm(w, sx, sy, fallback_r) + edge_delta,
                    0.6,
                )
                r_maj = min(r_maj, 0.5 * min(w_maj, h_maj) - 0.2)
                cutouts.append(
                    {
                        "x": xmid * sx,
                        "y": ymid * sy,
                        "shape": "roundrect",
                        "w": w_maj,
                        "h": h_maj,
                        "r": r_maj,
                    }
                )
                continue

            shape = _classify_cutout(xlen, ylen)
            if shape is None:
                continue
            entry = {"x": xmid * sx, "y": ymid * sy, "shape": shape["shape"]}
            if shape["shape"] == "circle":
                entry["d"] = shape["d"] * (sx + sy) * 0.5 + p.cutout_extra_mm
                max_dim = entry["d"]
                area = math.pi * (0.5 * entry["d"]) ** 2
            else:
                entry["w"] = max(shape["w"] * sx + p.cutout_extra_mm, 0.8)
                entry["h"] = max(shape["h"] * sy + p.cutout_extra_mm, 0.8)
                max_dim = max(entry["w"], entry["h"])
                area = entry["w"] * entry["h"]
            if max_dim < p.front_min_cutout_dim_mm or area < p.front_min_cutout_area_mm2:
                continue
            cutouts.append(entry)

    out = []
    seen = set()
    for c in cutouts:
        if c["shape"] == "circle":
            key = (c["shape"], round(c["x"], 2), round(c["y"], 2), round(c["d"], 2))
        elif c["shape"] == "roundrect":
            key = (
                c["shape"],
                round(c["x"], 2),
                round(c["y"], 2),
                round(c["w"], 2),
                round(c["h"], 2),
                round(c["r"], 2),
            )
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
    p: MakiCaseParams,
    target_side: str | None = None,
):
    """Derive a 3-panel vent pattern (8 rows each) from STEP side vents."""
    if target_side not in ("neg", "pos"):
        target_side = _resolve_tripanel_side(step_vents, p.tripod_expected_side)
    # Primary vent-bank candidates: large elongated side slots on tripod side,
    # excluding tiny front-region decorative/fastener loops.
    side_primary = []
    for v in step_vents:
        if v["axis"] != "y" or v["side"] != target_side:
            continue
        if v["z"] > -20.0:
            continue
        if v["slot_t"] < 8.0 or v["slot_z"] < 1.6:
            continue
        side_primary.append(v)

    # Fallback pattern if extraction is sparse.
    if not side_primary:
        z_centers = [
            p.clearance_mm + p.front_wall_mm + p.vent_start_from_front_mm + i * p.vent_pitch_mm
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

    # Use median slot size for consistency across the 3 panels.
    slot_t = float(np.median([v["slot_t"] * sx + p.side_feature_clearance_mm for v in side_primary]))
    slot_z = float(np.median([max(v["slot_z"] * sz + p.side_feature_clearance_mm, 0.8) for v in side_primary]))

    # Build z row centers (8 vents per panel) from primary bank only.
    z_vals_raw = _collapse_close([v["z"] for v in side_primary], tol=0.7)
    z_vals = sorted(z_vals_raw)
    if len(z_vals) >= p.vent_rows_per_panel:
        # Prefer the longest contiguous run with near-constant pitch.
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
        if len(z_vals) >= 2:
            pitch = float(np.median(np.diff(sorted(z_vals))))
        else:
            pitch = p.vent_pitch_mm
        z0 = z_vals[0] if z_vals else (p.clearance_mm + p.vent_start_from_front_mm)
        z_centers = [float(map_z(z0 + i * pitch)) for i in range(p.vent_rows_per_panel)]

    # Keep the 24-vent bank on the tripod-side bottom 3-panel family:
    # one center lane and two adjacent lanes on the same bottom-facing side.
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
    p: MakiCaseParams,
    size_override: tuple[float, float] | None = None,
):
    """Derive 3-per-side vents from side vent family in STEP."""
    family = []
    for v in step_vents:
        if v["axis"] != "x":
            continue
        # Select rear-end or front-end vents based on threshold direction.
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
            "z_center": p.clearance_mm + p.front_wall_mm + 12.0,
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

    if p.side_trio_per_side <= 1:
        y_centers = [0.0]
    elif p.side_trio_per_side == 2:
        y_centers = [-y_off, y_off]
    else:
        y_centers = [-y_off, 0.0, y_off]

    return {
        "y_centers": [float(y) for y in y_centers],
        "z_center": z_center,
        "slot_t": slot_t,
        "slot_z": slot_z,
        "source": "step_side_trio",
    }


def _derive_merged_tripanel_cutout(
    vent_pattern: dict,
    z_centers: list[float],
    tripod_feature: dict | None,
    map_x,
    map_z,
    sx: float,
    fallback_tripod_z: float,
    p: MakiCaseParams,
) -> dict | None:
    panels = vent_pattern.get("panels", [])
    if not panels or not z_centers:
        return None

    slot_t = float(vent_pattern["slot_t"])
    slot_z = float(vent_pattern["slot_z"])
    margin = max(float(p.merged_tripanel_cutout_margin_mm), 0.0)

    vent_min_x = min(float(panel.get("x", 0.0)) - 0.5 * slot_t for panel in panels)
    vent_max_x = max(float(panel.get("x", 0.0)) + 0.5 * slot_t for panel in panels)
    vent_min_z = min(float(z) - 0.5 * slot_z for z in z_centers)
    vent_max_z = max(float(z) + 0.5 * slot_z for z in z_centers)

    if tripod_feature is not None:
        tripod_x = float(map_x(float(tripod_feature["x"])))
        tripod_z = float(map_z(float(tripod_feature["z"])))
        tripod_d = max(2.0 * float(tripod_feature["r"]) * sx + p.tripod_cutout_extra_mm, 2.0)
    else:
        tripod_x = 0.0
        tripod_z = float(fallback_tripod_z)
        tripod_d = float(p.tripod_hole_diameter_mm)

    if p.tripod_use_rect_cutout:
        tripod_z += p.tripod_rect_z_shift_mm
        tripod_w = float(p.tripod_rect_short_mm if p.tripod_rect_long_along_z else p.tripod_rect_long_mm)
        tripod_h = float(p.tripod_rect_long_mm if p.tripod_rect_long_along_z else p.tripod_rect_short_mm)
    else:
        tripod_w = tripod_d
        tripod_h = tripod_d

    min_x = min(vent_min_x, tripod_x - 0.5 * tripod_w) - margin
    max_x = max(vent_max_x, tripod_x + 0.5 * tripod_w) + margin
    vent_only_min_z = vent_min_z - margin
    vent_only_max_z = vent_max_z + margin
    tripod_top_z = tripod_z + 0.5 * tripod_h
    bridge_gap = max(float(p.merged_tripanel_tripod_bridge_mm), 0.0)
    min_z = max(vent_only_min_z, tripod_top_z + bridge_gap)
    max_z = vent_only_max_z
    width = max(max_x - min_x, 1.0)
    height = max(max_z - min_z, 1.0)
    covered_logical_vents = sum(
        1
        for z in z_centers
        if (float(z) - 0.5 * slot_z) >= min_z and (float(z) + 0.5 * slot_z) <= max_z
    ) * len(panels)
    logical_vents = int(len(panels) * len(z_centers))

    return {
        "id": "tripod_side_separate_vent_panel",
        "axis": "y",
        "side": vent_pattern.get("panel_side", "neg"),
        "shape": "rect",
        "x": float(0.5 * (min_x + max_x)),
        "z": float(0.5 * (min_z + max_z)),
        "w": float(width),
        "h": float(height),
        "bounds": {
            "min_x": float(min_x),
            "max_x": float(max_x),
            "min_z": float(min_z),
            "max_z": float(max_z),
        },
        "vent_only_bounds_before_bridge": {
            "min_z": float(vent_only_min_z),
            "max_z": float(vent_only_max_z),
        },
        "covers_tripanel_logical_vents": int(covered_logical_vents),
        "logical_tripanel_vent_locations": logical_vents,
        "omitted_lowest_tripanel_logical_vents": int(logical_vents - covered_logical_vents),
        "merged_with_tripod_cutout": False,
        "separated_from_tripod_cutout": True,
        "tripod_bridge_mm": float(bridge_gap),
        "tripod_rect_top_z_mm": float(tripod_top_z),
        "tripod_rect_w_mm": float(tripod_w),
        "tripod_rect_h_mm": float(tripod_h),
        "margin_mm": float(margin),
        "pattern_source": vent_pattern.get("source", "unknown"),
    }


def _derive_large_other_side_cutouts(
    resolved_tripod_side: str,
    outer_w: float,
    outer_h: float,
    outer_corner_r: float,
    shell_depth: float,
    cavity_front_z: float,
    p: MakiCaseParams,
) -> list[dict]:
    if not p.include_large_other_side_cutouts:
        return []

    tripod_side = resolved_tripod_side if resolved_tripod_side in ("neg", "pos") else "neg"
    opposite_tripod_side = "pos" if tripod_side == "neg" else "neg"
    corner_margin = max(float(p.large_other_side_cutout_corner_margin_mm), 0.0)
    preserved_corner_band = min(
        max(float(outer_corner_r) + corner_margin, p.wall_mm + 1.0),
        0.5 * min(float(outer_w), float(outer_h)) - 2.0,
    )
    min_z = max(
        cavity_front_z + p.front_wall_mm + float(p.large_other_side_cutout_front_margin_mm),
        1.0,
    )
    max_z = min(shell_depth - float(p.large_other_side_cutout_rear_margin_mm), shell_depth - 1.0)
    if max_z <= min_z + 1.0:
        return []

    x_span = max(float(outer_w) - 2.0 * preserved_corner_band, 1.0)
    y_span = max(float(outer_h) - 2.0 * preserved_corner_band, 1.0)
    z_span = max_z - min_z
    z_center = 0.5 * (min_z + max_z)

    cutouts = [
        {
            "id": f"large_surface_panel_y_{opposite_tripod_side}",
            "axis": "y",
            "side": opposite_tripod_side,
            "shape": "rect",
            "x": 0.0,
            "z": float(z_center),
            "w": float(x_span),
            "h": float(z_span),
            "bounds": {
                "min_x": float(-0.5 * x_span),
                "max_x": float(0.5 * x_span),
                "min_z": float(min_z),
                "max_z": float(max_z),
            },
        },
        {
            "id": "large_surface_panel_x_neg",
            "axis": "x",
            "side": "neg",
            "shape": "rect",
            "y": 0.0,
            "z": float(z_center),
            "w": float(y_span),
            "h": float(z_span),
            "bounds": {
                "min_y": float(-0.5 * y_span),
                "max_y": float(0.5 * y_span),
                "min_z": float(min_z),
                "max_z": float(max_z),
            },
        },
        {
            "id": "large_surface_panel_x_pos",
            "axis": "x",
            "side": "pos",
            "shape": "rect",
            "y": 0.0,
            "z": float(z_center),
            "w": float(y_span),
            "h": float(z_span),
            "bounds": {
                "min_y": float(-0.5 * y_span),
                "max_y": float(0.5 * y_span),
                "min_z": float(min_z),
                "max_z": float(max_z),
            },
        },
    ]
    for cutout in cutouts:
        cutout.update(
            {
                "cutout_kind": "large_surface_panel",
                "preserves_tripod_side": True,
                "tripod_side": tripod_side,
                "preserved_corner_radius_mm": float(outer_corner_r),
                "corner_margin_mm": float(corner_margin),
                "preserved_corner_band_mm": float(preserved_corner_band),
                "front_margin_mm": float(p.large_other_side_cutout_front_margin_mm),
                "rear_margin_mm": float(p.large_other_side_cutout_rear_margin_mm),
            }
        )
    return cutouts


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
    tmp_dir = Path("tmp")
    tmp_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=tmp_dir, suffix=".stl", delete=False) as tmp_file:
        tmp_stl = Path(tmp_file.name)
    mesh, housing, step_features = _load_step_as_mesh(p.step_path, tmp_stl, p)

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

    inner_depth = p.nominal_length_mm + 2.0 * p.clearance_mm + p.rear_extension_mm
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

    resolved_tripod_side = _resolve_tripanel_side(step_features["vents"], p.tripod_expected_side)
    vents_used = []
    merged_vent_cutouts_used = []
    large_side_cutouts_used = []
    tripod_used = None
    tripod_detected = step_features["tripod"]
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
            extracted_front_cutouts = _extract_front_cutouts(housing, p, sx, sy, zmax)
            if p.front_single_circle_cutout_only:
                front_cutouts_detected = [
                    {"x": 0.0, "y": p.lens_center_y_mm, "shape": "circle", "d": p.lens_diameter_mm}
                ]
            else:
                front_cutouts_detected = extracted_front_cutouts
                if not front_cutouts_detected:
                    front_cutouts_detected = [
                        {"x": 0.0, "y": p.lens_center_y_mm, "shape": "circle", "d": p.lens_diameter_mm}
                    ]
            with BuildSketch(Plane.XY.offset(-0.2)):
                for c in front_cutouts_detected:
                    with Locations((c["x"], c["y"])):
                        if c["shape"] == "circle":
                            Circle(c["d"] * 0.5)
                        elif c["shape"] == "roundrect":
                            _add_rounded_rectangle(c["w"], c["h"], c["r"])
                        elif c["shape"] == "slot":
                            SlotOverall(c["w"], c["h"])
                        else:
                            Rectangle(c["w"], c["h"])
            extrude(amount=p.front_wall_mm + 0.6, mode=Mode.SUBTRACT)
            front_cutouts_applied = list(front_cutouts_detected)

        # Hood is built separately after main BuildPart and unioned (visor clip).
        _build_maki_hood = (p.front_integrated and p.lens_hood_enabled
                            and p.lens_hood_depth_mm > 0.0)
        if _build_maki_hood:
            hood_gap = max(p.lens_hood_perimeter_margin_mm, 0.6)
            hood_roundrect_cutouts = [
                c for c in front_cutouts_detected
                if c.get("shape") == "roundrect"
            ]
            if hood_roundrect_cutouts:
                hood_ref = max(hood_roundrect_cutouts, key=lambda c: float(c.get("w", 0.0)) * float(c.get("h", 0.0)))
                _maki_hood_center_x = float(hood_ref.get("x", 0.0))
                _maki_hood_center_y = float(hood_ref.get("y", 0.0))
                _maki_hood_reference_cutout_w = float(hood_ref.get("w", outer_w - 2.0 * p.wall_mm))
                _maki_hood_reference_cutout_h = float(hood_ref.get("h", outer_h - 2.0 * p.wall_mm))
                _maki_hood_reference_cutout_r = float(hood_ref.get("r", outer_corner_r))
            else:
                _maki_hood_center_x = 0.0
                _maki_hood_center_y = 0.0
                _maki_hood_reference_cutout_w = max(outer_w - 2.0 * p.wall_mm, 2.0)
                _maki_hood_reference_cutout_h = max(outer_h - 2.0 * p.wall_mm, 2.0)
                _maki_hood_reference_cutout_r = max(outer_corner_r - p.wall_mm, 0.6)
            _maki_hood_gap = hood_gap
            _maki_hood_inner_w = _maki_hood_reference_cutout_w + 2.0 * hood_gap
            _maki_hood_inner_h = _maki_hood_reference_cutout_h + 2.0 * hood_gap
            _maki_hood_inner_r = _maki_hood_reference_cutout_r + hood_gap
            _maki_hood_outer_w = _maki_hood_inner_w + 2.0 * p.lens_hood_wall_mm
            _maki_hood_outer_h = _maki_hood_inner_h + 2.0 * p.lens_hood_wall_mm
            _maki_hood_outer_r = _maki_hood_inner_r + p.lens_hood_wall_mm

        # Cold shoe mount (ISO 518 female receptor) on top (Y+) side, near rear.
        cold_shoe_info = None
        if p.cold_shoe_enabled and not p.include_sun_shade:
            cs_z_center = shell_depth - p.cold_shoe_z_from_rear_mm
            cs_boss_l = p.cold_shoe_boss_length_mm
            cs_boss_w = p.cold_shoe_boss_width_mm
            cs_slot_w = p.cold_shoe_slot_width_mm
            cs_rail_oh = p.cold_shoe_rail_overhang_mm
            cs_rail_t = p.cold_shoe_rail_thickness_mm
            cs_slot_d = p.cold_shoe_slot_depth_mm
            cs_boss_h = cs_slot_d + cs_rail_t  # derive boss height so rail_thickness controls geometry
            cs_opening = cs_slot_w - 2.0 * cs_rail_oh

            # 1) Raised boss on top surface (Y+ face)
            with BuildSketch(Plane.XZ.offset(max_y)):
                with Locations((0.0, cs_z_center)):
                    Rectangle(cs_boss_w, cs_boss_l)
            extrude(amount=cs_boss_h)

            # 2) T-slot channel cut from top of boss.
            # Slot runs from boss front wall to past rear end for slide-in access.
            boss_top_y = max_y + cs_boss_h
            cs_front_z = cs_z_center - cs_boss_l * 0.5  # front wall of boss (closed stop)
            cs_slot_len = shell_depth + 0.2 - cs_front_z  # extend past rear edge
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5

            # 2a) Narrow opening through entire boss height (shoe stem passage)
            with BuildSketch(Plane.XZ.offset(boss_top_y + 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_opening, cs_slot_len)
            extrude(amount=-(cs_boss_h + 0.2), mode=Mode.SUBTRACT)

            # 2b) Wide floor pocket from boss bottom upward (flange pocket + rails at top)
            with BuildSketch(Plane.XZ.offset(max_y - 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_slot_w, cs_slot_len)
            extrude(amount=cs_slot_d + 0.1, mode=Mode.SUBTRACT)

            cold_shoe_info = {
                "enabled": True,
                "z_center_mm": float(cs_z_center),
                "y_base_mm": float(max_y),
                "boss_height_mm": float(cs_boss_h),
                "boss_length_mm": float(cs_boss_l),
                "boss_width_mm": float(cs_boss_w),
                "slot_width_mm": float(cs_slot_w),
                "rail_opening_mm": float(cs_opening),
                "rail_overhang_mm": float(cs_rail_oh),
                "rail_thickness_mm": float(cs_rail_t),
                "slot_depth_mm": float(cs_slot_d),
                "slide_in_from": "rear",
            }

        # Snap-latch flexure clips on inner X walls for rear cap retention.
        snap_clip_info = None
        if p.include_snap_clips:
            clip_tip_z = shell_depth - p.snap_clip_setback_mm
            clip_base_z = clip_tip_z - p.snap_clip_beam_length_mm
            clip_mid_z = 0.5 * (clip_base_z + clip_tip_z)
            clip_y = p.snap_clip_y_position_mm
            inner_half_x = 0.5 * inner_w
            beam_t = p.snap_clip_beam_thickness_mm
            catch_h = p.snap_clip_catch_height_mm

            for side_sign in (-1.0, 1.0):
                beam_x = side_sign * (inner_half_x - 0.5 * beam_t)
                with Locations((beam_x, clip_y, clip_mid_z)):
                    Box(beam_t, p.snap_clip_beam_width_mm, p.snap_clip_beam_length_mm)
                nub_x = side_sign * (inner_half_x - beam_t - 0.5 * catch_h)
                nub_z = clip_tip_z - 0.5 * p.snap_clip_catch_depth_mm
                with Locations((nub_x, clip_y, nub_z)):
                    Box(catch_h, p.snap_clip_beam_width_mm, p.snap_clip_catch_depth_mm)

            snap_clip_info = {
                "enabled": True,
                "clip_count": 2,
                "beam_length_mm": float(p.snap_clip_beam_length_mm),
                "beam_width_mm": float(p.snap_clip_beam_width_mm),
                "beam_thickness_mm": float(beam_t),
                "catch_height_mm": float(catch_h),
                "catch_depth_mm": float(p.snap_clip_catch_depth_mm),
                "clip_tip_z_mm": float(clip_tip_z),
                "setback_mm": float(p.snap_clip_setback_mm),
            }

        # Detent pockets: 4 discrete pockets (one per wall center) for snap-in cap retention.
        # Oversized vs cap bumps for insertion tolerance.
        friction_ridge_info = None
        if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
            fr_z = shell_depth - p.friction_ridge_setback_mm
            fr_h = p.friction_ridge_height_mm  # pocket depth (radial)
            fr_w = p.friction_ridge_width_mm   # pocket extent along Z
            half_iw = 0.5 * inner_w
            half_ih = 0.5 * inner_h
            pocket_w = 8.0   # pocket width (tangential)
            pocket_d = 1.5   # pocket radial depth into wall
            pocket_z = 4.0   # pocket extent along Z
            # 2 pockets on X walls (at Y=0), 2 on Y walls (at X=0)
            # Position so pocket extends INTO the wall from inner surface outward.
            for sx_sign in (-1.0, 1.0):
                px = sx_sign * (half_iw + 0.5 * pocket_d)
                with Locations((px, 0.0, fr_z)):
                    Box(pocket_d, pocket_w, pocket_z, mode=Mode.SUBTRACT)
            for sy_sign in (-1.0, 1.0):
                py = sy_sign * (half_ih + 0.5 * pocket_d)
                with Locations((0.0, py, fr_z)):
                    Box(pocket_w, pocket_d, pocket_z, mode=Mode.SUBTRACT)
            friction_ridge_info = {
                "enabled": True,
                "type": "detent_pockets",
                "pocket_count": 4,
                "pocket_width_mm": float(pocket_w),
                "pocket_depth_mm": float(pocket_d),
                "pocket_z_mm": float(pocket_z),
                "setback_mm": float(p.friction_ridge_setback_mm),
                "z_mm": float(fr_z),
            }

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
                cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + 3.0)
                z_centers_shifted = [
                    min(max(float(z_c) + p.tripanel_vent_z_shift_mm, 1.0), shell_depth - 1.0)
                    for z_c in vent_pattern["z_centers"]
                ]
                merged_cutout = _derive_merged_tripanel_cutout(
                    vent_pattern,
                    z_centers_shifted,
                    tripod_detected,
                    map_x,
                    map_z,
                    sx,
                    cavity_front_z + p.clearance_mm + p.tripod_center_from_front_mm,
                    p,
                )
                if merged_cutout is not None:
                    on_neg = merged_cutout["side"] == "neg"
                    y_face = min_y - 0.2 if on_neg else max_y + 0.2
                    with BuildSketch(Plane.XZ.offset(y_face)):
                        with Locations((merged_cutout["x"], merged_cutout["z"])):
                            Rectangle(merged_cutout["w"], merged_cutout["h"])
                    extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                    merged_cutout["y"] = float(y_face)
                    merged_cutout["cut_depth_mm"] = float(cut_depth)
                    merged_vent_cutouts_used.append(merged_cutout)

                for large_cutout in _derive_large_other_side_cutouts(
                    resolved_tripod_side,
                    outer_w,
                    outer_h,
                    outer_corner_r,
                    shell_depth,
                    cavity_front_z,
                    p,
                ):
                    axis = large_cutout["axis"]
                    side = large_cutout["side"]
                    on_neg = side == "neg"
                    if axis == "y":
                        y_face = min_y - 0.2 if on_neg else max_y + 0.2
                        with BuildSketch(Plane.XZ.offset(y_face)):
                            with Locations((large_cutout["x"], large_cutout["z"])):
                                Rectangle(large_cutout["w"], large_cutout["h"])
                        extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                        large_cutout["y"] = float(y_face)
                    elif axis == "x":
                        x_face = min_x - 0.2 if on_neg else max_x + 0.2
                        with BuildSketch(Plane.YZ.offset(x_face)):
                            with Locations((large_cutout["y"], large_cutout["z"])):
                                Rectangle(large_cutout["w"], large_cutout["h"])
                        extrude(amount=cut_depth if on_neg else -cut_depth, mode=Mode.SUBTRACT)
                        large_cutout["x"] = float(x_face)
                    else:
                        continue
                    large_cutout["cut_depth_mm"] = float(cut_depth)
                    large_side_cutouts_used.append(large_cutout)

                for panel_idx, panel in enumerate(vent_pattern["panels"]):
                    for z_shifted in z_centers_shifted:
                        if panel["axis"] == "y":
                            on_neg = panel["side"] == "neg"
                            y_face = min_y - 0.2 if on_neg else max_y + 0.2
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
                                    "vent_family": "tripanel",
                                    "cutout_kind": "merged_panel",
                                    "merged_cutout_id": merged_cutout["id"] if merged_cutout else None,
                                }
                            )
                        else:
                            on_neg = panel["side"] == "neg"
                            x_face = panel["x"] - 0.2 if on_neg else panel["x"] + 0.2
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
                                    "vent_family": "tripanel",
                                    "cutout_kind": "merged_panel",
                                    "merged_cutout_id": merged_cutout["id"] if merged_cutout else None,
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
                    side_trio_z = (
                        (shell_depth + cavity_front_z) - trio["z_center"]
                        if p.side_trio_flip_end
                        else trio["z_center"]
                    )
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
                                    "cutout_kind": "large_surface_panel"
                                    if p.include_large_other_side_cutouts
                                    else "individual_slot",
                                    "large_cutout_id": f"large_surface_panel_x_{side}"
                                    if p.include_large_other_side_cutouts
                                    else None,
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
                        cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + 3.0)
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
                        cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + 3.0)
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

            t = tripod_detected
            if t is not None:
                x_c = map_x(t["x"])
                z_c = map_z(t["z"])
                d = max(2.0 * t["r"] * sx + p.tripod_cutout_extra_mm, 2.0)
                detected_on_neg = t["side"] == "neg"
                expected_on_neg = resolved_tripod_side == "neg"
                on_neg = expected_on_neg
                y_face = (
                    min_y - 0.2
                    if on_neg
                    else max_y + 0.2
                )
                tripod_from_step_front_mm = float(zmax - float(t["z"]))

                cut_depth = p.wall_mm + 3.0
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
                    "rect_x_mm": float(p.tripod_rect_short_mm if p.tripod_rect_long_along_z else p.tripod_rect_long_mm) if p.tripod_use_rect_cutout else None,
                    "rect_z_mm": float(p.tripod_rect_long_mm if p.tripod_rect_long_along_z else p.tripod_rect_short_mm) if p.tripod_use_rect_cutout else None,
                    "source": step_features.get("tripod_source", "unknown"),
                    "detected_radius_raw_mm": float(t.get("r", d * 0.5)),
                    "from_camera_front_mm_raw": tripod_from_step_front_mm,
                    "from_case_front_mm_mapped": float(z_c),
                }

        # Fallback if STEP-derived features were unavailable.
        if not vents_used:
            vent_z0 = cavity_front_z + p.clearance_mm + p.vent_start_from_front_mm
            fallback_on_neg = resolved_tripod_side == "neg"
            y_face = min_y - 0.2 if fallback_on_neg else max_y + 0.2
            cut_depth = max(p.vent_cut_depth_mm, p.wall_mm + 3.0)
            for i in range(p.vent_count):
                z = min(max(vent_z0 + i * p.vent_pitch_mm + p.tripanel_vent_z_shift_mm, 1.0), shell_depth - 1.0)
                with BuildSketch(Plane.XZ.offset(y_face)):
                    with Locations((0.0, z)):
                        SlotOverall(p.vent_slot_w_mm, p.vent_slot_h_mm)
                extrude(amount=cut_depth if fallback_on_neg else -cut_depth, mode=Mode.SUBTRACT)

        if tripod_used is None:
            tripod_z = cavity_front_z + p.clearance_mm + p.tripod_center_from_front_mm
            if p.tripod_use_rect_cutout:
                tripod_z += p.tripod_rect_z_shift_mm
            fallback_on_neg = resolved_tripod_side == "neg"

            fallback_y = (
                min_y - 0.2
                if fallback_on_neg
                else max_y + 0.2
            )
            fallback_depth = p.wall_mm + 3.0
            with BuildSketch(Plane.XZ.offset(fallback_y)):
                with Locations((0.0, tripod_z)):
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
                "z": tripod_z,
                "diameter": p.tripod_hole_diameter_mm,
            }

    sleeve = _largest_solid(sleeve_bp.part)

    # Build a perimeter-following front visor that stays near the shell edge so
    # it doesn't intrude into the MAKI's wide-angle view.
    if _build_maki_hood:
        half_hood_outer_h = 0.5 * _maki_hood_outer_h
        half_hood_inner_h = 0.5 * _maki_hood_inner_h
        bottom_panel_h = max(half_hood_outer_h - half_hood_inner_h + 1.0, p.lens_hood_wall_mm + 0.8)
        with BuildPart() as hood_bp:
            with BuildSketch(Plane.XY):
                _add_rounded_rectangle(_maki_hood_outer_w, _maki_hood_outer_h, _maki_hood_outer_r)
            extrude(amount=p.lens_hood_depth_mm)

            with BuildSketch(Plane.XY.offset(-0.1)):
                _add_rounded_rectangle(_maki_hood_inner_w, _maki_hood_inner_h, _maki_hood_inner_r)
            extrude(amount=p.lens_hood_depth_mm + 0.2, mode=Mode.SUBTRACT)

            # Open the bottom panel while keeping the top and side visor walls.
            with Locations((0.0, half_hood_inner_h + 0.5 * bottom_panel_h, 0.5 * p.lens_hood_depth_mm)):
                Box(_maki_hood_outer_w + 2.0, bottom_panel_h + 0.2, p.lens_hood_depth_mm + 1.0, mode=Mode.SUBTRACT)

        hood_solid = hood_bp.part
        for fillet_r in (2.0, 1.5, 1.0, 0.5):
            try:
                hood_solid = fillet(hood_solid.edges(), fillet_r)
                break
            except Exception:
                continue
        try:
            sleeve = _largest_solid(sleeve + hood_solid)
        except Exception:
            pass

    sleeve, sleeve_fillet_y = _apply_axis_fillet(sleeve, Axis.Y, (0.8, 0.6, 0.45, 0.3, 0.2))

    sun_shade_info = None
    if p.include_sun_shade:
        standoff = p.sun_shade_standoff_mm
        shade_w = p.sun_shade_wall_mm
        post_w = p.sun_shade_post_width_mm
        vent_relief_margin = max(p.sun_shade_vent_relief_mm, 0.2)

        shade_inner_w = outer_w + 2.0 * standoff
        shade_inner_h = outer_h + 2.0 * standoff
        shade_outer_w = shade_inner_w + 2.0 * shade_w
        shade_outer_h = shade_inner_h + 2.0 * shade_w
        shade_inner_r = min(
            outer_corner_r + standoff,
            0.49 * min(shade_inner_w, shade_inner_h),
        )
        shade_outer_r = min(
            shade_inner_r + shade_w,
            0.49 * min(shade_outer_w, shade_outer_h),
        )

        half_outer_w = 0.5 * outer_w
        half_outer_h = 0.5 * outer_h
        half_shade_outer_w = 0.5 * shade_outer_w
        half_shade_outer_h = 0.5 * shade_outer_h
        half_shade_inner_w = 0.5 * shade_inner_w
        shade_mid_z = 0.5 * shell_depth

        flat_extent_half = max(half_outer_w - outer_corner_r, 0.0)
        rib_offset = max(flat_extent_half - 2.0, 0.0)
        rib_radial = standoff + 2.0
        side_rib_x_centers = {
            "neg": -(half_outer_w + 0.5 * standoff),
            "pos": half_outer_w + 0.5 * standoff,
        }
        side_rib_y_centers = (-rib_offset, rib_offset)
        shade_support_relief_count = 0

        side_drop = min(
            max(p.sun_shade_side_drop_ratio * shade_outer_h, 0.25 * shade_outer_h),
            shade_outer_h - 2.0 * shade_w - 2.0,
        )
        side_panel_lower_y = half_shade_outer_h - side_drop
        side_trim_h = max(side_panel_lower_y + half_shade_outer_h, 0.0)
        side_trim_x_depth = max(shade_outer_w - shade_inner_w + 2.0, shade_w + 1.0)
        side_trim_x_center = 0.5 * (half_shade_outer_w + half_shade_inner_w)
        side_trim_corner_r = _safe_fillet_radius(
            side_trim_x_depth,
            side_trim_h + 0.2,
            p.sun_shade_side_skirt_corner_r_mm,
        ) if side_trim_h > 0.5 else 0.0
        lower_side_support_h = 0.0
        if p.sun_shade_side_support_height_mm > 0.0:
            lower_side_support_h = max(min(p.sun_shade_side_support_height_mm, side_drop - 1.0), 1.5)
            lower_side_support_y = -(side_panel_lower_y + 0.5 * lower_side_support_h)
            lower_side_support_x = 0.5 * (half_outer_w + half_shade_outer_w)
            lower_side_support_x_span = max(half_shade_outer_w - half_outer_w, shade_w + 0.8)

        try:
            with BuildPart() as shade_shell_bp:
                with BuildSketch(Plane.XY):
                    Rectangle(shade_outer_w, shade_outer_h)
                    fillet(vertices(), shade_outer_r)
                extrude(amount=shell_depth)
                with BuildSketch(Plane.XY.offset(-0.1)):
                    Rectangle(shade_inner_w, shade_inner_h)
                    fillet(vertices(), shade_inner_r)
                extrude(amount=shell_depth + 0.2, mode=Mode.SUBTRACT)

                # Keep the roof on the -Y side and open the underside (+Y).
                bottom_cut_h = half_shade_outer_h - half_outer_h + 1.0
                with Locations((0.0, half_outer_h + 0.5 * bottom_cut_h, shade_mid_z)):
                    Box(shade_outer_w + 2.0, bottom_cut_h + 0.2, shell_depth + 2.0, mode=Mode.SUBTRACT)

                # Trim the lower sections of the side walls on the +Y side so the
                # shade behaves like a top visor wrap, not a full enclosure.
                if side_trim_h > 0.5:
                    for sx_sign in (-1.0, 1.0):
                        with BuildSketch(Plane.XY.offset(-1.0)):
                            with Locations((sx_sign * side_trim_x_center, half_shade_outer_h - 0.5 * side_trim_h)):
                                Rectangle(side_trim_x_depth, side_trim_h + 0.2)
                                fillet(vertices(), side_trim_corner_r)
                        extrude(amount=shell_depth + 2.0, mode=Mode.SUBTRACT)

            with BuildPart() as shade_ribs_bp:
                for ry in (-1.0, 1.0):
                    with Locations((half_outer_w + 0.5 * standoff, ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shell_depth)
                for ry in (-1.0, 1.0):
                    with Locations((-(half_outer_w + 0.5 * standoff), ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shell_depth)
                for rx in (-1.0, 1.0):
                    with Locations((rx * rib_offset, -(half_outer_h + 0.5 * standoff), shade_mid_z)):
                        Box(post_w, rib_radial, shell_depth)

                if lower_side_support_h > 0.0:
                    for sx_sign in (-1.0, 1.0):
                        with Locations((sx_sign * lower_side_support_x, lower_side_support_y, shade_mid_z)):
                            Box(lower_side_support_x_span, lower_side_support_h, shell_depth)

                # Cut relief notches into the shade support ribs anywhere they would
                # otherwise bridge directly across existing vent openings.
                for vent in vents_used:
                    slot_t = float(vent.get("slot_t", vent.get("slot_w", 0.0)))
                    slot_z = float(vent.get("slot_z", vent.get("slot_h", 0.0)))
                    if slot_t <= 0.0 or slot_z <= 0.0:
                        continue

                    notch_z = slot_z + vent_relief_margin

                    if vent.get("axis") == "x":
                        rib_x = side_rib_x_centers.get(vent.get("side"))
                        if rib_x is None:
                            continue
                        notch_x = rib_radial + vent_relief_margin
                        notch_y = slot_t + vent_relief_margin
                        for rib_y in side_rib_y_centers:
                            if abs(float(vent.get("y", 0.0)) - rib_y) > 0.5 * (slot_t + post_w) + 0.3:
                                continue
                            with Locations((rib_x, float(vent.get("y", 0.0)), float(vent.get("z", shade_mid_z)))):
                                Box(notch_x, notch_y, notch_z, mode=Mode.SUBTRACT)
                            shade_support_relief_count += 1

            shade_solid = shade_shell_bp.part

            lower_edge_cut_y = half_shade_outer_h - side_trim_h - 0.1

            def fillet_side_skirt_edge_band(shape, min_abs_x: float, max_abs_x: float, preferred_r: float):
                lower_edges = []
                for edge in shape.edges():
                    bb = edge.bounding_box()
                    center = bb.center()
                    size = bb.size
                    is_lower_skirt_edge = (
                        abs(center.Y - lower_edge_cut_y) <= 0.5
                        and min_abs_x <= abs(center.X) <= max_abs_x
                        and size.Z >= 0.75 * shell_depth
                        and size.X <= 0.45
                        and size.Y <= 0.6
                    )
                    if is_lower_skirt_edge:
                        lower_edges.append(edge)

                if lower_edges and preferred_r > 0.0:
                    for fillet_r in (preferred_r, 1.5, 1.0, 0.75, 0.5, 0.3):
                        if fillet_r > preferred_r:
                            continue
                        try:
                            return fillet(lower_edges, fillet_r), fillet_r, len(lower_edges)
                        except Exception:
                            continue
                return shape, 0.0, len(lower_edges)

            shade_solid, side_skirt_outer_edge_fillet_applied, side_skirt_outer_edge_count = fillet_side_skirt_edge_band(
                shade_solid,
                half_shade_inner_w + 0.5,
                half_shade_outer_w + 0.5,
                p.sun_shade_side_skirt_corner_r_mm,
            )
            shade_solid, side_skirt_inner_edge_fillet_applied, side_skirt_inner_edge_count = fillet_side_skirt_edge_band(
                shade_solid,
                max(half_shade_inner_w - 1.2, 0.0),
                half_shade_inner_w + 0.3,
                p.sun_shade_side_skirt_inner_edge_fillet_mm,
            )

            for rib_solid in shade_ribs_bp.part.solids():
                shade_solid = _largest_solid(shade_solid + rib_solid)
            sleeve = _largest_solid(sleeve + shade_solid)

            if p.cold_shoe_enabled:
                cs_z_center = shell_depth - p.cold_shoe_z_from_rear_mm
                cs_boss_l = p.cold_shoe_boss_length_mm
                cs_boss_w = p.cold_shoe_boss_width_mm
                cs_slot_w = p.cold_shoe_slot_width_mm
                cs_rail_oh = p.cold_shoe_rail_overhang_mm
                cs_rail_t = p.cold_shoe_rail_thickness_mm
                cs_slot_d = p.cold_shoe_slot_depth_mm
                cs_boss_h = cs_slot_d + cs_rail_t
                cs_opening = cs_slot_w - 2.0 * cs_rail_oh
                boss_overlap = min(1.0, shade_w - 0.2)

                with BuildPart() as boss_bp:
                    with BuildSketch(Plane.XZ.offset(half_shade_outer_h - boss_overlap)):
                        with Locations((0.0, cs_z_center)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=cs_boss_h + boss_overlap)
                sleeve = _largest_solid(sleeve + boss_bp.part)

                with BuildPart() as left_boss_bp:
                    with BuildSketch(Plane.YZ.offset(-half_shade_outer_w + boss_overlap)):
                        with Locations((0.0, cs_z_center)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=-(cs_boss_h + boss_overlap))
                sleeve = _largest_solid(sleeve + left_boss_bp.part)

                with BuildPart() as right_boss_bp:
                    with BuildSketch(Plane.YZ.offset(half_shade_outer_w - boss_overlap)):
                        with Locations((0.0, cs_z_center)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=cs_boss_h + boss_overlap)
                sleeve = _largest_solid(sleeve + right_boss_bp.part)

                cs_front_z = cs_z_center - 0.5 * cs_boss_l
                cs_slot_len = shell_depth + 0.2 - cs_front_z
                cs_slot_mid_z = cs_front_z + 0.5 * cs_slot_len
                boss_top_y = half_shade_outer_h + cs_boss_h

                with BuildPart() as stem_cut_bp:
                    with BuildSketch(Plane.XZ.offset(boss_top_y + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=-(cs_boss_h + 0.2))
                sleeve = _largest_solid(sleeve - stem_cut_bp.part)

                with BuildPart() as floor_cut_bp:
                    with BuildSketch(Plane.XZ.offset(half_shade_outer_h - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=cs_slot_d + 0.2)
                sleeve = _largest_solid(sleeve - floor_cut_bp.part)

                left_outer_x = -half_shade_outer_w - cs_boss_h
                with BuildPart() as left_stem_cut_bp:
                    with BuildSketch(Plane.YZ.offset(left_outer_x - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=cs_boss_h + 0.2)
                sleeve = _largest_solid(sleeve - left_stem_cut_bp.part)

                with BuildPart() as left_floor_cut_bp:
                    with BuildSketch(Plane.YZ.offset(-half_shade_outer_w + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=-(cs_slot_d + 0.2))
                sleeve = _largest_solid(sleeve - left_floor_cut_bp.part)

                right_outer_x = half_shade_outer_w + cs_boss_h
                with BuildPart() as right_stem_cut_bp:
                    with BuildSketch(Plane.YZ.offset(right_outer_x + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=-(cs_boss_h + 0.2))
                sleeve = _largest_solid(sleeve - right_stem_cut_bp.part)

                with BuildPart() as right_floor_cut_bp:
                    with BuildSketch(Plane.YZ.offset(half_shade_outer_w - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=cs_slot_d + 0.2)
                sleeve = _largest_solid(sleeve - right_floor_cut_bp.part)

                cold_shoe_info = {
                    "enabled": True,
                    "locations": ["top", "left", "right"],
                    "pad_z_center_mm": float(cs_z_center),
                    "boss_height_mm": float(cs_boss_h),
                    "slot_width_mm": float(cs_slot_w),
                    "rail_opening_mm": float(cs_opening),
                    "slide_in_from": "rear",
                    "mounted_on": "shade_hood",
                    "style": "open_dual_rail",
                }
                sun_shade_info = sun_shade_info or {}
                sun_shade_info["cold_shoe_count"] = 3

            sun_shade_info = {
                "enabled": True,
                "standoff_mm": float(standoff),
                "wall_mm": float(shade_w),
                "shade_outer_w_mm": float(shade_outer_w),
                "shade_outer_h_mm": float(shade_outer_h),
                "post_width_mm": float(post_w),
                "side_drop_ratio": float(p.sun_shade_side_drop_ratio),
                "side_support_height_mm": float(lower_side_support_h),
                "vent_relief_mm": float(vent_relief_margin),
                "vent_relief_count": int(shade_support_relief_count),
                "side_skirt_corner_r_mm": float(side_trim_corner_r),
                "side_skirt_lower_trim_y_mm": float(half_shade_outer_h - side_trim_h - 0.1),
                "side_skirt_outer_edge_fillet_mm": float(side_skirt_outer_edge_fillet_applied),
                "side_skirt_outer_edge_count": int(side_skirt_outer_edge_count),
                "side_skirt_inner_edge_fillet_mm": float(side_skirt_inner_edge_fillet_applied),
                "side_skirt_inner_edge_count": int(side_skirt_inner_edge_count),
                "cold_shoe_count": int(3 if p.cold_shoe_enabled else 0),
                "coverage": "top + partial left/right sides (open bottom)",
            }
        except Exception as exc:
            print(f"  WARNING: sun shade failed to build: {exc}")

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
            "merged_vent_cutouts_applied": len(merged_vent_cutouts_used),
            "merged_vent_cutouts_applied_entries": merged_vent_cutouts_used,
            "large_side_cutouts_applied": len(large_side_cutouts_used),
            "large_side_cutouts_applied_entries": large_side_cutouts_used,
            "tripanel_vent_layout_enforced": p.enforce_tripanel_vent_layout,
            "tripod_detected": tripod_detected is not None,
            "tripod_source": step_features.get("tripod_source", "unknown"),
            "tripod_cyl_candidate_count": step_features.get("tripod_cyl_candidate_count", 0),
            "tripod_edge_candidate_count": step_features.get("tripod_edge_candidate_count", 0),
            "tripod_detected_raw": tripod_detected,
            "tripod_applied": tripod_used,
        },
        "derived": {
            "inner_depth_mm": float(inner_depth),
            "shell_depth_mm": float(shell_depth),
            "rear_extension_mm": float(p.rear_extension_mm),
            "open_sleeve": not bool(p.front_integrated),
            "front_integrated": bool(p.front_integrated),
            "lens_hood": {
                "enabled": bool(p.front_integrated and p.lens_hood_enabled),
                "type": "perimeter_visor",
                "depth_mm": float(max(p.lens_hood_depth_mm, 0.0)),
                "wall_mm": float(p.lens_hood_wall_mm),
                "aperture_gap_mm": float(_maki_hood_gap) if _build_maki_hood else float(p.lens_hood_perimeter_margin_mm),
                "center_x_mm": float(_maki_hood_center_x) if _build_maki_hood else float(0.0),
                "center_y_mm": float(_maki_hood_center_y) if _build_maki_hood else float(0.0),
                "reference_cutout_w_mm": float(_maki_hood_reference_cutout_w) if _build_maki_hood else float(max(outer_w - 2.0 * p.wall_mm, 2.0)),
                "reference_cutout_h_mm": float(_maki_hood_reference_cutout_h) if _build_maki_hood else float(max(outer_h - 2.0 * p.wall_mm, 2.0)),
                "reference_cutout_corner_r_mm": float(_maki_hood_reference_cutout_r) if _build_maki_hood else float(max(outer_corner_r - p.wall_mm, 0.6)),
                "side": "neg" if resolved_tripod_side == "neg" else "pos",
            },
            "cavity_front_z_mm": float(cavity_front_z),
            "inner_w_mm": float(inner_w),
            "inner_h_mm": float(inner_h),
            "outer_w_mm": float(outer_w),
            "outer_h_mm": float(outer_h),
            "end_clearance_each_mm": float(p.clearance_mm),
            "front_cutouts": {
                "enabled": bool(p.include_front_cutouts),
                "single_circle_only": bool(p.front_single_circle_cutout_only),
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
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "sun_shade": sun_shade_info if sun_shade_info else {"enabled": False},
            "snap_clips": snap_clip_info if snap_clip_info else {"enabled": False},
            "friction_ridge": friction_ridge_info if friction_ridge_info else {"enabled": False},
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
    parser.add_argument("--rear-extension", type=float, default=None, help="Extra rear-only depth added to the ASA shell (mm)")
    parser.add_argument("--wall", type=float, default=None, help="Wall thickness (mm)")
    parser.add_argument("--open-front", action="store_true", help="Legacy mode: keep front end fully open.")
    parser.add_argument("--no-front-cutouts", action="store_true", help="Disable front-wall cutouts in integrated-front mode.")
    parser.add_argument("--lens-d", type=float, default=None, help="Front lens opening diameter (mm)")
    parser.add_argument("--no-lens-hood", action="store_true", help="Disable integrated top-front glare hood.")
    parser.add_argument("--lens-hood-depth", type=float, default=None, help="Integrated glare hood projection depth (mm)")
    parser.add_argument("--no-sun-shade", action="store_true", help="Disable top-and-side floating sun shade")

    parser.add_argument("--tripod-z", type=float, default=None, help="Fallback tripod opening center from front (mm)")
    parser.add_argument("--no-cold-shoe", action="store_true", help="Disable cold shoe mount on top rear")
    parser.add_argument("--no-snap-clips", action="store_true", help="Disable snap-latch flexure clips")
    parser.add_argument("--no-friction-ridge", action="store_true", help="Disable continuous friction ridge")
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None, help="Cold shoe center distance from rear edge (mm)")
    parser.add_argument("--no-step-side-features", action="store_true", help="Disable STEP-derived side vents/tripod hole")
    parser.add_argument("--tripod-rect", action="store_true", help="Use rectangular cutout instead of circular tripod hole")
    parser.add_argument("--tripod-rect-along-width", action="store_true", help="Orient long side of rect along case width (default: along length)")
    parser.add_argument("--out-suffix", type=str, default="", help="Suffix appended to output filenames (e.g. '_orient_a')")
    args = parser.parse_args()

    params = MakiCaseParams(step_path=args.step)
    if args.clearance is not None:
        params.clearance_mm = args.clearance
    if args.rear_extension is not None:
        params.rear_extension_mm = args.rear_extension
    if args.wall is not None:
        params.wall_mm = args.wall
    if args.open_front:
        params.front_integrated = False
    if args.no_front_cutouts:
        params.include_front_cutouts = False
    if args.lens_d is not None:
        params.lens_diameter_mm = args.lens_d
    if args.no_lens_hood:
        params.lens_hood_enabled = False
    if args.lens_hood_depth is not None:
        params.lens_hood_depth_mm = max(float(args.lens_hood_depth), 0.0)
    if args.no_sun_shade:
        params.include_sun_shade = False
    if args.tripod_z is not None:
        params.tripod_center_from_front_mm = args.tripod_z
    if args.no_cold_shoe:
        params.cold_shoe_enabled = False
    if args.no_snap_clips:
        params.include_snap_clips = False
    if args.no_friction_ridge:
        params.include_friction_ridge = False
    if args.cold_shoe_z_from_rear is not None:
        params.cold_shoe_z_from_rear_mm = float(args.cold_shoe_z_from_rear)
    if args.no_step_side_features:
        params.use_step_side_features = False
    if args.tripod_rect:
        params.tripod_use_rect_cutout = True
    if args.tripod_rect_along_width:
        params.tripod_rect_long_along_z = False

    sleeve, report = build_case(params)

    suffix = args.out_suffix
    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    out_step = args.out / f"maki_live_case_sleeve{suffix}.step"
    out_json = reports_dir / f"maki_live_case_report{suffix}.json"
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
