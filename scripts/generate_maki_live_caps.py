#!/usr/bin/env python3
"""Generate front and rear caps for MAKI Live sleeve with STEP-derived cutouts."""

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
    BuildPart,
    BuildSketch,
    Circle,
    GeomType,
    Locations,
    Mode,
    Plane,
    Rectangle,
    SlotOverall,
    export_stl,
    export_step,
    extrude,
    fillet,
    import_step,
    vertices,
)
from scipy.spatial import ConvexHull
from shapely import affinity
from shapely.geometry import Point, Polygon


@dataclass
class MakiCapParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # Match ASA sleeve nominals and fit
    nominal_width_mm: float = 56.99
    nominal_height_mm: float = 56.99
    nominal_length_mm: float = 120.32
    sleeve_clearance_mm: float = 2.3
    sleeve_wall_mm: float = 3.0

    # Cap geometry
    cap_thickness_mm: float = 3.0
    plug_depth_mm: float = 1.8
    plug_clearance_mm: float = 0.28
    cap_plate_extra_mm: float = 0.0
    cutout_extra_mm: float = 1.5
    front_recess_depth_mm: float = 1.2
    front_recess_inset_mm: float = 3.0

    # End-hole extraction windows from source STEP (mm from each end)
    front_window_mm: float = 16.0
    rear_window_mm: float = 16.0
    end_plane_tol_mm: float = 3.0

    # Ignore very large loops that would remove almost entire cap
    max_cutout_ratio_xy: float = 0.80
    include_major_front_aperture: bool = True
    front_major_aperture_shrink_mm: float = 2.0
    rear_min_cutout_dim_mm: float = 4.0
    rear_min_cutout_area_mm2: float = 8.0

    # Processing
    section_z_ratio: float = 0.50
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
    solids = list(shape.solids())
    tmp_stl.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(tmp_stl))
    mesh = trimesh.load(str(tmp_stl), force="mesh")
    try:
        tmp_stl.unlink(missing_ok=True)
    except Exception:
        pass
    return mesh, solids


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


def _cutout_metrics(entry: dict) -> tuple[float, float]:
    if entry["shape"] == "circle":
        d = float(entry["d"])
        return d, math.pi * (0.5 * d) ** 2
    w = float(entry.get("w", 0.0))
    h = float(entry.get("h", 0.0))
    return max(w, h), w * h


def _extract_end_cutouts(solids, p: MakiCapParams, sx: float, sy: float, zmin: float, zmax: float):
    front_plane_z = None
    rear_plane_z = None
    for solid in solids:
        for f in solid.faces():
            if f.geom_type != GeomType.PLANE:
                continue
            try:
                n = f.normal_at()
                c = f.center()
            except Exception:
                continue
            if abs(n.Z) < 0.92:
                continue
            if n.Z > 0:
                front_plane_z = c.Z if front_plane_z is None else max(front_plane_z, c.Z)
            else:
                rear_plane_z = c.Z if rear_plane_z is None else min(rear_plane_z, c.Z)

    front = []
    rear = []
    for solid in solids:
        for f in solid.faces():
            wires = f.wires()
            if len(wires) <= 1:
                continue
            try:
                n = f.normal_at()
            except Exception:
                continue
            if abs(n.Z) < 0.92:
                continue
            fc = f.center()
            on_front_plane = front_plane_z is not None and abs(fc.Z - front_plane_z) <= p.end_plane_tol_mm
            on_rear_plane = rear_plane_z is not None and abs(fc.Z - rear_plane_z) <= p.end_plane_tol_mm
            for w in wires[1:]:
                bb = w.bounding_box()
                xlen = float(bb.size.X)
                ylen = float(bb.size.Y)
                zmid = float((bb.min.Z + bb.max.Z) * 0.5)
                xmid = float((bb.min.X + bb.max.X) * 0.5)
                ymid = float((bb.min.Y + bb.max.Y) * 0.5)

                too_large = (
                    xlen > p.nominal_width_mm * p.max_cutout_ratio_xy
                    and ylen > p.nominal_height_mm * p.max_cutout_ratio_xy
                )

                if too_large:
                    # Keep the main front camera aperture if present, skip other giant loops.
                    if not (p.include_major_front_aperture and n.Z > 0 and zmid > (zmax - p.front_window_mm)):
                        continue
                    # Major aperture behaves like a large circular opening.
                    d_maj = (
                        (xlen + ylen) * 0.5 * (sx + sy) * 0.5
                        + p.cutout_extra_mm
                        - p.front_major_aperture_shrink_mm
                    )
                    d_maj = max(d_maj, 1.0)
                    entry = {
                        "x": xmid * sx,
                        "y": ymid * sy,
                        "shape": "circle",
                        "d": d_maj,
                    }
                    front.append(entry)
                    continue

                shape = _classify_cutout(xlen, ylen)
                if shape is None:
                    continue
                entry = {
                    "x": xmid * sx,
                    "y": ymid * sy,
                    "shape": shape["shape"],
                }
                if shape["shape"] == "circle":
                    entry["d"] = shape["d"] * (sx + sy) * 0.5 + p.cutout_extra_mm
                else:
                    entry["w"] = max(shape["w"] * sx + p.cutout_extra_mm, 0.8)
                    entry["h"] = max(shape["h"] * sy + p.cutout_extra_mm, 0.8)

                if n.Z > 0 and (on_front_plane or zmid > (zmax - p.front_window_mm)):
                    front.append(entry)
                if n.Z < 0 and (on_rear_plane or zmid < (zmin + p.rear_window_mm)):
                    max_dim, area = _cutout_metrics(entry)
                    # Filter tiny rear fastener holes so rear cap keeps actual port openings.
                    if max_dim < p.rear_min_cutout_dim_mm or area < p.rear_min_cutout_area_mm2:
                        continue
                    rear.append(entry)

    # Deduplicate by rounded center+size
    def dedupe(items):
        kept = {}
        for c in items:
            key = (c["shape"], round(c["x"], 1), round(c["y"], 1))
            cur_max_dim, cur_area = _cutout_metrics(c)
            prev = kept.get(key)
            if prev is None:
                kept[key] = c
                continue
            _, prev_area = _cutout_metrics(prev)
            if cur_area > prev_area:
                kept[key] = c
        return list(kept.values())

    return dedupe(front), dedupe(rear), {
        "front_plane_z_mm": float(front_plane_z) if front_plane_z is not None else None,
        "rear_plane_z_mm": float(rear_plane_z) if rear_plane_z is not None else None,
        "end_plane_tol_mm": float(p.end_plane_tol_mm),
    }


def _build_cap(
    plate_w: float,
    plate_h: float,
    plate_r: float,
    plug_w: float,
    plug_h: float,
    plug_r: float,
    p: MakiCapParams,
    cutouts: list[dict],
):
    with BuildPart() as cap_bp:
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(plate_w, plate_h, plate_r)
        extrude(amount=p.cap_thickness_mm)

        with BuildSketch(Plane.XY.offset(p.cap_thickness_mm)):
            _add_rounded_rectangle(plug_w, plug_h, plug_r)
        extrude(amount=p.plug_depth_mm)

        cut_depth = p.cap_thickness_mm + p.plug_depth_mm + 1.5
        with BuildSketch(Plane.XY.offset(-1.0)):
            for c in cutouts:
                with Locations((c["x"], c["y"])):
                    if c["shape"] == "circle":
                        Circle(c["d"] * 0.5)
                    elif c["shape"] == "slot":
                        # Rear I/O connectors are generally rectilinear; use hard-edged openings.
                        Rectangle(c["w"], c["h"])
                    else:
                        Rectangle(c["w"], c["h"])
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    part = cap_bp.part
    # Light global break-edge pass for a machined look.
    try:
        part = fillet(part.edges(), 0.5)
    except Exception:
        pass
    return part


def _build_front_cap_inverted(
    plate_w: float,
    plate_h: float,
    plate_r: float,
    plug_w: float,
    plug_h: float,
    plug_r: float,
    p: MakiCapParams,
    cutouts: list[dict],
):
    recess_w = max(plate_w - 2.0 * p.front_recess_inset_mm, 2.0)
    recess_h = max(plate_h - 2.0 * p.front_recess_inset_mm, 2.0)
    recess_r = max(plate_r - p.front_recess_inset_mm, 0.6)
    recess_depth = max(min(p.front_recess_depth_mm, p.cap_thickness_mm - 0.4), 0.25)

    with BuildPart() as cap_bp:
        # Main plate (outside face is at +Z).
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(plate_w, plate_h, plate_r)
        extrude(amount=p.cap_thickness_mm)

        # Plug goes to back side so the front is not center-protruding.
        with BuildSketch(Plane.XY):
            _add_rounded_rectangle(plug_w, plug_h, plug_r)
        extrude(amount=-p.plug_depth_mm)

        # Recess center panel on front face, leaving raised perimeter rim.
        with BuildSketch(Plane.XY.offset(p.cap_thickness_mm)):
            _add_rounded_rectangle(recess_w, recess_h, recess_r)
        extrude(amount=-(recess_depth + 0.12), mode=Mode.SUBTRACT)

        # Front cutouts go through full cap stack.
        cut_depth = p.cap_thickness_mm + p.plug_depth_mm + 2.0
        with BuildSketch(Plane.XY.offset(p.cap_thickness_mm + 1.0)):
            for c in cutouts:
                with Locations((c["x"], c["y"])):
                    if c["shape"] == "circle":
                        Circle(c["d"] * 0.5)
                    elif c["shape"] == "slot":
                        SlotOverall(c["w"], c["h"])
                    else:
                        Rectangle(c["w"], c["h"])
        extrude(amount=-cut_depth, mode=Mode.SUBTRACT)

    part = cap_bp.part
    try:
        part = fillet(part.edges(), 0.5)
    except Exception:
        pass
    return part


def build_caps(p: MakiCapParams):
    tmp_stl = Path("tmp/maki_device_from_step_caps_ref.stl")
    mesh, solids = _load_step_as_mesh(p.step_path, tmp_stl)
    zmin, zmax = [float(v) for v in mesh.bounds[:, 2]]
    z_section = zmin + (zmax - zmin) * p.section_z_ratio
    base_profile = _extract_profile_xy(mesh, z_section)

    raw_min_x, raw_min_y, raw_max_x, raw_max_y = base_profile.bounds
    raw_w = raw_max_x - raw_min_x
    raw_h = raw_max_y - raw_min_y

    sx = p.nominal_width_mm / raw_w
    sy = p.nominal_height_mm / raw_h
    base_profile = affinity.scale(base_profile, xfact=sx, yfact=sy, origin=(0.0, 0.0))
    base_corner_r = _estimate_corner_radius(base_profile)

    inner_w = p.nominal_width_mm + 2.0 * p.sleeve_clearance_mm
    inner_h = p.nominal_height_mm + 2.0 * p.sleeve_clearance_mm
    sleeve_outer_w = inner_w + 2.0 * p.sleeve_wall_mm
    sleeve_outer_h = inner_h + 2.0 * p.sleeve_wall_mm

    inner_corner_r = min(base_corner_r + p.sleeve_clearance_mm, 0.5 * min(inner_w, inner_h) - 0.2)
    sleeve_outer_corner_r = min(inner_corner_r + p.sleeve_wall_mm, 0.5 * min(sleeve_outer_w, sleeve_outer_h) - 0.2)

    plate_w = sleeve_outer_w + 2.0 * p.cap_plate_extra_mm
    plate_h = sleeve_outer_h + 2.0 * p.cap_plate_extra_mm
    plate_corner_r = min(
        sleeve_outer_corner_r + p.cap_plate_extra_mm,
        0.5 * min(plate_w, plate_h) - 0.2,
    )
    plug_w = max(inner_w - 2.0 * p.plug_clearance_mm, 2.0)
    plug_h = max(inner_h - 2.0 * p.plug_clearance_mm, 2.0)
    plug_corner_r = max(inner_corner_r - p.plug_clearance_mm, 0.6)

    front_cutouts, rear_cutouts, end_plane_meta = _extract_end_cutouts(solids, p, sx, sy, zmin, zmax)

    front_cap = _build_front_cap_inverted(
        plate_w, plate_h, plate_corner_r, plug_w, plug_h, plug_corner_r, p, front_cutouts
    )
    rear_cap = _build_cap(plate_w, plate_h, plate_corner_r, plug_w, plug_h, plug_corner_r, p, rear_cutouts)

    report = {
        "mesh_bounds_mm": {
            "min": [float(v) for v in mesh.bounds[0]],
            "max": [float(v) for v in mesh.bounds[1]],
            "extents": [float(v) for v in mesh.extents],
        },
        "profile_raw_mm": {"width": float(raw_w), "height": float(raw_h)},
        "profile_scale": {"sx": float(sx), "sy": float(sy)},
        "derived": {
            "sleeve_outer_w_mm": float(sleeve_outer_w),
            "sleeve_outer_h_mm": float(sleeve_outer_h),
            "plate_w_mm": float(plate_w),
            "plate_h_mm": float(plate_h),
            "inner_w_mm": float(inner_w),
            "inner_h_mm": float(inner_h),
            "plug_w_mm": float(plug_w),
            "plug_h_mm": float(plug_h),
            "corner_radius_mm": {
                "base_section": float(base_corner_r),
                "inner": float(inner_corner_r),
                "sleeve_outer": float(sleeve_outer_corner_r),
                "plate_outer": float(plate_corner_r),
                "plug": float(plug_corner_r),
            },
            "front_bezel_mm": {
                "recess_depth": float(max(min(p.front_recess_depth_mm, p.cap_thickness_mm - 0.4), 0.25)),
                "recess_inset": float(p.front_recess_inset_mm),
            },
        },
        "cutouts": {
            "end_planes": end_plane_meta,
            "front_count": len(front_cutouts),
            "rear_count": len(rear_cutouts),
            "front": front_cutouts,
            "rear": rear_cutouts,
        },
    }
    return front_cap, rear_cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate MAKI Live front and rear caps")
    parser.add_argument(
        "--profile",
        choices=["asa", "tpu"],
        default="asa",
        help="Material profile presets: asa outer shell caps, or tpu liner caps",
    )
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--cap-thickness", type=float, default=None, help="Cap plate thickness (mm)")
    parser.add_argument("--plug-depth", type=float, default=None, help="Cap plug insertion depth (mm)")
    parser.add_argument("--plug-clearance", type=float, default=None, help="Plug clearance inside sleeve (mm)")
    parser.add_argument("--cap-plate-extra", type=float, default=None, help="Extra radial plate overhang beyond sleeve OD (mm)")
    parser.add_argument("--cutout-extra", type=float, default=None, help="Extra clearance added around extracted cutouts (mm)")
    parser.add_argument("--front-recess-depth", type=float, default=None, help="Front cap center recess depth (mm)")
    parser.add_argument("--front-recess-inset", type=float, default=None, help="Front cap recess inset from OD edge (mm)")
    parser.add_argument(
        "--include-front-cap",
        action="store_true",
        help="Also export the separate front cap (legacy mode). Default is back-cap only.",
    )
    args = parser.parse_args()

    params = MakiCapParams(step_path=args.step)
    if args.profile == "tpu":
        # TPU liner-matched defaults.
        params.sleeve_clearance_mm = 0.2
        params.sleeve_wall_mm = 2.0
        params.cap_plate_extra_mm = 0.55
        params.cap_thickness_mm = 2.4
        params.plug_depth_mm = 3.4
        params.plug_clearance_mm = 0.30
    if args.cap_thickness is not None:
        params.cap_thickness_mm = args.cap_thickness
    if args.plug_depth is not None:
        params.plug_depth_mm = args.plug_depth
    if args.plug_clearance is not None:
        params.plug_clearance_mm = args.plug_clearance
    if args.cap_plate_extra is not None:
        params.cap_plate_extra_mm = args.cap_plate_extra
    if args.cutout_extra is not None:
        params.cutout_extra_mm = args.cutout_extra
    if args.front_recess_depth is not None:
        params.front_recess_depth_mm = args.front_recess_depth
    if args.front_recess_inset is not None:
        params.front_recess_inset_mm = args.front_recess_inset

    front_cap, rear_cap, report = build_caps(params)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    if args.profile == "asa":
        front_step = args.out / "maki_live_front_cap.step"
        rear_step = args.out / "maki_live_rear_cap.step"
        report_json = reports_dir / "maki_live_caps_report.json"
        legacy_report_json = args.out / "maki_live_caps_report.json"
    else:
        front_step = args.out / "maki_live_tpu_front_cap.step"
        rear_step = args.out / "maki_live_tpu_rear_cap.step"
        report_json = reports_dir / "maki_live_tpu_caps_report.json"
        legacy_report_json = args.out / "maki_live_tpu_caps_report.json"

    # Front cap is optional now; default workflow is integrated front body + separate rear cap.
    archived = _archive_existing([front_step, rear_step, report_json, legacy_report_json], args.out)

    if args.include_front_cap:
        export_step(front_cap, str(front_step))
    export_step(rear_cap, str(rear_step))
    payload = {"profile": args.profile, "params": asdict(params), "report": report}
    payload["include_front_cap"] = bool(args.include_front_cap)
    payload["params"]["step_path"] = str(params.step_path)
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    if args.include_front_cap:
        print(f"Wrote {front_step}")
    print(f"Wrote {rear_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
