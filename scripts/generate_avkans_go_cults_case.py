#!/usr/bin/env python3
"""Generate the purchased AVKANS Go STL case derivative.

This script intentionally keeps the purchased STL as mesh input and emits a
mesh STL derivative. The purchased STL is repaired into one watertight main
shell, the front LED slot is extended for better visibility, and passive
hot-shoe-style receivers are fused to both flat side walls. It does not add
any extra lens hood or shade geometry.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np
import trimesh
from matplotlib.collections import PolyCollection
from shapely.geometry import LineString
from shapely.ops import polygonize, triangulate, unary_union

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DEFAULT_SOURCE = Path("/Users/nathanhicks/Downloads/Avkans%20Go4k%20shade%20cover-Cults3d.stl")
DEFAULT_OUT = Path("models/avkans_go_case/avkans_go4k_cults_shade_extended_led_slot.stl")


@dataclass
class AvkansCultsParams:
    front_z_mm: float = 0.0
    led_slot_extend_up_mm: float = 7.0
    led_slot_extend_down_mm: float = 18.0
    led_slot_detection_x_window_mm: float = 6.0
    led_slot_detection_y_margin_below_lens_mm: float = 5.0
    led_slot_z_cut_margin_mm: float = 0.5
    ransac_seed: int = 4
    ransac_iterations: int = 5000
    circle_inlier_tolerance_mm: float = 0.60
    # Side receivers duplicate the measured channel profile of the existing
    # top receiver. They are passive cold shoes (no electrical contacts).
    side_shoe_center_y_mm: float = -15.0
    side_shoe_z_start_mm: float = 88.325
    side_shoe_length_mm: float = 22.775
    side_shoe_boss_width_mm: float = 25.250
    side_shoe_slot_width_mm: float = 19.796
    side_shoe_opening_width_mm: float = 15.049
    side_shoe_slot_depth_mm: float = 2.525
    side_shoe_rail_thickness_mm: float = 2.273
    side_shoe_root_overlap_mm: float = 0.35
    side_shoe_join_overlap_mm: float = 0.08


def _archive_existing(paths: list[Path], out_dir: Path) -> list[tuple[str, str]]:
    archive_dir = out_dir / "archive"
    archive_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    moved: list[tuple[str, str]] = []
    for path in paths:
        if not path.exists():
            continue
        target = archive_dir / f"{path.stem}_{stamp}{path.suffix}"
        suffix = 1
        while target.exists():
            target = archive_dir / f"{path.stem}_{stamp}_{suffix}{path.suffix}"
            suffix += 1
        shutil.move(str(path), str(target))
        moved.append((str(path), str(target)))
    return moved


def _circle_from_3(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> tuple[float, float, float] | None:
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    temp = x2 * x2 + y2 * y2
    bc = (x1 * x1 + y1 * y1 - temp) / 2.0
    cd = (temp - x3 * x3 - y3 * y3) / 2.0
    det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
    if abs(det) < 1e-6:
        return None
    cx = (bc * (y2 - y3) - cd * (y1 - y2)) / det
    cy = ((x1 - x2) * cd - (x2 - x3) * bc) / det
    r = math.hypot(cx - x1, cy - y1)
    return cx, cy, r


def fit_front_lens_circle(mesh: trimesh.Trimesh, params: AvkansCultsParams) -> dict:
    """Fit the large circular lens opening on the front Z plane."""
    vertices = np.asarray(mesh.vertices)
    front_pts = vertices[np.abs(vertices[:, 2] - params.front_z_mm) < 1e-4][:, :2]
    if len(front_pts) < 40:
        raise RuntimeError(f"Not enough vertices on front Z={params.front_z_mm} plane to fit lens opening")

    bounds = mesh.bounds
    x_mid = 0.5 * (bounds[0, 0] + bounds[1, 0])
    y_span = bounds[1, 1] - bounds[0, 1]
    # The AVKANS Go front lens opening is the large upper circle in X/Y.
    candidates = front_pts[
        (front_pts[:, 1] > bounds[0, 1] + 0.45 * y_span)
        & (np.abs(front_pts[:, 0] - x_mid) > 1.5)
    ]
    if len(candidates) < 40:
        raise RuntimeError("Not enough front-plane upper-circle candidates for lens opening fit")

    rng = random.Random(params.ransac_seed)
    best: tuple[int, float, float, float] | None = None
    for _ in range(params.ransac_iterations):
        p = candidates[rng.sample(range(len(candidates)), 3)]
        circle = _circle_from_3(p[0], p[1], p[2])
        if circle is None:
            continue
        cx, cy, r = circle
        # Keep the search constrained to a realistic centered upper opening.
        if abs(cx - x_mid) > 3.0:
            continue
        if not (0.30 * y_span < cy - bounds[0, 1] < 0.80 * y_span):
            continue
        if not (0.32 * (bounds[1, 0] - bounds[0, 0]) < 2.0 * r < 1.05 * (bounds[1, 0] - bounds[0, 0])):
            continue
        d = np.sqrt((candidates[:, 0] - cx) ** 2 + (candidates[:, 1] - cy) ** 2)
        score = int(np.count_nonzero(np.abs(d - r) < 0.35))
        if best is None or score > best[0]:
            best = (score, cx, cy, r)

    if best is None:
        raise RuntimeError("Unable to fit lens opening circle")

    _, cx, cy, r = best
    d = np.sqrt((candidates[:, 0] - cx) ** 2 + (candidates[:, 1] - cy) ** 2)
    inliers = candidates[np.abs(d - r) < params.circle_inlier_tolerance_mm]
    if len(inliers) >= 12:
        a = np.c_[2.0 * inliers[:, 0], 2.0 * inliers[:, 1], np.ones(len(inliers))]
        b = inliers[:, 0] ** 2 + inliers[:, 1] ** 2
        refined = np.linalg.lstsq(a, b, rcond=None)[0]
        cx, cy, c = [float(v) for v in refined]
        r = math.sqrt(c + cx * cx + cy * cy)
    return {
        "center_x_mm": float(cx),
        "center_y_mm": float(cy),
        "radius_mm": float(r),
        "diameter_mm": float(2.0 * r),
        "front_z_mm": float(params.front_z_mm),
        "front_plane_vertex_count": int(len(front_pts)),
        "fit_candidate_count": int(len(candidates)),
        "inlier_count": int(len(inliers)),
    }


def edge_quality(mesh: trimesh.Trimesh) -> dict:
    counts: Counter[tuple[int, int]] = Counter()
    for face in mesh.faces:
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = tuple(sorted((int(a), int(b))))
            counts[edge] += 1
    return {
        "boundary_edges": int(sum(1 for count in counts.values() if count == 1)),
        "nonmanifold_edges": int(sum(1 for count in counts.values() if count > 2)),
        "unique_edges": int(len(counts)),
    }


def clean_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=6)
    if hasattr(mesh, "unique_faces"):
        mesh.update_faces(mesh.unique_faces())
    if hasattr(mesh, "nondegenerate_faces"):
        mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    return mesh


def boundary_edges(mesh: trimesh.Trimesh) -> list[tuple[int, int]]:
    counts: Counter[tuple[int, int]] = Counter()
    for face in mesh.faces:
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = tuple(sorted((int(a), int(b))))
            counts[edge] += 1
    return [edge for edge, count in counts.items() if count == 1]


def repair_source_shell(source: trimesh.Trimesh, params: AvkansCultsParams) -> tuple[trimesh.Trimesh, dict]:
    """Drop loose STL fragments and repair the main front-plane shell loops."""
    source_components = source.split(only_watertight=False)
    main = max(source_components, key=lambda component: len(component.faces))
    before = mesh_stats(main)
    before["edge_quality"] = edge_quality(main)

    lines = []
    for a, b in boundary_edges(main):
        pa = main.vertices[a]
        pb = main.vertices[b]
        if abs(pa[2] - params.front_z_mm) > 1e-5 or abs(pb[2] - params.front_z_mm) > 1e-5:
            continue
        lines.append(LineString([(float(pa[0]), float(pa[1])), (float(pb[0]), float(pb[1]))]))

    patch_vertices: list[tuple[float, float, float]] = []
    patch_faces: list[tuple[int, int, int]] = []
    patch_polygons = []
    if lines:
        polygons = list(polygonize(lines))
        merged = unary_union(polygons)
        patch_polygons = [merged] if merged.geom_type == "Polygon" else list(merged.geoms)
        for polygon in patch_polygons:
            for triangle in triangulate(polygon):
                if not polygon.covers(triangle.representative_point()):
                    continue
                indices = []
                for x, y in list(triangle.exterior.coords)[:-1]:
                    indices.append(len(patch_vertices))
                    patch_vertices.append((float(x), float(y), float(params.front_z_mm)))
                patch_faces.append(tuple(indices))

    if patch_faces:
        patch = trimesh.Trimesh(vertices=np.asarray(patch_vertices), faces=np.asarray(patch_faces), process=False)
        repaired = trimesh.util.concatenate([main, patch])
    else:
        patch = trimesh.Trimesh(vertices=np.empty((0, 3)), faces=np.empty((0, 3), dtype=int), process=False)
        repaired = main.copy()

    repaired = clean_mesh(repaired)
    after = mesh_stats(repaired)
    after["edge_quality"] = edge_quality(repaired)

    return repaired, {
        "source_component_count": int(len(source_components)),
        "kept_largest_component_faces": int(len(main.faces)),
        "dropped_loose_component_count": int(max(len(source_components) - 1, 0)),
        "front_boundary_segments_used": int(len(lines)),
        "front_patch_polygon_count": int(len(patch_polygons)),
        "front_patch_faces_added": int(len(patch.faces)),
        "before": before,
        "after": after,
    }

def detect_front_led_slot(source: trimesh.Trimesh, circle: dict, params: AvkansCultsParams) -> dict:
    """Detect the centered lower front-face oval slot from source STL vertices."""
    vertices = np.asarray(source.vertices)
    bounds = source.bounds
    front_z_min = float(vertices[:, 2].min())
    lens_bottom_y = circle["center_y_mm"] - circle["radius_mm"]
    y_limit = lens_bottom_y - params.led_slot_detection_y_margin_below_lens_mm
    y_floor = float(bounds[0, 1] + 0.18 * (bounds[1, 1] - bounds[0, 1]))
    z_mask = (np.abs(vertices[:, 2] - params.front_z_mm) < 1e-4) | (np.abs(vertices[:, 2] - front_z_min) < 1e-4)
    slot_vertices = vertices[
        z_mask
        & (np.abs(vertices[:, 0] - circle["center_x_mm"]) <= params.led_slot_detection_x_window_mm)
        & (vertices[:, 1] < y_limit)
        & (vertices[:, 1] > y_floor)
    ]
    if len(slot_vertices) < 12:
        raise RuntimeError(f"Unable to detect AVKANS front LED slot; matched {len(slot_vertices)} vertices")

    x_min = float(slot_vertices[:, 0].min())
    x_max = float(slot_vertices[:, 0].max())
    y_min = float(slot_vertices[:, 1].min())
    y_max = float(slot_vertices[:, 1].max())
    width = x_max - x_min
    height = y_max - y_min
    if height <= width:
        raise RuntimeError(f"Detected LED slot is not vertically oriented: width={width:.3f}, height={height:.3f}")

    extended_y_min = y_min - params.led_slot_extend_down_mm
    extended_y_max = y_max + params.led_slot_extend_up_mm
    return {
        "enabled": True,
        "matched_vertex_count": int(len(slot_vertices)),
        "center_x_mm": float(0.5 * (x_min + x_max)),
        "center_y_mm": float(0.5 * (y_min + y_max)),
        "width_mm": float(width),
        "height_mm": float(height),
        "x_min_mm": x_min,
        "x_max_mm": x_max,
        "y_min_mm": y_min,
        "y_max_mm": y_max,
        "extended_center_y_mm": float(0.5 * (extended_y_min + extended_y_max)),
        "extended_height_mm": float(height + params.led_slot_extend_down_mm + params.led_slot_extend_up_mm),
        "extended_y_min_mm": float(extended_y_min),
        "extended_y_max_mm": float(extended_y_max),
        "extend_up_mm": float(params.led_slot_extend_up_mm),
        "extend_down_mm": float(params.led_slot_extend_down_mm),
        "z_min_mm": float(front_z_min - params.led_slot_z_cut_margin_mm),
        "z_max_mm": float(params.front_z_mm + params.led_slot_z_cut_margin_mm),
    }


def build_vertical_slot_cutter(slot: dict, segments: int = 16) -> trimesh.Trimesh:
    """Build a watertight vertical rounded-slot prism for front-face subtraction."""
    width = slot["width_mm"]
    height = slot["extended_height_mm"]
    radius = 0.5 * width
    if height <= width:
        raise RuntimeError(f"LED slot cutter height must exceed width: width={width:.3f}, height={height:.3f}")

    cx = slot["center_x_mm"]
    y_bottom_center = slot["extended_y_min_mm"] + radius
    y_top_center = slot["extended_y_max_mm"] - radius
    points: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = math.pi * i / segments
        points.append((cx + radius * math.cos(angle), y_top_center + radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi + math.pi * i / segments
        points.append((cx + radius * math.cos(angle), y_bottom_center + radius * math.sin(angle)))

    z_min = slot["z_min_mm"]
    z_max = slot["z_max_mm"]
    vertices = [(x, y, z_min) for x, y in points] + [(x, y, z_max) for x, y in points]
    n = len(points)
    faces: list[tuple[int, int, int]] = []
    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))

    cutter = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=True)
    cutter.fix_normals()
    return clean_mesh(cutter)


def extend_front_led_slot(
    mesh: trimesh.Trimesh,
    source: trimesh.Trimesh,
    circle: dict,
    params: AvkansCultsParams,
) -> tuple[trimesh.Trimesh, dict]:
    slot = detect_front_led_slot(source, circle, params)
    cutter = build_vertical_slot_cutter(slot)
    cut = trimesh.boolean.difference([mesh, cutter], engine="manifold")
    if cut is None:
        raise RuntimeError("LED slot extension Boolean difference returned None")
    cut = clean_mesh(cut)
    quality = edge_quality(cut)
    component_count = len(cut.split(only_watertight=False))
    succeeded = bool(
        cut.is_watertight
        and component_count == 1
        and quality["boundary_edges"] == 0
        and quality["nonmanifold_edges"] == 0
    )
    slot.update(
        {
            "succeeded": succeeded,
            "cutter": mesh_stats(cutter),
            "final_component_count": int(component_count),
            "final_edge_quality": quality,
        }
    )
    if not succeeded:
        raise RuntimeError(f"LED slot extension did not produce one clean watertight component: {slot}")
    return cut, slot


def _box_from_bounds(minimum: tuple[float, float, float], maximum: tuple[float, float, float]) -> trimesh.Trimesh:
    minimum_array = np.asarray(minimum, dtype=float)
    maximum_array = np.asarray(maximum, dtype=float)
    extents = maximum_array - minimum_array
    if np.any(extents <= 0.0):
        raise RuntimeError(f"Invalid side-shoe box bounds: minimum={minimum}, maximum={maximum}")
    transform = np.eye(4)
    transform[:3, 3] = 0.5 * (minimum_array + maximum_array)
    return trimesh.creation.box(extents=extents, transform=transform)


def _section_x_crossings(mesh: trimesh.Trimesh, y: float, z: float) -> list[float]:
    section = mesh.section(plane_origin=(0.0, 0.0, z), plane_normal=(0.0, 0.0, 1.0))
    if section is None:
        raise RuntimeError(f"Unable to section AVKANS shell at Z={z:.3f} mm")

    horizontal = LineString([(-1000.0, y), (1000.0, y)])
    crossings: list[float] = []

    def collect_points(geometry) -> None:
        if geometry.is_empty:
            return
        if geometry.geom_type == "Point":
            crossings.append(float(geometry.x))
            return
        if hasattr(geometry, "geoms"):
            for child in geometry.geoms:
                collect_points(child)

    for path in section.discrete:
        collect_points(LineString(path[:, :2]).intersection(horizontal))

    return sorted(set(round(value, 6) for value in crossings))


def add_side_hot_shoes(
    mesh: trimesh.Trimesh,
    params: AvkansCultsParams,
) -> tuple[trimesh.Trimesh, dict]:
    """Fuse one rear-loading passive shoe receiver to each flat side wall."""
    boss_width = params.side_shoe_boss_width_mm
    slot_width = params.side_shoe_slot_width_mm
    opening_width = params.side_shoe_opening_width_mm
    slot_depth = params.side_shoe_slot_depth_mm
    rail_thickness = params.side_shoe_rail_thickness_mm
    root_overlap = params.side_shoe_root_overlap_mm
    join_overlap = params.side_shoe_join_overlap_mm
    if not (boss_width > slot_width > opening_width > 0.0):
        raise RuntimeError(
            "Side-shoe widths must satisfy boss_width > slot_width > opening_width > 0"
        )
    if min(slot_depth, rail_thickness, root_overlap) <= 0.0:
        raise RuntimeError("Side-shoe depth, rail thickness, and root overlap must be positive")

    z_start = params.side_shoe_z_start_mm
    z_end = min(z_start + params.side_shoe_length_mm, float(mesh.bounds[1, 2]))
    if z_end - z_start < 10.0:
        raise RuntimeError(f"Side-shoe rail length is too short: {z_end - z_start:.3f} mm")

    z_probe = 0.5 * (z_start + z_end)
    crossings = _section_x_crossings(mesh, params.side_shoe_center_y_mm, z_probe)
    if len(crossings) < 4:
        raise RuntimeError(
            f"Expected outer and inner side-wall crossings at Y={params.side_shoe_center_y_mm:.3f}, "
            f"Z={z_probe:.3f}; found {crossings}"
        )
    left_wall_x = float(min(crossings))
    right_wall_x = float(max(crossings))

    boss_height = slot_depth + rail_thickness
    half_boss = 0.5 * boss_width
    half_slot = 0.5 * slot_width
    half_opening = 0.5 * opening_width
    center_y = params.side_shoe_center_y_mm
    shoe_parts: list[trimesh.Trimesh] = []

    for wall_x, outward_sign in ((left_wall_x, -1.0), (right_wall_x, 1.0)):
        if outward_sign > 0.0:
            wall_x_bounds = (wall_x - root_overlap, wall_x + boss_height)
            rail_x_bounds = (wall_x + slot_depth - join_overlap, wall_x + boss_height)
        else:
            wall_x_bounds = (wall_x - boss_height, wall_x + root_overlap)
            rail_x_bounds = (wall_x - boss_height, wall_x - slot_depth + join_overlap)

        for tangential_sign in (-1.0, 1.0):
            if tangential_sign > 0.0:
                wall_y_bounds = (center_y + half_slot, center_y + half_boss)
                rail_y_bounds = (
                    center_y + half_opening,
                    center_y + half_slot + join_overlap,
                )
            else:
                wall_y_bounds = (center_y - half_boss, center_y - half_slot)
                rail_y_bounds = (
                    center_y - half_slot - join_overlap,
                    center_y - half_opening,
                )

            shoe_parts.append(
                _box_from_bounds(
                    (wall_x_bounds[0], wall_y_bounds[0], z_start),
                    (wall_x_bounds[1], wall_y_bounds[1], z_end),
                )
            )
            shoe_parts.append(
                _box_from_bounds(
                    (rail_x_bounds[0], rail_y_bounds[0], z_start),
                    (rail_x_bounds[1], rail_y_bounds[1], z_end),
                )
            )

    combined = trimesh.boolean.union([mesh, *shoe_parts], engine="manifold")
    if combined is None:
        raise RuntimeError("Side hot-shoe Boolean union returned None")
    combined = clean_mesh(combined)
    quality = edge_quality(combined)
    component_count = len(combined.split(only_watertight=False))
    succeeded = bool(
        combined.is_watertight
        and component_count == 1
        and quality["boundary_edges"] == 0
        and quality["nonmanifold_edges"] == 0
    )
    if not succeeded:
        raise RuntimeError(
            "Side hot-shoe union did not produce one clean watertight component: "
            f"components={component_count}, edge_quality={quality}"
        )

    return combined, {
        "enabled": True,
        "type": "passive_hot_shoe_style_cold_shoe_receiver",
        "count": 2,
        "locations": ["left_X-", "right_X+"],
        "profile_source": "measured from existing purchased top receiver",
        "slide_in_from": "rear_positive_Z",
        "center_y_mm": float(center_y),
        "y_span_mm": [float(center_y - half_boss), float(center_y + half_boss)],
        "z_start_mm": float(z_start),
        "z_end_mm": float(z_end),
        "length_mm": float(z_end - z_start),
        "detected_side_wall_x_mm": {
            "left": left_wall_x,
            "right": right_wall_x,
            "section_crossings": [float(value) for value in crossings],
        },
        "boss_width_mm": float(boss_width),
        "slot_width_mm": float(slot_width),
        "rail_opening_width_mm": float(opening_width),
        "slot_depth_mm": float(slot_depth),
        "rail_thickness_mm": float(rail_thickness),
        "projection_from_wall_mm": float(boss_height),
        "root_overlap_mm": float(root_overlap),
        "join_overlap_mm": float(join_overlap),
        "final_component_count": int(component_count),
        "final_edge_quality": quality,
    }


def make_preview(base: trimesh.Trimesh, combined: trimesh.Trimesh, out_png: Path) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)
    views = [
        ("Front X/Y", (0, 1), "X width (mm)", "Y height (mm)"),
        ("Side Y/Z", (1, 2), "Y height (mm)", "Z depth (mm)"),
        ("Top X/Z", (0, 2), "X width (mm)", "Z depth (mm)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8), dpi=180)
    for ax, (title, dims, xlabel, ylabel) in zip(axes, views):
        base_polys = [base.vertices[list(face)][:, dims] for face in base.faces]
        combined_polys = [combined.vertices[list(face)][:, dims] for face in combined.faces]
        ax.add_collection(
            PolyCollection(base_polys, facecolors="#c9c9c9", edgecolors="#707070", linewidths=0.02, alpha=0.45)
        )
        ax.add_collection(
            PolyCollection(combined_polys, facecolors="#9bb7d4", edgecolors="#29516f", linewidths=0.025, alpha=0.45)
        )
        xy = combined.vertices[:, dims]
        pad = 6.0
        ax.set_xlim(float(xy[:, 0].min()) - pad, float(xy[:, 0].max()) + pad)
        ax.set_ylim(float(xy[:, 1].min()) - pad, float(xy[:, 1].max()) + pad)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(title)
        ax.set_xlabel(xlabel, fontsize=8)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, linewidth=0.2, alpha=0.35)
        ax.tick_params(labelsize=7)
    fig.suptitle("AVKANS Go4k shade cover with extended LED slot and side shoe mounts")
    fig.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)


def mesh_stats(mesh: trimesh.Trimesh) -> dict:
    stats_mesh = mesh.copy()
    stats_mesh.merge_vertices()
    return {
        "vertices": int(len(stats_mesh.vertices)),
        "faces": int(len(stats_mesh.faces)),
        "bounds_min_mm": [float(v) for v in stats_mesh.bounds[0]],
        "bounds_max_mm": [float(v) for v in stats_mesh.bounds[1]],
        "extents_mm": [float(v) for v in stats_mesh.extents],
        "is_watertight": bool(stats_mesh.is_watertight),
        "component_count": int(len(stats_mesh.split(only_watertight=False))),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate AVKANS Go4k STL derivative with side shoe mounts and no added lens hood"
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--report", type=Path, default=Path("models/avkans_go_case/reports/avkans_go4k_cults_case_report.json"))
    parser.add_argument("--preview", type=Path, default=Path("models/avkans_go_case/reports/avkans_go4k_cults_case_preview.png"))
    parser.add_argument("--led-slot-extend-up", type=float, default=7.0)
    parser.add_argument("--led-slot-extend-down", type=float, default=18.0)
    args = parser.parse_args()

    params = AvkansCultsParams(
        led_slot_extend_up_mm=args.led_slot_extend_up,
        led_slot_extend_down_mm=args.led_slot_extend_down,
    )

    source_mesh = trimesh.load(args.source, force="mesh")
    circle = fit_front_lens_circle(source_mesh, params)
    repaired_base, repair_report = repair_source_shell(source_mesh, params)
    combined, led_slot_report = extend_front_led_slot(repaired_base, source_mesh, circle, params)
    combined, side_hot_shoe_report = add_side_hot_shoes(combined, params)
    archived = _archive_existing([args.out, args.report, args.preview], args.out.parent)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    combined.export(args.out)

    make_preview(repaired_base, combined, args.preview)

    report = {
        "source_file": str(args.source),
        "output_stl": str(args.out),
        "params": asdict(params),
        "added_lens_hood": {
            "enabled": False,
            "removed": True,
            "reason": "User requested complete removal of the added AVKANS lens hood/shade geometry.",
        },
        "source_mesh": mesh_stats(source_mesh),
        "source_repair": repair_report,
        "lens_opening_fit": circle,
        "combined_mesh": mesh_stats(combined),
        "combined_edge_quality": edge_quality(combined),
        "front_led_slot_extension": led_slot_report,
        "side_hot_shoes": side_hot_shoe_report,
        "archived_previous_outputs": [
            {"source": source, "archived_to": archived_to}
            for source, archived_to in archived
        ],
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.out}")
    print(f"Wrote {args.report}")
    print(f"Wrote {args.preview}")


if __name__ == "__main__":
    main()
