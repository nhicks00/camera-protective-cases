#!/usr/bin/env python3
"""Add a rounded half-circle lens hood to a purchased AVKANS Go STL case.

This script intentionally keeps the purchased STL as mesh input and emits a
mesh STL derivative. The added hood is generated as a watertight rounded sweep
mesh. The purchased STL is repaired into one watertight main shell before the
hood is fused with a manifold Boolean union for the final printable STL.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from collections import Counter
from dataclasses import asdict, dataclass
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
DEFAULT_OUT = Path("models/avkans_go_case/avkans_go4k_cults_shade_with_lens_hood.stl")


@dataclass
class HoodParams:
    front_z_mm: float = 0.0
    hood_depth_mm: float = 17.78
    wall_mm: float = 3.0
    root_overlap_mm: float = -0.30
    edge_round_mm: float = 1.2
    arc_segments: int = 96
    corner_segments: int = 8
    ransac_seed: int = 4
    ransac_iterations: int = 5000
    circle_inlier_tolerance_mm: float = 0.60


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


def _source_front_lip(circle: tuple[float, float, float], vertices: np.ndarray, params: HoodParams) -> dict:
    """Measure the STL's existing upper front lip for a flush hood root."""
    cx, cy, inner_r = circle
    radial = np.hypot(vertices[:, 0] - cx, vertices[:, 1] - cy)
    source_front_z_min = float(vertices[:, 2].min())
    lip_mask = (
        (vertices[:, 1] >= cy)
        & (vertices[:, 2] >= source_front_z_min - 0.05)
        & (vertices[:, 2] <= params.front_z_mm + 0.10)
        & (radial >= inner_r + 0.35)
        & (radial <= inner_r + params.wall_mm + 1.0)
    )
    lip_vertices = vertices[lip_mask]
    lip_radial = radial[lip_mask]
    if len(lip_vertices) < 8:
        return {
            "upper_lip_vertex_count": int(len(lip_vertices)),
            "upper_lip_outer_radius_mm": float(inner_r + params.wall_mm),
            "upper_lip_z_min_mm": float(params.front_z_mm),
            "upper_lip_radius_match_applied": False,
        }

    return {
        "upper_lip_vertex_count": int(len(lip_vertices)),
        "upper_lip_outer_radius_mm": float(np.percentile(lip_radial, 98.0)),
        "upper_lip_z_min_mm": float(lip_vertices[:, 2].min()),
        "upper_lip_radius_match_applied": True,
    }


def fit_front_lens_circle(mesh: trimesh.Trimesh, params: HoodParams) -> dict:
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
    lip = _source_front_lip((cx, cy, r), vertices, params)

    return {
        "center_x_mm": float(cx),
        "center_y_mm": float(cy),
        "radius_mm": float(r),
        "diameter_mm": float(2.0 * r),
        "front_z_mm": float(params.front_z_mm),
        "front_plane_vertex_count": int(len(front_pts)),
        "fit_candidate_count": int(len(candidates)),
        "inlier_count": int(len(inliers)),
        **lip,
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


def repair_source_shell(source: trimesh.Trimesh, params: HoodParams) -> tuple[trimesh.Trimesh, dict]:
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


def _smoothstep(value: float) -> float:
    value = min(max(float(value), 0.0), 1.0)
    return value * value * (3.0 - 2.0 * value)


def _source_lip_cross_section(
    width: float,
    z_min: float,
    root_inner_z: float,
    root_outer_z: float,
    radius: float,
    segments: int,
) -> np.ndarray:
    """Return a hood loop whose rear edge follows the source lip curve."""
    half_w = 0.5 * width
    half_h = 0.5 * (max(root_inner_z, root_outer_z) - z_min)
    radius = min(max(radius, 0.0), half_w * 0.95, half_h * 0.90)
    segments = max(int(segments), 3)

    if radius <= 0.0:
        return np.array(
            [
                [half_w, root_outer_z],
                [-half_w, root_inner_z],
                [-half_w, z_min],
                [half_w, z_min],
            ],
            dtype=float,
        )

    points: list[tuple[float, float]] = []
    root_segments = max(int(segments) * 2, 8)
    for i in range(root_segments + 1):
        t = i / root_segments
        u = half_w - width * t
        z = root_outer_z + (root_inner_z - root_outer_z) * _smoothstep(t)
        points.append((u, z))

    points.append((-half_w, z_min + radius))

    for i in range(1, segments + 1):
        angle = math.pi + 0.5 * math.pi * i / segments
        points.append((-half_w + radius + radius * math.cos(angle), z_min + radius + radius * math.sin(angle)))

    points.append((half_w - radius, z_min))
    for i in range(1, segments + 1):
        angle = 1.5 * math.pi + 0.5 * math.pi * i / segments
        points.append((half_w - radius + radius * math.cos(angle), z_min + radius + radius * math.sin(angle)))

    return np.asarray(points, dtype=float)


def _swept_arc_mesh(
    cx: float,
    cy: float,
    center_radius: float,
    cross_section: np.ndarray,
    arc_segments: int,
    theta_min: float = 0.0,
    theta_max: float = math.pi,
) -> trimesh.Trimesh:
    """Sweep a closed radial/Z profile around the upper lens-opening arc."""
    arc_segments = max(int(arc_segments), 16)
    cross_count = len(cross_section)
    z_center = float(cross_section[:, 1].mean())
    radial_center = float(cross_section[:, 0].mean())

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for arc_i in range(arc_segments + 1):
        theta = theta_min + (theta_max - theta_min) * arc_i / arc_segments
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        for radial_offset, z in cross_section:
            radius = center_radius + radial_offset
            vertices.append((cx + radius * cos_t, cy + radius * sin_t, z))

    for arc_i in range(arc_segments):
        for cross_i in range(cross_count):
            a = arc_i * cross_count + cross_i
            b = arc_i * cross_count + (cross_i + 1) % cross_count
            c = (arc_i + 1) * cross_count + (cross_i + 1) % cross_count
            d = (arc_i + 1) * cross_count + cross_i
            faces.append((a, b, c))
            faces.append((a, c, d))

    for arc_i, flip in ((0, True), (arc_segments, False)):
        theta = theta_min + (theta_max - theta_min) * arc_i / arc_segments
        center_index = len(vertices)
        vertices.append(
            (
                cx + (center_radius + radial_center) * math.cos(theta),
                cy + (center_radius + radial_center) * math.sin(theta),
                z_center,
            )
        )
        for cross_i in range(cross_count):
            a = arc_i * cross_count + cross_i
            b = arc_i * cross_count + (cross_i + 1) % cross_count
            faces.append((center_index, b, a) if flip else (center_index, a, b))

    swept = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=True)
    swept.fix_normals()
    return clean_mesh(swept)


def build_rounded_half_hood(circle: dict, params: HoodParams, hood_stl: Path) -> dict:
    """Build a watertight rounded semi-circular hood mesh and export it to STL."""
    cx = circle["center_x_mm"]
    cy = circle["center_y_mm"]
    inner_r = circle["radius_mm"]
    measured_lip_outer_r = circle.get("upper_lip_outer_radius_mm", inner_r + params.wall_mm)
    outer_r = max(inner_r + params.wall_mm, measured_lip_outer_r)
    effective_wall = outer_r - inner_r
    center_r = inner_r + 0.5 * effective_wall
    start_z = params.front_z_mm - params.hood_depth_mm
    measured_lip_z = circle.get("upper_lip_z_min_mm", params.front_z_mm)
    if circle.get("upper_lip_radius_match_applied") and measured_lip_z < params.front_z_mm:
        root_anchor = "source upper front lip"
        root_anchor_z = measured_lip_z
    else:
        root_anchor = "front face"
        root_anchor_z = params.front_z_mm
    root_outer_z = root_anchor_z + params.root_overlap_mm
    root_inner_z = params.front_z_mm
    cross_section = _source_lip_cross_section(
        effective_wall,
        start_z,
        root_inner_z,
        root_outer_z,
        params.edge_round_mm,
        params.corner_segments,
    )
    arc_segments = max(int(params.arc_segments), 16)
    hood_mesh = _swept_arc_mesh(cx, cy, center_r, cross_section, arc_segments)
    terminal_round_report = {
        "enabled": False,
        "style": "removed",
        "is_watertight": bool(hood_mesh.is_watertight),
    }

    hood_stl.parent.mkdir(parents=True, exist_ok=True)
    hood_mesh.export(hood_stl)

    return {
        "inner_radius_mm": float(inner_r),
        "outer_radius_mm": float(outer_r),
        "centerline_radius_mm": float(center_r),
        "requested_wall_mm": float(params.wall_mm),
        "effective_wall_mm": float(effective_wall),
        "depth_mm": float(params.hood_depth_mm),
        "sweep_length_mm": float(root_outer_z - start_z),
        "root_overlap_mm": float(params.root_overlap_mm),
        "root_anchor": root_anchor,
        "root_anchor_z_mm": float(root_anchor_z),
        "root_profile": "source lip curved rear edge",
        "root_inner_z_mm": float(root_inner_z),
        "root_outer_z_mm": float(root_outer_z),
        "front_z_mm": float(params.front_z_mm),
        "root_z_mm": float(root_outer_z),
        "z_min_mm": float(start_z),
        "z_max_mm": float(max(root_inner_z, root_outer_z)),
        "source_upper_lip_outer_radius_mm": float(measured_lip_outer_r),
        "source_upper_lip_z_min_mm": float(measured_lip_z),
        "arc": "upper half circle",
        "edge_round_mm": float(params.edge_round_mm),
        "terminal_cap_rounding": terminal_round_report,
        "arc_segments": int(arc_segments),
        "corner_segments": int(params.corner_segments),
        "is_watertight": bool(hood_mesh.is_watertight),
        "component_count": int(len(hood_mesh.split(only_watertight=False))),
        "hood_only_stl": str(hood_stl),
    }


def _root_curve_z(radial_offset: float, half_width: float, root_outer_z: float, root_inner_z: float) -> float:
    clamped = min(max(float(radial_offset), -half_width), half_width)
    t = (half_width - clamped) / (2.0 * half_width)
    return root_outer_z + (root_inner_z - root_outer_z) * _smoothstep(t)


def build_root_lip_cleanup_cutter(
    circle: dict,
    hood_report: dict,
    trim_depth_mm: float,
    curve_epsilon_mm: float,
    radial_margin_mm: float,
) -> trimesh.Trimesh:
    """Cut away the old source-STL upper lip so the hood curve becomes visible."""
    center_radius = hood_report["centerline_radius_mm"]
    half_width = 0.5 * hood_report["effective_wall_mm"]
    root_outer_z = hood_report["root_outer_z_mm"]
    root_inner_z = hood_report["front_z_mm"]
    z_lower = min(circle["upper_lip_z_min_mm"], root_outer_z) - trim_depth_mm
    offsets = np.linspace(-half_width - radial_margin_mm, half_width + radial_margin_mm, 36)

    cross_section: list[tuple[float, float]] = [(float(offset), float(z_lower)) for offset in offsets]
    for offset in offsets[::-1]:
        curve_z = _root_curve_z(offset, half_width, root_outer_z, root_inner_z) - curve_epsilon_mm
        cross_section.append((float(offset), float(max(curve_z, z_lower + 0.01))))

    return _swept_arc_mesh(
        circle["center_x_mm"],
        circle["center_y_mm"],
        center_radius,
        np.asarray(cross_section, dtype=float),
        arc_segments=128,
        theta_min=-0.04,
        theta_max=math.pi + 0.04,
    )


def build_root_internal_weld(
    circle: dict,
    hood_report: dict,
    trim_depth_mm: float,
    radial_padding_mm: float,
) -> tuple[trimesh.Trimesh, dict]:
    """Add a root weld after lip cleanup without growing the exterior profile."""
    half_width = 0.5 * hood_report["effective_wall_mm"]
    root_outer_z = hood_report["root_outer_z_mm"]
    source_lip_z = circle["upper_lip_z_min_mm"]
    z_lower = min(source_lip_z, root_outer_z) - trim_depth_mm
    weld_z_min = z_lower - 0.06
    weld_z_max = source_lip_z + 0.12
    radial_min = -half_width - radial_padding_mm
    radial_max = half_width + radial_padding_mm
    cross_section = np.asarray(
        [
            [radial_min, weld_z_min],
            [radial_max, weld_z_min],
            [radial_max, weld_z_max],
            [radial_min, weld_z_max],
        ],
        dtype=float,
    )
    weld = _swept_arc_mesh(
        circle["center_x_mm"],
        circle["center_y_mm"],
        hood_report["centerline_radius_mm"],
        cross_section,
        arc_segments=96,
        theta_min=0.0,
        theta_max=math.pi,
    )
    return weld, {
        "radial_min_offset_mm": float(radial_min),
        "radial_max_offset_mm": float(radial_max),
        "z_min_mm": float(weld_z_min),
        "z_max_mm": float(weld_z_max),
        "theta_min_rad": 0.0,
        "theta_max_rad": float(math.pi),
    }


def replace_visible_source_lip(combined: trimesh.Trimesh, circle: dict, hood_report: dict) -> tuple[trimesh.Trimesh, dict]:
    """Replace the source STL's old upper lip with the hood's curved root profile."""
    trim_depth_mm = 0.03
    curve_epsilon_mm = 0.02
    trim_radial_margin_mm = 0.45
    weld_radial_padding_mm = 0.0
    report = {
        "attempted": True,
        "engine": "manifold",
        "trim_depth_mm": trim_depth_mm,
        "curve_epsilon_mm": curve_epsilon_mm,
        "trim_radial_margin_mm": trim_radial_margin_mm,
        "weld_radial_padding_mm": weld_radial_padding_mm,
    }

    cutter = build_root_lip_cleanup_cutter(
        circle,
        hood_report,
        trim_depth_mm=trim_depth_mm,
        curve_epsilon_mm=curve_epsilon_mm,
        radial_margin_mm=trim_radial_margin_mm,
    )
    trimmed = trimesh.boolean.difference([combined, cutter], engine="manifold")
    if trimmed is None:
        raise RuntimeError("Root lip cleanup Boolean difference returned None")
    trimmed = clean_mesh(trimmed)
    trimmed_components = trimmed.split(only_watertight=False)
    report["trimmed"] = {
        "mesh": mesh_stats(trimmed),
        "edge_quality": edge_quality(trimmed),
        "component_face_counts": [int(len(component.faces)) for component in trimmed_components],
    }

    weld, weld_report = build_root_internal_weld(circle, hood_report, trim_depth_mm, weld_radial_padding_mm)
    cleaned = trimesh.boolean.union([trimmed, weld], engine="manifold")
    if cleaned is None:
        raise RuntimeError("Root lip cleanup weld Boolean union returned None")
    cleaned = clean_mesh(cleaned)
    quality = edge_quality(cleaned)
    component_count = len(cleaned.split(only_watertight=False))
    succeeded = bool(cleaned.is_watertight and component_count == 1 and quality["boundary_edges"] == 0 and quality["nonmanifold_edges"] == 0)
    report.update(
        {
            "succeeded": succeeded,
            "weld": weld_report,
            "final_edge_quality": quality,
            "final_component_count": int(component_count),
        }
    )
    if not succeeded:
        raise RuntimeError(f"Root lip cleanup did not produce one clean watertight component: {report}")
    return cleaned, report


def make_preview(base: trimesh.Trimesh, hood: trimesh.Trimesh, combined: trimesh.Trimesh, out_png: Path) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)
    views = [
        ("Front X/Y", (0, 1), "X width (mm)", "Y height (mm)"),
        ("Side Y/Z", (1, 2), "Y height (mm)", "Z depth (mm)"),
        ("Top X/Z", (0, 2), "X width (mm)", "Z depth (mm)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8), dpi=180)
    for ax, (title, dims, xlabel, ylabel) in zip(axes, views):
        base_polys = [base.vertices[list(face)][:, dims] for face in base.faces]
        hood_polys = [hood.vertices[list(face)][:, dims] for face in hood.faces]
        ax.add_collection(
            PolyCollection(base_polys, facecolors="#c9c9c9", edgecolors="#707070", linewidths=0.02, alpha=0.55)
        )
        ax.add_collection(
            PolyCollection(hood_polys, facecolors="#9bb7d4", edgecolors="#29516f", linewidths=0.035, alpha=0.90)
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
    fig.suptitle("AVKANS Go4k shade cover with rounded half-circle lens hood")
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


def fuse_repaired_shell_and_hood(shell: trimesh.Trimesh, hood: trimesh.Trimesh) -> tuple[trimesh.Trimesh, dict]:
    """Boolean-union two watertight meshes with manifold, falling back loudly."""
    report = {
        "attempted": True,
        "engine": "manifold",
        "fallback_used": False,
    }
    try:
        fused = trimesh.boolean.union([shell, hood], engine="manifold")
        if fused is None:
            raise RuntimeError("manifold returned None")
        fused = clean_mesh(fused)
        report["succeeded"] = bool(fused.is_watertight and len(fused.split(only_watertight=False)) == 1)
        report["edge_quality"] = edge_quality(fused)
        if not report["succeeded"]:
            raise RuntimeError("Boolean result was not one watertight component")
        return fused, report
    except Exception as exc:
        fallback = trimesh.util.concatenate([shell, hood])
        fallback = clean_mesh(fallback)
        report.update(
            {
                "succeeded": False,
                "fallback_used": True,
                "failure": f"{type(exc).__name__}: {exc}",
                "edge_quality": edge_quality(fallback),
            }
        )
        return fallback, report


def main() -> None:
    parser = argparse.ArgumentParser(description="Add rounded half-circle lens hood to AVKANS Go4k STL")
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--hood-only", type=Path, default=Path("models/avkans_go_case/avkans_go4k_added_lens_hood_only.stl"))
    parser.add_argument("--report", type=Path, default=Path("models/avkans_go_case/reports/avkans_go4k_cults_hood_report.json"))
    parser.add_argument("--preview", type=Path, default=Path("models/avkans_go_case/reports/avkans_go4k_cults_hood_preview.png"))
    parser.add_argument("--hood-depth", type=float, default=17.78)
    parser.add_argument("--wall", type=float, default=3.0)
    parser.add_argument("--root-overlap", type=float, default=-0.30)
    parser.add_argument("--edge-round", type=float, default=1.2)
    args = parser.parse_args()

    params = HoodParams(
        hood_depth_mm=args.hood_depth,
        wall_mm=args.wall,
        root_overlap_mm=args.root_overlap,
        edge_round_mm=args.edge_round,
    )

    source_mesh = trimesh.load(args.source, force="mesh")
    circle = fit_front_lens_circle(source_mesh, params)
    repaired_base, repair_report = repair_source_shell(source_mesh, params)
    hood_report = build_rounded_half_hood(circle, params, args.hood_only)
    hood_mesh = trimesh.load(args.hood_only, force="mesh")

    combined, union_report = fuse_repaired_shell_and_hood(repaired_base, hood_mesh)
    root_cleanup_report = {"attempted": False, "succeeded": False}
    if union_report.get("succeeded") and not union_report.get("fallback_used"):
        combined, root_cleanup_report = replace_visible_source_lip(combined, circle, hood_report)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    combined.export(args.out)

    make_preview(repaired_base, hood_mesh, combined, args.preview)

    report = {
        "source_file": str(args.source),
        "output_stl": str(args.out),
        "params": asdict(params),
        "source_mesh": mesh_stats(source_mesh),
        "source_repair": repair_report,
        "lens_opening_fit": circle,
        "hood": hood_report,
        "combined_mesh": mesh_stats(combined),
        "combined_edge_quality": edge_quality(combined),
        "boolean_union": union_report,
        "root_lip_cleanup": root_cleanup_report,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.out}")
    print(f"Wrote {args.hood_only}")
    print(f"Wrote {args.report}")
    print(f"Wrote {args.preview}")


if __name__ == "__main__":
    main()
