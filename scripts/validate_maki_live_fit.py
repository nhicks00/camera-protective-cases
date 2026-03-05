#!/usr/bin/env python3
"""Validate MAKI Live body/back-cap fit against the manufacturer STEP.

Outputs:
- models/maki_case/maki_live_fit_validation_assembly.step
- models/maki_case/maki_live_device_aligned.step
- models/maki_case/maki_live_body_aligned.step
- models/maki_case/maki_live_rear_cap_aligned.step
- models/maki_case/reports/maki_live_fit_validation_report.json

Validation coverage:
1) Global collision/clearance checks (device/body/cap, ASA/TPU).
2) Side-feature alignment checks for every opening:
   - 24 tripanel vents
   - 6 side-trio vents
   - tripod opening
   Compared for both ASA and TPU against STEP-derived expected positions.
3) Rear-cap port-opening checks:
   - extract cutouts from the generated rear-cap STEP
   - compare against STEP-derived expected rear cutouts.
"""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path

from build123d import Axis, Compound, GeomType, Plane, export_step, import_step

try:
    import generate_maki_live_case as maki_case_gen
except Exception as exc:  # pragma: no cover - defensive import guard
    maki_case_gen = None
    _MAKI_CASE_IMPORT_ERROR = str(exc)
else:
    _MAKI_CASE_IMPORT_ERROR = None

try:
    import generate_maki_live_caps as maki_caps_gen
except Exception as exc:  # pragma: no cover - defensive import guard
    maki_caps_gen = None
    _MAKI_CAPS_IMPORT_ERROR = str(exc)
else:
    _MAKI_CAPS_IMPORT_ERROR = None


# Strict enough to catch regressions, relaxed enough for kernel tolerance.
VENT_CENTER_TOL_MM = 0.35
VENT_SIZE_TOL_MM = 0.35
TRIPOD_CENTER_TOL_MM = 0.35
TRIPOD_DIAM_TOL_MM = 0.45
REAR_CUTOUT_CENTER_TOL_MM = 0.45
# Rear-cap openings are edge-broken (0.5 mm fillet), which widens measured face
# apertures by ~1.0 mm compared to sketch dimensions.
REAR_CUTOUT_SIZE_TOL_MM = 1.15


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


def _safe_intersection_volume(a, b) -> float:
    try:
        inter = a & b
    except Exception:
        return 0.0
    try:
        solids = inter.solids()
        if solids:
            return float(sum(s.volume for s in solids))
    except Exception:
        pass
    v = getattr(inter, "volume", None)
    return float(v) if v is not None else 0.0


def _safe_distance(a, b) -> float:
    try:
        return float(a.distance(b))
    except Exception:
        return -1.0


def _to_compound(shape) -> Compound:
    if isinstance(shape, Compound):
        return shape
    return Compound(shape.wrapped)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _transform_device_to_case_frame(device_step: Path, dual_report: dict) -> Compound:
    dev = import_step(str(device_step))
    asa_src = dual_report["report"]["sources"]["asa_case_report"]
    cavity_front_z = float(asa_src["derived"]["cavity_front_z_mm"])
    clearance = float(
        dual_report.get("params", {}).get(
            "asa_clearance_mm", asa_src["derived"].get("end_clearance_each_mm", 2.3)
        )
    )
    front_target_z = cavity_front_z + clearance
    # rotate(Y, 180) flips X and Z (front/back reversal + X mirror for cap ports).
    # rotate(Z, 180) then flips X and Y, placing device vents on the same Y-side
    # as the case vents.  Net effect: (x,y,z) -> (x, -y, -z).
    rotated = dev.rotate(Axis.Y, 180).rotate(Axis.Z, 180)
    bb = rotated.bounding_box()
    tz = front_target_z - float(bb.min.Z)
    transformed = _to_compound(rotated.translate((0.0, 0.0, tz)))
    transformed.label = "MAKI_Device_Aligned"
    return transformed


def _pick_body_solids(body_step: Path):
    body = _to_compound(import_step(str(body_step)))
    solids = sorted(body.solids(), key=lambda s: s.volume, reverse=True)
    if len(solids) < 2:
        raise RuntimeError("Expected two solids in MAKI body dual-material STEP")
    asa, tpu = solids[0], solids[1]
    asa.label = "ASA_Shell"
    tpu.label = "TPU_Sleeve"
    return body, asa, tpu


def _pick_cap_solids(cap_step: Path):
    cap = _to_compound(import_step(str(cap_step)))
    solids = sorted(cap.solids(), key=lambda s: s.volume, reverse=True)
    if not solids:
        raise RuntimeError("No solids found in MAKI rear cap STEP")
    asa = solids[0]
    asa.label = "ASA_Back_Cap"
    tpu = solids[1] if len(solids) > 1 else None
    if tpu is not None:
        tpu.label = "TPU_Back_Gasket"
    return cap, asa, tpu


def _place_cap(cap, shell_depth: float, cap_thickness: float, axis: str):
    if axis == "x":
        placed = cap.rotate(Axis.X, 180.0).translate((0.0, 0.0, shell_depth + cap_thickness))
    elif axis == "y":
        placed = cap.rotate(Axis.Y, 180.0).translate((0.0, 0.0, shell_depth + cap_thickness))
    else:
        raise ValueError(f"Unsupported cap axis '{axis}'")
    return _to_compound(placed)


def _score_orientation(
    cap_comp: Compound,
    axis: str,
    shell_depth: float,
    cap_thickness: float,
    body_asa,
    body_tpu,
    device_solids,
):
    placed = _place_cap(cap_comp, shell_depth, cap_thickness, axis)
    cap_solids = sorted(placed.solids(), key=lambda s: s.volume, reverse=True)
    cap_asa = cap_solids[0]
    cap_tpu = cap_solids[1] if len(cap_solids) > 1 else None

    score = 0.0
    score += 1000.0 * _safe_intersection_volume(cap_asa, body_asa)
    score += 1000.0 * _safe_intersection_volume(cap_asa, body_tpu)
    if cap_tpu is not None:
        score += 200.0 * _safe_intersection_volume(cap_tpu, body_asa)
        score += 200.0 * _safe_intersection_volume(cap_tpu, body_tpu)
    for ds in device_solids:
        score += 100.0 * _safe_intersection_volume(cap_asa, ds)
        if cap_tpu is not None:
            score += 20.0 * _safe_intersection_volume(cap_tpu, ds)

    return score, placed, cap_asa, cap_tpu


def _normalize_vent(entry: dict, z_offset: float = 0.0) -> dict:
    family = entry.get("vent_family", "tripanel")
    axis = entry["axis"]
    side = entry["side"]
    x = float(entry.get("x", 0.0)) if axis == "y" else 0.0
    y = float(entry.get("y", 0.0)) if axis == "x" else 0.0
    return {
        "family": family,
        "axis": axis,
        "side": side,
        "x": x,
        "y": y,
        "z": float(entry["z"]) + z_offset,
        "slot_t": float(entry.get("slot_t", entry.get("slot_w", 0.0))),
        "slot_z": float(entry.get("slot_z", entry.get("slot_h", 0.0))),
    }


def _match_vent_sets(actual: list[dict], expected: list[dict]) -> dict:
    used_expected = set()
    center_dz = []
    center_t = []
    size_t = []
    size_z = []
    unmatched_actual = 0

    for a in actual:
        best_idx = None
        best_score = None
        for idx, e in enumerate(expected):
            if idx in used_expected:
                continue
            if e["family"] != a["family"] or e["axis"] != a["axis"] or e["side"] != a["side"]:
                continue
            tangent_delta = abs(a["x"] - e["x"]) if a["axis"] == "y" else abs(a["y"] - e["y"])
            dz = abs(a["z"] - e["z"])
            dt = abs(a["slot_t"] - e["slot_t"])
            ds = abs(a["slot_z"] - e["slot_z"])
            score = dz + 0.35 * tangent_delta + 0.15 * (dt + ds)
            if best_score is None or score < best_score:
                best_score = score
                best_idx = idx
        if best_idx is None:
            unmatched_actual += 1
            continue
        used_expected.add(best_idx)
        e = expected[best_idx]
        tangent_delta = abs(a["x"] - e["x"]) if a["axis"] == "y" else abs(a["y"] - e["y"])
        center_t.append(float(tangent_delta))
        center_dz.append(float(abs(a["z"] - e["z"])))
        size_t.append(float(abs(a["slot_t"] - e["slot_t"])))
        size_z.append(float(abs(a["slot_z"] - e["slot_z"])))

    unmatched_expected = len(expected) - len(used_expected)
    max_center_dz = max(center_dz) if center_dz else None
    max_center_t = max(center_t) if center_t else None
    max_size_t = max(size_t) if size_t else None
    max_size_z = max(size_z) if size_z else None
    passed = (
        unmatched_actual == 0
        and unmatched_expected == 0
        and (max_center_dz is not None and max_center_dz <= VENT_CENTER_TOL_MM)
        and (max_center_t is not None and max_center_t <= VENT_CENTER_TOL_MM)
        and (max_size_t is not None and max_size_t <= VENT_SIZE_TOL_MM)
        and (max_size_z is not None and max_size_z <= VENT_SIZE_TOL_MM)
    )
    return {
        "passed": bool(passed),
        "count_actual": len(actual),
        "count_expected": len(expected),
        "unmatched_actual": int(unmatched_actual),
        "unmatched_expected": int(unmatched_expected),
        "max_center_delta_z_mm": max_center_dz,
        "max_center_delta_tangent_mm": max_center_t,
        "max_size_delta_t_mm": max_size_t,
        "max_size_delta_z_mm": max_size_z,
        "tolerances_mm": {
            "center": float(VENT_CENTER_TOL_MM),
            "size": float(VENT_SIZE_TOL_MM),
        },
    }


def _tripod_delta(actual: dict | None, expected: dict | None) -> dict:
    if actual is None or expected is None:
        return {
            "passed": False,
            "missing_actual": actual is None,
            "missing_expected": expected is None,
        }
    dx = float(abs(float(actual["x"]) - float(expected["x"])))
    dz = float(abs(float(actual["z"]) - float(expected["z"])))
    dd = float(abs(float(actual["diameter"]) - float(expected["diameter"])))
    side_ok = str(actual["side"]) == str(expected["side"])
    passed = side_ok and dx <= TRIPOD_CENTER_TOL_MM and dz <= TRIPOD_CENTER_TOL_MM and dd <= TRIPOD_DIAM_TOL_MM
    return {
        "passed": bool(passed),
        "side_match": bool(side_ok),
        "delta_x_mm": dx,
        "delta_z_mm": dz,
        "delta_diameter_mm": dd,
        "tolerances_mm": {
            "center": float(TRIPOD_CENTER_TOL_MM),
            "diameter": float(TRIPOD_DIAM_TOL_MM),
        },
    }


def _extract_expected_case_features(device_step: Path, asa_case_report: dict) -> dict:
    if maki_case_gen is None:
        raise RuntimeError(f"Could not import generate_maki_live_case.py: {_MAKI_CASE_IMPORT_ERROR}")

    p = maki_case_gen.MakiCaseParams(step_path=device_step)
    tmp_stl = Path("tmp/maki_validate_device_side_features.stl")
    mesh, _, step_features = maki_case_gen._load_step_as_mesh(device_step, tmp_stl, p)
    zmax = float(mesh.bounds[1][2])

    sx = float(asa_case_report["profile_scale"]["sx"])
    sy = float(asa_case_report["profile_scale"]["sy"])
    sz = float(asa_case_report["profile_scale"]["sz"])
    shell_depth = float(asa_case_report["derived"]["shell_depth_mm"])
    cavity_front_z = float(asa_case_report["derived"]["cavity_front_z_mm"])
    clearance = float(asa_case_report["derived"]["end_clearance_each_mm"])
    outer_w = float(asa_case_report["derived"]["outer_w_mm"])
    resolved_side = asa_case_report["step_side_features"].get("resolved_tripod_side", p.tripod_expected_side)

    def map_x(x_dev: float) -> float:
        return float(x_dev * sx)

    def map_y(y_dev: float) -> float:
        return float(y_dev * sy)

    def map_z(z_dev: float) -> float:
        return float(cavity_front_z + (zmax - z_dev) * sz + clearance)

    vent_pattern = maki_case_gen._derive_tripanel_vents(
        step_features["vents"],
        map_x,
        map_y,
        map_z,
        sx,
        sz,
        outer_w,
        p,
        target_side=resolved_side,
    )
    resolved_side = vent_pattern.get("panel_side", resolved_side)

    expected_vents: list[dict] = []
    for panel in vent_pattern["panels"]:
        for z_c in vent_pattern["z_centers"]:
            z_shifted = min(max(float(z_c) + p.tripanel_vent_z_shift_mm, 1.0), shell_depth - 1.0)
            expected_vents.append(
                {
                    "family": "tripanel",
                    "axis": panel["axis"],
                    "side": panel["side"],
                    "x": float(panel.get("x", 0.0)) if panel["axis"] == "y" else 0.0,
                    "y": float(panel.get("y", 0.0)) if panel["axis"] == "x" else 0.0,
                    "z": float(z_shifted),
                    "slot_t": float(vent_pattern["slot_t"]),
                    "slot_z": float(vent_pattern["slot_z"]),
                }
            )

    trio = maki_case_gen._derive_side_trio_vents(
        step_features["vents"],
        map_y,
        map_z,
        sy,
        sz,
        p,
        size_override=(vent_pattern["slot_t"], vent_pattern["slot_z"]),
    )
    side_trio_z = (
        (shell_depth + cavity_front_z) - trio["z_center"] if p.side_trio_flip_end else trio["z_center"]
    )
    side_trio_z = min(max(side_trio_z + p.side_trio_vent_z_shift_mm, 1.0), shell_depth - 1.0)
    for side in ("neg", "pos"):
        for y_c in trio["y_centers"]:
            expected_vents.append(
                {
                    "family": "side_trio",
                    "axis": "x",
                    "side": side,
                    "x": 0.0,
                    "y": float(y_c),
                    "z": float(side_trio_z),
                    "slot_t": float(trio["slot_t"]),
                    "slot_z": float(trio["slot_z"]),
                }
            )

    expected_tripod = None
    t = step_features.get("tripod")
    if t is not None:
        expected_z = float(map_z(float(t["z"])))
        if p.tripod_use_rect_cutout:
            expected_z += p.tripod_rect_z_shift_mm
        expected_tripod = {
            "side": "neg" if resolved_side == "neg" else "pos",
            "x": float(map_x(float(t["x"]))),
            "z": expected_z,
            "diameter": float(max(2.0 * float(t["r"]) * sx + p.tripod_cutout_extra_mm, 2.0)),
        }

    return {
        "expected_vents": expected_vents,
        "expected_tripod": expected_tripod,
        "raw_detected": {
            "vents_detected": int(len(step_features.get("vents", []))),
            "tripod_detected": bool(t is not None),
            "tripod_source": step_features.get("tripod_source", "unknown"),
        },
    }


def _normalize_cutout(c: dict) -> dict:
    shape = str(c.get("shape", "rect")).lower()
    if shape == "circle":
        d = float(c["d"])
        return {
            "family": "round",
            "x": float(c["x"]),
            "y": float(c["y"]),
            "major": d,
            "minor": d,
        }
    w = float(c.get("w", 0.0))
    h = float(c.get("h", 0.0))
    return {
        "family": "elongated",
        "x": float(c["x"]),
        "y": float(c["y"]),
        "major": max(w, h),
        "minor": min(w, h),
    }


def _extract_rear_cutouts_from_cap_step(cap_step: Path) -> list[dict]:
    cap = _to_compound(import_step(str(cap_step)))
    solids = sorted(cap.solids(), key=lambda s: s.volume, reverse=True)
    if not solids:
        return []
    asa = solids[0]

    faces = []
    for f in asa.faces():
        if f.geom_type != GeomType.PLANE:
            continue
        try:
            n = f.normal_at()
            c = f.center()
            wires = f.wires()
        except Exception:
            continue
        if float(n.Z) > -0.95:
            continue
        if len(wires) <= 1:
            continue
        faces.append((float(c.Z), f))
    if not faces:
        return []
    # Use the outer weather face where inner loops correspond to actual port holes.
    _, face = min(faces, key=lambda t: t[0])

    cutouts = []
    for loop in face.wires()[1:]:
        bb = loop.bounding_box()
        xlen = float(bb.size.X)
        ylen = float(bb.size.Y)
        xmid = float((bb.min.X + bb.max.X) * 0.5)
        ymid = float((bb.min.Y + bb.max.Y) * 0.5)
        ratio = max(xlen, ylen) / max(min(xlen, ylen), 1e-9)
        if ratio <= 1.18:
            cutouts.append({"shape": "circle", "x": xmid, "y": ymid, "d": 0.5 * (xlen + ylen)})
        else:
            cutouts.append({"shape": "slot", "x": xmid, "y": ymid, "w": max(xlen, ylen), "h": min(xlen, ylen)})
    return cutouts


def _extract_expected_rear_cutouts(device_step: Path, rear_cap_report: dict | None) -> list[dict]:
    if maki_caps_gen is None:
        raise RuntimeError(f"Could not import generate_maki_live_caps.py: {_MAKI_CAPS_IMPORT_ERROR}")

    p = maki_caps_gen.MakiCapParams(step_path=device_step)
    if rear_cap_report is not None:
        src_params = rear_cap_report.get("params", {})
        if "cutout_extra_mm" in src_params:
            p.cutout_extra_mm = float(src_params["cutout_extra_mm"])
        if "cap_thickness_mm" in src_params:
            p.cap_thickness_mm = float(src_params["cap_thickness_mm"])
        if "plug_depth_mm" in src_params:
            p.plug_depth_mm = float(src_params["plug_depth_mm"])
        if "plug_clearance_mm" in src_params:
            p.plug_clearance_mm = float(src_params["plug_clearance_mm"])

    _, _, caps_report = maki_caps_gen.build_caps(p)
    return list(caps_report.get("cutouts", {}).get("rear", []))


def _match_cutouts(actual: list[dict], expected: list[dict]) -> dict:
    a_norm = [_normalize_cutout(c) for c in actual]
    e_norm = [_normalize_cutout(c) for c in expected]

    used_expected = set()
    center_deltas = []
    major_deltas = []
    minor_deltas = []
    unmatched_actual = 0

    for a in a_norm:
        best_idx = None
        best_score = None
        for idx, e in enumerate(e_norm):
            if idx in used_expected:
                continue
            if e["family"] != a["family"]:
                continue
            dc = ((a["x"] - e["x"]) ** 2 + (a["y"] - e["y"]) ** 2) ** 0.5
            dmaj = abs(a["major"] - e["major"])
            dmin = abs(a["minor"] - e["minor"])
            score = dc + 0.2 * (dmaj + dmin)
            if best_score is None or score < best_score:
                best_score = score
                best_idx = idx
        if best_idx is None:
            unmatched_actual += 1
            continue
        used_expected.add(best_idx)
        e = e_norm[best_idx]
        center_deltas.append(float(((a["x"] - e["x"]) ** 2 + (a["y"] - e["y"]) ** 2) ** 0.5))
        major_deltas.append(float(abs(a["major"] - e["major"])))
        minor_deltas.append(float(abs(a["minor"] - e["minor"])))

    unmatched_expected = len(e_norm) - len(used_expected)
    max_center = max(center_deltas) if center_deltas else None
    max_major = max(major_deltas) if major_deltas else None
    max_minor = max(minor_deltas) if minor_deltas else None
    passed = (
        unmatched_actual == 0
        and unmatched_expected == 0
        and (max_center is not None and max_center <= REAR_CUTOUT_CENTER_TOL_MM)
        and (max_major is not None and max_major <= REAR_CUTOUT_SIZE_TOL_MM)
        and (max_minor is not None and max_minor <= REAR_CUTOUT_SIZE_TOL_MM)
    )
    return {
        "passed": bool(passed),
        "count_actual": len(a_norm),
        "count_expected": len(e_norm),
        "unmatched_actual": int(unmatched_actual),
        "unmatched_expected": int(unmatched_expected),
        "max_center_delta_mm": max_center,
        "max_major_size_delta_mm": max_major,
        "max_minor_size_delta_mm": max_minor,
        "tolerances_mm": {
            "center": float(REAR_CUTOUT_CENTER_TOL_MM),
            "size": float(REAR_CUTOUT_SIZE_TOL_MM),
        },
    }


def validate_fit(
    device_step: Path,
    body_step: Path,
    cap_step: Path,
    dual_report_path: Path,
    rear_cap_report_path: Path | None = None,
):
    dual = _load_json(dual_report_path)
    rear_cap_report = None
    if rear_cap_report_path is not None and rear_cap_report_path.exists():
        rear_cap_report = _load_json(rear_cap_report_path)

    body, body_asa, body_tpu = _pick_body_solids(body_step)
    cap_raw, _, _ = _pick_cap_solids(cap_step)
    device_comp = _transform_device_to_case_frame(device_step, dual)
    device_solids = list(device_comp.solids())

    shell_depth = float(dual["report"]["sources"]["asa_case_report"]["derived"]["shell_depth_mm"])
    cap_thickness = 3.0
    cap_axes = ("x", "y")

    best = None
    for axis in cap_axes:
        score, placed, cap_asa, cap_tpu = _score_orientation(
            cap_raw, axis, shell_depth, cap_thickness, body_asa, body_tpu, device_solids
        )
        if best is None or score < best["score"]:
            best = {
                "axis": axis,
                "score": score,
                "placed": placed,
                "cap_asa": cap_asa,
                "cap_tpu": cap_tpu,
            }

    cap_placed = best["placed"]
    cap_asa = best["cap_asa"]
    cap_tpu = best["cap_tpu"]

    # Global collision/clearance metrics.
    metrics = {}
    pair_defs = [
        ("device_vs_body_asa", device_solids, [body_asa]),
        ("device_vs_body_tpu", device_solids, [body_tpu]),
        ("device_vs_cap_asa", device_solids, [cap_asa]),
        ("cap_asa_vs_body_asa", [cap_asa], [body_asa]),
        ("cap_asa_vs_body_tpu", [cap_asa], [body_tpu]),
    ]
    if cap_tpu is not None:
        pair_defs.extend(
            [
                ("device_vs_cap_tpu", device_solids, [cap_tpu]),
                ("cap_tpu_vs_body_asa", [cap_tpu], [body_asa]),
                ("cap_tpu_vs_body_tpu", [cap_tpu], [body_tpu]),
            ]
        )

    for key, a_list, b_list in pair_defs:
        vol = 0.0
        min_dist = None
        for a in a_list:
            for b in b_list:
                iv = _safe_intersection_volume(a, b)
                vol += iv
                d = _safe_distance(a, b)
                if d >= 0.0:
                    min_dist = d if min_dist is None else min(min_dist, d)
        metrics[key] = {
            "intersection_volume_mm3": float(vol),
            "min_distance_mm": float(min_dist) if min_dist is not None else None,
            "hard_collision": bool(vol > 0.10),
        }

    hard_collision_pairs = [k for k, v in metrics.items() if bool(v.get("hard_collision"))]

    # Feature-by-feature validation.
    asa_case_report = dual["report"]["sources"]["asa_case_report"]
    tpu_case_report = dual["report"]["sources"]["tpu_sleeve_report"]
    tpu_z_offset = float(dual["report"]["alignment_mm"]["tpu_z_offset_into_asa"])

    feature_warnings: list[str] = []
    case_feature_check = {}
    rear_cutout_check = {}

    try:
        expected_case = _extract_expected_case_features(device_step, asa_case_report)
        expected_vents = expected_case["expected_vents"]
        expected_tripod = expected_case["expected_tripod"]

        asa_actual_vents = [
            _normalize_vent(v, z_offset=0.0)
            for v in asa_case_report["step_side_features"].get("vents_applied_entries", [])
        ]
        tpu_actual_vents = [
            _normalize_vent(v, z_offset=tpu_z_offset)
            for v in tpu_case_report["step_side_features"].get("vents_applied_entries", [])
        ]
        asa_vents_check = _match_vent_sets(asa_actual_vents, expected_vents)
        tpu_vents_check = _match_vent_sets(tpu_actual_vents, expected_vents)

        asa_tripod = asa_case_report["step_side_features"].get("tripod_applied")
        tpu_tripod = tpu_case_report["step_side_features"].get("tripod_applied")
        if tpu_tripod is not None:
            tpu_tripod = dict(tpu_tripod)
            tpu_tripod["z"] = float(tpu_tripod["z"]) + tpu_z_offset
        asa_tripod_check = _tripod_delta(asa_tripod, expected_tripod)
        tpu_tripod_check = _tripod_delta(tpu_tripod, expected_tripod)

        case_feature_check = {
            "passed": bool(
                asa_vents_check["passed"]
                and tpu_vents_check["passed"]
                and asa_tripod_check["passed"]
                and tpu_tripod_check["passed"]
            ),
            "expected_feature_source": expected_case["raw_detected"],
            "asa_vents": asa_vents_check,
            "tpu_vents": tpu_vents_check,
            "asa_tripod": asa_tripod_check,
            "tpu_tripod": tpu_tripod_check,
            "counts": {
                "expected_total_vents": len(expected_vents),
                "expected_tripanel": len([v for v in expected_vents if v["family"] == "tripanel"]),
                "expected_side_trio": len([v for v in expected_vents if v["family"] == "side_trio"]),
            },
        }
    except Exception as exc:
        case_feature_check = {
            "passed": False,
            "error": str(exc),
        }
        feature_warnings.append(f"Case-feature validation error: {exc}")

    try:
        actual_rear_cutouts = _extract_rear_cutouts_from_cap_step(cap_step)
        expected_rear_cutouts = _extract_expected_rear_cutouts(device_step, rear_cap_report)
        rear_cutout_check = _match_cutouts(actual_rear_cutouts, expected_rear_cutouts)
        rear_cutout_check["source"] = {
            "actual_from_cap_step": str(cap_step),
            "expected_from_device_step": str(device_step),
            "rear_cap_report_used": str(rear_cap_report_path) if rear_cap_report_path and rear_cap_report else None,
        }
    except Exception as exc:
        rear_cutout_check = {
            "passed": False,
            "error": str(exc),
        }
        feature_warnings.append(f"Rear-cutout validation error: {exc}")

    # ---- Physical pin-through-vent test ----
    # For each case vent, create a thin probe box at the vent center,
    # oriented radially through the case wall, and check if it collides with
    # the device.  If it does, the vent is blocked by the device shell wall
    # (i.e. you can't stick your finger straight through the vent).
    # A small collision volume (< threshold) is expected due to curved-wall
    # clipping and is treated as "pass".
    PIN_BLOCK_THRESHOLD_MM3 = 0.5
    pin_test: dict = {"passed": True, "tested": 0, "blocked": 0, "clear": 0, "blocked_vents": []}
    try:
        from build123d import BuildPart, Box, Align, Location, Vector

        asa_vents_for_pin = asa_case_report["step_side_features"].get("vents_applied_entries", [])
        for vi, vent in enumerate(asa_vents_for_pin):
            axis = vent["axis"]
            side = vent["side"]
            vz = float(vent["z"])
            slot_t = float(vent.get("slot_t", vent.get("slot_w", 2.0)))
            slot_z = float(vent.get("slot_z", vent.get("slot_h", 2.0)))
            # Thin probe box centered in the vent slot, extending radially
            probe_t = min(slot_t, 1.5)  # tangential width (narrow)
            probe_z = min(slot_z, 1.0)  # axial height (narrow)
            probe_len = 30.0  # radial length

            if axis == "y":
                vx = float(vent.get("x", 0.0))
                # Probe box: long in Y, narrow in X and Z
                with BuildPart() as _bp:
                    Box(probe_t, probe_len, probe_z, align=(Align.CENTER, Align.CENTER, Align.CENTER))
                probe = _bp.part.translate((vx, 0.0, vz))
            elif axis == "x":
                vy = float(vent.get("y", 0.0))
                # Probe box: long in X, narrow in Y and Z
                with BuildPart() as _bp:
                    Box(probe_len, probe_t, probe_z, align=(Align.CENTER, Align.CENTER, Align.CENTER))
                probe = _bp.part.translate((0.0, vy, vz))
            else:
                continue

            pin_test["tested"] += 1
            max_vol = 0.0
            for ds in device_solids:
                vol = _safe_intersection_volume(probe, ds)
                max_vol = max(max_vol, vol)
            if max_vol > PIN_BLOCK_THRESHOLD_MM3:
                pin_test["blocked"] += 1
                pin_test["blocked_vents"].append({
                    "index": vi,
                    "axis": axis,
                    "side": side,
                    "z": vz,
                    "collision_volume_mm3": float(max_vol),
                })
            else:
                pin_test["clear"] += 1

        pin_test["passed"] = pin_test["blocked"] == 0
        pin_test["threshold_mm3"] = float(PIN_BLOCK_THRESHOLD_MM3)
    except Exception as exc:
        pin_test = {"passed": False, "error": str(exc)}
        feature_warnings.append(f"Pin-through-vent test error: {exc}")

    all_feature_checks_passed = bool(
        case_feature_check.get("passed", False)
        and rear_cutout_check.get("passed", False)
        and pin_test.get("passed", False)
    )

    blocking_issues = []
    if hard_collision_pairs:
        blocking_issues.append("Hard-collision pairs detected.")
    if not all_feature_checks_passed:
        blocking_issues.append("One or more feature-alignment checks failed.")
    if pin_test.get("blocked", 0) > 0:
        blocking_issues.append(
            f"Pin-through test: {pin_test['blocked']}/{pin_test['tested']} vents blocked by device."
        )

    assembly = Compound(children=[device_comp, body, cap_placed], label="MAKI_Fit_Validation_Assembly")

    report = {
        "inputs": {
            "device_step": str(device_step),
            "body_step": str(body_step),
            "cap_step": str(cap_step),
            "dual_report": str(dual_report_path),
            "rear_cap_report": str(rear_cap_report_path) if rear_cap_report_path is not None else None,
        },
        "device_to_case_transform": {
            "mode": "rotate_y180_z180_plus_translate_z",
            "sx": 1.0,
            "sy": -1.0,
            "sz": -1.0,
            "tx": 0.0,
            "ty": 0.0,
            "tz": float(dual["report"]["sources"]["asa_case_report"]["derived"]["cavity_front_z_mm"])
            + float(
                dual.get("params", {}).get(
                    "asa_clearance_mm",
                    dual["report"]["sources"]["asa_case_report"]["derived"].get("end_clearance_each_mm", 2.3),
                )
            ),
        },
        "cap_placement": {
            "shell_depth_mm": shell_depth,
            "assumed_cap_thickness_mm": cap_thickness,
            "selected_flip_axis": best["axis"],
            "orientation_score": float(best["score"]),
        },
        "fit_metrics": metrics,
        "hard_collision_pairs": hard_collision_pairs,
        "feature_validation": {
            "all_feature_checks_passed": all_feature_checks_passed,
            "case_openings_vs_device": case_feature_check,
            "rear_cap_cutouts_vs_device": rear_cutout_check,
            "pin_through_vent_test": pin_test,
            "warnings": feature_warnings,
        },
        "blocking_issues": blocking_issues,
    }

    extras = {
        "device_aligned": device_comp,
        "cap_aligned": cap_placed,
        "body_aligned": body,
    }
    return assembly, report, extras


def main():
    parser = argparse.ArgumentParser(description="Validate MAKI body/cap fit with real device STEP")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--device-step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="Manufacturer device STEP",
    )
    parser.add_argument(
        "--body-step",
        type=Path,
        default=Path("models/maki_case/maki_live_body_dual_material.step"),
        help="Generated MAKI body dual-material STEP",
    )
    parser.add_argument(
        "--cap-step",
        type=Path,
        default=Path("models/maki_case/maki_live_rear_cap_dual_material.step"),
        help="Generated MAKI rear cap dual-material STEP",
    )
    parser.add_argument(
        "--dual-report",
        type=Path,
        default=Path("models/maki_case/reports/maki_live_dual_material_report.json"),
        help="Dual-material body report JSON",
    )
    parser.add_argument(
        "--rear-cap-report",
        type=Path,
        default=Path("models/maki_case/reports/maki_live_rear_cap_dual_material_report.json"),
        help="Rear-cap dual-material report JSON (used for cap cutout expectation params)",
    )
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    out_step = args.out / "maki_live_fit_validation_assembly.step"
    out_device = args.out / "maki_live_device_aligned.step"
    out_cap = args.out / "maki_live_rear_cap_aligned.step"
    out_body = args.out / "maki_live_body_aligned.step"
    out_json = reports_dir / "maki_live_fit_validation_report.json"
    archived = _archive_existing([out_step, out_device, out_cap, out_body, out_json], args.out)

    assembly, report, extras = validate_fit(
        device_step=args.device_step,
        body_step=args.body_step,
        cap_step=args.cap_step,
        dual_report_path=args.dual_report,
        rear_cap_report_path=args.rear_cap_report,
    )

    export_step(assembly, str(out_step))
    export_step(extras["device_aligned"], str(out_device))
    export_step(extras["cap_aligned"], str(out_cap))
    export_step(extras["body_aligned"], str(out_body))
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")
    print(f"Wrote {out_device}")
    print(f"Wrote {out_cap}")
    print(f"Wrote {out_body}")


if __name__ == "__main__":
    main()
