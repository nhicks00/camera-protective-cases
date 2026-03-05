#!/usr/bin/env python3
"""
Independent fit validation for MAKI Live case — v2.

Strategy:
  1. Load device STEP, extract vent/tripod positions from B-REP geometry
  2. Transform device features into case coordinate frame
  3. Compare against case vent positions from the dual-material report
     (the report records where the generator actually placed cuts)
  4. Also do direct geometry collision checks
  5. Report all deltas

This avoids the circular validation in the existing validator which
re-derives "expected" positions using the same mapping functions.
"""

import json
import sys
import math
from pathlib import Path
from collections import defaultdict

from build123d import (
    import_step, Solid, Compound, Shape, Location, Plane,
    Axis, Vector, BoundBox
)
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_ShapeEnum
from OCP.TopExp import TopExp_Explorer
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.TopoDS import TopoDS
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Pnt, gp_Vec


ROOT = Path(__file__).resolve().parent.parent
DEVICE_STEP = ROOT / "refs" / "BirdDog_MAKI-Live_3D-file.step"
BODY_STEP = ROOT / "models" / "maki_case" / "maki_live_body_dual_material.step"
CAP_STEP = ROOT / "models" / "maki_case" / "maki_live_rear_cap_dual_material.step"
DUAL_REPORT = ROOT / "models" / "maki_case" / "reports" / "maki_live_dual_material_report.json"


def get_face_bbox(face):
    bbox = Bnd_Box()
    BRepBndLib.Add_s(face, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return {
        "cx": (xmin + xmax) / 2, "cy": (ymin + ymax) / 2, "cz": (zmin + zmax) / 2,
        "sx": xmax - xmin, "sy": ymax - ymin, "sz": zmax - zmin,
    }


def get_face_normal(face):
    surf = BRepAdaptor_Surface(face)
    u = (surf.FirstUParameter() + surf.LastUParameter()) / 2
    v = (surf.FirstVParameter() + surf.LastVParameter()) / 2
    p = gp_Pnt()
    d1u = gp_Vec()
    d1v = gp_Vec()
    surf.D1(u, v, p, d1u, d1v)
    n = d1u.Crossed(d1v)
    mag = n.Magnitude()
    if mag < 1e-10:
        return (0, 0, 0)
    return (n.X() / mag, n.Y() / mag, n.Z() / mag)


def count_wires(face):
    exp = TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_WIRE)
    count = 0
    while exp.More():
        count += 1
        exp.Next()
    return count


def get_inner_wire_bboxes(face):
    exp = TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_WIRE)
    wires = []
    while exp.More():
        wires.append(exp.Current())
        exp.Next()
    if len(wires) < 2:
        return []
    results = []
    for wire in wires[1:]:
        bbox = Bnd_Box()
        BRepBndLib.Add_s(wire, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        results.append({
            "cx": (xmin + xmax) / 2, "cy": (ymin + ymax) / 2, "cz": (zmin + zmax) / 2,
            "sx": xmax - xmin, "sy": ymax - ymin, "sz": zmax - zmin,
        })
    return results


def extract_device_features(housing):
    """Extract vent slots and tripod from device STEP B-REP. Returns in DEVICE native frame."""
    vents = []
    tripod_candidates = []

    bb = housing.bounding_box()
    dev_zmin, dev_zmax = bb.min.Z, bb.max.Z
    print(f"  Device BB: X[{bb.min.X:.3f}, {bb.max.X:.3f}] "
          f"Y[{bb.min.Y:.3f}, {bb.max.Y:.3f}] "
          f"Z[{bb.min.Z:.3f}, {bb.max.Z:.3f}]")
    print(f"  Device extents: {bb.max.X - bb.min.X:.3f} x {bb.max.Y - bb.min.Y:.3f} x {bb.max.Z - bb.min.Z:.3f}")

    exp = TopExp_Explorer(housing.wrapped, TopAbs_ShapeEnum.TopAbs_FACE)
    face_count = 0
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        face_count += 1
        n = get_face_normal(face)

        # --- TRIPOD: cylindrical faces ---
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() == GeomAbs_Cylinder:
            cyl = surf.Cylinder()
            r = cyl.Radius()
            loc = cyl.Location()
            fb = get_face_bbox(face)
            if 2.5 <= r <= 5.5 and abs(loc.X()) <= 8.0:
                tripod_candidates.append({
                    "x": loc.X(), "y": loc.Y(), "z": loc.Z(), "r": r,
                    "side": "neg" if loc.Y() < 0 else "pos",
                    "bbox_sz": fb["sz"],
                })

        # --- VENTS: faces with inner wire loops on side faces ---
        nwires = count_wires(face)
        if nwires >= 2 and abs(n[2]) < 0.70:
            inner_bbs = get_inner_wire_bboxes(face)
            for ibb in inner_bbs:
                dims = sorted([ibb["sx"], ibb["sy"], ibb["sz"]])
                d_long, d_mid, d_small = dims[2], dims[1], dims[0]
                if 3.0 <= d_long <= 20.0 and 0.5 <= d_mid <= 4.0 and d_small <= 2.0:
                    if d_long / d_mid >= 1.5:
                        if abs(n[1]) >= abs(n[0]):
                            axis, side = "y", ("neg" if ibb["cy"] < 0 else "pos")
                        else:
                            axis, side = "x", ("neg" if ibb["cx"] < 0 else "pos")
                        vents.append({
                            "axis": axis, "side": side,
                            "x": ibb["cx"], "y": ibb["cy"], "z": ibb["cz"],
                            "slot_long": d_long, "slot_mid": d_mid,
                        })
        exp.Next()

    # Best tripod: closest to 3.175mm radius, on neg-Y side, in mid-body Z range
    tripod = None
    print(f"  All tripod cylinder candidates:")
    for c in tripod_candidates:
        print(f"    r={c['r']:.3f} at ({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f}) "
              f"side={c['side']} bbox_sz={c['bbox_sz']:.2f}")

    # Filter: must be on neg-Y, Z in mid-body range (-110 to -20), centerline X < 6
    neg_candidates = [c for c in tripod_candidates
                      if c["side"] == "neg"
                      and c["bbox_sz"] >= 2.0
                      and -110.0 <= c["z"] <= -20.0
                      and abs(c["x"]) <= 6.0
                      and 2.5 <= c["r"] <= 5.0]
    if neg_candidates:
        tripod = min(neg_candidates, key=lambda c: abs(c["r"] - 3.175))
    print(f"  Filtered tripod candidates: {len(neg_candidates)}")

    # Deduplicate vents
    seen = set()
    unique = []
    for v in vents:
        key = (v["axis"], v["side"], round(v["x"], 1), round(v["y"], 1), round(v["z"], 1))
        if key not in seen:
            seen.add(key)
            unique.append(v)

    print(f"  Scanned {face_count} faces, {len(vents)} raw vents -> {len(unique)} unique")
    if tripod:
        print(f"  Tripod: r={tripod['r']:.3f} at ({tripod['x']:.3f}, {tripod['y']:.3f}, {tripod['z']:.3f})")

    return unique, tripod, (dev_zmin, dev_zmax)


def transform_to_case_frame(vents, tripod, dev_zrange):
    """
    Transform device features into case coordinate frame.

    The case generator uses:
      map_z(z_dev) = cavity_front_z + (zmax - z_dev) * sz + clearance
                   = 3.0 + (zmax - z_dev) * sz + 2.3

    The fit validator uses:
      mirror XY (z -> -z), then translate so device front at z = 5.3
      z_case = (zmax - z_dev) + 5.3  (when sz=1)

    But sz = 120.32 / (zmax - zmin) which is NOT exactly 1.0.
    Similarly sx = 56.99 / raw_width, sy = 56.99 / raw_height.

    We have two choices:
      A) Use the physical device positions (no scaling) - this is what
         physically happens when you insert the camera
      B) Use scaled positions - this is what the case generator uses
         to decide where to cut

    We do BOTH and compare, to find discrepancies.
    """
    dev_zmin, dev_zmax = dev_zrange

    # Physical transform (what actually happens when device goes in case)
    # Mirror XY + translate Z so device front at cavity_front_z + clearance = 5.3
    def physical_z(z_dev):
        return (dev_zmax - z_dev) + 5.3

    # Scaled transform (what the case generator uses to place cuts)
    raw_w = 55.978  # from report profile_raw_mm
    raw_h = 55.978
    raw_l = dev_zmax - dev_zmin  # ~114.8
    sx = 56.99 / raw_w  # ~1.0181
    sy = 56.99 / raw_h
    sz = 120.32 / raw_l  # ~1.0018 (from report)

    def scaled_z(z_dev):
        return 3.0 + (dev_zmax - z_dev) * sz + 2.3

    def scaled_x(x_dev):
        return x_dev * sx

    def scaled_y(y_dev):
        return y_dev * sy

    physical = []
    scaled = []
    for v in vents:
        pv = dict(v)
        pv["z_case"] = physical_z(v["z"])
        pv["x_case"] = v["x"]
        pv["y_case"] = v["y"]
        physical.append(pv)

        sv = dict(v)
        sv["z_case"] = scaled_z(v["z"])
        sv["x_case"] = scaled_x(v["x"])
        sv["y_case"] = scaled_y(v["y"])
        scaled.append(sv)

    phys_tripod = scaled_tripod = None
    if tripod:
        phys_tripod = dict(tripod)
        phys_tripod["z_case"] = physical_z(tripod["z"])
        phys_tripod["x_case"] = tripod["x"]
        phys_tripod["y_case"] = tripod["y"]

        scaled_tripod = dict(tripod)
        scaled_tripod["z_case"] = scaled_z(tripod["z"])
        scaled_tripod["x_case"] = scaled_x(tripod["x"])
        scaled_tripod["y_case"] = scaled_y(tripod["y"])

    # Report the scaling error (difference between physical and scaled)
    print(f"\n  Scale factors: sx={sx:.6f} sy={sy:.6f} sz={sz:.6f}")
    if physical and scaled:
        z_diffs = [abs(p["z_case"] - s["z_case"]) for p, s in zip(physical, scaled)]
        x_diffs = [abs(p["x_case"] - s["x_case"]) for p, s in zip(physical, scaled)]
        print(f"  Max physical-vs-scaled Z difference: {max(z_diffs):.3f} mm")
        print(f"  Max physical-vs-scaled X difference: {max(x_diffs):.3f} mm")

    return physical, scaled, phys_tripod, scaled_tripod


def load_report_vents():
    """Load case vent positions from the dual-material report."""
    with open(DUAL_REPORT) as f:
        report = json.load(f)

    asa_entries = report["report"]["sources"]["asa_case_report"]["step_side_features"]["vents_applied_entries"]
    tpu_entries = report["report"]["sources"]["tpu_sleeve_report"]["step_side_features"]["vents_applied_entries"]
    asa_tripod = report["report"]["sources"]["asa_case_report"]["step_side_features"]["tripod_applied"]
    tpu_tripod = report["report"]["sources"]["tpu_sleeve_report"]["step_side_features"]["tripod_applied"]
    tpu_z_offset = report["report"]["alignment_mm"]["tpu_z_offset_into_asa"]

    return asa_entries, tpu_entries, asa_tripod, tpu_tripod, tpu_z_offset


def match_vents(dev_vents, case_vents, label, case_is_tpu=False, tpu_z_offset=0.0):
    """
    Match device vents (in case frame) against case vent positions (from report).
    Case report positions are in the body assembly frame (ASA frame).
    TPU vents need tpu_z_offset added to get them into ASA frame.
    """
    print(f"\n  === {label}: {len(dev_vents)} device vents vs {len(case_vents)} case vents ===")

    matches = []
    used_case = set()

    for dv in dev_vents:
        dz = dv["z_case"]
        best_dist = float("inf")
        best_cv = None
        best_idx = -1

        for ci, cv in enumerate(case_vents):
            if ci in used_case:
                continue

            cz = cv["z"]
            if case_is_tpu:
                cz += tpu_z_offset  # TPU vents stored in TPU frame, shift to body frame

            # Match axis and side
            if cv["axis"] != dv["axis"] or cv["side"] != dv["side"]:
                continue

            # Distance metric
            if dv["axis"] == "y":
                dist = math.sqrt((dz - cz) ** 2 + (dv["x_case"] - cv["x"]) ** 2)
            else:
                dist = math.sqrt((dz - cz) ** 2 + (dv["y_case"] - cv["y"]) ** 2)

            if dist < best_dist:
                best_dist = dist
                best_cv = cv
                best_idx = ci

        if best_cv and best_dist < 30.0:
            used_case.add(best_idx)
            cz = best_cv["z"] + (tpu_z_offset if case_is_tpu else 0.0)
            delta_z = dz - cz
            if dv["axis"] == "y":
                delta_t = dv["x_case"] - best_cv["x"]
            else:
                delta_t = dv["y_case"] - best_cv["y"]
            matches.append({
                "dev": dv, "case": best_cv,
                "delta_z": delta_z, "delta_tangent": delta_t,
                "dist": best_dist,
                "pattern": best_cv.get("pattern_source", "?"),
            })

    unmatched_dev = len(dev_vents) - len(matches)
    unmatched_case = len(case_vents) - len(used_case)

    print(f"  Matched: {len(matches)}, Unmatched dev: {unmatched_dev}, Unmatched case: {unmatched_case}")

    if matches:
        # Group by pattern source
        by_pattern = defaultdict(list)
        for m in matches:
            by_pattern[m["pattern"]].append(m)

        for pattern, group in sorted(by_pattern.items()):
            z_deltas = [m["delta_z"] for m in group]
            t_deltas = [m["delta_tangent"] for m in group]
            print(f"\n  Pattern: {pattern} ({len(group)} vents)")
            print(f"    Z deltas:  min={min(z_deltas):.3f}  max={max(z_deltas):.3f}  "
                  f"mean={sum(z_deltas)/len(z_deltas):.3f}")
            print(f"    T deltas:  min={min(t_deltas):.3f}  max={max(t_deltas):.3f}  "
                  f"mean={sum(t_deltas)/len(t_deltas):.3f}")

            # Are all Z deltas the same? (systematic offset)
            z_spread = max(z_deltas) - min(z_deltas)
            if z_spread < 0.1 and abs(sum(z_deltas)/len(z_deltas)) > 0.3:
                mean_z = sum(z_deltas) / len(z_deltas)
                print(f"    >>> SYSTEMATIC Z OFFSET: {mean_z:.3f} mm <<<")

        # Overall stats
        all_z = [abs(m["delta_z"]) for m in matches]
        all_t = [abs(m["delta_tangent"]) for m in matches]
        print(f"\n  Overall max |Δz|={max(all_z):.3f}  max |Δt|={max(all_t):.3f}")
        print(f"  Pass threshold: 1.0 mm")
        status = "PASS" if max(all_z) < 1.0 and max(all_t) < 1.0 else "FAIL"
        print(f"  Status: {status}")
        return matches, status == "PASS"

    return matches, False


def check_collision(solid_a, solid_b, label):
    """Check intersection volume between two solids."""
    try:
        common = BRepAlgoAPI_Common(solid_a.wrapped, solid_b.wrapped)
        if common.IsDone():
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(common.Shape(), props)
            vol = abs(props.Mass())
            status = "PASS" if vol < 0.1 else f"FAIL ({vol:.2f} mm³)"
            print(f"  {label}: {status}")
            return vol
    except Exception as e:
        print(f"  {label}: ERROR - {e}")
    return -1


def main():
    print("=" * 72)
    print("INDEPENDENT MAKI LIVE FIT VALIDATION (v2)")
    print("=" * 72)

    # ═══ 1. LOAD DEVICE ═══
    print("\n[1] Loading device STEP...")
    dev_shape = import_step(str(DEVICE_STEP))
    housing = max(dev_shape.solids(), key=lambda s: s.volume)

    # ═══ 2. EXTRACT DEVICE FEATURES ═══
    print("\n[2] Extracting device features from B-REP...")
    dev_vents, dev_tripod, dev_zrange = extract_device_features(housing)

    # Classify
    y_neg = [v for v in dev_vents if v["axis"] == "y" and v["side"] == "neg"]
    x_all = [v for v in dev_vents if v["axis"] == "x"]
    y_pos = [v for v in dev_vents if v["axis"] == "y" and v["side"] == "pos"]
    print(f"\n  Classification:")
    print(f"    Y-neg (bottom/tripanel): {len(y_neg)}")
    print(f"    Y-pos (top): {len(y_pos)}")
    print(f"    X-neg (left side trio): {len([v for v in x_all if v['side']=='neg'])}")
    print(f"    X-pos (right side trio): {len([v for v in x_all if v['side']=='pos'])}")

    # ═══ 3. TRANSFORM TO CASE FRAME ═══
    print("\n[3] Transforming to case frame (physical + scaled)...")
    phys_vents, scaled_vents, phys_tripod, scaled_tripod = transform_to_case_frame(
        dev_vents, dev_tripod, dev_zrange
    )

    # Print all device vent positions
    print("\n  All device vents (PHYSICAL frame, sorted by Z):")
    for v in sorted(phys_vents, key=lambda v: v["z_case"]):
        print(f"    {v['axis']}/{v['side']:3s}  x={v['x_case']:8.3f}  y={v['y_case']:8.3f}  z={v['z_case']:8.3f}")

    print("\n  All device vents (SCALED frame, sorted by Z):")
    for v in sorted(scaled_vents, key=lambda v: v["z_case"]):
        print(f"    {v['axis']}/{v['side']:3s}  x={v['x_case']:8.3f}  y={v['y_case']:8.3f}  z={v['z_case']:8.3f}")

    # ═══ 4. LOAD CASE REPORT DATA ═══
    print("\n[4] Loading case vent positions from dual-material report...")
    asa_entries, tpu_entries, asa_tripod, tpu_tripod, tpu_z_offset = load_report_vents()
    print(f"  ASA vents: {len(asa_entries)}")
    print(f"  TPU vents: {len(tpu_entries)}")
    print(f"  TPU Z-offset into ASA: {tpu_z_offset:.3f} mm")

    # Print ASA vent positions
    print("\n  ASA case vent positions (from report, sorted by Z):")
    for v in sorted(asa_entries, key=lambda v: v["z"]):
        src = v.get("pattern_source", "?")
        print(f"    {v['axis']}/{v['side']:3s}  x={v['x']:8.3f}  y={v['y']:8.3f}  z={v['z']:8.3f}  [{src}]")

    # ═══ 5. MATCH: PHYSICAL device vents vs ASA case cuts ═══
    print("\n" + "=" * 72)
    print("[5] PHYSICAL device position vs ASA case cuts")
    print("    (where the camera vents actually are vs where case holes are)")
    phys_asa_matches, phys_asa_ok = match_vents(phys_vents, asa_entries, "Physical vs ASA")

    # ═══ 6. MATCH: SCALED device vents vs ASA case cuts ═══
    print("\n" + "=" * 72)
    print("[6] SCALED device position vs ASA case cuts")
    print("    (same mapping pipeline the generator uses)")
    scaled_asa_matches, scaled_asa_ok = match_vents(scaled_vents, asa_entries, "Scaled vs ASA")

    # ═══ 7. MATCH: PHYSICAL device vents vs TPU case cuts ═══
    print("\n" + "=" * 72)
    print("[7] PHYSICAL device position vs TPU case cuts")
    phys_tpu_matches, phys_tpu_ok = match_vents(
        phys_vents, tpu_entries, "Physical vs TPU",
        case_is_tpu=True, tpu_z_offset=tpu_z_offset
    )

    # ═══ 8. TRIPOD CHECK ═══
    print("\n" + "=" * 72)
    print("[8] TRIPOD ALIGNMENT")
    if phys_tripod:
        print(f"\n  Device tripod (physical): x={phys_tripod['x_case']:.3f} "
              f"y={phys_tripod['y_case']:.3f} z={phys_tripod['z_case']:.3f} r={phys_tripod['r']:.3f}")
        print(f"  Device tripod (scaled):   x={scaled_tripod['x_case']:.3f} "
              f"y={scaled_tripod['y_case']:.3f} z={scaled_tripod['z_case']:.3f}")

        print(f"\n  ASA case tripod: x={asa_tripod['x']:.3f} z={asa_tripod['z']:.3f} "
              f"d={asa_tripod['diameter']:.3f}")
        print(f"  TPU case tripod: x={tpu_tripod['x']:.3f} z={tpu_tripod['z']:.3f} "
              f"d={tpu_tripod['diameter']:.3f}")

        # Physical comparison
        dz_asa = phys_tripod["z_case"] - asa_tripod["z"]
        dx_asa = phys_tripod["x_case"] - asa_tripod["x"]
        print(f"\n  Physical device vs ASA tripod:")
        print(f"    Δx = {dx_asa:.3f} mm, Δz = {dz_asa:.3f} mm")
        print(f"    Status: {'PASS' if abs(dz_asa) < 1.0 and abs(dx_asa) < 1.0 else 'FAIL'}")

        # Scaled comparison
        dz_asa_s = scaled_tripod["z_case"] - asa_tripod["z"]
        dx_asa_s = scaled_tripod["x_case"] - asa_tripod["x"]
        print(f"\n  Scaled device vs ASA tripod:")
        print(f"    Δx = {dx_asa_s:.3f} mm, Δz = {dz_asa_s:.3f} mm")
        print(f"    Status: {'PASS' if abs(dz_asa_s) < 1.0 and abs(dx_asa_s) < 1.0 else 'FAIL'}")

        # TPU comparison (TPU tripod z is in TPU frame, add offset)
        tpu_tripod_z_in_asa = tpu_tripod["z"] + tpu_z_offset
        dz_tpu = phys_tripod["z_case"] - tpu_tripod_z_in_asa
        print(f"\n  Physical device vs TPU tripod (z adjusted by +{tpu_z_offset:.1f}):")
        print(f"    Δz = {dz_tpu:.3f} mm")
        print(f"    Status: {'PASS' if abs(dz_tpu) < 1.0 else 'FAIL'}")
    else:
        print("  WARNING: No tripod detected on device!")

    # ═══ 9. COLLISION CHECKS ═══
    print("\n" + "=" * 72)
    print("[9] COLLISION CHECKS")

    # Transform device into case frame
    dev_mirrored = housing.mirror(Plane.XY)
    dev_bb = dev_mirrored.bounding_box()
    tz = 5.3 - dev_bb.min.Z
    dev_in_case = dev_mirrored.translate((0, 0, tz))
    dbb = dev_in_case.bounding_box()
    print(f"  Device in case frame: X[{dbb.min.X:.2f},{dbb.max.X:.2f}] "
          f"Y[{dbb.min.Y:.2f},{dbb.max.Y:.2f}] Z[{dbb.min.Z:.2f},{dbb.max.Z:.2f}]")

    # Load body solids
    body_shape = import_step(str(BODY_STEP))
    body_solids = {}
    for s in body_shape.solids():
        sbb = s.bounding_box()
        w = sbb.max.X - sbb.min.X
        label = "body_ASA" if w > 64 else "body_TPU"
        body_solids[label] = s
        print(f"  {label} BB: X[{sbb.min.X:.2f},{sbb.max.X:.2f}] "
              f"Y[{sbb.min.Y:.2f},{sbb.max.Y:.2f}] Z[{sbb.min.Z:.2f},{sbb.max.Z:.2f}]")

    # Load cap and place it
    cap_shape = import_step(str(CAP_STEP))
    shell_depth = 127.92
    cap_thickness = 3.0
    cap_solids = {}
    for cs in cap_shape.solids():
        cs_placed = cs.rotate(Axis.Y, 180).translate((0, 0, shell_depth + cap_thickness))
        csbb = cs_placed.bounding_box()
        w = csbb.max.X - csbb.min.X
        label = "cap_ASA" if w > 64 else "cap_TPU"
        cap_solids[label] = cs_placed

    print()
    all_solids = {**body_solids, **cap_solids}
    collision_vols = {}
    for label, solid in all_solids.items():
        vol = check_collision(dev_in_case, solid, f"Device vs {label}")
        collision_vols[label] = vol

    # ═══ 10. FINAL SUMMARY ═══
    print("\n" + "=" * 72)
    print("FINAL SUMMARY")
    print("=" * 72)

    issues = []

    # Collision issues
    for label, vol in collision_vols.items():
        if vol > 0.1:
            issues.append(f"COLLISION: Device vs {label} = {vol:.2f} mm³")

    # Vent alignment (physical = what matters for real fitment)
    if phys_asa_matches:
        max_z = max(abs(m["delta_z"]) for m in phys_asa_matches)
        max_t = max(abs(m["delta_tangent"]) for m in phys_asa_matches)
        if max_z > 1.0:
            issues.append(f"ASA vent Z misalignment: max {max_z:.2f} mm")
        if max_t > 1.0:
            issues.append(f"ASA vent tangent misalignment: max {max_t:.2f} mm")

        # Check for systematic offset per vent group
        by_pattern = defaultdict(list)
        for m in phys_asa_matches:
            by_pattern[m["pattern"]].append(m)
        for pattern, group in by_pattern.items():
            mean_dz = sum(m["delta_z"] for m in group) / len(group)
            if abs(mean_dz) > 0.5:
                issues.append(f"ASA {pattern}: systematic Z offset = {mean_dz:.2f} mm ({len(group)} vents)")

    # Tripod
    if phys_tripod:
        dz_asa = phys_tripod["z_case"] - asa_tripod["z"]
        if abs(dz_asa) > 1.0:
            issues.append(f"ASA tripod Z misalignment: {dz_asa:.2f} mm")

    if issues:
        print("\nISSUES FOUND:")
        for issue in issues:
            print(f"  >> {issue}")
        print(f"\nTotal: {len(issues)} issues")
    else:
        print("\n  All checks passed!")

    return len(issues) == 0


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)
