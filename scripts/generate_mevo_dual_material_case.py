#!/usr/bin/env python3
"""Generate a Mevo Start dual-material case plus separate ASA back cap.

Primary outputs:
- models/mevo_case/mevo_start_body_dual_material.step
  - contains two named bodies: TPU_Sleeve and ASA_Shell
- models/mevo_case/mevo_start_back_cap_asa.step
  - separate pure-ASA back cap
- models/mevo_case/reports/mevo_start_dual_material_report.json

This generator implements the reviewed design spec while preserving
the ovular/capsule Mevo front profile style:
- TPU inner cavity: 34.3 x 50.3 x 85.0 mm
- TPU wall: 1.8 mm
- ASA wall: 2.2 mm
- Interface gap TPU<->ASA: 0.0 mm
- Front profile: open ovular/capsule opening by default (no front circular cutout)
- Bottom tripod hole: 20.5 mm dia, center at Z=43.2 mm from front face
- Back cap lip depth: 5.0 mm, lip undersize: 0.1 mm vs ASA opening
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import Box, BuildPart, BuildSketch, Circle, Compound, Locations, Mode, Plane, Rectangle, export_step, extrude, fillet


@dataclass
class DualMaterialParams:
    # Device nominal for reference
    device_nominal_w_mm: float = 34.0
    device_nominal_h_mm: float = 50.0
    device_nominal_l_mm: float = 87.0

    # Core reviewed dimensions
    tpu_inner_w_mm: float = 34.30
    tpu_inner_h_mm: float = 50.30
    tpu_inner_depth_mm: float = 85.00
    tpu_wall_mm: float = 1.80

    asa_wall_mm: float = 2.20
    interface_gap_mm: float = 0.0

    # Profile style: keep the Mevo ovular/capsule cross-section.
    enforce_capsule_profile: bool = True

    # Front behavior
    open_front_ovular: bool = True
    include_front_lens_led_cutouts: bool = False
    sun_hood_depth_mm: float = 3.0
    lens_cutout_d_mm: float = 32.0
    led_hole_d_mm: float = 3.0
    led_hole_offset_y_mm: float = 12.0

    # Bottom tripod well
    tripod_hole_d_mm: float = 20.50
    tripod_center_from_front_mm: float = 43.20

    # Optional side vents (kept for prior workflow compatibility)
    vent_count: int = 3
    vent_h_mm: float = 2.2
    vent_len_mm: float = 12.0
    vent_pitch_mm: float = 4.8
    vent_z_ratio: float = 0.57
    include_side_vents: bool = True

    # Back cap (pure ASA)
    back_cap_thickness_mm: float = 3.0
    back_cap_lip_depth_mm: float = 5.0
    back_cap_lip_undersize_total_mm: float = 0.10
    back_cap_edge_fillet_mm: float = 0.6

    # Single utility slot on back cap
    utility_slot_width_mm: float = 15.0
    utility_slot_top_margin_mm: float = 15.0
    utility_slot_bottom_margin_mm: float = 10.0


def _add_vertical_stadium(width: float, height: float) -> None:
    """Centered stadium aligned with Y axis (vertical)."""
    if height <= width:
        Circle(0.5 * width)
        return
    straight = height - width
    Rectangle(width, straight)
    with Locations((0.0, 0.5 * straight)):
        Circle(0.5 * width)
    with Locations((0.0, -0.5 * straight)):
        Circle(0.5 * width)


def _add_profile(width: float, height: float, capsule: bool) -> None:
    if capsule:
        _add_vertical_stadium(width, height)
        return
    Rectangle(width, height)


def _largest_solid(shape):
    solids = shape.solids() if hasattr(shape, "solids") else []
    if len(solids) <= 1:
        return shape
    return max(solids, key=lambda s: s.volume)


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


def _derived(p: DualMaterialParams) -> dict:
    tpu_outer_w = p.tpu_inner_w_mm + 2.0 * p.tpu_wall_mm
    tpu_outer_h = p.tpu_inner_h_mm + 2.0 * p.tpu_wall_mm

    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    # For capsule profile, effective cap radius is half of the minor axis.
    tpu_outer_profile_r = 0.5 * min(tpu_outer_w, tpu_outer_h)
    asa_inner_profile_r = 0.5 * min(asa_inner_w, asa_inner_h)
    asa_outer_profile_r = 0.5 * min(asa_outer_w, asa_outer_h)

    if p.open_front_ovular:
        cavity_start_z = 0.0
        body_depth = p.tpu_inner_depth_mm
    else:
        cavity_start_z = p.sun_hood_depth_mm
        body_depth = p.sun_hood_depth_mm + p.tpu_inner_depth_mm

    utility_slot_h = max(
        asa_outer_h - p.utility_slot_top_margin_mm - p.utility_slot_bottom_margin_mm,
        p.utility_slot_width_mm,
    )
    utility_slot_center_y = 0.5 * (
        (0.5 * asa_outer_h - p.utility_slot_top_margin_mm)
        + (-0.5 * asa_outer_h + p.utility_slot_bottom_margin_mm)
    )

    return {
        "tpu_outer_w_mm": tpu_outer_w,
        "tpu_outer_h_mm": tpu_outer_h,
        "asa_inner_w_mm": asa_inner_w,
        "asa_inner_h_mm": asa_inner_h,
        "asa_outer_w_mm": asa_outer_w,
        "asa_outer_h_mm": asa_outer_h,
        "tpu_outer_profile_r_mm": tpu_outer_profile_r,
        "asa_inner_profile_r_mm": asa_inner_profile_r,
        "asa_outer_profile_r_mm": asa_outer_profile_r,
        "cavity_start_z_mm": cavity_start_z,
        "body_depth_mm": body_depth,
        "utility_slot_h_mm": utility_slot_h,
        "utility_slot_center_y_mm": utility_slot_center_y,
    }


def build_dual_material_body(p: DualMaterialParams):
    d = _derived(p)

    min_y_asa = -0.5 * d["asa_outer_h_mm"]
    max_x_asa = 0.5 * d["asa_outer_w_mm"]
    min_y_tpu = -0.5 * d["tpu_outer_h_mm"]

    cavity_start_z = d["cavity_start_z_mm"]
    cavity_depth = p.tpu_inner_depth_mm

    vent_z = cavity_start_z + cavity_depth * p.vent_z_ratio
    tripod_z = p.tripod_center_from_front_mm

    # ASA bucket body (front wall + side shell, open at rear)
    with BuildPart() as asa_bp:
        with BuildSketch(Plane.XY):
            _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=d["body_depth_mm"])

        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            _add_profile(d["asa_inner_w_mm"], d["asa_inner_h_mm"], p.enforce_capsule_profile)
        extrude(amount=cavity_depth + 0.2, mode=Mode.SUBTRACT)

        # Optional lens/LED openings when a closed front wall is used.
        if (not p.open_front_ovular) and p.include_front_lens_led_cutouts:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((0.0, 0.0)):
                    Circle(0.5 * p.lens_cutout_d_mm)
                with Locations((0.0, p.led_hole_offset_y_mm)):
                    Circle(0.5 * p.led_hole_d_mm)
            extrude(amount=p.sun_hood_depth_mm + 0.6, mode=Mode.SUBTRACT)

        # Optional side vents.
        if p.include_side_vents:
            for side in (-1.0, 1.0):
                x = side * (max_x_asa - p.asa_wall_mm * 0.55)
                for i in range(p.vent_count):
                    y = (i - (p.vent_count - 1) * 0.5) * p.vent_pitch_mm
                    with Locations((x, y, vent_z)):
                        Box(
                            p.asa_wall_mm * 2.4,
                            p.vent_h_mm,
                            p.vent_len_mm,
                            mode=Mode.SUBTRACT,
                        )

        # Bottom tripod hole through ASA.
        with BuildSketch(Plane.XZ.offset(min_y_asa + 0.12)):
            with Locations((0.0, tripod_z)):
                Circle(0.5 * p.tripod_hole_d_mm)
        extrude(amount=-(p.asa_wall_mm + 2.0), mode=Mode.SUBTRACT)

    asa_shell = _largest_solid(asa_bp.part)
    asa_shell.label = "ASA_Shell"

    # TPU sleeve fused in body region only (starts after sun hood).
    with BuildPart() as tpu_bp:
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            _add_profile(d["tpu_outer_w_mm"], d["tpu_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=cavity_depth)

        with BuildSketch(Plane.XY.offset(cavity_start_z - 0.2)):
            _add_profile(p.tpu_inner_w_mm, p.tpu_inner_h_mm, p.enforce_capsule_profile)
        extrude(amount=cavity_depth + 0.4, mode=Mode.SUBTRACT)

        # Bottom tripod hole through TPU too.
        with BuildSketch(Plane.XZ.offset(min_y_tpu + 0.12)):
            with Locations((0.0, tripod_z)):
                Circle(0.5 * p.tripod_hole_d_mm)
        extrude(amount=-(p.tpu_wall_mm + 1.5), mode=Mode.SUBTRACT)

    tpu_sleeve = _largest_solid(tpu_bp.part)
    tpu_sleeve.label = "TPU_Sleeve"

    assembly = Compound(children=[tpu_sleeve, asa_shell], label="Mevo_Body_Assembly")

    report = {
        "derived_mm": d,
        "features_mm": {
            "open_front_ovular": bool(p.open_front_ovular),
            "include_front_lens_led_cutouts": bool(p.include_front_lens_led_cutouts),
            "lens_cutout_d": float(p.lens_cutout_d_mm),
            "led_hole_d": float(p.led_hole_d_mm),
            "led_hole_offset_y": float(p.led_hole_offset_y_mm),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "tripod_hole_d": float(p.tripod_hole_d_mm),
            "tripod_center_from_front": float(p.tripod_center_from_front_mm),
        },
        "named_bodies": ["TPU_Sleeve", "ASA_Shell"],
        "warnings": [],
    }

    return assembly, report


def build_back_cap(p: DualMaterialParams):
    d = _derived(p)

    lip_w = max(d["asa_inner_w_mm"] - p.back_cap_lip_undersize_total_mm, 2.0)
    lip_h = max(d["asa_inner_h_mm"] - p.back_cap_lip_undersize_total_mm, 2.0)

    slot_w = p.utility_slot_width_mm
    slot_h = d["utility_slot_h_mm"]
    slot_center_y = d["utility_slot_center_y_mm"]

    with BuildPart() as cap_bp:
        with BuildSketch(Plane.XY):
            _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=p.back_cap_thickness_mm)

        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
            _add_profile(lip_w, lip_h, p.enforce_capsule_profile)
        extrude(amount=p.back_cap_lip_depth_mm)

        cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + 1.0
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((0.0, slot_center_y)):
                _add_vertical_stadium(slot_w, slot_h)
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    cap = _largest_solid(cap_bp.part)
    try:
        cap = fillet(cap.edges(), p.back_cap_edge_fillet_mm)
    except Exception:
        pass
    cap = _largest_solid(cap)
    cap.label = "ASA_Back_Cap"

    report = {
        "back_cap_mm": {
            "plate_w": float(d["asa_outer_w_mm"]),
            "plate_h": float(d["asa_outer_h_mm"]),
            "lip_w": float(lip_w),
            "lip_h": float(lip_h),
            "lip_depth": float(p.back_cap_lip_depth_mm),
            "thickness": float(p.back_cap_thickness_mm),
            "lip_undersize_total": float(p.back_cap_lip_undersize_total_mm),
        },
        "utility_slot_mm": {
            "shape": "stadium",
            "width": float(slot_w),
            "height": float(slot_h),
            "center_y": float(slot_center_y),
            "top_margin": float(p.utility_slot_top_margin_mm),
            "bottom_margin": float(p.utility_slot_bottom_margin_mm),
        },
    }

    return cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate reviewed Mevo dual-material body + ASA back cap")
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument("--closed-front", action="store_true", help="Use a closed front wall (bucket mode).")
    parser.add_argument(
        "--enable-front-lens-led-cutouts",
        action="store_true",
        help="When used with --closed-front, cut lens and LED holes into the front wall.",
    )
    parser.add_argument(
        "--disable-capsule-profile",
        action="store_true",
        help="Use rectangular profile instead of ovular capsule profile.",
    )
    args = parser.parse_args()

    p = DualMaterialParams()
    p.open_front_ovular = not bool(args.closed_front)
    p.include_front_lens_led_cutouts = bool(args.enable_front_lens_led_cutouts)
    p.enforce_capsule_profile = not bool(args.disable_capsule_profile)

    body_asm, body_report = build_dual_material_body(p)
    back_cap, cap_report = build_back_cap(p)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    body_step = args.out / "mevo_start_body_dual_material.step"
    cap_step = args.out / "mevo_start_back_cap_asa.step"
    report_json = reports_dir / "mevo_start_dual_material_report.json"

    archived = _archive_existing([body_step, cap_step, report_json], args.out)

    export_step(body_asm, str(body_step))
    export_step(back_cap, str(cap_step))

    payload = {
        "params": asdict(p),
        "body_report": body_report,
        "back_cap_report": cap_report,
    }
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {body_step}")
    print(f"Wrote {cap_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
