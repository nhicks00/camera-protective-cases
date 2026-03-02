#!/usr/bin/env python3
"""Generate a Mevo Start dual-material case plus separate back-cap assembly.

Primary outputs:
- models/mevo_case/mevo_start_body_dual_material.step
  - contains two named bodies: TPU_Sleeve and ASA_Shell
- models/mevo_case/mevo_start_back_cap_dual_material.step
  - contains two named bodies: ASA_Back_Cap and TPU_Back_Gasket
- models/mevo_case/mevo_start_back_cap_asa.step
  - compatibility export of ASA-only cap body
- models/mevo_case/reports/mevo_start_dual_material_report.json

This generator implements the reviewed design spec while preserving
the ovular/capsule Mevo cross-section profile:
- TPU inner cavity: 34.3 x 75.8 x 87.3 mm (0.15 mm fit clearance around 34 x 75.5 x 87)
- TPU wall: 1.8 mm
- ASA wall: 2.2 mm
- Interface gap TPU<->ASA: 0.0 mm
- Front profile: integrated front wall by default with offset lens opening
- Bottom tripod hole: 20.5 mm dia, center at Z=43.2 mm from front face
- Back cap lip depth: 5.0 mm, TPU-aware fit clearance: 0.28 mm total
- Back cap engagement: two-stage tongue + body rear groove seat
- Back utility slot is disabled by default (enable explicitly when port mapping is confirmed)
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Locations,
    Mode,
    Plane,
    Rectangle,
    export_step,
    extrude,
    fillet,
)


@dataclass
class DualMaterialParams:
    # Device nominal for reference
    device_nominal_w_mm: float = 34.0
    device_nominal_h_mm: float = 75.5
    device_nominal_l_mm: float = 87.0

    # Core fit dimensions (snug: nominal + 0.15 mm per side/end)
    tpu_inner_w_mm: float = 34.30
    tpu_inner_h_mm: float = 75.80
    tpu_inner_depth_mm: float = 87.30
    tpu_wall_mm: float = 1.80

    asa_wall_mm: float = 2.20
    interface_gap_mm: float = 0.0

    # Profile style: keep the Mevo ovular/capsule cross-section.
    enforce_capsule_profile: bool = True

    # Front behavior
    open_front_ovular: bool = False
    include_front_lens_led_cutouts: bool = True
    sun_hood_depth_mm: float = 3.0
    lens_cutout_d_mm: float = 32.0
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 16.5
    led_hole_d_mm: float = 3.0
    led_hole_offset_from_lens_y_mm: float = 12.0
    tpu_front_edge_wrap_depth_mm: float = 2.5
    tpu_front_edge_wrap_radial_mm: float = 2.0
    include_tpu_front_edge_wrap: bool = True
    include_tpu_rear_edge_wrap: bool = False

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

    # Back cap engagement geometry
    back_cap_thickness_mm: float = 3.0
    back_cap_lip_depth_mm: float = 5.0
    back_cap_lip_undersize_total_mm: float = 0.28
    back_cap_tongue_depth_mm: float = 1.4
    back_cap_tongue_radial_step_mm: float = 0.30
    back_cap_groove_radial_clearance_mm: float = 0.08
    back_cap_groove_axial_clearance_mm: float = 0.15
    back_cap_edge_fillet_mm: float = 0.6

    # Optional TPU gasket on cap inner face (recommended)
    include_back_cap_tpu_gasket: bool = True
    back_cap_tpu_gasket_thickness_mm: float = 1.2
    back_cap_tpu_gasket_outer_inset_mm: float = 1.0
    back_cap_tpu_gasket_ring_width_mm: float = 1.6

    # Single utility slot on back cap
    include_back_utility_slot: bool = False
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

    # Two-stage cap tongue and matching rear body groove.
    lip_tip_w = max(asa_inner_w - p.back_cap_lip_undersize_total_mm, 2.0)
    lip_tip_h = max(asa_inner_h - p.back_cap_lip_undersize_total_mm, 2.0)
    tongue_w = lip_tip_w + 2.0 * p.back_cap_tongue_radial_step_mm
    tongue_h = lip_tip_h + 2.0 * p.back_cap_tongue_radial_step_mm
    tongue_depth = min(max(p.back_cap_tongue_depth_mm, 0.6), max(p.back_cap_lip_depth_mm - 0.8, 0.6))
    lip_tip_depth = max(p.back_cap_lip_depth_mm - tongue_depth, 0.6)

    groove_w = tongue_w + 2.0 * p.back_cap_groove_radial_clearance_mm
    groove_h = tongue_h + 2.0 * p.back_cap_groove_radial_clearance_mm
    groove_depth = tongue_depth + p.back_cap_groove_axial_clearance_mm
    groove_start_z = body_depth - groove_depth

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
        "lip_tip_w_mm": lip_tip_w,
        "lip_tip_h_mm": lip_tip_h,
        "tongue_w_mm": tongue_w,
        "tongue_h_mm": tongue_h,
        "tongue_depth_mm": tongue_depth,
        "lip_tip_depth_mm": lip_tip_depth,
        "groove_w_mm": groove_w,
        "groove_h_mm": groove_h,
        "groove_depth_mm": groove_depth,
        "groove_start_z_mm": groove_start_z,
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
    wrap_depth = max(min(p.tpu_front_edge_wrap_depth_mm, 0.45 * cavity_depth), 0.6)
    wrap_radial = max(p.tpu_front_edge_wrap_radial_mm, 0.6)
    wrap_inner_w = max(p.tpu_inner_w_mm - 2.0 * wrap_radial, 2.0)
    wrap_inner_h = max(p.tpu_inner_h_mm - 2.0 * wrap_radial, wrap_inner_w + 0.2)

    # ASA bucket body (front wall + side shell, open at rear)
    with BuildPart() as asa_bp:
        with BuildSketch(Plane.XY):
            _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=d["body_depth_mm"])

        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            _add_profile(d["asa_inner_w_mm"], d["asa_inner_h_mm"], p.enforce_capsule_profile)
        extrude(amount=cavity_depth + 0.2, mode=Mode.SUBTRACT)

        # Rear groove seat for the cap's wider tongue stage (tongue-and-groove guidance).
        with BuildSketch(Plane.XY.offset(d["groove_start_z_mm"])):
            _add_profile(d["groove_w_mm"], d["groove_h_mm"], p.enforce_capsule_profile)
        extrude(amount=d["groove_depth_mm"] + 0.2, mode=Mode.SUBTRACT)

        # Optional lens/LED openings when a closed front wall is used.
        if (not p.open_front_ovular) and p.include_front_lens_led_cutouts:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                    Circle(0.5 * p.lens_cutout_d_mm)
                with Locations((p.lens_center_x_mm, p.lens_center_y_mm + p.led_hole_offset_from_lens_y_mm)):
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

        # Front-only edge wrap by default so insertion side remains open.
        if p.include_tpu_front_edge_wrap:
            with BuildSketch(Plane.XY.offset(cavity_start_z)):
                _add_profile(p.tpu_inner_w_mm, p.tpu_inner_h_mm, p.enforce_capsule_profile)
            extrude(amount=wrap_depth)
            with BuildSketch(Plane.XY.offset(cavity_start_z - 0.2)):
                _add_profile(wrap_inner_w, wrap_inner_h, p.enforce_capsule_profile)
            extrude(amount=wrap_depth + 0.4, mode=Mode.SUBTRACT)

        if p.include_tpu_rear_edge_wrap:
            rear_start_z = cavity_start_z + cavity_depth - wrap_depth
            with BuildSketch(Plane.XY.offset(rear_start_z)):
                _add_profile(p.tpu_inner_w_mm, p.tpu_inner_h_mm, p.enforce_capsule_profile)
            extrude(amount=wrap_depth)
            with BuildSketch(Plane.XY.offset(rear_start_z - 0.2)):
                _add_profile(wrap_inner_w, wrap_inner_h, p.enforce_capsule_profile)
            extrude(amount=wrap_depth + 0.4, mode=Mode.SUBTRACT)

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
            "lens_center_x": float(p.lens_center_x_mm),
            "lens_center_y": float(p.lens_center_y_mm),
            "led_hole_d": float(p.led_hole_d_mm),
            "led_hole_offset_from_lens_y": float(p.led_hole_offset_from_lens_y_mm),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "tripod_hole_d": float(p.tripod_hole_d_mm),
            "tripod_center_from_front": float(p.tripod_center_from_front_mm),
            "tpu_edge_wrap_depth": float(wrap_depth),
            "tpu_edge_wrap_radial": float(wrap_radial),
            "tpu_edge_wrap_enabled": {
                "front": bool(p.include_tpu_front_edge_wrap),
                "rear": bool(p.include_tpu_rear_edge_wrap),
            },
            "back_cap_fit_clearance_total": float(p.back_cap_lip_undersize_total_mm),
            "back_cap_tongue_depth": float(d["tongue_depth_mm"]),
            "back_cap_tongue_radial_step": float(p.back_cap_tongue_radial_step_mm),
            "body_rear_groove_depth": float(d["groove_depth_mm"]),
            "body_rear_groove_start_z": float(d["groove_start_z_mm"]),
        },
        "named_bodies": ["TPU_Sleeve", "ASA_Shell"],
        "warnings": [],
    }

    return assembly, report


def build_back_cap(p: DualMaterialParams):
    d = _derived(p)

    slot_w = p.utility_slot_width_mm
    slot_h = d["utility_slot_h_mm"]
    slot_center_y = d["utility_slot_center_y_mm"]

    with BuildPart() as asa_cap_bp:
        with BuildSketch(Plane.XY):
            _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=p.back_cap_thickness_mm)

        # Stage 1: wider tongue that keys into the body rear groove.
        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
            _add_profile(d["tongue_w_mm"], d["tongue_h_mm"], p.enforce_capsule_profile)
        extrude(amount=d["tongue_depth_mm"])

        # Stage 2: slimmer tip for controlled friction fit deeper in the sleeve opening.
        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm + d["tongue_depth_mm"])):
            _add_profile(d["lip_tip_w_mm"], d["lip_tip_h_mm"], p.enforce_capsule_profile)
        extrude(amount=d["lip_tip_depth_mm"])

        if p.include_back_utility_slot:
            cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + p.back_cap_tpu_gasket_thickness_mm + 1.0
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((0.0, slot_center_y)):
                    _add_vertical_stadium(slot_w, slot_h)
            extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    asa_cap = _largest_solid(asa_cap_bp.part)
    try:
        asa_cap = fillet(asa_cap.edges(), p.back_cap_edge_fillet_mm)
    except Exception:
        pass
    asa_cap = _largest_solid(asa_cap)
    asa_cap.label = "ASA_Back_Cap"

    gasket = None
    if p.include_back_cap_tpu_gasket and p.back_cap_tpu_gasket_thickness_mm > 0.0:
        gasket_outer_w = max(d["asa_outer_w_mm"] - 2.0 * p.back_cap_tpu_gasket_outer_inset_mm, 2.0)
        gasket_outer_h = max(d["asa_outer_h_mm"] - 2.0 * p.back_cap_tpu_gasket_outer_inset_mm, 2.0)
        gasket_inner_w = max(gasket_outer_w - 2.0 * p.back_cap_tpu_gasket_ring_width_mm, 1.0)
        gasket_inner_h = max(gasket_outer_h - 2.0 * p.back_cap_tpu_gasket_ring_width_mm, 1.0)
        with BuildPart() as gasket_bp:
            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
                _add_profile(gasket_outer_w, gasket_outer_h, p.enforce_capsule_profile)
            extrude(amount=p.back_cap_tpu_gasket_thickness_mm)

            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm - 0.1)):
                _add_profile(gasket_inner_w, gasket_inner_h, p.enforce_capsule_profile)
            extrude(amount=p.back_cap_tpu_gasket_thickness_mm + 0.2, mode=Mode.SUBTRACT)

            if p.include_back_utility_slot:
                with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm - 0.1)):
                    with Locations((0.0, slot_center_y)):
                        _add_vertical_stadium(slot_w, slot_h)
                extrude(amount=p.back_cap_tpu_gasket_thickness_mm + 0.2, mode=Mode.SUBTRACT)

        gasket = _largest_solid(gasket_bp.part)
        gasket.label = "TPU_Back_Gasket"

    if gasket is not None:
        cap_asm = Compound(children=[gasket, asa_cap], label="Mevo_Back_Cap_Assembly")
        named_bodies = ["ASA_Back_Cap", "TPU_Back_Gasket"]
    else:
        cap_asm = asa_cap
        named_bodies = ["ASA_Back_Cap"]

    report = {
        "back_cap_mm": {
            "plate_w": float(d["asa_outer_w_mm"]),
            "plate_h": float(d["asa_outer_h_mm"]),
            "lip_tip_w": float(d["lip_tip_w_mm"]),
            "lip_tip_h": float(d["lip_tip_h_mm"]),
            "tongue_w": float(d["tongue_w_mm"]),
            "tongue_h": float(d["tongue_h_mm"]),
            "tongue_depth": float(d["tongue_depth_mm"]),
            "lip_tip_depth": float(d["lip_tip_depth_mm"]),
            "lip_depth": float(p.back_cap_lip_depth_mm),
            "thickness": float(p.back_cap_thickness_mm),
            "lip_undersize_total": float(p.back_cap_lip_undersize_total_mm),
            "groove_radial_clearance": float(p.back_cap_groove_radial_clearance_mm),
            "groove_axial_clearance": float(p.back_cap_groove_axial_clearance_mm),
        },
        "named_bodies": named_bodies,
        "utility_slot_mm": {
            "enabled": bool(p.include_back_utility_slot),
            "shape": "stadium",
            "width": float(slot_w),
            "height": float(slot_h),
            "center_y": float(slot_center_y),
            "top_margin": float(p.utility_slot_top_margin_mm),
            "bottom_margin": float(p.utility_slot_bottom_margin_mm),
        },
        "tpu_gasket_mm": {
            "enabled": bool(gasket is not None),
            "thickness": float(p.back_cap_tpu_gasket_thickness_mm),
            "outer_inset": float(p.back_cap_tpu_gasket_outer_inset_mm),
            "ring_width": float(p.back_cap_tpu_gasket_ring_width_mm),
        },
    }

    return cap_asm, asa_cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate reviewed Mevo dual-material body + ASA back cap")
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument("--open-front-ovular", action="store_true", help="Keep the front open (legacy mode).")
    parser.add_argument(
        "--enable-front-lens-led-cutouts",
        action="store_true",
        help="Cut lens and LED holes into the front wall (closed-front mode).",
    )
    parser.add_argument(
        "--disable-front-lens-led-cutouts",
        action="store_true",
        help="Disable front lens/LED cutouts in closed-front mode.",
    )
    parser.add_argument(
        "--enable-back-utility-slot",
        action="store_true",
        help="Enable the large rear utility slot cutout on the back cap.",
    )
    parser.add_argument(
        "--disable-back-utility-slot",
        action="store_true",
        help="Disable rear utility slot cutout on the back cap (default).",
    )
    parser.add_argument(
        "--disable-capsule-profile",
        action="store_true",
        help="Use rectangular profile instead of ovular capsule profile.",
    )
    parser.add_argument(
        "--disable-back-cap-tpu-gasket",
        action="store_true",
        help="Disable TPU gasket body on the back-cap assembly.",
    )
    parser.add_argument(
        "--disable-front-tpu-edge-wrap",
        action="store_true",
        help="Disable TPU front perimeter wrap in the main body.",
    )
    parser.add_argument(
        "--enable-rear-tpu-edge-wrap",
        action="store_true",
        help="Enable TPU rear perimeter wrap in the main body (disabled by default for insertion).",
    )
    parser.add_argument(
        "--back-cap-fit-clearance-total",
        type=float,
        default=None,
        help="Total undersize clearance for the cap lip vs body opening (mm).",
    )
    parser.add_argument(
        "--back-cap-tpu-gasket-thickness",
        type=float,
        default=None,
        help="TPU gasket thickness on cap inner face (mm).",
    )
    args = parser.parse_args()

    p = DualMaterialParams()
    p.open_front_ovular = bool(args.open_front_ovular)
    if args.enable_front_lens_led_cutouts:
        p.include_front_lens_led_cutouts = True
    if args.disable_front_lens_led_cutouts:
        p.include_front_lens_led_cutouts = False
    if args.enable_back_utility_slot:
        p.include_back_utility_slot = True
    if args.disable_back_utility_slot:
        p.include_back_utility_slot = False
    if args.disable_back_cap_tpu_gasket:
        p.include_back_cap_tpu_gasket = False
    if args.disable_front_tpu_edge_wrap:
        p.include_tpu_front_edge_wrap = False
    if args.enable_rear_tpu_edge_wrap:
        p.include_tpu_rear_edge_wrap = True
    if args.back_cap_fit_clearance_total is not None:
        p.back_cap_lip_undersize_total_mm = max(float(args.back_cap_fit_clearance_total), 0.0)
    if args.back_cap_tpu_gasket_thickness is not None:
        p.back_cap_tpu_gasket_thickness_mm = max(float(args.back_cap_tpu_gasket_thickness), 0.0)
    p.enforce_capsule_profile = not bool(args.disable_capsule_profile)

    body_asm, body_report = build_dual_material_body(p)
    back_cap_asm, back_cap_asa, cap_report = build_back_cap(p)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    body_step = args.out / "mevo_start_body_dual_material.step"
    cap_step_dual = args.out / "mevo_start_back_cap_dual_material.step"
    cap_step_asa = args.out / "mevo_start_back_cap_asa.step"
    report_json = reports_dir / "mevo_start_dual_material_report.json"

    archived = _archive_existing([body_step, cap_step_dual, cap_step_asa, report_json], args.out)

    export_step(body_asm, str(body_step))
    export_step(back_cap_asm, str(cap_step_dual))
    export_step(back_cap_asa, str(cap_step_asa))

    payload = {
        "params": asdict(p),
        "body_report": body_report,
        "back_cap_report": cap_report,
    }
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {body_step}")
    print(f"Wrote {cap_step_dual}")
    print(f"Wrote {cap_step_asa}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
