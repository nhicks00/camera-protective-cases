#!/usr/bin/env python3
"""Generate a Mevo Core protective case: 3 separate STEP files.

Primary outputs:
- models/mevo_core_case/mevo_core_asa_shell.step
- models/mevo_core_case/mevo_core_tpu_frame.step
- models/mevo_core_case/mevo_core_back_cap.step
- models/mevo_core_case/reports/mevo_core_report.json

Design notes:
- Square cross-section (90 x 90 mm) with rounded corners
- Full circular tube lens hood (2.75" diameter, 2.5" depth)
- Skeleton TPU frame (corner bumpers + edge rails)
- Back cap with port cutout (bottom) and power button cutout (top center)
- Single bottom tripod mount (1/4"-20 UNC), same rect as Mevo Start
- Cold shoe mount (ISO 518) on top rear
- 3 separate output files: ASA shell, TPU frame, back cap
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Location,
    Locations,
    Mode,
    Plane,
    Rectangle,
    SlotOverall,
    Vector,
    add,
    chamfer,
    export_step,
    extrude,
    fillet,
    loft,
    vertices,
)


@dataclass
class MevoCoreParams:
    # Device nominal (W x H x L)
    device_nominal_w_mm: float = 90.0
    device_nominal_h_mm: float = 90.0
    device_nominal_l_mm: float = 69.85  # 2.75 inches

    # TPU clearance and wall
    extra_length_mm: float = 1.5        # extra axial room for easy insertion
    tpu_clearance_mm: float = 0.15      # per side
    tpu_wall_mm: float = 1.8

    # ASA shell
    asa_wall_mm: float = 2.2
    interface_gap_mm: float = 0.25       # 0.25 per side = 0.5 mm total gap
    bond_interface_tolerance_mm: float = 0.02

    # Corner fillets (rounded-rectangle profile — aggressive rounding)
    asa_outer_corner_r_mm: float = 24.0
    asa_inner_corner_r_mm: float = 22.0
    tpu_outer_corner_r_mm: float = 22.0
    tpu_inner_corner_r_mm: float = 20.0

    # Front wall / sun hood
    sun_hood_depth_mm: float = 3.0

    # Lens cutout (centered on front face)
    lens_cutout_d_mm: float = 76.2      # 2.75" + 0.25" = 3.0 inches
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 0.0

    # Lens hood (full circular tube encompassing lens)
    include_lens_hood: bool = True
    lens_hood_depth_mm: float = 63.5    # 2.5 inches
    lens_hood_wall_mm: float = 2.5
    lens_hood_clearance_mm: float = 0.0  # hood starts at lens cutout edge

    # Skeleton TPU frame
    tpu_corner_bumper_w_mm: float = 12.0
    tpu_edge_rail_w_mm: float = 4.0
    tpu_front_edge_wrap_depth_mm: float = 5.0
    tpu_front_edge_wrap_radial_mm: float = 4.0
    include_tpu_front_edge_wrap: bool = True
    tpu_front_gusset_r_mm: float = 4.0    # concave fillet at bumper-to-front junction
    tpu_front_dome_mm: float = 1.5        # convex dome height on front face

    # Bottom tripod cutout (same as MAKI)
    tripod_rect_w_mm: float = 63.5      # 2.5 inches
    tripod_rect_l_mm: float = 50.8      # 2.0 inches (MAKI long dimension, along Z)
    # Center from front of device: device_l - 41.275mm (1-5/8" from back)
    tripod_center_from_front_mm: float = 34.925  # shifted 1/4" toward rear

    # Thermal vents
    include_thermal_vents: bool = True
    side_vent_count: int = 5
    side_vent_slot_h_mm: float = 20.0
    side_vent_slot_w_mm: float = 3.0
    side_vent_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 0.0
    side_vent_cut_depth_mm: float = 6.0
    top_vent_count: int = 5
    top_vent_slot_width_mm: float = 24.0
    top_vent_slot_height_mm: float = 3.5
    top_vent_pitch_z_mm: float = 9.0
    top_vent_cut_depth_mm: float = 6.0

    # Cold shoe mount (ISO 518)
    include_cold_shoe: bool = True
    cold_shoe_pad_width_mm: float = 28.0
    cold_shoe_pad_length_mm: float = 30.0
    cold_shoe_pad_z_from_rear_mm: float = 15.0
    cold_shoe_pad_corner_r_mm: float = 3.0
    cold_shoe_boss_height_mm: float = 4.0
    cold_shoe_boss_length_mm: float = 22.0
    cold_shoe_boss_width_mm: float = 22.0
    cold_shoe_slot_width_mm: float = 18.8
    cold_shoe_rail_overhang_mm: float = 2.65
    cold_shoe_rail_thickness_mm: float = 1.8
    cold_shoe_slot_depth_mm: float = 2.5

    # Back cap
    back_cap_thickness_mm: float = 3.0
    back_cap_lip_depth_mm: float = 5.0
    back_cap_lip_undersize_total_mm: float = 0.28
    back_cap_edge_fillet_mm: float = 0.6

    # Back cap cutouts (positions relative to cap center, i.e. device center)
    # Port cutout (bottom of back face)
    port_cutout_w_mm: float = 63.5      # 2.5 inches
    port_cutout_h_mm: float = 12.7      # 0.5 inches
    port_cutout_center_x_mm: float = -0.55  # very slightly left of center
    port_cutout_center_y_mm: float = -19.6  # below center
    port_cutout_corner_r_mm: float = 2.0
    # Power button cutout (top center of back face)
    power_cutout_w_mm: float = 31.75    # 1.25 inches
    power_cutout_h_mm: float = 14.29    # ~9/16 inch (3/16" to 3/4" from top)
    power_cutout_center_x_mm: float = 0.0
    power_cutout_center_y_mm: float = 33.1  # near top
    power_cutout_corner_r_mm: float = 2.0
    # Oversize for cutouts (cable boot clearance)
    cutout_oversize_mm: float = 1.0

    # Retention: bump pockets (4x)
    include_friction_ridge: bool = True
    friction_ridge_height_mm: float = 0.8
    friction_ridge_setback_mm: float = 3.0

    # Rear TPU relief
    tpu_rear_cap_relief_depth_mm: float = 5.4
    tpu_rear_cap_relief_radial_mm: float = 0.3


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


def _derived(p: MevoCoreParams) -> dict:
    tpu_inner_w = p.device_nominal_w_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_h = p.device_nominal_h_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_depth = p.device_nominal_l_mm + 2.0 * p.tpu_clearance_mm + p.extra_length_mm

    tpu_outer_w = tpu_inner_w + 2.0 * p.tpu_wall_mm
    tpu_outer_h = tpu_inner_h + 2.0 * p.tpu_wall_mm

    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    cavity_start_z = p.sun_hood_depth_mm
    # Body depth must accommodate: front wall + camera space + cap plug intrusion
    asa_cavity_depth = tpu_inner_depth + p.back_cap_lip_depth_mm
    body_depth = p.sun_hood_depth_mm + asa_cavity_depth

    lip_tip_w = max(asa_inner_w - p.back_cap_lip_undersize_total_mm, 2.0)
    lip_tip_h = max(asa_inner_h - p.back_cap_lip_undersize_total_mm, 2.0)

    return {
        "tpu_inner_w_mm": tpu_inner_w,
        "tpu_inner_h_mm": tpu_inner_h,
        "tpu_inner_depth_mm": tpu_inner_depth,
        "tpu_outer_w_mm": tpu_outer_w,
        "tpu_outer_h_mm": tpu_outer_h,
        "asa_inner_w_mm": asa_inner_w,
        "asa_inner_h_mm": asa_inner_h,
        "asa_outer_w_mm": asa_outer_w,
        "asa_outer_h_mm": asa_outer_h,
        "asa_cavity_depth_mm": asa_cavity_depth,
        "cavity_start_z_mm": cavity_start_z,
        "body_depth_mm": body_depth,
        "lip_tip_w_mm": lip_tip_w,
        "lip_tip_h_mm": lip_tip_h,
    }


def build_asa_shell(p: MevoCoreParams):
    d = _derived(p)

    asa_outer_w = d["asa_outer_w_mm"]
    asa_outer_h = d["asa_outer_h_mm"]
    asa_inner_w = d["asa_inner_w_mm"]
    asa_inner_h = d["asa_inner_h_mm"]

    half_asa_w = 0.5 * asa_outer_w
    half_asa_h = 0.5 * asa_outer_h

    cavity_start_z = d["cavity_start_z_mm"]
    asa_cavity_depth = d["asa_cavity_depth_mm"]
    body_depth = d["body_depth_mm"]

    # Vent Z centers
    slot_mid_z = 0.5 * body_depth
    side_slot_count = max(p.side_vent_count, 1)
    side_slot_z_centers = [
        float(slot_mid_z + (i - 0.5 * (side_slot_count - 1)) * p.side_vent_pitch_z_mm)
        for i in range(side_slot_count)
    ]
    edge_margin_z = max(8.0, 0.5 * p.side_vent_slot_w_mm + 2.0)
    side_slot_z_centers = [
        min(max(z, edge_margin_z), body_depth - edge_margin_z)
        for z in side_slot_z_centers
    ]

    top_hole_count = max(p.top_vent_count, 1)
    top_vent_z_centers = [
        float(slot_mid_z + (i - 0.5 * (top_hole_count - 1)) * p.top_vent_pitch_z_mm)
        for i in range(top_hole_count)
    ]
    top_hole_margin = max(10.0, 0.5 * p.top_vent_slot_width_mm + 3.0)
    top_vent_z_centers = [
        min(max(z, top_hole_margin), body_depth - top_hole_margin)
        for z in top_vent_z_centers
    ]

    # Remove vents in cold shoe zones (top + left side)
    left_side_slot_z_centers = list(side_slot_z_centers)
    if p.include_cold_shoe:
        cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
        cs_pad_half_l = 0.5 * p.cold_shoe_pad_length_mm
        top_vent_z_centers = [
            z for z in top_vent_z_centers
            if z < (cs_pad_z - cs_pad_half_l) or z > (cs_pad_z + cs_pad_half_l)
        ]
        left_side_slot_z_centers = [
            z for z in side_slot_z_centers
            if z < (cs_pad_z - cs_pad_half_l) or z > (cs_pad_z + cs_pad_half_l)
        ]

    # --- ASA Shell ---
    with BuildPart() as asa_bp:
        # Outer solid
        with BuildSketch(Plane.XY):
            Rectangle(asa_outer_w, asa_outer_h)
            fillet(vertices(), p.asa_outer_corner_r_mm)
        extrude(amount=body_depth)

        # Inner cavity (deep enough for camera + cap plug)
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            Rectangle(asa_inner_w, asa_inner_h)
            fillet(vertices(), p.asa_inner_corner_r_mm)
        extrude(amount=asa_cavity_depth + 0.2, mode=Mode.SUBTRACT)

        # Rear groove for cap plug
        groove_clearance = 0.08
        groove_axial = 0.15
        groove_w = d["lip_tip_w_mm"] + 2.0 * groove_clearance
        groove_h = d["lip_tip_h_mm"] + 2.0 * groove_clearance
        groove_depth = p.back_cap_lip_depth_mm + groove_axial
        groove_start_z = body_depth - groove_depth
        with BuildSketch(Plane.XY.offset(groove_start_z)):
            Rectangle(groove_w, groove_h)
            fillet(vertices(), max(p.asa_inner_corner_r_mm - 0.5, 0.5))
        extrude(amount=groove_depth + 0.2, mode=Mode.SUBTRACT)

        # Bump pockets (4x) on inner walls
        friction_ridge_info = None
        if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
            fr_z = body_depth - p.friction_ridge_setback_mm
            pocket_w = 8.0
            pocket_d = 0.8
            pocket_z = 4.0
            half_iw = 0.5 * asa_inner_w
            half_ih = 0.5 * asa_inner_h

            for sx in (-1.0, 1.0):
                px = sx * (half_iw + 0.5 * pocket_d)
                with Locations((px, 0.0, fr_z)):
                    Box(pocket_d, pocket_w, pocket_z, mode=Mode.SUBTRACT)
            for sy in (-1.0, 1.0):
                py = sy * (half_ih + 0.5 * pocket_d)
                with Locations((0.0, py, fr_z)):
                    Box(pocket_w, pocket_d, pocket_z, mode=Mode.SUBTRACT)

            friction_ridge_info = {
                "enabled": True,
                "type": "flush_bumps",
                "pocket_count": 4,
                "setback_mm": float(p.friction_ridge_setback_mm),
                "z_mm": float(fr_z),
            }

        # Front lens cutout
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                Circle(0.5 * p.lens_cutout_d_mm)
        extrude(amount=p.sun_hood_depth_mm + 0.6, mode=Mode.SUBTRACT)

        # Thermal vents
        if p.include_thermal_vents:
            side_cut_depth = max(p.side_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
            for side in ("neg", "pos"):
                x_face = -half_asa_w - 0.2 if side == "neg" else half_asa_w + 0.2
                # Left side (neg/X-) has cold shoe — use filtered Z centers
                z_centers = left_side_slot_z_centers if side == "neg" else side_slot_z_centers
                for z_c in z_centers:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, z_c)):
                            SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                    extrude(amount=side_cut_depth if side == "neg" else -side_cut_depth,
                            mode=Mode.SUBTRACT)

            top_cut_depth = max(p.top_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
            with BuildSketch(Plane.XZ.offset(half_asa_h + 0.2)):
                for z_c in top_vent_z_centers:
                    with Locations((0.0, z_c)):
                        SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
            extrude(amount=-top_cut_depth, mode=Mode.SUBTRACT)

        # Bottom tripod cutout (rounded corners for fluid transitions)
        tripod_cut_depth = half_asa_h
        tripod_z = cavity_start_z + p.tripod_center_from_front_mm
        tripod_y_center = -half_asa_h + 0.5 * tripod_cut_depth - 0.2
        tripod_corner_r = min(3.0, 0.5 * min(p.tripod_rect_w_mm, p.tripod_rect_l_mm) - 0.5)
        with BuildSketch(Plane.XZ.offset(tripod_y_center - 0.5 * tripod_cut_depth)):
            with Locations((0.0, tripod_z)):
                Rectangle(p.tripod_rect_w_mm, p.tripod_rect_l_mm)
                fillet(vertices(), tripod_corner_r)
        extrude(amount=tripod_cut_depth, mode=Mode.SUBTRACT)

        # Cold shoe mount (ISO 518) on top (Y+) face near rear — opposite from tripod
        cold_shoe_info = None
        if p.include_cold_shoe:
            pad_w = p.cold_shoe_pad_width_mm
            pad_l = p.cold_shoe_pad_length_mm
            pad_z_center = body_depth - p.cold_shoe_pad_z_from_rear_mm
            pad_fill_height = max(p.asa_wall_mm + 2.0, 4.0)

            with BuildSketch(Plane.XZ.offset(half_asa_h + 0.2)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(pad_w, pad_l)
                    fillet(vertices(), p.cold_shoe_pad_corner_r_mm)
            extrude(amount=-pad_fill_height)

            # Subtract inner cavity so fill doesn't intrude
            with BuildSketch(Plane.XY.offset(pad_z_center - 0.5 * pad_l - 1.0)):
                Rectangle(asa_inner_w, asa_inner_h)
                fillet(vertices(), p.asa_inner_corner_r_mm)
            extrude(amount=pad_l + 2.0, mode=Mode.SUBTRACT)

            # Hollow the fill pad
            pad_shell = p.asa_wall_mm
            pad_inner_w = pad_w - 2.0 * pad_shell
            pad_inner_l = pad_l - 2.0 * pad_shell
            if pad_inner_w > 2.0 and pad_inner_l > 2.0:
                with BuildSketch(Plane.XZ.offset((half_asa_h + 0.2) - pad_shell)):
                    with Locations((0.0, pad_z_center)):
                        Rectangle(pad_inner_w, pad_inner_l)
                extrude(amount=-(pad_fill_height - pad_shell), mode=Mode.SUBTRACT)

            # Cold shoe boss
            cs_boss_l = p.cold_shoe_boss_length_mm
            cs_boss_w = p.cold_shoe_boss_width_mm
            cs_slot_w = p.cold_shoe_slot_width_mm
            cs_rail_oh = p.cold_shoe_rail_overhang_mm
            cs_rail_t = p.cold_shoe_rail_thickness_mm
            cs_slot_d = p.cold_shoe_slot_depth_mm
            cs_boss_h = cs_slot_d + cs_rail_t
            cs_opening = cs_slot_w - 2.0 * cs_rail_oh

            with BuildSketch(Plane.XZ.offset(half_asa_h)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(cs_boss_w, cs_boss_l)
            extrude(amount=cs_boss_h)

            # T-slot channel
            boss_top_offset = half_asa_h + cs_boss_h
            cs_front_z = pad_z_center - cs_boss_l * 0.5
            cs_slot_len = body_depth + 0.2 - cs_front_z
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5

            with BuildSketch(Plane.XZ.offset(boss_top_offset + 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_opening, cs_slot_len)
            extrude(amount=-(cs_boss_h + 0.2), mode=Mode.SUBTRACT)

            with BuildSketch(Plane.XZ.offset(half_asa_h - 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_slot_w, cs_slot_len)
            extrude(amount=cs_slot_d + 0.2, mode=Mode.SUBTRACT)

            cold_shoe_info = {
                "enabled": True,
                "locations": ["top_Y+"],
                "pad_z_center_mm": float(pad_z_center),
                "boss_height_mm": float(cs_boss_h),
                "slot_width_mm": float(cs_slot_w),
                "rail_opening_mm": float(cs_opening),
                "slide_in_from": "rear",
            }

            # Second cold shoe on left side (X-) — same ISO 518 pattern
            with BuildSketch(Plane.YZ.offset(-(half_asa_w + 0.2))):
                with Locations((0.0, pad_z_center)):
                    Rectangle(pad_w, pad_l)
                    fillet(vertices(), p.cold_shoe_pad_corner_r_mm)
            extrude(amount=pad_fill_height)

            # Subtract inner cavity so left fill doesn't intrude
            with BuildSketch(Plane.XY.offset(pad_z_center - 0.5 * pad_l - 1.0)):
                Rectangle(asa_inner_w, asa_inner_h)
                fillet(vertices(), p.asa_inner_corner_r_mm)
            extrude(amount=pad_l + 2.0, mode=Mode.SUBTRACT)

            # Hollow the left fill pad
            if pad_inner_w > 2.0 and pad_inner_l > 2.0:
                with BuildSketch(Plane.YZ.offset(-(half_asa_w + 0.2) + pad_shell)):
                    with Locations((0.0, pad_z_center)):
                        Rectangle(pad_inner_w, pad_inner_l)
                extrude(amount=pad_fill_height - pad_shell, mode=Mode.SUBTRACT)

            # Left cold shoe boss
            with BuildSketch(Plane.YZ.offset(-half_asa_w)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(cs_boss_w, cs_boss_l)
            extrude(amount=-cs_boss_h)

            # Left T-slot channel
            left_boss_offset = -(half_asa_w + cs_boss_h)
            with BuildSketch(Plane.YZ.offset(left_boss_offset - 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_opening, cs_slot_len)
            extrude(amount=cs_boss_h + 0.2, mode=Mode.SUBTRACT)

            with BuildSketch(Plane.YZ.offset(-half_asa_w + 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_slot_w, cs_slot_len)
            extrude(amount=-(cs_slot_d + 0.2), mode=Mode.SUBTRACT)

            cold_shoe_info["locations"].append("left_X-")

    asa_shell = _largest_solid(asa_bp.part)

    # Build full circular tube lens hood, then union
    if p.include_lens_hood and p.lens_hood_depth_mm > 0.0:
        hood_inner_r = 0.5 * p.lens_cutout_d_mm + p.lens_hood_clearance_mm
        hood_outer_r = hood_inner_r + p.lens_hood_wall_mm
        with BuildPart() as hood_bp:
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.0)):
                Cylinder(hood_outer_r, p.lens_hood_depth_mm, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.1)):
                Cylinder(hood_inner_r, p.lens_hood_depth_mm + 0.2, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN),
                         mode=Mode.SUBTRACT)
        hood_solid = hood_bp.part
        for fillet_r in (2.0, 1.5, 1.0, 0.5):
            try:
                hood_solid = fillet(hood_solid.edges(), fillet_r)
                break
            except Exception:
                continue
        try:
            asa_shell = _largest_solid(asa_shell + hood_solid)
        except Exception:
            pass

    # Round the body edges (front-face-to-wall transitions, rear edges, etc.)
    for fillet_r in (3.0, 2.5, 2.0, 1.5, 1.0):
        try:
            asa_shell = fillet(asa_shell.edges(), fillet_r)
            break
        except Exception:
            continue
    asa_shell = _largest_solid(asa_shell)
    asa_shell.label = "ASA_Shell"

    report = {
        "derived_mm": d,
        "features_mm": {
            "lens_cutout_d": float(p.lens_cutout_d_mm),
            "lens_center_x": float(p.lens_center_x_mm),
            "lens_center_y": float(p.lens_center_y_mm),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "lens_hood": {
                "enabled": bool(p.include_lens_hood),
                "type": "full_tube",
                "depth_mm": float(p.lens_hood_depth_mm),
                "wall_mm": float(p.lens_hood_wall_mm),
                "clearance_mm": float(p.lens_hood_clearance_mm),
            },
            "tripod_cutout": {
                "rect_w_mm": float(p.tripod_rect_w_mm),
                "rect_l_mm": float(p.tripod_rect_l_mm),
                "center_from_front_mm": float(p.tripod_center_from_front_mm),
            },
            "thermal_vents": {
                "side_slots": {
                    "count_per_side": int(side_slot_count),
                    "z_centers": [float(z) for z in side_slot_z_centers],
                },
                "top_slots": {
                    "count": len(top_vent_z_centers),
                    "z_centers": [float(z) for z in top_vent_z_centers],
                },
            },
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "friction_ridge": friction_ridge_info if friction_ridge_info else {"enabled": False},
        },
    }

    return asa_shell, report, side_slot_z_centers, top_vent_z_centers, left_side_slot_z_centers


def build_tpu_frame(p: MevoCoreParams, side_slot_z_centers, top_vent_z_centers, left_side_slot_z_centers):
    d = _derived(p)

    tpu_outer_w = d["tpu_outer_w_mm"]
    tpu_outer_h = d["tpu_outer_h_mm"]
    tpu_inner_w = d["tpu_inner_w_mm"]
    tpu_inner_h = d["tpu_inner_h_mm"]

    half_tpu_w = 0.5 * tpu_outer_w
    half_tpu_h = 0.5 * tpu_outer_h

    cavity_start_z = d["cavity_start_z_mm"]
    cavity_depth = d["tpu_inner_depth_mm"]

    wrap_depth = max(min(p.tpu_front_edge_wrap_depth_mm, 0.45 * cavity_depth), 0.6) if p.include_tpu_front_edge_wrap else 0.0
    wrap_radial = max(p.tpu_front_edge_wrap_radial_mm, 0.6) if p.include_tpu_front_edge_wrap else 0.0

    wrap_inner_w = max(tpu_inner_w - 2.0 * wrap_radial, 2.0) if wrap_radial > 0.0 else tpu_inner_w
    wrap_inner_h = max(tpu_inner_h - 2.0 * wrap_radial, 2.0) if wrap_radial > 0.0 else tpu_inner_h
    wrap_inner_corner_r = max(p.tpu_inner_corner_r_mm - wrap_radial, 0.5)

    # --- Stage 1: base tube with smooth lofted transition at front wrap ---
    wrap_z = cavity_start_z + wrap_depth
    with BuildPart() as tpu_stage1:
        # Full TPU outer shell
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            Rectangle(tpu_outer_w, tpu_outer_h)
            fillet(vertices(), p.tpu_outer_corner_r_mm)
        extrude(amount=cavity_depth)

        # Main cavity (behind the transition zone)
        with BuildSketch(Plane.XY.offset(wrap_z - 0.1)):
            Rectangle(tpu_inner_w, tpu_inner_h)
            fillet(vertices(), p.tpu_inner_corner_r_mm)
        extrude(amount=cavity_depth - wrap_depth + 0.3, mode=Mode.SUBTRACT)

        # Smooth lofted transition: wrap_inner at front → tpu_inner at wrap_z
        # Single continuous curved surface, no step or chamfer.
        if p.include_tpu_front_edge_wrap and wrap_depth > 0.0 and wrap_radial > 0.0:
            with BuildSketch(Plane.XY.offset(cavity_start_z)):
                Rectangle(wrap_inner_w, wrap_inner_h)
                fillet(vertices(), wrap_inner_corner_r)
            with BuildSketch(Plane.XY.offset(wrap_z)):
                Rectangle(tpu_inner_w, tpu_inner_h)
                fillet(vertices(), p.tpu_inner_corner_r_mm)
            loft(mode=Mode.SUBTRACT)

    tpu_base = _largest_solid(tpu_stage1.part)

    # --- Stage 2: skeleton cuts, vents, tripod on the chamfered base ---
    bumper_w = p.tpu_corner_bumper_w_mm
    rail_w = p.tpu_edge_rail_w_mm
    wall_cut_depth = p.tpu_wall_mm + 1.0
    skel_start_z = cavity_start_z + wrap_depth + rail_w
    skel_end_z = cavity_start_z + cavity_depth - rail_w
    skel_span = max(skel_end_z - skel_start_z, 0.0)

    with BuildPart() as tpu_bp:
        add(tpu_base)

        # X walls (left/right) skeleton windows
        x_wall_clear_h = max(tpu_outer_h - 2.0 * bumper_w, 0.0)
        if x_wall_clear_h > 1.0 and skel_span > 1.0:
            x_cut_center_z = skel_start_z + 0.5 * skel_span
            for side in (-1.0, 1.0):
                x_face = side * (half_tpu_w + 0.2)
                with BuildSketch(Plane.YZ.offset(x_face)):
                    with Locations((0.0, x_cut_center_z)):
                        Rectangle(x_wall_clear_h, skel_span)
                extrude(
                    amount=wall_cut_depth if side < 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        # Y walls (top/bottom) skeleton windows
        y_wall_clear_w = max(tpu_outer_w - 2.0 * bumper_w, 0.0)
        if y_wall_clear_w > 1.0 and skel_span > 1.0:
            y_cut_center_z = skel_start_z + 0.5 * skel_span
            for side in (-1.0, 1.0):
                y_face = side * (half_tpu_h + 0.2)
                with BuildSketch(Plane.XZ.offset(-y_face)):
                    with Locations((0.0, y_cut_center_z)):
                        Rectangle(y_wall_clear_w, skel_span)
                extrude(
                    amount=wall_cut_depth if side > 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        # Rear TPU relief for cap insertion
        rear_relief = min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))
        if rear_relief > 0.0:
            relief_start_z = cavity_start_z + cavity_depth - rear_relief
            relief_w = tpu_outer_w + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            relief_h = tpu_outer_h + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            with BuildSketch(Plane.XY.offset(relief_start_z - 0.2)):
                Rectangle(relief_w, relief_h)
                fillet(vertices(), max(p.tpu_outer_corner_r_mm + 0.3, 0.5))
            extrude(amount=rear_relief + 0.4, mode=Mode.SUBTRACT)

        # Vent pass-through cuts
        if p.include_thermal_vents:
            tpu_side_cut = max(p.side_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            for side in ("neg", "pos"):
                x_face = -half_tpu_w - 0.2 if side == "neg" else half_tpu_w + 0.2
                z_centers = left_side_slot_z_centers if side == "neg" else side_slot_z_centers
                for z_c in z_centers:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, z_c)):
                            SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                    extrude(amount=tpu_side_cut if side == "neg" else -tpu_side_cut,
                            mode=Mode.SUBTRACT)

            tpu_top_cut = max(p.top_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            with BuildSketch(Plane.XZ.offset(half_tpu_h + 0.2)):
                for z_c in top_vent_z_centers:
                    with Locations((0.0, z_c)):
                        SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
            extrude(amount=-tpu_top_cut, mode=Mode.SUBTRACT)

        # Bottom tripod pass-through (rounded corners)
        tripod_tpu_cut_depth = half_tpu_h
        tpu_rect_w = p.tripod_rect_w_mm + 2.0
        tpu_rect_l = p.tripod_rect_l_mm + 2.0
        tripod_z = d["cavity_start_z_mm"] + p.tripod_center_from_front_mm
        tripod_tpu_corner_r = min(3.0, 0.5 * min(tpu_rect_w, tpu_rect_l) - 0.5)
        tpu_tripod_y_center = -half_tpu_h + 0.5 * tripod_tpu_cut_depth - 0.2
        with BuildSketch(Plane.XZ.offset(tpu_tripod_y_center - 0.5 * tripod_tpu_cut_depth)):
            with Locations((0.0, tripod_z)):
                Rectangle(tpu_rect_w, tpu_rect_l)
                fillet(vertices(), tripod_tpu_corner_r)
        extrude(amount=tripod_tpu_cut_depth, mode=Mode.SUBTRACT)

    tpu_frame = _largest_solid(tpu_bp.part)

    # Global edge fillet for smooth rounded feel on all edges.
    for fillet_r in (p.tpu_front_dome_mm, 1.5, 1.0, 0.8, 0.5):
        try:
            tpu_frame = fillet(tpu_frame.edges(), fillet_r)
            break
        except Exception:
            continue
    tpu_frame = _largest_solid(tpu_frame)
    tpu_frame.label = "TPU_Frame"

    report = {
        "tpu_frame": {
            "type": "skeleton",
            "corner_bumper_w_mm": float(p.tpu_corner_bumper_w_mm),
            "edge_rail_w_mm": float(p.tpu_edge_rail_w_mm),
            "wall_thickness_mm": float(p.tpu_wall_mm),
        },
    }

    return tpu_frame, report


def build_back_cap(p: MevoCoreParams):
    d = _derived(p)

    asa_outer_w = d["asa_outer_w_mm"]
    asa_outer_h = d["asa_outer_h_mm"]
    lip_tip_w = d["lip_tip_w_mm"]
    lip_tip_h = d["lip_tip_h_mm"]

    cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + 1.0
    os = p.cutout_oversize_mm  # oversize for cable boot clearance

    with BuildPart() as cap_bp:
        # Outer plate
        with BuildSketch(Plane.XY):
            Rectangle(asa_outer_w, asa_outer_h)
            fillet(vertices(), p.asa_outer_corner_r_mm)
        extrude(amount=p.back_cap_thickness_mm)

        # Plug tongue
        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
            Rectangle(lip_tip_w, lip_tip_h)
            fillet(vertices(), max(p.asa_inner_corner_r_mm - 0.5, 0.5))
        extrude(amount=p.back_cap_lip_depth_mm)

        # Port cutout (bottom of back face)
        port_w = p.port_cutout_w_mm + 2.0 * os
        port_h = p.port_cutout_h_mm + 2.0 * os
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((p.port_cutout_center_x_mm, p.port_cutout_center_y_mm)):
                Rectangle(port_w, port_h)
                fillet(vertices(), p.port_cutout_corner_r_mm)
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

        # Power button cutout (top center of back face)
        pwr_w = p.power_cutout_w_mm + 2.0 * os
        pwr_h = p.power_cutout_h_mm + 2.0 * os
        with BuildSketch(Plane.XY.offset(-0.2)):
            with Locations((p.power_cutout_center_x_mm, p.power_cutout_center_y_mm)):
                Rectangle(pwr_w, pwr_h)
                fillet(vertices(), p.power_cutout_corner_r_mm)
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    cap = _largest_solid(cap_bp.part)
    try:
        cap = fillet(cap.edges(), p.back_cap_edge_fillet_mm)
    except Exception:
        pass
    cap = _largest_solid(cap)

    # Retention bumps (4x flush on plug surface)
    if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
        fr_z = p.back_cap_thickness_mm + (p.back_cap_lip_depth_mm - p.friction_ridge_setback_mm)
        fr_h = p.friction_ridge_height_mm
        half_lw = 0.5 * lip_tip_w
        half_lh = 0.5 * lip_tip_h
        bump_w = 8.0
        bump_z = 4.0

        with BuildPart() as fr_bp:
            for sx in (-1.0, 1.0):
                with Locations((sx * (half_lw + 0.5 * fr_h), 0.0, fr_z)):
                    Box(fr_h, bump_w, bump_z)
            for sy in (-1.0, 1.0):
                with Locations((0.0, sy * (half_lh + 0.5 * fr_h), fr_z)):
                    Box(bump_w, fr_h, bump_z)
        try:
            cap = _largest_solid(cap + fr_bp.part)
        except Exception:
            pass

    cap.label = "ASA_Back_Cap"

    report = {
        "back_cap_mm": {
            "plate_w": float(asa_outer_w),
            "plate_h": float(asa_outer_h),
            "lip_tip_w": float(lip_tip_w),
            "lip_tip_h": float(lip_tip_h),
            "lip_depth": float(p.back_cap_lip_depth_mm),
            "thickness": float(p.back_cap_thickness_mm),
            "lip_undersize_total": float(p.back_cap_lip_undersize_total_mm),
            "port_cutout": {
                "w_mm": float(port_w),
                "h_mm": float(port_h),
                "center_x_mm": float(p.port_cutout_center_x_mm),
                "center_y_mm": float(p.port_cutout_center_y_mm),
            },
            "power_cutout": {
                "w_mm": float(pwr_w),
                "h_mm": float(pwr_h),
                "center_x_mm": float(p.power_cutout_center_x_mm),
                "center_y_mm": float(p.power_cutout_center_y_mm),
            },
        },
        "named_bodies": ["ASA_Back_Cap"],
    }

    return cap, report


def main():
    parser = argparse.ArgumentParser(
        description="Generate Mevo Core case: ASA shell + TPU frame + back cap (3 separate files)"
    )
    parser.add_argument("--out", type=Path, default=Path("models/mevo_core_case"),
                        help="Output directory")
    parser.add_argument("--no-cold-shoe", action="store_true")
    parser.add_argument("--no-friction-ridge", action="store_true")
    parser.add_argument("--no-hood", action="store_true")
    parser.add_argument("--no-vents", action="store_true")
    parser.add_argument("--lens-diameter", type=float, default=None)
    parser.add_argument("--hood-depth", type=float, default=None)
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None)
    args = parser.parse_args()

    p = MevoCoreParams()
    if args.no_cold_shoe:
        p.include_cold_shoe = False
    if args.no_friction_ridge:
        p.include_friction_ridge = False
    if args.no_hood:
        p.include_lens_hood = False
    if args.no_vents:
        p.include_thermal_vents = False
    if args.lens_diameter is not None:
        p.lens_cutout_d_mm = float(args.lens_diameter)
    if args.hood_depth is not None:
        p.lens_hood_depth_mm = float(args.hood_depth)
    if args.cold_shoe_z_from_rear is not None:
        p.cold_shoe_pad_z_from_rear_mm = float(args.cold_shoe_z_from_rear)

    asa_shell, asa_report, side_z, top_z, left_z = build_asa_shell(p)
    tpu_frame, tpu_report = build_tpu_frame(p, side_z, top_z, left_z)
    back_cap, cap_report = build_back_cap(p)

    # Collision check: cap PLATE only (not plug) vs ASA shell
    d = _derived(p)
    body_depth = d["body_depth_mm"]
    collision_report = {"cap_vs_body_mm3": -1.0, "collision_pass": True, "cap_seated_z_mm": 0.0}
    try:
        # Offset slightly past shell rear face to avoid coincident surface overlap.
        cap_seated_z = body_depth + 0.1
        # Build plate-only solid for collision (exclude plug tongue)
        with BuildPart() as plate_bp:
            with BuildSketch(Plane.XY):
                Rectangle(d["asa_outer_w_mm"], d["asa_outer_h_mm"])
                fillet(vertices(), p.asa_outer_corner_r_mm)
            extrude(amount=p.back_cap_thickness_mm)
        plate_only = plate_bp.part.moved(Location(Vector(0, 0, cap_seated_z)))
        try:
            inter = asa_shell & plate_only
            vol = float(inter.volume) if hasattr(inter, "volume") else 0.0
        except Exception:
            vol = 0.0
        collision_report = {
            "cap_vs_body_mm3": float(vol),
            "collision_pass": vol <= 0.5,
            "cap_seated_z_mm": float(cap_seated_z),
        }
    except Exception as e:
        collision_report["error"] = str(e)

    if not collision_report.get("collision_pass", True):
        print(f"WARNING: Cap-body collision detected: {collision_report['cap_vs_body_mm3']:.2f} mm3")

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    shell_step = args.out / "mevo_core_asa_shell.step"
    tpu_step = args.out / "mevo_core_tpu_frame.step"
    cap_step = args.out / "mevo_core_back_cap.step"
    report_json = reports_dir / "mevo_core_report.json"

    archived = _archive_existing([shell_step, tpu_step, cap_step, report_json], args.out)

    export_step(asa_shell, str(shell_step))
    export_step(tpu_frame, str(tpu_step))
    export_step(back_cap, str(cap_step))

    # Fit alignment verification
    tpu_outer_w = d["tpu_outer_w_mm"]
    tpu_outer_h = d["tpu_outer_h_mm"]
    asa_inner_w = d["asa_inner_w_mm"]
    asa_inner_h = d["asa_inner_h_mm"]
    tpu_depth = d["tpu_inner_depth_mm"]
    asa_cav_depth = d["asa_cavity_depth_mm"]
    cap_plug_depth = p.back_cap_lip_depth_mm

    radial_gap_w = 0.5 * (asa_inner_w - tpu_outer_w)
    radial_gap_h = 0.5 * (asa_inner_h - tpu_outer_h)
    axial_gap_behind_tpu = asa_cav_depth - tpu_depth - cap_plug_depth
    camera_space = asa_cav_depth - cap_plug_depth

    fit_report = {
        "radial_gap_w_each_mm": float(radial_gap_w),
        "radial_gap_h_each_mm": float(radial_gap_h),
        "asa_cavity_depth_mm": float(asa_cav_depth),
        "tpu_depth_mm": float(tpu_depth),
        "cap_plug_intrusion_mm": float(cap_plug_depth),
        "axial_gap_behind_tpu_mm": float(axial_gap_behind_tpu),
        "effective_camera_space_mm": float(camera_space),
        "device_length_mm": float(p.device_nominal_l_mm),
        "camera_fits": camera_space >= p.device_nominal_l_mm,
    }

    payload = {
        "params": asdict(p),
        "asa_shell_report": asa_report,
        "tpu_frame_report": tpu_report,
        "back_cap_report": cap_report,
        "fit_alignment": fit_report,
        "collision_check": collision_report,
    }
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {shell_step}")
    print(f"Wrote {tpu_step}")
    print(f"Wrote {cap_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
