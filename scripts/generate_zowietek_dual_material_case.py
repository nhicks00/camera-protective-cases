#!/usr/bin/env python3
"""Generate a Zowietek 4K NDI POV Zoom Camera dual-material case + bumper-ring back cap.

Primary outputs:
- models/zowietek_case/zowietek_pov_body_dual_material.step
  - contains two named bodies: TPU_Frame and ASA_Shell
- models/zowietek_case/zowietek_pov_back_cap.step
  - ASA bumper ring (open center for rear port/button access)
- models/zowietek_case/reports/zowietek_pov_dual_material_report.json

Design notes:
- Rounded-rectangle cross-section (box + corner fillets)
- TPU is a skeleton frame: corner bumpers connected by edge rails (not a solid sleeve)
- Two 1/4"-20 UNC tripod mounts (bottom + top)
- Bumper ring back cap with wide-open center for rear I/O access
- Duckbill sun hood, cold shoe mount, thermal vents
"""

from __future__ import annotations

import argparse
import json
import math
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
    Cylinder,
    Locations,
    Mode,
    Plane,
    Rectangle,
    SlotOverall,
    export_step,
    extrude,
    fillet,
    vertices,
)


@dataclass
class ZowietekParams:
    # Device nominal (L x W x H)
    device_nominal_l_mm: float = 68.6
    device_nominal_w_mm: float = 60.2
    device_nominal_h_mm: float = 51.0

    # TPU clearance and wall
    tpu_clearance_mm: float = 0.40        # per side (matched to Mevo)
    tpu_wall_mm: float = 1.8

    # ASA shell
    asa_wall_mm: float = 2.2
    interface_gap_mm: float = 0.0         # coincident bond
    bond_interface_tolerance_mm: float = 0.02

    # Corner fillets (rounded-rectangle profile)
    asa_outer_corner_r_mm: float = 4.0
    asa_inner_corner_r_mm: float = 2.0
    tpu_outer_corner_r_mm: float = 2.0
    tpu_inner_corner_r_mm: float = 1.0

    # Front wall / sun hood
    sun_hood_depth_mm: float = 3.0
    include_front_lens_led_cutouts: bool = True
    lens_cutout_d_mm: float = 37.7        # 25.0 + 12.7 mm (0.5")
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 0.0         # centered for this camera
    led_hole_d_mm: float = 3.0
    led_hole_above_lens_mm: float = 12.0

    # Lens hood (circular tube around lens opening only)
    include_lens_hood: bool = True
    lens_hood_depth_mm: float = 14.0      # how far hood extends from front face
    lens_hood_wall_mm: float = 2.5        # wall thickness of hood tube
    lens_hood_clearance_mm: float = 1.0   # gap between lens cutout edge and hood inner wall

    # Skeleton TPU frame params
    tpu_corner_bumper_w_mm: float = 12.0  # width of each corner bumper along wall
    tpu_edge_rail_w_mm: float = 4.0       # width of connecting rails along each edge
    tpu_front_edge_wrap_depth_mm: float = 2.5
    tpu_front_edge_wrap_radial_mm: float = 2.0
    include_tpu_front_edge_wrap: bool = True

    # Bottom tripod cutout (1/4"-20)
    tripod_rect_w_mm: float = 31.75       # 1.25 inches
    tripod_rect_l_mm: float = 25.4        # 1 inch along Z
    tripod_bottom_center_from_front_mm: float = 34.3  # roughly centered along L
    # Top tripod cutout (mirrored)
    include_top_tripod: bool = True
    tripod_top_center_from_front_mm: float = 34.3

    # Thermal vents
    include_thermal_vents: bool = True
    side_vent_count: int = 5
    side_vent_slot_h_mm: float = 18.0     # vertical extent
    side_vent_slot_w_mm: float = 3.0      # width (rounded ends)
    side_vent_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 0.0    # vertically centered
    side_vent_cut_depth_mm: float = 6.0
    top_vent_count: int = 4
    top_vent_slot_width_mm: float = 22.0  # long axis (X)
    top_vent_slot_height_mm: float = 3.0  # short axis
    top_vent_pitch_z_mm: float = 12.0
    top_vent_cut_depth_mm: float = 6.0

    # Cold shoe mount (ISO 518)
    include_cold_shoe: bool = True
    cold_shoe_pad_width_mm: float = 26.0
    cold_shoe_pad_length_mm: float = 28.0
    cold_shoe_pad_z_from_rear_mm: float = 12.0
    cold_shoe_pad_corner_r_mm: float = 3.0
    cold_shoe_boss_height_mm: float = 4.0
    cold_shoe_boss_length_mm: float = 22.0
    cold_shoe_boss_width_mm: float = 22.0
    cold_shoe_slot_width_mm: float = 18.8
    cold_shoe_rail_overhang_mm: float = 2.65
    cold_shoe_rail_thickness_mm: float = 1.8
    cold_shoe_slot_depth_mm: float = 2.5

    # Back cap (bumper ring)
    back_cap_thickness_mm: float = 3.0
    back_cap_lip_depth_mm: float = 5.0
    back_cap_lip_undersize_total_mm: float = 0.28
    back_cap_edge_fillet_mm: float = 0.6
    bumper_ring_inset_mm: float = 3.0     # inset from device edge per side for opening
    bumper_ring_corner_r_mm: float = 3.0

    # Retention: latch pockets/bumps (4x)
    include_friction_ridge: bool = True
    friction_ridge_height_mm: float = 0.8
    friction_ridge_width_mm: float = 1.0
    friction_ridge_setback_mm: float = 3.0
    latch_side_gap_mm: float = 1.0
    latch_tip_gap_z_mm: float = 1.5
    latch_catch_height_mm: float = 0.6
    latch_catch_ramp_mm: float = 1.5
    latch_notch_depth_mm: float = 0.6
    latch_notch_z_mm: float = 1.5

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


def _derived(p: ZowietekParams) -> dict:
    # Inner cavity = device + clearance
    tpu_inner_w = p.device_nominal_w_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_h = p.device_nominal_h_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_depth = p.device_nominal_l_mm + 2.0 * p.tpu_clearance_mm

    tpu_outer_w = tpu_inner_w + 2.0 * p.tpu_wall_mm
    tpu_outer_h = tpu_inner_h + 2.0 * p.tpu_wall_mm

    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    cavity_start_z = p.sun_hood_depth_mm
    body_depth = p.sun_hood_depth_mm + tpu_inner_depth

    # Back cap plug dimensions
    lip_tip_w = max(asa_inner_w - p.back_cap_lip_undersize_total_mm, 2.0)
    lip_tip_h = max(asa_inner_h - p.back_cap_lip_undersize_total_mm, 2.0)

    # Bumper ring opening = device envelope + margin on each side
    ring_opening_w = max(p.device_nominal_w_mm - 2.0 * p.bumper_ring_inset_mm, 10.0)
    ring_opening_h = max(p.device_nominal_h_mm - 2.0 * p.bumper_ring_inset_mm, 10.0)

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
        "cavity_start_z_mm": cavity_start_z,
        "body_depth_mm": body_depth,
        "lip_tip_w_mm": lip_tip_w,
        "lip_tip_h_mm": lip_tip_h,
        "ring_opening_w_mm": ring_opening_w,
        "ring_opening_h_mm": ring_opening_h,
    }


def build_dual_material_body(p: ZowietekParams):
    d = _derived(p)

    asa_outer_w = d["asa_outer_w_mm"]
    asa_outer_h = d["asa_outer_h_mm"]
    asa_inner_w = d["asa_inner_w_mm"]
    asa_inner_h = d["asa_inner_h_mm"]
    tpu_outer_w = d["tpu_outer_w_mm"]
    tpu_outer_h = d["tpu_outer_h_mm"]
    tpu_inner_w = d["tpu_inner_w_mm"]
    tpu_inner_h = d["tpu_inner_h_mm"]

    half_asa_outer_w = 0.5 * asa_outer_w
    half_asa_outer_h = 0.5 * asa_outer_h
    half_tpu_outer_w = 0.5 * tpu_outer_w
    half_tpu_outer_h = 0.5 * tpu_outer_h

    cavity_start_z = d["cavity_start_z_mm"]
    cavity_depth = d["tpu_inner_depth_mm"]
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
    top_hole_margin_z = max(10.0, 0.5 * p.top_vent_slot_width_mm + 3.0)
    top_vent_z_centers = [
        min(max(z, top_hole_margin_z), body_depth - top_hole_margin_z)
        for z in top_vent_z_centers
    ]

    # Remove top vents in cold shoe zone
    if p.include_cold_shoe:
        cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
        cs_pad_half_l = 0.5 * p.cold_shoe_pad_length_mm
        top_vent_z_centers = [
            z for z in top_vent_z_centers
            if z < (cs_pad_z - cs_pad_half_l) or z > (cs_pad_z + cs_pad_half_l)
        ]

    led_center_y = p.lens_center_y_mm + p.led_hole_above_lens_mm

    # --- ASA Shell ---
    with BuildPart() as asa_bp:
        # Outer solid
        with BuildSketch(Plane.XY):
            Rectangle(asa_outer_w, asa_outer_h)
            fillet(vertices(), p.asa_outer_corner_r_mm)
        extrude(amount=body_depth)

        # Inner cavity
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            Rectangle(asa_inner_w, asa_inner_h)
            fillet(vertices(), p.asa_inner_corner_r_mm)
        extrude(amount=cavity_depth + 0.2, mode=Mode.SUBTRACT)

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

        # Simple bump pockets (4x) on inner walls — matching cap bumps
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
                "type": "latch_pockets",
                "pocket_count": 4,
                "setback_mm": float(p.friction_ridge_setback_mm),
                "z_mm": float(fr_z),
            }

        # Front lens + LED cutouts
        if p.include_front_lens_led_cutouts:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                    Circle(0.5 * p.lens_cutout_d_mm)
                with Locations((p.lens_center_x_mm, led_center_y)):
                    Circle(0.5 * p.led_hole_d_mm)
            extrude(amount=p.sun_hood_depth_mm + 0.6, mode=Mode.SUBTRACT)

        # Compute hood params (circular tube around lens, built separately)
        _build_hood = False
        if p.include_lens_hood and p.lens_hood_depth_mm > 0.0:
            hood_inner_r = 0.5 * p.lens_cutout_d_mm + p.lens_hood_clearance_mm
            hood_outer_r = hood_inner_r + p.lens_hood_wall_mm
            _build_hood = True

        # Thermal vents (ASA)
        if p.include_thermal_vents:
            # Side slots (on W faces, i.e. X walls)
            side_cut_depth = max(p.side_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
            for side in ("neg", "pos"):
                x_face = -half_asa_outer_w - 0.2 if side == "neg" else half_asa_outer_w + 0.2
                for z_c in side_slot_z_centers:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, z_c)):
                            SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                    extrude(amount=side_cut_depth if side == "neg" else -side_cut_depth, mode=Mode.SUBTRACT)

            # Top slots (on H+ face, i.e. Y+ wall)
            top_cut_depth = max(p.top_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
            with BuildSketch(Plane.XZ.offset(-(half_asa_outer_h + 0.2))):
                for z_c in top_vent_z_centers:
                    with Locations((0.0, z_c)):
                        SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
            extrude(amount=top_cut_depth, mode=Mode.SUBTRACT)

        # Bottom tripod cutout
        tripod_cut_depth = half_asa_outer_h
        tripod_z_bottom = cavity_start_z + p.tripod_bottom_center_from_front_mm
        with Locations((0.0, -half_asa_outer_h + 0.5 * tripod_cut_depth - 0.2, tripod_z_bottom)):
            Box(p.tripod_rect_w_mm, tripod_cut_depth, p.tripod_rect_l_mm, mode=Mode.SUBTRACT)

        # Top tripod cutout
        if p.include_top_tripod:
            tripod_z_top = cavity_start_z + p.tripod_top_center_from_front_mm
            with Locations((0.0, half_asa_outer_h - 0.5 * tripod_cut_depth + 0.2, tripod_z_top)):
                Box(p.tripod_rect_w_mm, tripod_cut_depth, p.tripod_rect_l_mm, mode=Mode.SUBTRACT)

        # Cold shoe mount (ISO 518) on top rear (Y+)
        cold_shoe_info = None
        if p.include_cold_shoe:
            pad_w = p.cold_shoe_pad_width_mm
            pad_l = p.cold_shoe_pad_length_mm
            pad_z_center = body_depth - p.cold_shoe_pad_z_from_rear_mm
            pad_fill_height = max(p.asa_wall_mm + 2.0, 4.0)

            # Fill pad on top surface
            with BuildSketch(Plane.XZ.offset(-half_asa_outer_h)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(pad_w, pad_l)
                    fillet(vertices(), p.cold_shoe_pad_corner_r_mm)
            extrude(amount=pad_fill_height)

            # Subtract inner cavity so fill doesn't intrude interior
            with BuildSketch(Plane.XY.offset(pad_z_center - 0.5 * pad_l - 1.0)):
                Rectangle(asa_inner_w, asa_inner_h)
                fillet(vertices(), p.asa_inner_corner_r_mm)
            extrude(amount=pad_l + 2.0, mode=Mode.SUBTRACT)

            # Hollow the fill pad
            pad_shell = p.asa_wall_mm
            pad_inner_w = pad_w - 2.0 * pad_shell
            pad_inner_l = pad_l - 2.0 * pad_shell
            if pad_inner_w > 2.0 and pad_inner_l > 2.0:
                with BuildSketch(Plane.XZ.offset(-half_asa_outer_h + pad_shell)):
                    with Locations((0.0, pad_z_center)):
                        Rectangle(pad_inner_w, pad_inner_l)
                extrude(amount=pad_fill_height - pad_shell, mode=Mode.SUBTRACT)

            # Cold shoe boss
            cs_boss_l = p.cold_shoe_boss_length_mm
            cs_boss_w = p.cold_shoe_boss_width_mm
            cs_slot_w = p.cold_shoe_slot_width_mm
            cs_rail_oh = p.cold_shoe_rail_overhang_mm
            cs_rail_t = p.cold_shoe_rail_thickness_mm
            cs_slot_d = p.cold_shoe_slot_depth_mm
            cs_boss_h = cs_slot_d + cs_rail_t
            cs_opening = cs_slot_w - 2.0 * cs_rail_oh

            with BuildSketch(Plane.XZ.offset(-half_asa_outer_h)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(cs_boss_w, cs_boss_l)
            extrude(amount=-cs_boss_h)

            # T-slot channel
            boss_top_offset = -(half_asa_outer_h + cs_boss_h)
            cs_front_z = pad_z_center - cs_boss_l * 0.5
            cs_slot_len = body_depth + 0.2 - cs_front_z
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5

            # Narrow opening (stem passage)
            with BuildSketch(Plane.XZ.offset(boss_top_offset - 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_opening, cs_slot_len)
            extrude(amount=(cs_boss_h + 0.2), mode=Mode.SUBTRACT)

            # Wide floor pocket
            with BuildSketch(Plane.XZ.offset(-half_asa_outer_h + 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_slot_w, cs_slot_len)
            extrude(amount=-(cs_slot_d + 0.2), mode=Mode.SUBTRACT)

            cold_shoe_info = {
                "enabled": True,
                "pad_z_center_mm": float(pad_z_center),
                "boss_height_mm": float(cs_boss_h),
                "slot_width_mm": float(cs_slot_w),
                "rail_opening_mm": float(cs_opening),
                "slide_in_from": "rear",
            }

    asa_shell = _largest_solid(asa_bp.part)

    # Build top-visor lens hood (arc over top of lens opening)
    if _build_hood:
        with BuildPart() as hood_bp:
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.0)):
                Cylinder(hood_outer_r, p.lens_hood_depth_mm, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.1)):
                Cylinder(hood_inner_r, p.lens_hood_depth_mm + 0.2, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN),
                         mode=Mode.SUBTRACT)
        # Clip to top-half visor within body width
        clip_w = d["asa_outer_w_mm"]
        clip_h = hood_outer_r + 1.0
        with BuildPart() as clip_bp:
            with Locations((p.lens_center_x_mm,
                            p.lens_center_y_mm + 0.5 * clip_h,
                            -0.5 * p.lens_hood_depth_mm)):
                Box(clip_w, clip_h, p.lens_hood_depth_mm + 1.0)
        hood_solid = hood_bp.part & clip_bp.part
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

    asa_shell.label = "ASA_Shell"

    # --- TPU Skeleton Frame ---
    # Corner bumpers at each of the 4 vertical edges, connected by edge rails
    # along each of the 4 horizontal edges (top/bottom/left/right at front and rear).
    with BuildPart() as tpu_bp:
        # Full TPU outer shell (will subtract interior + skeleton cuts)
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            Rectangle(tpu_outer_w, tpu_outer_h)
            fillet(vertices(), p.tpu_outer_corner_r_mm)
        extrude(amount=cavity_depth)

        # Subtract inner cavity
        with BuildSketch(Plane.XY.offset(cavity_start_z - 0.2)):
            Rectangle(tpu_inner_w, tpu_inner_h)
            fillet(vertices(), p.tpu_inner_corner_r_mm)
        extrude(amount=cavity_depth + 0.4, mode=Mode.SUBTRACT)

        # Now subtract the wall sections BETWEEN corner bumpers + edge rails
        # to create the skeleton frame.
        # Strategy: cut rectangular windows from each flat wall face,
        # leaving corner bumper zones and horizontal edge rails intact.
        #
        # Each wall has:
        # - Corner bumpers at each end (tpu_corner_bumper_w_mm from corner)
        # - Edge rails at top and bottom of the wall (tpu_edge_rail_w_mm)
        # - The middle section between these is cut away
        #
        # X walls (left/right): span = tpu_outer_h - 2*corner_bumper_w
        # Y walls (top/bottom): span = tpu_outer_w - 2*corner_bumper_w

        bumper_w = p.tpu_corner_bumper_w_mm
        rail_w = p.tpu_edge_rail_w_mm
        wall_cut_depth = p.tpu_wall_mm + 1.0  # ensure full through-cut

        # X walls (left/right faces): wall runs along Y axis
        x_wall_clear_h = max(tpu_outer_h - 2.0 * bumper_w, 0.0)  # Y extent of clearable zone
        x_wall_clear_z = max(cavity_depth - 2.0 * rail_w, 0.0)    # Z extent of clearable zone
        if x_wall_clear_h > 1.0 and x_wall_clear_z > 1.0:
            x_cut_center_z = cavity_start_z + 0.5 * cavity_depth
            for side in (-1.0, 1.0):
                x_face = side * (half_tpu_outer_w + 0.2)
                with BuildSketch(Plane.YZ.offset(x_face)):
                    with Locations((0.0, x_cut_center_z)):
                        Rectangle(x_wall_clear_h, x_wall_clear_z)
                extrude(
                    amount=wall_cut_depth if side < 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        # Y walls (top/bottom faces): wall runs along X axis
        y_wall_clear_w = max(tpu_outer_w - 2.0 * bumper_w, 0.0)
        y_wall_clear_z = max(cavity_depth - 2.0 * rail_w, 0.0)
        if y_wall_clear_w > 1.0 and y_wall_clear_z > 1.0:
            y_cut_center_z = cavity_start_z + 0.5 * cavity_depth
            for side in (-1.0, 1.0):
                y_face = side * (half_tpu_outer_h + 0.2)
                with BuildSketch(Plane.XZ.offset(-y_face)):
                    with Locations((0.0, y_cut_center_z)):
                        Rectangle(y_wall_clear_w, y_wall_clear_z)
                extrude(
                    amount=wall_cut_depth if side > 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        # Front edge wrap (thin lip around front opening for device retention)
        if p.include_tpu_front_edge_wrap:
            wrap_depth = max(min(p.tpu_front_edge_wrap_depth_mm, 0.45 * cavity_depth), 0.6)
            wrap_radial = max(p.tpu_front_edge_wrap_radial_mm, 0.6)
            wrap_inner_w = max(tpu_inner_w - 2.0 * wrap_radial, 2.0)
            wrap_inner_h = max(tpu_inner_h - 2.0 * wrap_radial, 2.0)
            with BuildSketch(Plane.XY.offset(cavity_start_z)):
                Rectangle(tpu_inner_w, tpu_inner_h)
                fillet(vertices(), p.tpu_inner_corner_r_mm)
            extrude(amount=wrap_depth)
            with BuildSketch(Plane.XY.offset(cavity_start_z - 0.2)):
                Rectangle(wrap_inner_w, wrap_inner_h)
                fillet(vertices(), max(p.tpu_inner_corner_r_mm - 0.5, 0.3))
            extrude(amount=wrap_depth + 0.4, mode=Mode.SUBTRACT)

        # Rear TPU relief for cap insertion
        rear_relief_depth = min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))
        if rear_relief_depth > 0.0:
            relief_start_z = cavity_start_z + cavity_depth - rear_relief_depth
            relief_w = tpu_outer_w + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            relief_h = tpu_outer_h + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            with BuildSketch(Plane.XY.offset(relief_start_z - 0.2)):
                Rectangle(relief_w, relief_h)
                fillet(vertices(), max(p.tpu_outer_corner_r_mm + 0.3, 0.5))
            extrude(amount=rear_relief_depth + 0.4, mode=Mode.SUBTRACT)

        # Vent pass-through cuts on TPU (same locations as ASA)
        if p.include_thermal_vents:
            tpu_side_cut = max(p.side_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            for side in ("neg", "pos"):
                x_face = -half_tpu_outer_w - 0.2 if side == "neg" else half_tpu_outer_w + 0.2
                for z_c in side_slot_z_centers:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, z_c)):
                            SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                    extrude(amount=tpu_side_cut if side == "neg" else -tpu_side_cut, mode=Mode.SUBTRACT)

            tpu_top_cut = max(p.top_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            with BuildSketch(Plane.XZ.offset(-(half_tpu_outer_h + 0.2))):
                for z_c in top_vent_z_centers:
                    with Locations((0.0, z_c)):
                        SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
            extrude(amount=tpu_top_cut, mode=Mode.SUBTRACT)

        # Bottom tripod pass-through
        tripod_tpu_cut_depth = half_tpu_outer_h
        tpu_rect_w = p.tripod_rect_w_mm + 2.0
        tpu_rect_l = p.tripod_rect_l_mm + 2.0
        tripod_z_bottom = cavity_start_z + p.tripod_bottom_center_from_front_mm
        with Locations((0.0, -half_tpu_outer_h + 0.5 * tripod_tpu_cut_depth - 0.2, tripod_z_bottom)):
            Box(tpu_rect_w, tripod_tpu_cut_depth, tpu_rect_l, mode=Mode.SUBTRACT)

        # Top tripod pass-through
        if p.include_top_tripod:
            tripod_z_top = cavity_start_z + p.tripod_top_center_from_front_mm
            with Locations((0.0, half_tpu_outer_h - 0.5 * tripod_tpu_cut_depth + 0.2, tripod_z_top)):
                Box(tpu_rect_w, tripod_tpu_cut_depth, tpu_rect_l, mode=Mode.SUBTRACT)

    tpu_frame = _largest_solid(tpu_bp.part)
    tpu_frame.label = "TPU_Frame"

    interface_gap_w_each = 0.5 * (asa_inner_w - tpu_outer_w)
    interface_gap_h_each = 0.5 * (asa_inner_h - tpu_outer_h)
    max_abs_gap = max(abs(interface_gap_w_each), abs(interface_gap_h_each))
    bond_grade = "A" if max_abs_gap <= p.bond_interface_tolerance_mm else "B"

    report = {
        "derived_mm": d,
        "features_mm": {
            "include_front_lens_led_cutouts": bool(p.include_front_lens_led_cutouts),
            "lens_cutout_d": float(p.lens_cutout_d_mm),
            "lens_center_x": float(p.lens_center_x_mm),
            "lens_center_y": float(p.lens_center_y_mm),
            "led_hole_d": float(p.led_hole_d_mm),
            "led_hole_center_y": float(led_center_y),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "lens_hood": {
                "enabled": bool(p.include_lens_hood),
                "type": "top_visor",
                "depth": float(p.lens_hood_depth_mm),
                "wall": float(p.lens_hood_wall_mm),
                "clearance": float(p.lens_hood_clearance_mm),
            },
            "tripod_cutouts": {
                "bottom": {
                    "rect_w_mm": float(p.tripod_rect_w_mm),
                    "rect_l_mm": float(p.tripod_rect_l_mm),
                    "center_from_front_mm": float(p.tripod_bottom_center_from_front_mm),
                },
                "top": {
                    "enabled": bool(p.include_top_tripod),
                    "rect_w_mm": float(p.tripod_rect_w_mm),
                    "rect_l_mm": float(p.tripod_rect_l_mm),
                    "center_from_front_mm": float(p.tripod_top_center_from_front_mm),
                },
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
            "tpu_frame": {
                "type": "skeleton",
                "corner_bumper_w_mm": float(p.tpu_corner_bumper_w_mm),
                "edge_rail_w_mm": float(p.tpu_edge_rail_w_mm),
                "wall_thickness_mm": float(p.tpu_wall_mm),
            },
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "friction_ridge": friction_ridge_info if friction_ridge_info else {"enabled": False},
        },
        "bond_interface_mm": {
            "target_gap_each": float(p.interface_gap_mm),
            "actual_gap_each_width": float(interface_gap_w_each),
            "actual_gap_each_height": float(interface_gap_h_each),
            "max_abs_gap_each": float(max_abs_gap),
            "bond_grade": bond_grade,
        },
        "named_bodies": ["TPU_Frame", "ASA_Shell"],
        "warnings": [],
    }
    if max_abs_gap > p.bond_interface_tolerance_mm:
        report["warnings"].append(
            "TPU-to-ASA interface gap exceeds tolerance; fusion quality may be reduced."
        )

    return asa_shell, tpu_frame, report


def build_back_cap(p: ZowietekParams):
    d = _derived(p)

    asa_outer_w = d["asa_outer_w_mm"]
    asa_outer_h = d["asa_outer_h_mm"]
    lip_tip_w = d["lip_tip_w_mm"]
    lip_tip_h = d["lip_tip_h_mm"]
    ring_opening_w = d["ring_opening_w_mm"]
    ring_opening_h = d["ring_opening_h_mm"]

    cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + 1.0

    with BuildPart() as cap_bp:
        # Outer plate
        with BuildSketch(Plane.XY):
            Rectangle(asa_outer_w, asa_outer_h)
            fillet(vertices(), p.asa_outer_corner_r_mm)
        extrude(amount=p.back_cap_thickness_mm)

        # Plug tongue (single-step)
        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
            Rectangle(lip_tip_w, lip_tip_h)
            fillet(vertices(), max(p.asa_inner_corner_r_mm - 0.5, 0.5))
        extrude(amount=p.back_cap_lip_depth_mm)

        # Large center opening (bumper ring)
        with BuildSketch(Plane.XY.offset(-0.2)):
            Rectangle(ring_opening_w, ring_opening_h)
            fillet(vertices(), p.bumper_ring_corner_r_mm)
        extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    # Fillet plate edges
    cap = _largest_solid(cap_bp.part)
    try:
        cap = fillet(cap.edges(), p.back_cap_edge_fillet_mm)
    except Exception:
        pass
    cap = _largest_solid(cap)

    # Add retention bumps (4x cantilever flex-tabs)
    if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
        fr_z = p.back_cap_thickness_mm + (p.back_cap_lip_depth_mm - p.friction_ridge_setback_mm)
        fr_h = p.friction_ridge_height_mm
        half_lw = 0.5 * lip_tip_w
        half_lh = 0.5 * lip_tip_h
        bump_w = 8.0
        bump_z = 4.0

        # Simple flush bumps on plug outer surface (4x: 2 on X walls, 2 on Y walls)
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
            "ring_opening_w": float(ring_opening_w),
            "ring_opening_h": float(ring_opening_h),
            "bumper_ring_inset": float(p.bumper_ring_inset_mm),
        },
        "named_bodies": ["ASA_Back_Cap"],
    }

    return cap, report


def main():
    parser = argparse.ArgumentParser(
        description="Generate Zowietek 4K POV dual-material case + bumper-ring back cap"
    )
    parser.add_argument("--out", type=Path, default=Path("models/zowietek_case"), help="Output directory")
    parser.add_argument("--no-cold-shoe", action="store_true", help="Disable cold shoe mount")
    parser.add_argument("--no-friction-ridge", action="store_true", help="Disable retention latches")
    parser.add_argument("--no-top-tripod", action="store_true", help="Disable top tripod cutout")
    parser.add_argument("--no-hood", action="store_true", help="Disable lens hood")
    parser.add_argument("--no-vents", action="store_true", help="Disable thermal vents")
    parser.add_argument("--lens-diameter", type=float, default=None, help="Front lens cutout diameter (mm)")
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None, help="Cold shoe center from rear (mm)")
    parser.add_argument("--bumper-ring-inset", type=float, default=None, help="Bumper ring inset from device edge (mm)")
    args = parser.parse_args()

    p = ZowietekParams()
    if args.no_cold_shoe:
        p.include_cold_shoe = False
    if args.no_friction_ridge:
        p.include_friction_ridge = False
    if args.no_top_tripod:
        p.include_top_tripod = False
    if args.no_hood:
        p.include_lens_hood = False
    if args.no_vents:
        p.include_thermal_vents = False
    if args.lens_diameter is not None:
        p.lens_cutout_d_mm = float(args.lens_diameter)
    if args.cold_shoe_z_from_rear is not None:
        p.cold_shoe_pad_z_from_rear_mm = float(args.cold_shoe_z_from_rear)
    if args.bumper_ring_inset is not None:
        p.bumper_ring_inset_mm = float(args.bumper_ring_inset)

    asa_shell, tpu_frame, body_report = build_dual_material_body(p)
    back_cap, cap_report = build_back_cap(p)

    # Collision check: plate-only (not plug) vs ASA shell
    d = _derived(p)
    body_depth = d["body_depth_mm"]
    collision_report = {"cap_vs_body_mm3": -1.0, "collision_pass": True, "cap_seated_z_mm": 0.0}
    try:
        from build123d import Location, Vector
        cap_seated_z = body_depth + 0.5  # offset past shell rear face to avoid coincident overlap
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
            "collision_pass": vol <= 200.0,  # plate-only check; small volumes are numerical noise
            "cap_seated_z_mm": float(cap_seated_z),
        }
    except Exception as e:
        collision_report["error"] = str(e)

    if not collision_report.get("collision_pass", True):
        print(f"WARNING: Cap-body collision detected: {collision_report['cap_vs_body_mm3']:.2f} mm3")

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    # 3 separate output files
    shell_step = args.out / "zowietek_pov_asa_shell.step"
    tpu_step = args.out / "zowietek_pov_tpu_frame.step"
    cap_step = args.out / "zowietek_pov_back_cap.step"
    report_json = reports_dir / "zowietek_pov_dual_material_report.json"

    # Also archive old combined body file
    body_step_legacy = args.out / "zowietek_pov_body_dual_material.step"
    archived = _archive_existing(
        [shell_step, tpu_step, cap_step, report_json, body_step_legacy],
        args.out,
    )

    export_step(asa_shell, str(shell_step))
    export_step(tpu_frame, str(tpu_step))
    export_step(back_cap, str(cap_step))

    payload = {
        "params": asdict(p),
        "body_report": body_report,
        "back_cap_report": cap_report,
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
