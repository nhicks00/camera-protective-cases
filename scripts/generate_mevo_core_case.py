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
    Cone,
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
    asa_wall_mm: float = 3.3
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
    lens_hood_clearance_mm: float = 2.5  # hood bore wider than front face hole for ledge/strength
    lens_hood_base_flare_mm: float = 6.0   # extra outer radius at root for strength
    lens_hood_base_depth_mm: float = 8.0   # axial depth of the taper zone

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
    side_vent_count: int = 7              # non-cold-shoe side gets all 7; cold-shoe side filtered
    side_vent_slot_h_mm: float = 26.0     # +30% longer
    side_vent_slot_w_mm: float = 3.0
    side_vent_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 0.0
    side_vent_cut_depth_mm: float = 6.0
    top_vent_count: int = 7               # cold-shoe filter removes 2 rear slots; net +1 vs old 5
    top_vent_slot_width_mm: float = 31.0  # +30% longer
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

    # Retention: bump pockets — disabled
    include_friction_ridge: bool = False
    friction_ridge_height_mm: float = 0.8
    friction_ridge_setback_mm: float = 3.0

    # Snap-fit clips — disabled
    include_snap_clips: bool = False
    snap_clip_beam_length_mm: float = 8.0
    snap_clip_beam_width_mm: float = 6.0
    snap_clip_beam_thickness_mm: float = 1.5
    snap_clip_catch_height_mm: float = 1.0
    snap_clip_catch_depth_mm: float = 1.0
    snap_clip_setback_mm: float = 3.0
    snap_clips_per_wall: int = 2
    snap_clip_spread_mm: float = 30.0
    include_snap_ridge: bool = False
    snap_ridge_height_mm: float = 1.0
    snap_ridge_depth_mm: float = 1.0
    snap_ridge_setback_mm: float = 3.0

    # Cantilever snap-fit latches (beams on cap plug, through-holes in shell)
    include_snap_latches: bool = True
    snap_latch_beam_length_mm: float = 4.0   # free cantilever length along Z axis
    snap_latch_beam_width_mm: float = 8.0    # lateral width of beam strip
    snap_latch_beam_thickness_mm: float = 1.5 # radial thickness of beam (thin for flex)
    snap_latch_hook_height_mm: float = 1.5   # how far hook protrudes beyond plug surface
    snap_latch_hook_ramp_mm: float = 2.5     # Z extent of hook ramp (angled for insertion)
    snap_latch_flex_gap_mm: float = 1.2      # radial gap behind beam for deflection
    snap_latch_side_slot_width_mm: float = 1.0  # width of side isolation slots
    snap_latch_hole_width_mm: float = 10.0   # through-hole width in shell (press-to-release)
    snap_latch_hole_height_mm: float = 4.0   # through-hole Z extent in shell

    # Retention bumps on non-latch walls (low-profile press-fit)
    include_retention_bumps: bool = True
    retention_bump_height_mm: float = 0.8    # protrusion from plug surface
    retention_bump_width_mm: float = 8.0     # lateral width of bump
    retention_bump_z_extent_mm: float = 4.0  # Z extent of bump
    retention_bump_setback_mm: float = 3.0   # from plug tip

    # Sun shade canopy (floating external shell, top + sides, open bottom)
    include_sun_shade: bool = True
    sun_shade_standoff_mm: float = 6.0    # air gap between shell and shade
    sun_shade_wall_mm: float = 2.0        # shade panel thickness
    sun_shade_post_width_mm: float = 4.0   # rib width along each face

    # Rear TPU corner bumpers
    include_rear_tpu_bumpers: bool = True
    tpu_rear_bumper_depth_mm: float = 3.0    # how far bumper wraps around rear corners
    tpu_rear_bumper_wall_mm: float = 1.8     # wall thickness of rear bumper (same as TPU wall)

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
    # Drop the front-most vent (closest to front face) to reduce stress riser
    if len(side_slot_z_centers) > 1:
        side_slot_z_centers = side_slot_z_centers[1:]

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
    # After margin clamping, redistribute evenly within the available range
    # to avoid overlapping slots near the margins
    if len(top_vent_z_centers) > 1:
        z_min = top_vent_z_centers[0]
        z_max = top_vent_z_centers[-1]
        n = len(top_vent_z_centers)
        if n > 1 and (z_max - z_min) > 0:
            top_vent_z_centers = [
                z_min + i * (z_max - z_min) / (n - 1) for i in range(n)
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

        # (Through-holes and bump pockets are cut AFTER cold shoe fills below
        #  so that the fill material doesn't cover the holes.)
        snap_latch_info = None
        retention_bump_info = None

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
                # Both sides get full vent set (cold shoes moved to shade hood)
                z_centers = side_slot_z_centers
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

        # Cold shoe mounts are placed on the shade hood (not the shell).
        # Just initialize the info dict here; geometry is added in shade BuildPart.
        cold_shoe_info = None

        # Snap-latch through-holes — cut AFTER cold shoe fills so holes
        # aren't covered by fill material on X- and Y+ walls.
        if p.include_snap_latches:
            hook_body_z = body_depth - p.back_cap_lip_depth_mm + 0.5 * p.snap_latch_hook_ramp_mm
            hole_z = hook_body_z
            hole_w = p.snap_latch_hole_width_mm
            hole_h = p.snap_latch_hole_height_mm
            wall_cut = p.asa_wall_mm + 2.0

            half_ow = 0.5 * asa_outer_w
            half_oh = 0.5 * asa_outer_h

            # X+ wall
            with BuildSketch(Plane.YZ.offset(half_ow + 0.2)):
                with Locations((0.0, hole_z)):
                    Rectangle(hole_w, hole_h)
            extrude(amount=-wall_cut, mode=Mode.SUBTRACT)

            # X- wall (through cold shoe fill)
            with BuildSketch(Plane.YZ.offset(-(half_ow + 0.2))):
                with Locations((0.0, hole_z)):
                    Rectangle(hole_w, hole_h)
            extrude(amount=wall_cut, mode=Mode.SUBTRACT)

            # Y- wall
            with BuildSketch(Plane.XZ.offset(-(half_oh + 0.2))):
                with Locations((0.0, hole_z)):
                    Rectangle(hole_w, hole_h)
            extrude(amount=wall_cut, mode=Mode.SUBTRACT)

            latch_walls = ["X+", "X-", "Y-"]
            snap_latch_info = {
                "enabled": True,
                "count": len(latch_walls),
                "walls": latch_walls,
                "hole_z_mm": float(hole_z),
                "hole_width_mm": float(hole_w),
                "hole_height_mm": float(hole_h),
            }

        # Retention bump pocket on non-latch wall (Y+ only)
        if p.include_retention_bumps and p.include_snap_latches:
            rb_h = p.retention_bump_height_mm
            rb_w = p.retention_bump_width_mm
            rb_z_ext = p.retention_bump_z_extent_mm
            rb_z = body_depth - p.retention_bump_setback_mm
            half_ih = 0.5 * asa_inner_h
            pocket_d = rb_h + 0.2

            with Locations((0.0, half_ih + 0.5 * pocket_d, rb_z)):
                Box(rb_w, pocket_d, rb_z_ext, mode=Mode.SUBTRACT)

            retention_bump_info = {
                "enabled": True,
                "walls": ["Y+"],
                "bump_height_mm": float(rb_h),
                "bump_width_mm": float(rb_w),
                "z_mm": float(rb_z),
            }

    asa_shell = _largest_solid(asa_bp.part)

    # Build full circular tube lens hood with flared base, then union
    if p.include_lens_hood and p.lens_hood_depth_mm > 0.0:
        hood_inner_r = 0.5 * p.lens_cutout_d_mm + p.lens_hood_clearance_mm
        hood_outer_r = hood_inner_r + p.lens_hood_wall_mm
        cx, cy = p.lens_center_x_mm, p.lens_center_y_mm
        flare_r = hood_outer_r + p.lens_hood_base_flare_mm
        flare_d = min(p.lens_hood_base_depth_mm, p.lens_hood_depth_mm * 0.4)
        with BuildPart() as hood_bp:
            # Main straight tube
            with Locations((cx, cy, 0.0)):
                Cylinder(hood_outer_r, p.lens_hood_depth_mm, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Flared base cone: wider at Z=0, tapers to hood_outer_r at flare_d depth
            if p.lens_hood_base_flare_mm > 0.0 and flare_d > 0.0:
                with Locations((cx, cy, 0.0)):
                    Cone(flare_r, hood_outer_r, flare_d, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Subtract inner bore through full depth (constant inner radius)
            with Locations((cx, cy, 0.1)):
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

    # Sun shade canopy: floating external shell (top + sides), open bottom.
    # Connected to shell via full-length ribs on flat face sections.
    # Like a carport awning providing sun/heat protection.
    sun_shade_info = None
    if p.include_sun_shade:
        standoff = p.sun_shade_standoff_mm
        shade_w = p.sun_shade_wall_mm
        post_w = p.sun_shade_post_width_mm

        # Shade profile dimensions (larger rounded rect offset outward)
        shade_inner_w = asa_outer_w + 2.0 * standoff
        shade_inner_h = asa_outer_h + 2.0 * standoff
        shade_outer_w = shade_inner_w + 2.0 * shade_w
        shade_outer_h = shade_inner_h + 2.0 * shade_w
        shade_inner_r = min(p.asa_outer_corner_r_mm + standoff, 0.49 * min(shade_inner_w, shade_inner_h))
        shade_outer_r = min(shade_inner_r + shade_w, 0.49 * min(shade_outer_w, shade_outer_h))

        half_shade_outer_w = 0.5 * shade_outer_w
        half_shade_outer_h = 0.5 * shade_outer_h

        shade_z_start = 0.0
        shade_z_len = body_depth
        shade_mid_z = shade_z_start + 0.5 * shade_z_len

        # The flat section of each face starts at corner_r from the edge.
        # Place ribs at the boundaries of the flat sections (near fillets)
        # so they don't block vents in the middle.
        flat_extent_half = half_asa_w - p.asa_outer_corner_r_mm  # half-width of flat section
        # Rib positions along each face (near the fillet transitions)
        rib_offset = max(flat_extent_half - 2.0, 0.0)  # 2mm inside from fillet start

        # Rib radial span: must overlap both shell outer wall and shade inner wall
        # Shell outer at half_asa_w, shade inner at half_asa_w + standoff
        # Rib extends from shell_outer - 1mm to shade_inner + 1mm
        rib_radial = standoff + 2.0  # 1mm overlap each side

        try:
            # Cold shoe boss dimensions (needed for shade BuildPart)
            cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
            cs_boss_l = p.cold_shoe_boss_length_mm
            cs_boss_w = p.cold_shoe_boss_width_mm
            cs_slot_w = p.cold_shoe_slot_width_mm
            cs_rail_oh = p.cold_shoe_rail_overhang_mm
            cs_rail_t = p.cold_shoe_rail_thickness_mm
            cs_slot_d = p.cold_shoe_slot_depth_mm
            cs_boss_h = cs_slot_d + cs_rail_t
            cs_opening = cs_slot_w - 2.0 * cs_rail_oh
            cs_front_z = cs_pad_z - cs_boss_l * 0.5
            cs_slot_len = body_depth + 0.2 - cs_front_z
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5
            boss_overlap = min(1.5, shade_w - 0.2)

            half_shade_ow = 0.5 * shade_outer_w
            half_shade_oh = 0.5 * shade_outer_h

            with BuildPart() as shade_bp:
                # Shade tube (outer - inner)
                with BuildSketch(Plane.XY.offset(shade_z_start)):
                    Rectangle(shade_outer_w, shade_outer_h)
                    fillet(vertices(), shade_outer_r)
                extrude(amount=shade_z_len)
                with BuildSketch(Plane.XY.offset(shade_z_start - 0.1)):
                    Rectangle(shade_inner_w, shade_inner_h)
                    fillet(vertices(), shade_inner_r)
                extrude(amount=shade_z_len + 0.2, mode=Mode.SUBTRACT)

                # Cut away Y+ panel (user's bottom / tripod side)
                # Shade covers Y- (user's top) + X sides
                cut_height = half_shade_outer_h - half_asa_h + 1.0
                with Locations((0.0, half_asa_h + 0.5 * cut_height, shade_mid_z)):
                    Box(shade_outer_w + 2.0, cut_height + 0.2, shade_z_len + 2.0,
                        mode=Mode.SUBTRACT)

                # Connecting ribs (2 per face, near fillet transitions)
                for ry in (-1.0, 1.0):
                    with Locations((half_asa_w + 0.5 * standoff, ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shade_z_len)
                for ry in (-1.0, 1.0):
                    with Locations((-(half_asa_w + 0.5 * standoff), ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shade_z_len)
                # Y- face ribs (user's top)
                for rx in (-1.0, 1.0):
                    with Locations((rx * rib_offset, -(half_asa_h + 0.5 * standoff), shade_mid_z)):
                        Box(post_w, rib_radial, shade_z_len)

                # Cold shoe solid bosses on shade outer surfaces (no T-slot
                # cuts yet — cuts would break the overlap and _largest_solid
                # would strip the bosses). T-slots are cut after full union.
                if p.include_cold_shoe:
                    # Y- boss built separately after shade+shell union (see below).
                    # Building it inside the shade BuildPart causes _largest_solid
                    # to strip it even with overlap, while X bosses survive fine.

                    # X- boss (left side)
                    with BuildSketch(Plane.YZ.offset(-half_shade_ow + boss_overlap)):
                        with Locations((0.0, cs_pad_z)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=-(cs_boss_h + boss_overlap))

                    # X+ boss (right side)
                    with BuildSketch(Plane.YZ.offset(half_shade_ow - boss_overlap)):
                        with Locations((0.0, cs_pad_z)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=cs_boss_h + boss_overlap)

            shade_solid = _largest_solid(shade_bp.part)
            asa_shell = _largest_solid(asa_shell + shade_solid)

            # Y- cold shoe boss (user's top) — built AFTER shade+shell union
            # so it fuses with the large connected solid reliably.
            # NOTE: Plane.XZ normal is (0,-1,0), so offset(+val) moves to Y=-val.
            if p.include_cold_shoe:
                with BuildPart() as yn_boss_bp:
                    with BuildSketch(Plane.XZ.offset(half_shade_oh - boss_overlap)):
                        with Locations((0.0, cs_pad_z)):
                            Rectangle(cs_boss_w, cs_boss_l)
                    extrude(amount=cs_boss_h + boss_overlap)
                asa_shell = _largest_solid(asa_shell + yn_boss_bp.part)

            sun_shade_info = {
                "enabled": True,
                "standoff_mm": float(standoff),
                "wall_mm": float(shade_w),
                "shade_outer_w_mm": float(shade_outer_w),
                "shade_outer_h_mm": float(shade_outer_h),
                "post_width_mm": float(post_w),
                "rib_count": 6,
                "coverage": "top + left + right (open bottom)",
            }
            print(f"  Sun shade canopy added: {shade_outer_w:.1f} x {shade_outer_h:.1f} mm outer, {standoff:.1f}mm gap, 6 ribs")

            # Now cut T-slots into the fused bosses (safe — bosses are
            # already part of the single fused solid).
            if p.include_cold_shoe:
                cs_locations = []

                # Y- T-slot (user's top)
                # Plane.XZ normal is (0,-1,0): offset(+v) → Y=-v, positive extrude → -Y
                yn_outer_offset = half_shade_oh + cs_boss_h  # offset value (positive → Y=-val)
                with BuildPart() as sn:
                    with BuildSketch(Plane.XZ.offset(yn_outer_offset + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=-(cs_boss_h + 0.2))  # toward +Y (inward)
                asa_shell = _largest_solid(asa_shell - sn.part)
                with BuildPart() as sw:
                    with BuildSketch(Plane.XZ.offset(half_shade_oh - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=cs_slot_d + 0.2)  # toward -Y (into boss)
                asa_shell = _largest_solid(asa_shell - sw.part)
                cs_locations.append("top_Y-")

                # X- T-slot (left side)
                xn_outer = -half_shade_ow - cs_boss_h
                with BuildPart() as sn:
                    with BuildSketch(Plane.YZ.offset(xn_outer - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=cs_boss_h + 0.2)
                asa_shell = _largest_solid(asa_shell - sn.part)
                with BuildPart() as sw:
                    with BuildSketch(Plane.YZ.offset(-half_shade_ow + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=-(cs_slot_d + 0.2))
                asa_shell = _largest_solid(asa_shell - sw.part)
                cs_locations.append("left_X-")

                # X+ T-slot (right side)
                xp_outer = half_shade_ow + cs_boss_h
                with BuildPart() as sn:
                    with BuildSketch(Plane.YZ.offset(xp_outer + 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_opening, cs_slot_len)
                    extrude(amount=-(cs_boss_h + 0.2))
                asa_shell = _largest_solid(asa_shell - sn.part)
                with BuildPart() as sw:
                    with BuildSketch(Plane.YZ.offset(half_shade_ow - 0.1)):
                        with Locations((0.0, cs_slot_mid_z)):
                            Rectangle(cs_slot_w, cs_slot_len)
                    extrude(amount=cs_slot_d + 0.2)
                asa_shell = _largest_solid(asa_shell - sw.part)
                cs_locations.append("right_X+")

                cold_shoe_info = {
                    "enabled": True,
                    "locations": cs_locations,
                    "pad_z_center_mm": float(cs_pad_z),
                    "boss_height_mm": float(cs_boss_h),
                    "slot_width_mm": float(cs_slot_w),
                    "rail_opening_mm": float(cs_opening),
                    "slide_in_from": "rear",
                    "mounted_on": "shade_hood",
                }
                sun_shade_info["cold_shoe_count"] = 3
                print(f"  3 cold shoes on shade hood (top, left, right)")

        except Exception as e:
            print(f"  WARNING: sun shade failed to build: {e}")

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
            "snap_latches": snap_latch_info if snap_latch_info else {"enabled": False},
            "sun_shade": sun_shade_info if sun_shade_info else {"enabled": False},
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

    # Compute rear relief depth here so it's accessible outside BuildPart
    rear_relief = min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))

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

    # Rear edge wrap: full continuous rim mirroring the front edge wrap.
    # Smooth lofted transition from normal wall thickness to a thicker rim
    # that wraps over the camera's rear face — symmetrical with the front.
    if p.include_rear_tpu_bumpers:
        rear_z = cavity_start_z + cavity_depth
        rb_overlap = 4.0  # overlap into surviving skeleton for fusion

        # Use same wrap dimensions as front edge wrap for symmetry
        rear_wrap_depth = wrap_depth   # same as front (5.0 mm)
        rear_wrap_radial = wrap_radial  # same as front (4.0 mm)

        rear_wrap_inner_w = max(tpu_inner_w - 2.0 * rear_wrap_radial, 2.0)
        rear_wrap_inner_h = max(tpu_inner_h - 2.0 * rear_wrap_radial, 2.0)
        rear_wrap_inner_r = max(p.tpu_inner_corner_r_mm - rear_wrap_radial, 0.5)

        rear_wrap_start = rear_z - rear_wrap_depth  # where transition begins
        overlap_start = rear_wrap_start - rb_overlap  # extends into frame for fusion

        with BuildPart() as _rear_wrap:
            # Full outer tube from overlap zone to rear face
            with BuildSketch(Plane.XY.offset(overlap_start)):
                Rectangle(tpu_outer_w, tpu_outer_h)
                fillet(vertices(), p.tpu_outer_corner_r_mm)
            extrude(amount=rear_wrap_depth + rb_overlap)

            # Subtract normal cavity in the overlap zone (thin walls)
            with BuildSketch(Plane.XY.offset(overlap_start - 0.1)):
                Rectangle(tpu_inner_w, tpu_inner_h)
                fillet(vertices(), p.tpu_inner_corner_r_mm)
            extrude(amount=rb_overlap + 0.2, mode=Mode.SUBTRACT)

            # Smooth lofted transition: tpu_inner at wrap_start → wrap_inner at rear_z
            with BuildSketch(Plane.XY.offset(rear_wrap_start)):
                Rectangle(tpu_inner_w, tpu_inner_h)
                fillet(vertices(), p.tpu_inner_corner_r_mm)
            with BuildSketch(Plane.XY.offset(rear_z)):
                Rectangle(rear_wrap_inner_w, rear_wrap_inner_h)
                fillet(vertices(), rear_wrap_inner_r)
            loft(mode=Mode.SUBTRACT)

        pre_vol = tpu_frame.volume
        try:
            tpu_frame = tpu_frame + _rear_wrap.part
            post_vol = tpu_frame.volume
            print(f"  Rear edge wrap (full rim): +{post_vol - pre_vol:.0f} mm³ (Z to {rear_z:.1f}, wrap_depth {rear_wrap_depth:.1f}mm)")
        except Exception:
            print("  WARNING: rear edge wrap failed to fuse")

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

    # Flush wall bumps on 3 plug walls (X+, X-, Y-) that click into
    # the shell through-holes. No channel cuts or isolated beams —
    # the bump is part of the plug wall surface.
    snap_latch_cap_info = None
    if p.include_snap_latches:
        hook_h = p.snap_latch_hook_height_mm
        hook_ramp = p.snap_latch_hook_ramp_mm
        beam_w = p.snap_latch_beam_width_mm

        half_lw = 0.5 * lip_tip_w
        half_lh = 0.5 * lip_tip_h
        plug_tip_z = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm

        # Position bump near plug tip to align with shell through-holes
        bump_z_top = plug_tip_z
        bump_z_center = bump_z_top - 0.5 * hook_ramp

        # Bump walls: X+, X-, Y- (Y+ gets retention bump only)
        latch_defs = [("X", 1.0), ("X", -1.0), ("Y", -1.0)]

        for axis, sign in latch_defs:
            half_wall = half_lw if axis == "X" else half_lh

            if axis == "X":
                bump_cx = sign * (half_wall + 0.5 * hook_h)
                with BuildPart() as _bump:
                    with Locations((bump_cx, 0.0, bump_z_center)):
                        Box(hook_h, beam_w, hook_ramp)
                cap = cap + _bump.part
            else:
                bump_cy = sign * (half_wall + 0.5 * hook_h)
                with BuildPart() as _bump:
                    with Locations((0.0, bump_cy, bump_z_center)):
                        Box(beam_w, hook_h, hook_ramp)
                cap = cap + _bump.part

        snap_latch_cap_info = {
            "enabled": True,
            "type": "flush_wall_bump",
            "count": 3,
            "walls": ["X+", "X-", "Y-"],
            "bump_protrusion_mm": float(hook_h),
            "bump_width_mm": float(beam_w),
            "bump_z_extent_mm": float(hook_ramp),
        }

    # Low-profile retention bump on non-latch wall (Y+ only)
    retention_bump_cap_info = None
    if p.include_retention_bumps and p.include_snap_latches:
        rb_h = p.retention_bump_height_mm
        rb_w = p.retention_bump_width_mm
        rb_z_ext = p.retention_bump_z_extent_mm
        half_lh = 0.5 * lip_tip_h
        plug_tip_z = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm
        rb_z = plug_tip_z - p.retention_bump_setback_mm

        # Bump on Y+ wall (protrudes outward in +Y direction)
        with BuildPart() as _rb_yp:
            with Locations((0.0, half_lh + 0.5 * rb_h, rb_z)):
                Box(rb_w, rb_h, rb_z_ext)
        cap = cap + _rb_yp.part

        retention_bump_cap_info = {
            "enabled": True,
            "walls": ["Y+"],
            "bump_height_mm": float(rb_h),
            "bump_width_mm": float(rb_w),
        }

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
            "snap_latches": snap_latch_cap_info if snap_latch_cap_info else {"enabled": False},
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
    parser.add_argument("--no-snap-clips", action="store_true")
    parser.add_argument("--no-snap-ridge", action="store_true")
    parser.add_argument("--no-snap-latches", action="store_true", help="Disable cantilever snap-fit latches")
    parser.add_argument("--no-hood", action="store_true")
    parser.add_argument("--no-vents", action="store_true")
    parser.add_argument("--no-sun-shade", action="store_true", help="Disable sun shade canopy")
    parser.add_argument("--lens-diameter", type=float, default=None)
    parser.add_argument("--hood-depth", type=float, default=None)
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None)
    args = parser.parse_args()

    p = MevoCoreParams()
    if args.no_cold_shoe:
        p.include_cold_shoe = False
    if args.no_friction_ridge:
        p.include_friction_ridge = False
    if args.no_snap_clips:
        p.include_snap_clips = False
    if args.no_snap_ridge:
        p.include_snap_ridge = False
    if args.no_snap_latches:
        p.include_snap_latches = False
    if args.no_hood:
        p.include_lens_hood = False
    if args.no_vents:
        p.include_thermal_vents = False
    if args.no_sun_shade:
        p.include_sun_shade = False
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
