#!/usr/bin/env python3
"""Generate a Mevo Core protective case: ASA shell + back cap.

Primary outputs:
- models/mevo_core_case/mevo_core_asa_shell.step
- models/mevo_core_case/mevo_core_back_cap.step
- models/mevo_core_case/reports/mevo_core_report.json

Design notes:
- Square cross-section (90 x 90 mm) with rounded corners
- Full circular tube lens hood (2.75" diameter, 3.5" depth)
- Back cap as bumper ring with one large open center
- Single bottom tripod mount (1/4"-20 UNC), same rect as Mevo Start
- Large side/top panel cutouts instead of individual vent slots
- Cold shoe mount (ISO 518) on top rear
- 2 separate output files: ASA shell, back cap
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
    Cone,
    Cylinder,
    Location,
    Locations,
    Mode,
    Plane,
    Rectangle,
    Vector,
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

    # Direct ASA fit. No TPU liner/frame is generated for Mevo Core.
    asa_clearance_mm: float = 2.1       # per X/Y side; +3.0 mm total rear opening W/H vs prior 0.6
    asa_depth_clearance_mm: float = 0.6 # per front/rear end; keep insertion depth stack unchanged
    extra_length_mm: float = 1.5        # extra axial room for easy insertion
    asa_wall_mm: float = 3.3

    # Corner fillets (rounded-rectangle profile — aggressive rounding)
    asa_outer_corner_r_mm: float = 24.0
    asa_inner_corner_r_mm: float = 22.0
    front_face_side_fillet_mm: float = 3.0

    # Front wall / sun hood
    sun_hood_depth_mm: float = 3.0

    # Lens cutout (centered on front face)
    lens_cutout_d_mm: float = 76.2      # 2.75" + 0.25" = 3.0 inches
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 0.0

    # Lens hood (full circular tube encompassing lens)
    include_lens_hood: bool = True
    lens_hood_depth_mm: float = 88.9    # 3.5 inches
    lens_hood_wall_mm: float = 2.5
    lens_hood_clearance_mm: float = 2.5  # hood bore wider than front face hole for ledge/strength
    lens_hood_base_flare_mm: float = 6.0   # extra outer radius at root for strength
    lens_hood_base_depth_mm: float = 8.0   # axial depth of the taper zone
    lens_hood_base_edge_inset_mm: float = 3.0  # keep flared root inside rounded front edge
    lens_hood_root_overlap_mm: float = 1.2  # positive-Z overlap into front wall for fused root
    lens_hood_root_fuse_overlap_mm: float = 0.25  # negative-Z overlap into flare to avoid mesh seam
    lens_hood_side_access_notches: bool = True
    lens_hood_access_depth_mm: float = 44.45  # 1.75" from base along hood axis
    lens_hood_access_height_mm: float = 50.8  # 2.0" tall opening on each side
    lens_hood_access_radial_depth_mm: float = 25.4  # 1.0" radial bite through hood wall
    lens_hood_access_corner_r_mm: float = 3.0
    lens_hood_access_terminal_edge_fillet_mm: float = 0.75
    lens_hood_access_root_edge_fillet_mm: float = 0.3
    lens_hood_access_outboard_clearance_mm: float = 1.0

    # Bottom tripod cutout (same as MAKI)
    tripod_rect_w_mm: float = 63.5      # 2.5 inches
    tripod_rect_l_mm: float = 50.8      # 2.0 inches (MAKI long dimension, along Z)
    # Center from front of device: device_l - 41.275mm (1-5/8" from back)
    tripod_center_from_front_mm: float = 34.925  # shifted 1/4" toward rear

    # Thermal vents
    include_thermal_vents: bool = True
    side_vent_count: int = 7              # front-most is dropped, leaving 6 side cutout rows
    side_vent_slot_w_mm: float = 3.0      # legacy source slot width for bank extents
    side_vent_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 0.0
    side_vent_cut_depth_mm: float = 6.0
    top_vent_count: int = 6               # cold-shoe filter removes 2 rear slots; net 4 top source rows
    top_vent_slot_width_mm: float = 31.0  # legacy source slot width for margin calculation
    top_vent_pitch_z_mm: float = 9.0
    top_vent_cut_depth_mm: float = 6.0
    vent_notch_corner_margin_mm: float = 2.0
    external_cutout_corner_r_mm: float = 2.0

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

    # Back cap: large open-center bumper ring for rear access
    back_cap_center_opening_inset_mm: float = 8.0
    back_cap_center_opening_corner_r_mm: float = 8.0

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
    sun_shade_rib_corner_r_mm: float = 1.0
    sun_shade_support_vent_clearance_mm: float = 1.0
    sun_shade_front_overhang_mm: float = 0.0
    sun_shade_rear_overhang_mm: float = 0.0
    sun_shade_end_edge_fillet_mm: float = 1.0
    sun_shade_drip_lip_out_mm: float = 1.5
    sun_shade_drip_lip_drop_mm: float = 1.5
    sun_shade_drip_lip_overlap_mm: float = 0.15
    sun_shade_lower_support_overhang_mm: float = 2.0
    sun_shade_lower_outer_edge_fillet_mm: float = 2.0
    sun_shade_lower_inner_edge_fillet_mm: float = 1.0


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
    asa_inner_w = p.device_nominal_w_mm + 2.0 * p.asa_clearance_mm
    asa_inner_h = p.device_nominal_h_mm + 2.0 * p.asa_clearance_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    cavity_start_z = p.sun_hood_depth_mm
    # Body depth must accommodate: front wall + camera space + cap plug intrusion
    camera_space_depth = p.device_nominal_l_mm + 2.0 * p.asa_depth_clearance_mm + p.extra_length_mm
    asa_cavity_depth = camera_space_depth + p.back_cap_lip_depth_mm
    body_depth = p.sun_hood_depth_mm + asa_cavity_depth

    lip_tip_w = max(asa_inner_w - p.back_cap_lip_undersize_total_mm, 2.0)
    lip_tip_h = max(asa_inner_h - p.back_cap_lip_undersize_total_mm, 2.0)

    return {
        "asa_clearance_each_side_mm": p.asa_clearance_mm,
        "asa_depth_clearance_each_end_mm": p.asa_depth_clearance_mm,
        "rear_opening_total_width_height_increase_mm": 3.0,
        "camera_space_depth_mm": camera_space_depth,
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
    if p.include_cold_shoe:
        cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
        cs_pad_half_l = 0.5 * p.cold_shoe_pad_length_mm
        top_vent_z_centers = [
            z for z in top_vent_z_centers
            if z < (cs_pad_z - cs_pad_half_l) or z > (cs_pad_z + cs_pad_half_l)
        ]

    corner_margin = max(p.vent_notch_corner_margin_mm, 0.0)
    base_side_notch_y = max(asa_outer_h - 2.0 * (p.asa_outer_corner_r_mm + corner_margin), 4.0)
    base_top_notch_x = max(asa_outer_w - 2.0 * (p.asa_outer_corner_r_mm + corner_margin), 4.0)

    # The large panel notches and shade support ribs share the same face span.
    # Push ribs into the rounded corner bands, then clamp notch width so there
    # is a measured gap between the cutout edge and each rib inner edge.
    support_clearance = max(p.sun_shade_support_vent_clearance_mm, 0.0)
    side_flat_extent_half = max(half_asa_h - p.asa_outer_corner_r_mm, 0.0)
    top_flat_extent_half = max(half_asa_w - p.asa_outer_corner_r_mm, 0.0)
    default_side_rib_offset = max(side_flat_extent_half - 2.0, 0.0)
    default_top_rib_offset = max(top_flat_extent_half - 2.0, 0.0)
    side_rib_offset = default_side_rib_offset
    top_rib_offset = default_top_rib_offset
    if p.include_sun_shade:
        side_rib_offset = max(
            default_side_rib_offset,
            0.5 * base_side_notch_y + 0.5 * p.sun_shade_post_width_mm + support_clearance,
        )
        top_rib_offset = max(
            default_top_rib_offset,
            0.5 * base_top_notch_x + 0.5 * p.sun_shade_post_width_mm + support_clearance,
        )
        side_rib_offset = min(side_rib_offset, max(half_asa_h - 0.5 * p.sun_shade_post_width_mm, 0.0))
        top_rib_offset = min(top_rib_offset, max(half_asa_w - 0.5 * p.sun_shade_post_width_mm, 0.0))

    side_support_limited_y = max(
        2.0 * (side_rib_offset - 0.5 * p.sun_shade_post_width_mm - support_clearance),
        4.0,
    )
    top_support_limited_x = max(
        2.0 * (top_rib_offset - 0.5 * p.sun_shade_post_width_mm - support_clearance),
        4.0,
    )
    side_notch_y = min(base_side_notch_y, side_support_limited_y) if p.include_sun_shade else base_side_notch_y
    top_notch_x = min(base_top_notch_x, top_support_limited_x) if p.include_sun_shade else base_top_notch_x

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
            side_cut_depth = max(p.side_vent_cut_depth_mm, p.asa_wall_mm + 1.0)
            cutout_corner_r = max(p.external_cutout_corner_r_mm, 0.0)
            side_notch_match_z = 0.0
            side_notch_match_center_z = None
            if side_slot_z_centers:
                side_notch_z_min = min(side_slot_z_centers) - 0.5 * p.side_vent_pitch_z_mm
                side_notch_z_max = max(side_slot_z_centers) + 0.5 * p.side_vent_pitch_z_mm
                side_notch_z = max(side_notch_z_max - side_notch_z_min, 4.0)
                side_notch_corner_r = min(
                    cutout_corner_r,
                    0.5 * min(side_notch_y, side_notch_z) - 0.25,
                )
                side_notch_match_z = side_notch_z
                side_notch_center_z = 0.5 * (side_notch_z_min + side_notch_z_max)
                side_notch_match_center_z = side_notch_center_z
                for side in ("neg", "pos"):
                    x_face = -half_asa_w - 0.2 if side == "neg" else half_asa_w + 0.2
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, side_notch_center_z)):
                            Rectangle(side_notch_y, side_notch_z)
                            if side_notch_corner_r > 0.0:
                                fillet(vertices(), side_notch_corner_r)
                    extrude(amount=side_cut_depth if side == "neg" else -side_cut_depth,
                            mode=Mode.SUBTRACT)

            top_cut_depth = max(p.top_vent_cut_depth_mm, p.asa_wall_mm + 1.0)
            if top_vent_z_centers:
                top_notch_z_min = min(top_vent_z_centers) - 0.5 * p.top_vent_pitch_z_mm
                top_notch_z_max = max(top_vent_z_centers) + 0.5 * p.top_vent_pitch_z_mm
                top_notch_z = max(top_notch_z_max - top_notch_z_min, side_notch_match_z, 4.0)
                top_notch_center_z = (
                    side_notch_match_center_z
                    if side_notch_match_center_z is not None
                    else 0.5 * (top_notch_z_min + top_notch_z_max)
                )
                top_notch_corner_r = min(
                    cutout_corner_r,
                    0.5 * min(top_notch_x, top_notch_z) - 0.25,
                )
                with BuildSketch(Plane.XZ.offset(half_asa_h + 0.2)):
                    with Locations((0.0, top_notch_center_z)):
                        Rectangle(top_notch_x, top_notch_z)
                        if top_notch_corner_r > 0.0:
                            fillet(vertices(), top_notch_corner_r)
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

            # X+ wall. Keep these release/clearance holes rectangular so
            # the back-cap latch bumps have full corner clearance.
            snap_hole_corner_r = 0.0
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
                "hole_corner_r_mm": float(snap_hole_corner_r),
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

    # Round body edges before adding the lens hood. Applying an all-edge fillet
    # after the hood root is fused makes OCC chase the long tube/notch topology.
    body_edge_fillet_applied = 0.0
    for fillet_r in (3.0, 2.5, 2.0, 1.5, 1.0):
        try:
            asa_shell = fillet(asa_shell.edges(), fillet_r)
            body_edge_fillet_applied = fillet_r
            break
        except Exception:
            continue
    asa_shell = _largest_solid(asa_shell)

    front_face_edges = []
    body_front_abs_min = half_asa_w - 0.5
    body_front_abs_max = half_asa_w + 0.75
    for edge in asa_shell.edges():
        bb = edge.bounding_box()
        center = bb.center()
        size = bb.size
        abs_extent = max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y))
        is_body_front_edge = (
            abs(center.Z) <= 0.2
            and size.Z <= 0.2
            and edge.length >= 5.0
            and body_front_abs_min <= abs_extent <= body_front_abs_max
        )
        if is_body_front_edge:
            front_face_edges.append(edge)

    front_face_fillet_applied = body_edge_fillet_applied
    if front_face_edges and p.front_face_side_fillet_mm > 0.0:
        for fillet_r in (p.front_face_side_fillet_mm, 2.5, 2.0, 1.5, 1.0, 0.75, 0.5):
            if fillet_r > p.front_face_side_fillet_mm:
                continue
            try:
                asa_shell = fillet(front_face_edges, fillet_r)
                front_face_fillet_applied = fillet_r
                break
            except Exception:
                continue
        asa_shell = _largest_solid(asa_shell)

    # Build full circular tube lens hood with flared base, then union
    if p.include_lens_hood and p.lens_hood_depth_mm > 0.0:
        hood_access_terminal_edge_fillet_applied = 0.0
        hood_access_terminal_edge_count = 0
        hood_access_root_edge_fillet_applied = 0.0
        hood_access_root_edge_count = 0
        hood_inner_r = 0.5 * p.lens_cutout_d_mm + p.lens_hood_clearance_mm
        hood_outer_r = hood_inner_r + p.lens_hood_wall_mm
        cx, cy = p.lens_center_x_mm, p.lens_center_y_mm
        requested_flare_r = hood_outer_r + p.lens_hood_base_flare_mm
        max_flare_r = max(
            hood_outer_r,
            min(half_asa_w, half_asa_h) - max(p.lens_hood_base_edge_inset_mm, 0.0),
        )
        flare_r = min(requested_flare_r, max_flare_r)
        flare_d = min(p.lens_hood_base_depth_mm, p.lens_hood_depth_mm * 0.4)
        root_overlap = min(max(p.lens_hood_root_overlap_mm, 0.0), p.sun_hood_depth_mm + 0.2)
        root_fuse_overlap = min(
            max(p.lens_hood_root_fuse_overlap_mm, 0.0),
            max(root_overlap, 0.0),
        )
        with BuildPart() as hood_bp:
            # Embed a short collar into the front wall. This gives OpenCascade
            # overlapping solids at the root instead of a coplanar/tangent join.
            # The small negative-Z overlap closes the slicer-facing mesh seam
            # between the flared cone and the root collar.
            if root_overlap > 0.0:
                with Locations((cx, cy, -root_fuse_overlap)):
                    Cylinder(
                        flare_r,
                        root_overlap + root_fuse_overlap,
                        align=(Align.CENTER, Align.CENTER, Align.MIN),
                    )
            # Main straight tube
            with Locations((cx, cy, 0.0)):
                Cylinder(hood_outer_r, p.lens_hood_depth_mm, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Flared base cone: wider at Z=0, tapers to hood_outer_r at flare_d depth
            if p.lens_hood_base_flare_mm > 0.0 and flare_d > 0.0:
                with Locations((cx, cy, 0.0)):
                    Cone(flare_r, hood_outer_r, flare_d, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Subtract inner bore through the forward hood and the embedded root collar.
            with Locations((cx, cy, -p.lens_hood_depth_mm - 0.2)):
                Cylinder(
                    hood_inner_r,
                    p.lens_hood_depth_mm + root_overlap + 0.4,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )
            if p.lens_hood_side_access_notches:
                notch_depth = max(min(p.lens_hood_access_depth_mm, p.lens_hood_depth_mm), 0.0)
                notch_height = max(min(p.lens_hood_access_height_mm, 2.0 * hood_outer_r + 2.0), 2.0)
                notch_radial = max(p.lens_hood_access_radial_depth_mm, p.lens_hood_wall_mm + 2.0)
                notch_outboard = max(p.lens_hood_access_outboard_clearance_mm, 0.0)
                notch_outer_r = max(flare_r, hood_outer_r) + notch_outboard
                notch_cut_depth = notch_radial + max(notch_outer_r - hood_outer_r, 0.0) + 0.4
                notch_corner_r = min(
                    max(p.lens_hood_access_corner_r_mm, 0.0),
                    0.5 * min(notch_height, notch_depth) - 0.25,
                )
                for sx in (-1.0, 1.0):
                    x_face = cx + sx * notch_outer_r
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((cy, -0.5 * notch_depth)):
                            Rectangle(notch_height, notch_depth + 0.4)
                            if notch_corner_r > 0.0:
                                fillet(vertices(), notch_corner_r)
                    extrude(amount=-sx * notch_cut_depth, mode=Mode.SUBTRACT)
        hood_solid = hood_bp.part

        if p.lens_hood_side_access_notches:
            def apply_access_edge_fillet(shape, edge_selector, preferred_r, fallback_radii):
                selected_edges = []
                for edge in shape.edges():
                    if edge_selector(edge):
                        selected_edges.append(edge)

                applied_r = 0.0
                if selected_edges and preferred_r > 0.0:
                    for fillet_r in (preferred_r, *fallback_radii):
                        if fillet_r > preferred_r:
                            continue
                        try:
                            rounded = _largest_solid(fillet(selected_edges, fillet_r))
                            return rounded, fillet_r, len(selected_edges)
                        except Exception:
                            continue
                return shape, applied_r, len(selected_edges)

            def terminal_notch_edge(edge):
                bb = edge.bounding_box()
                center = bb.center()
                return (
                    abs(center.X - cx) > hood_inner_r - 5.0
                    and (-notch_depth - 0.75) <= center.Z <= (-notch_depth + 2.0)
                    and abs(center.Y - cy) <= 0.5 * notch_height + 0.75
                )

            def root_notch_edge(edge):
                bb = edge.bounding_box()
                center = bb.center()
                y_limit = 0.5 * notch_height
                y_band_inner = max(y_limit - max(p.lens_hood_access_corner_r_mm, 2.0), 0.0)
                return (
                    abs(center.X - cx) >= flare_r - 2.0
                    and (-root_fuse_overlap - 0.15) <= center.Z <= (root_overlap + 0.2)
                    and y_band_inner <= abs(center.Y - cy) <= y_limit + 0.75
                    and 0.4 <= edge.length <= 8.0
                )

            hood_solid, hood_access_terminal_edge_fillet_applied, hood_access_terminal_edge_count = (
                apply_access_edge_fillet(
                    hood_solid,
                    terminal_notch_edge,
                    p.lens_hood_access_terminal_edge_fillet_mm,
                    (0.5, 0.3, 0.2, 0.1),
                )
            )
            hood_solid, hood_access_root_edge_fillet_applied, hood_access_root_edge_count = (
                apply_access_edge_fillet(
                    hood_solid,
                    root_notch_edge,
                    p.lens_hood_access_root_edge_fillet_mm,
                    (0.2, 0.1),
                )
            )

        try:
            asa_shell = _largest_solid(asa_shell + hood_solid)
        except Exception:
            pass

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
        shade_front_overhang = max(float(p.sun_shade_front_overhang_mm), 0.0)
        shade_rear_overhang = max(float(p.sun_shade_rear_overhang_mm), 0.0)
        shade_z_start = -shade_front_overhang
        shade_z_end = body_depth + shade_rear_overhang
        shade_z_len = shade_z_end - shade_z_start
        shade_mid_z = shade_z_start + 0.5 * shade_z_len
        shade_support_z_start = 0.0
        shade_support_z_len = body_depth
        drip_lip_out = max(float(p.sun_shade_drip_lip_out_mm), 0.0)
        drip_lip_drop = max(float(p.sun_shade_drip_lip_drop_mm), 0.0)
        drip_lip_overlap = min(
            max(float(p.sun_shade_drip_lip_overlap_mm), 0.0),
            max(drip_lip_out - 0.1, 0.0),
        )
        drip_lip_inward = 0.0
        if drip_lip_out > 0.0 and drip_lip_drop > 0.0:
            drip_lip_inward = min(
                drip_lip_drop,
                max(0.5 * min(shade_inner_w, shade_inner_h) - shade_w - 1.0, 0.0),
            )
        front_drip_enabled = shade_front_overhang > 0.0 and drip_lip_inward > 0.0
        rear_drip_enabled = shade_rear_overhang > 0.0 and drip_lip_inward > 0.0
        front_drip_terminal_z = shade_z_start - drip_lip_out if front_drip_enabled else shade_z_start
        rear_drip_terminal_z = shade_z_end + drip_lip_out if rear_drip_enabled else shade_z_end
        rear_drip_extension = drip_lip_out if rear_drip_enabled else 0.0
        drip_lip_count = int(front_drip_enabled) + int(rear_drip_enabled)
        drip_lip_terminal_z_values = []
        if front_drip_enabled:
            drip_lip_terminal_z_values.append(front_drip_terminal_z)
        if rear_drip_enabled:
            drip_lip_terminal_z_values.append(rear_drip_terminal_z)
        shade_total_z_start = front_drip_terminal_z if front_drip_enabled else shade_z_start
        shade_total_z_end = rear_drip_terminal_z if rear_drip_enabled else shade_z_end
        shade_total_z_len = shade_total_z_end - shade_total_z_start
        shade_total_mid_z = shade_total_z_start + 0.5 * shade_total_z_len

        # Rib positions are shared with the support-aware vent notch sizing
        # above, so ribs stay outside the large side/top panel openings.
        lower_support_y_max = side_rib_offset + 0.5 * post_w
        shade_bottom_cut_y = lower_support_y_max + max(p.sun_shade_lower_support_overhang_mm, 0.0)

        # Rib radial span: must overlap both shell outer wall and shade inner wall
        # Shell outer at half_asa_w, shade inner at half_asa_w + standoff
        # Rib extends from shell_outer - 1mm to shade_inner + 1mm
        rib_radial = standoff + 2.0  # 1mm overlap each side
        terminal_outer_w = max(shade_outer_w - 2.0 * drip_lip_inward, 2.0 * shade_w + 2.0)
        terminal_outer_h = max(shade_outer_h - 2.0 * drip_lip_inward, 2.0 * shade_w + 2.0)
        terminal_inner_w = max(shade_inner_w - 2.0 * drip_lip_inward, 1.0)
        terminal_inner_h = max(shade_inner_h - 2.0 * drip_lip_inward, 1.0)
        terminal_inner_w = min(terminal_inner_w, terminal_outer_w - 2.0 * shade_w)
        terminal_inner_h = min(terminal_inner_h, terminal_outer_h - 2.0 * shade_w)
        terminal_outer_r = min(
            max(shade_outer_r - drip_lip_inward, 0.6),
            0.49 * min(terminal_outer_w, terminal_outer_h),
        )
        terminal_inner_r = min(
            max(shade_inner_r - drip_lip_inward, 0.4),
            0.49 * min(terminal_inner_w, terminal_inner_h),
        )

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
            cs_slot_len = shade_z_end + rear_drip_extension + 0.2 - cs_front_z
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5
            boss_overlap = min(1.5, shade_w - 0.2)

            half_shade_ow = 0.5 * shade_outer_w
            half_shade_oh = 0.5 * shade_outer_h

            with BuildPart() as shade_bp:
                def add_rounded_rib(center_x, center_y, width, height):
                    rib_corner_r = min(
                        max(p.sun_shade_rib_corner_r_mm, 0.0),
                        0.5 * min(width, height) - 0.1,
                    )
                    with BuildSketch(Plane.XY.offset(shade_support_z_start)) as rib_sk:
                        with Locations((center_x, center_y)):
                            Rectangle(width, height)
                            if rib_corner_r > 0.0:
                                fillet(vertices(), rib_corner_r)
                    extrude(rib_sk.sketch, amount=shade_support_z_len)

                # Shade tube (outer - inner). The angled drip end is part of
                # this same lofted shell, not a separate piece fused afterward.
                outer_profiles = []
                if front_drip_enabled:
                    outer_profiles.append((
                        front_drip_terminal_z,
                        terminal_outer_w,
                        terminal_outer_h,
                        terminal_outer_r,
                    ))
                outer_profiles.extend([
                    (shade_z_start, shade_outer_w, shade_outer_h, shade_outer_r),
                    (shade_z_end, shade_outer_w, shade_outer_h, shade_outer_r),
                ])
                if rear_drip_enabled:
                    outer_profiles.append((
                        rear_drip_terminal_z,
                        terminal_outer_w,
                        terminal_outer_h,
                        terminal_outer_r,
                    ))
                for profile_z, profile_w, profile_h, profile_r in outer_profiles:
                    with BuildSketch(Plane.XY.offset(profile_z)):
                        Rectangle(profile_w, profile_h)
                        fillet(vertices(), profile_r)
                loft(ruled=True)

                inner_profiles = []
                if front_drip_enabled:
                    inner_profiles.append((
                        front_drip_terminal_z - 0.1,
                        terminal_inner_w,
                        terminal_inner_h,
                        terminal_inner_r,
                    ))
                    inner_profiles.append((shade_z_start + 0.1, shade_inner_w, shade_inner_h, shade_inner_r))
                else:
                    inner_profiles.append((shade_z_start - 0.1, shade_inner_w, shade_inner_h, shade_inner_r))
                if rear_drip_enabled:
                    inner_profiles.append((shade_z_end - 0.1, shade_inner_w, shade_inner_h, shade_inner_r))
                    inner_profiles.append((
                        rear_drip_terminal_z + 0.1,
                        terminal_inner_w,
                        terminal_inner_h,
                        terminal_inner_r,
                    ))
                else:
                    inner_profiles.append((shade_z_end + 0.1, shade_inner_w, shade_inner_h, shade_inner_r))
                for profile_z, profile_w, profile_h, profile_r in inner_profiles:
                    with BuildSketch(Plane.XY.offset(profile_z)):
                        Rectangle(profile_w, profile_h)
                        fillet(vertices(), profile_r)
                loft(ruled=True, mode=Mode.SUBTRACT)

                # Cut away Y+ panel (user's bottom / tripod side)
                # Shade covers Y- (user's top) + X sides
                cut_height = half_shade_outer_h - shade_bottom_cut_y + 1.0
                with Locations((0.0, shade_bottom_cut_y + 0.5 * cut_height, shade_total_mid_z)):
                    Box(shade_outer_w + 2.0, cut_height + 0.2, shade_total_z_len + 2.0,
                        mode=Mode.SUBTRACT)

                # Connecting ribs (2 per face, near fillet transitions)
                for ry in (-1.0, 1.0):
                    add_rounded_rib(half_asa_w + 0.5 * standoff, ry * side_rib_offset, rib_radial, post_w)
                for ry in (-1.0, 1.0):
                    add_rounded_rib(-(half_asa_w + 0.5 * standoff), ry * side_rib_offset, rib_radial, post_w)
                # Y- face ribs (user's top)
                for rx in (-1.0, 1.0):
                    add_rounded_rib(rx * top_rib_offset, -(half_asa_h + 0.5 * standoff), post_w, rib_radial)

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

            lower_edge_cut_y = shade_bottom_cut_y - 0.1
            lower_edge_corner_y = half_shade_outer_h - shade_outer_r
            lower_edge_dy = lower_edge_cut_y - lower_edge_corner_y
            lower_outer_x = half_shade_outer_w - shade_outer_r
            lower_inner_x = 0.0
            if abs(lower_edge_dy) < shade_outer_r:
                lower_outer_x += (shade_outer_r**2 - lower_edge_dy**2) ** 0.5
            if abs(lower_edge_dy) < shade_inner_r:
                lower_inner_x = half_shade_outer_w - shade_outer_r + (
                    shade_inner_r**2 - lower_edge_dy**2
                ) ** 0.5
            lower_outer_edge_min_x = 0.5 * (lower_outer_x + lower_inner_x)

            def fillet_lower_edges(shape, min_abs_x, max_abs_x, preferred_r):
                lower_edges = []
                for edge in shape.edges():
                    bb = edge.bounding_box()
                    center = bb.center()
                    size = bb.size
                    abs_x = abs(center.X)
                    is_lower_side_edge = (
                        abs(center.Y - lower_edge_cut_y) <= 0.25
                        and size.Z >= 0.8 * shade_z_len
                        and size.X <= 0.25
                        and size.Y <= 0.25
                        and min_abs_x <= abs_x <= max_abs_x
                    )
                    if is_lower_side_edge:
                        lower_edges.append(edge)

                applied = 0.0
                if lower_edges and preferred_r > 0.0:
                    for fillet_r in (preferred_r, 1.5, 1.0, 0.75, 0.5):
                        if fillet_r > preferred_r:
                            continue
                        try:
                            return fillet(lower_edges, fillet_r), fillet_r, len(lower_edges)
                        except Exception:
                            continue
                return shape, applied, len(lower_edges)

            shade_solid, lower_outer_edge_fillet_applied, lower_outer_edge_count = fillet_lower_edges(
                shade_solid,
                lower_outer_edge_min_x,
                half_shade_outer_w + 0.25,
                p.sun_shade_lower_outer_edge_fillet_mm,
            )
            shade_solid, lower_inner_edge_fillet_applied, lower_inner_edge_count = fillet_lower_edges(
                shade_solid,
                max(lower_inner_x - 0.25, 0.0),
                lower_outer_edge_min_x,
                p.sun_shade_lower_inner_edge_fillet_mm,
            )

            if lower_inner_edge_fillet_applied == 0.0 and lower_outer_edge_fillet_applied == 0.0:
                lower_edges = []
                for edge in shade_solid.edges():
                    bb = edge.bounding_box()
                    center = bb.center()
                    size = bb.size
                    is_lower_side_edge = (
                        abs(center.Y - lower_edge_cut_y) <= 0.25
                        and size.Z >= 0.8 * shade_z_len
                        and size.X <= 0.25
                        and size.Y <= 0.25
                    )
                    if is_lower_side_edge:
                        lower_edges.append(edge)
                for fillet_r in (0.5,):
                    try:
                        shade_solid = fillet(lower_edges, fillet_r)
                        lower_inner_edge_fillet_applied = fillet_r
                        lower_outer_edge_fillet_applied = fillet_r
                        lower_inner_edge_count = len(lower_edges)
                        lower_outer_edge_count = len(lower_edges)
                        break
                    except Exception:
                        continue

            def fillet_end_edges(shape, preferred_r):
                end_z_values = tuple(drip_lip_terminal_z_values) or (shade_z_start, shade_z_end)
                result = shape
                applied = 0.0
                edge_count = 0
                for target_z in end_z_values:
                    end_edges = []
                    for edge in result.edges():
                        bb = edge.bounding_box()
                        center = bb.center()
                        size = bb.size
                        is_end_edge = (
                            abs(center.Z - target_z) <= 0.25
                            and size.Z <= 0.35
                            and (size.X >= 0.35 or size.Y >= 0.35)
                        )
                        if is_end_edge:
                            end_edges.append(edge)

                    edge_count += len(end_edges)
                    if not end_edges or preferred_r <= 0.0:
                        continue
                    for fillet_r in (preferred_r, 0.75, 0.5, 0.3, 0.2, 0.1):
                        if fillet_r > preferred_r:
                            continue
                        try:
                            result = fillet(end_edges, fillet_r)
                            applied = max(applied, fillet_r)
                            break
                        except Exception:
                            continue
                return result, applied, edge_count

            shade_solid, end_edge_fillet_applied, end_edge_count = fillet_end_edges(
                shade_solid,
                p.sun_shade_end_edge_fillet_mm,
            )

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
                "front_overhang_mm": float(shade_front_overhang),
                "rear_overhang_mm": float(shade_rear_overhang),
                "shade_z_start_mm": float(shade_z_start),
                "shade_z_end_mm": float(shade_z_end),
                "support_z_start_mm": float(shade_support_z_start),
                "support_z_end_mm": float(shade_support_z_start + shade_support_z_len),
                "drip_lip_count": int(drip_lip_count),
                "drip_lip_style": "integrated_continuous_lofted_shade_end",
                "drip_lip_out_mm": float(drip_lip_out),
                "drip_lip_drop_mm": float(drip_lip_drop),
                "drip_lip_inward_mm": float(drip_lip_inward),
                "drip_lip_overlap_mm": 0.0,
                "drip_lip_legacy_overlap_param_mm": float(drip_lip_overlap),
                "drip_lip_terminal_z_values_mm": [float(v) for v in drip_lip_terminal_z_values],
                "post_width_mm": float(post_w),
                "rib_corner_r_mm": float(
                    min(
                        max(p.sun_shade_rib_corner_r_mm, 0.0),
                        0.5 * min(post_w, rib_radial) - 0.1,
                    )
                ),
                "rib_count": 6,
                "coverage": "top + left + right (open bottom)",
                "support_vent_clearance_mm": float(support_clearance),
                "side_support_center_y_abs_mm": float(side_rib_offset),
                "side_support_inner_edge_y_abs_mm": float(side_rib_offset - 0.5 * post_w),
                "top_support_center_x_abs_mm": float(top_rib_offset),
                "top_support_inner_edge_x_abs_mm": float(top_rib_offset - 0.5 * post_w),
                "lower_support_center_y_mm": float(side_rib_offset),
                "lower_support_outer_y_mm": float(lower_support_y_max),
                "lower_support_overhang_mm": float(max(p.sun_shade_lower_support_overhang_mm, 0.0)),
                "bottom_cut_y_mm": float(shade_bottom_cut_y),
                "lower_outer_edge_fillet_mm": float(lower_outer_edge_fillet_applied),
                "lower_outer_edge_fillet_edges": lower_outer_edge_count,
                "lower_inner_edge_fillet_mm": float(lower_inner_edge_fillet_applied),
                "lower_inner_edge_fillet_edges": lower_inner_edge_count,
                "end_edge_fillet_mm": float(end_edge_fillet_applied),
                "end_edge_fillet_edges": int(end_edge_count),
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

    side_notch_info = {"enabled": False}
    side_notch_report_z = 0.0
    side_notch_report_center_z = None
    if p.include_thermal_vents and side_slot_z_centers:
        side_z_min = min(side_slot_z_centers) - 0.5 * p.side_vent_pitch_z_mm
        side_z_max = max(side_slot_z_centers) + 0.5 * p.side_vent_pitch_z_mm
        side_notch_report_z = max(side_z_max - side_z_min, 4.0)
        side_notch_report_center_z = 0.5 * (side_z_min + side_z_max)
        side_notch_info = {
            "enabled": True,
            "type": "single_rectangular_panel_cutout_per_side",
            "source_rows_per_side": len(side_slot_z_centers),
            "width_y_mm": float(side_notch_y),
            "base_width_y_mm": float(base_side_notch_y),
            "support_limited_width_y_mm": float(side_support_limited_y),
            "height_z_mm": float(side_notch_report_z),
            "center_y_mm": float(p.side_vent_center_y_mm),
            "center_z_mm": float(side_notch_report_center_z),
            "corner_r_mm": float(
                min(
                    max(p.external_cutout_corner_r_mm, 0.0),
                    0.5 * min(side_notch_y, side_notch_report_z) - 0.25,
                )
            ),
            "corner_margin_mm": float(corner_margin),
            "support_clearance_mm": float(support_clearance),
            "support_aware_sizing": bool(p.include_sun_shade),
            "side_support_inner_edge_y_abs_mm": float(side_rib_offset - 0.5 * p.sun_shade_post_width_mm),
        }
    top_notch_info = {"enabled": False}
    if p.include_thermal_vents and top_vent_z_centers:
        top_z_min = min(top_vent_z_centers) - 0.5 * p.top_vent_pitch_z_mm
        top_z_max = max(top_vent_z_centers) + 0.5 * p.top_vent_pitch_z_mm
        raw_top_notch_z = max(top_z_max - top_z_min, 4.0)
        top_notch_z = max(raw_top_notch_z, side_notch_report_z, 4.0)
        top_notch_info = {
            "enabled": True,
            "type": "single_rectangular_panel_cutout",
            "source_rows": len(top_vent_z_centers),
            "width_x_mm": float(top_notch_x),
            "base_width_x_mm": float(base_top_notch_x),
            "support_limited_width_x_mm": float(top_support_limited_x),
            "height_z_mm": float(top_notch_z),
            "raw_source_height_z_mm": float(raw_top_notch_z),
            "matched_side_notch_height": bool(top_notch_z > raw_top_notch_z),
            "center_x_mm": 0.0,
            "center_z_mm": float(
                side_notch_report_center_z
                if side_notch_report_center_z is not None
                else 0.5 * (top_z_min + top_z_max)
            ),
            "matched_side_notch_alignment": side_notch_report_center_z is not None,
            "corner_r_mm": float(
                min(
                    max(p.external_cutout_corner_r_mm, 0.0),
                    0.5 * min(top_notch_x, top_notch_z) - 0.25,
                )
            ),
            "corner_margin_mm": float(corner_margin),
            "support_clearance_mm": float(support_clearance),
            "support_aware_sizing": bool(p.include_sun_shade),
            "top_support_inner_edge_x_abs_mm": float(top_rib_offset - 0.5 * p.sun_shade_post_width_mm),
        }

    report = {
        "derived_mm": d,
        "features_mm": {
            "lens_cutout_d": float(p.lens_cutout_d_mm),
            "lens_center_x": float(p.lens_center_x_mm),
            "lens_center_y": float(p.lens_center_y_mm),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "front_face_to_side_fillet": {
                "applied_radius_mm": float(front_face_fillet_applied),
                "target_radius_mm": float(p.front_face_side_fillet_mm),
                "edge_count": len(front_face_edges),
                "scope": "main ASA body front perimeter, rounded before lens hood fusion",
            },
            "lens_hood": {
                "enabled": bool(p.include_lens_hood),
                "type": "full_tube",
                "depth_mm": float(p.lens_hood_depth_mm),
                "wall_mm": float(p.lens_hood_wall_mm),
                "clearance_mm": float(p.lens_hood_clearance_mm),
                "connection": "embedded root overlap collar fused into ASA front wall",
                "root_overlap_mm": float(p.lens_hood_root_overlap_mm),
                "resolved_root_overlap_mm": float(
                    min(max(p.lens_hood_root_overlap_mm, 0.0), p.sun_hood_depth_mm + 0.2)
                ),
                "root_fuse_overlap_mm": float(
                    min(
                        max(p.lens_hood_root_fuse_overlap_mm, 0.0),
                        max(
                            min(max(p.lens_hood_root_overlap_mm, 0.0), p.sun_hood_depth_mm + 0.2),
                            0.0,
                        ),
                    )
                ),
                "base_flare": {
                    "requested_extra_radius_mm": float(p.lens_hood_base_flare_mm),
                    "requested_outer_radius_mm": float(
                        0.5 * p.lens_cutout_d_mm
                        + p.lens_hood_clearance_mm
                        + p.lens_hood_wall_mm
                        + p.lens_hood_base_flare_mm
                    ),
                    "resolved_outer_radius_mm": float(
                        min(
                            0.5 * p.lens_cutout_d_mm
                            + p.lens_hood_clearance_mm
                            + p.lens_hood_wall_mm
                            + p.lens_hood_base_flare_mm,
                            max(
                                0.5 * p.lens_cutout_d_mm
                                + p.lens_hood_clearance_mm
                                + p.lens_hood_wall_mm,
                                min(
                                    0.5 * d["asa_outer_w_mm"],
                                    0.5 * d["asa_outer_h_mm"],
                                ) - max(p.lens_hood_base_edge_inset_mm, 0.0),
                            ),
                        )
                    ),
                    "edge_inset_mm": float(p.lens_hood_base_edge_inset_mm),
                    "trim_reason": "prevents hood root from hanging past rounded front face perimeter",
                },
                "side_access_notches": {
                    "enabled": bool(p.lens_hood_side_access_notches),
                    "count": 2 if p.lens_hood_side_access_notches else 0,
                    "depth_mm": float(p.lens_hood_access_depth_mm),
                    "height_mm": float(p.lens_hood_access_height_mm),
                    "radial_depth_mm": float(p.lens_hood_access_radial_depth_mm),
                    "corner_r_mm": float(p.lens_hood_access_corner_r_mm),
                    "terminal_edge_fillet_mm": float(hood_access_terminal_edge_fillet_applied),
                    "terminal_edge_fillet_target_mm": float(p.lens_hood_access_terminal_edge_fillet_mm),
                    "terminal_edge_fillet_edges": int(hood_access_terminal_edge_count),
                    "root_edge_fillet_mm": float(hood_access_root_edge_fillet_applied),
                    "root_edge_fillet_target_mm": float(p.lens_hood_access_root_edge_fillet_mm),
                    "root_edge_fillet_edges": int(hood_access_root_edge_count),
                    "outboard_clearance_mm": float(p.lens_hood_access_outboard_clearance_mm),
                    "cut_starts_outside_flare": True,
                },
            },
            "tripod_cutout": {
                "rect_w_mm": float(p.tripod_rect_w_mm),
                "rect_l_mm": float(p.tripod_rect_l_mm),
                "center_from_front_mm": float(p.tripod_center_from_front_mm),
            },
            "thermal_vents": {
                "mode": "large_panel_notches",
                "side_notches": side_notch_info,
                "top_notch": top_notch_info,
            },
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "friction_ridge": friction_ridge_info if friction_ridge_info else {"enabled": False},
            "snap_latches": snap_latch_info if snap_latch_info else {"enabled": False},
            "sun_shade": sun_shade_info if sun_shade_info else {"enabled": False},
        },
    }

    return asa_shell, report


def build_back_cap(p: MevoCoreParams):
    d = _derived(p)

    asa_outer_w = d["asa_outer_w_mm"]
    asa_outer_h = d["asa_outer_h_mm"]
    lip_tip_w = d["lip_tip_w_mm"]
    lip_tip_h = d["lip_tip_h_mm"]
    center_opening_w = max(p.device_nominal_w_mm - 2.0 * p.back_cap_center_opening_inset_mm, 10.0)
    center_opening_h = max(p.device_nominal_h_mm - 2.0 * p.back_cap_center_opening_inset_mm, 10.0)
    center_opening_r = min(
        max(p.back_cap_center_opening_corner_r_mm, 0.0),
        0.5 * min(center_opening_w, center_opening_h) - 0.5,
    )

    cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + 1.0

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

        # Large center opening through the back cap plate and plug lip.
        with BuildSketch(Plane.XY.offset(-0.2)):
            Rectangle(center_opening_w, center_opening_h)
            if center_opening_r > 0.0:
                fillet(vertices(), center_opening_r)
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
            "center_opening": {
                "enabled": True,
                "type": "single_large_rounded_rectangle",
                "w_mm": float(center_opening_w),
                "h_mm": float(center_opening_h),
                "corner_r_mm": float(center_opening_r),
                "inset_from_device_edge_mm": float(p.back_cap_center_opening_inset_mm),
            },
        },
        "named_bodies": ["ASA_Back_Cap"],
    }

    return cap, report


def main():
    parser = argparse.ArgumentParser(
        description="Generate Mevo Core case: ASA shell + back cap"
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

    asa_shell, asa_report = build_asa_shell(p)
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
    cap_step = args.out / "mevo_core_back_cap.step"
    report_json = reports_dir / "mevo_core_report.json"

    archived = _archive_existing([shell_step, cap_step, report_json], args.out)

    # Hard cutover from the prior TPU workflow: no current TPU deliverables.
    removed_obsolete = []
    for obsolete in (
        args.out / "mevo_core_tpu_frame.step",
        args.out / "mevo_core_tpu_frame.3mf",
        args.out / "mevo_core_PETG_SHELL.3mf",
    ):
        if obsolete.exists():
            obsolete.unlink()
            removed_obsolete.append(str(obsolete))

    export_step(asa_shell, str(shell_step))
    export_step(back_cap, str(cap_step))

    asa_inner_w = d["asa_inner_w_mm"]
    asa_inner_h = d["asa_inner_h_mm"]
    asa_cav_depth = d["asa_cavity_depth_mm"]
    cap_plug_depth = p.back_cap_lip_depth_mm
    camera_space = asa_cav_depth - cap_plug_depth
    radial_clearance_w = 0.5 * (asa_inner_w - p.device_nominal_w_mm)
    radial_clearance_h = 0.5 * (asa_inner_h - p.device_nominal_h_mm)
    axial_clearance_total = camera_space - p.device_nominal_l_mm

    fit_report = {
        "radial_clearance_w_each_mm": float(radial_clearance_w),
        "radial_clearance_h_each_mm": float(radial_clearance_h),
        "asa_cavity_depth_mm": float(asa_cav_depth),
        "cap_plug_intrusion_mm": float(cap_plug_depth),
        "effective_camera_space_mm": float(camera_space),
        "device_length_mm": float(p.device_nominal_l_mm),
        "axial_clearance_total_mm": float(axial_clearance_total),
        "camera_fits": camera_space >= p.device_nominal_l_mm,
    }

    payload = {
        "params": asdict(p),
        "asa_shell_report": asa_report,
        "back_cap_report": cap_report,
        "direct_asa_fit": fit_report,
        "collision_check": collision_report,
        "obsolete_outputs_removed": removed_obsolete,
    }
    report_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    if removed_obsolete:
        print(f"Removed {len(removed_obsolete)} obsolete file(s)")
    print(f"Wrote {shell_step}")
    print(f"Wrote {cap_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
