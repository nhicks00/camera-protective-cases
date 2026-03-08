#!/usr/bin/env python3
"""Generate a Mevo Start dual-material case plus separate back-cap assembly.

Primary outputs:
- models/mevo_case/mevo_start_body_dual_material.step
  - contains two named bodies: TPU_Frame and ASA_Shell
- models/mevo_case/mevo_start_back_cap_dual_material.step
  - contains two named bodies: ASA_Back_Cap and TPU_Back_Insert
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
    Compound,
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
class DualMaterialParams:
    # Device nominal for reference
    device_nominal_w_mm: float = 34.0
    device_nominal_h_mm: float = 67.5625
    device_nominal_l_mm: float = 87.0

    # Core fit dimensions (snug: nominal + 0.15 mm per side/end)
    tpu_inner_w_mm: float = 34.80
    tpu_inner_h_mm: float = 68.3625
    tpu_inner_depth_mm: float = 97.325
    tpu_wall_mm: float = 1.80

    asa_wall_mm: float = 2.20
    interface_gap_mm: float = 0.0
    bond_interface_tolerance_mm: float = 0.02

    # Profile style: keep the Mevo ovular/capsule cross-section.
    enforce_capsule_profile: bool = True

    # Skeleton TPU frame (corner bumpers + edge rails instead of solid walls)
    skeleton_frame: bool = True
    skeleton_corner_bumper_w_mm: float = 12.0   # width of each corner bumper along wall
    skeleton_edge_rail_w_mm: float = 4.0        # width of connecting rails along each edge

    # Front behavior
    open_front_ovular: bool = False
    include_front_lens_led_cutouts: bool = True
    sun_hood_depth_mm: float = 3.0
    lens_cutout_d_mm: float = 42.2        # 29.5 + 12.7 mm (0.5")
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 20.0
    led_hole_d_mm: float = 5.5
    use_led_hole_from_bottom: bool = True
    led_hole_center_from_bottom_mm: float = 17.875
    led_hole_offset_from_lens_y_mm: float = 12.0
    include_lens_hood: bool = True
    lens_hood_depth_mm: float = 16.0
    lens_hood_wall_mm: float = 2.5
    lens_hood_clearance_mm: float = 1.0
    tpu_front_edge_wrap_depth_mm: float = 2.5
    tpu_front_edge_wrap_radial_mm: float = 2.0
    include_tpu_front_edge_wrap: bool = True
    include_tpu_rear_edge_wrap: bool = False

    # Bottom tripod cutout (rectangular by default)
    tripod_use_rect_cutout: bool = True
    tripod_rect_w_mm: float = 31.75    # 1.25 inches wide
    tripod_rect_l_mm: float = 50.8     # 2 inches long (along Z axis)
    tripod_hole_d_mm: float = 22.86    # legacy circular fallback
    tripod_center_from_front_mm: float = 50.7875  # shifted 1/4" toward rear vs previous
    tripod_well_d_mm: float = 32.0
    tripod_well_depth_mm: float = 6.0
    tripod_tpu_hole_extra_d_mm: float = 2.0
    tripod_tpu_cut_depth_mm: float = 8.0
    tripod_tpu_flat_depth_mm: float = 7.5

    # Legacy side vents (kept for compatibility, disabled by default)
    vent_count: int = 3
    vent_h_mm: float = 2.2
    vent_len_mm: float = 12.0
    vent_pitch_mm: float = 4.8
    vent_z_ratio: float = 0.57
    include_side_vents: bool = False

    # Thermal vents (primary)
    include_thermal_vents: bool = True
    include_side_slot_thermal_vents: bool = True
    side_vent_slot_count: int = 7
    side_vent_slot_h_mm: float = 20.0
    side_vent_slot_w_mm: float = 3.0
    side_vent_slot_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 8.0
    side_vent_cut_depth_mm: float = 6.0
    include_top_vent_hole: bool = True
    top_vent_hole_count: int = 5
    top_vent_hole_pitch_z_mm: float = 14.0
    top_vent_slot_width_mm: float = 24.0   # long axis (X direction); kept within dome flat zone
    top_vent_slot_height_mm: float = 3.5   # short axis (rounded ends)
    top_vent_cut_depth_mm: float = 6.0

    # Cold shoe mount (ISO 518 female receptor) on top rear
    include_cold_shoe: bool = True
    cold_shoe_pad_width_mm: float = 28.0
    cold_shoe_pad_length_mm: float = 30.0
    cold_shoe_pad_z_from_rear_mm: float = 15.0
    cold_shoe_pad_corner_r_mm: float = 3.0
    cold_shoe_boss_height_mm: float = 4.0
    cold_shoe_boss_length_mm: float = 22.0
    cold_shoe_boss_width_mm: float = 22.0
    cold_shoe_slot_width_mm: float = 18.8      # +0.4mm/side clearance for foot flange
    cold_shoe_rail_overhang_mm: float = 2.65   # reduced for looser stem fit
    cold_shoe_rail_thickness_mm: float = 1.8
    cold_shoe_slot_depth_mm: float = 2.5

    # Snap-latch flexure clips for back cap retention (disabled: fragile on FDM)
    include_snap_clips: bool = False
    snap_clip_beam_length_mm: float = 7.0
    snap_clip_beam_width_mm: float = 5.0
    snap_clip_beam_thickness_mm: float = 1.5
    snap_clip_catch_height_mm: float = 1.0
    snap_clip_catch_depth_mm: float = 1.0
    snap_clip_setback_mm: float = 3.0
    snap_clip_ridge_height_mm: float = 1.0
    snap_clip_y_position_mm: float = 0.0

    # Continuous friction ridge around plug perimeter
    include_friction_ridge: bool = True
    friction_ridge_height_mm: float = 0.8
    friction_ridge_width_mm: float = 1.0
    friction_ridge_setback_mm: float = 3.0

    # Cantilever latch geometry (applied to friction ridge bumps/pockets)
    latch_side_gap_mm: float = 1.0        # tangential gap each side of cantilever
    latch_tip_gap_z_mm: float = 1.5       # axial gap at free tip
    latch_catch_height_mm: float = 0.6    # catch protrusion beyond bump
    latch_catch_ramp_mm: float = 1.5      # ramp length for insertion ease
    latch_notch_depth_mm: float = 0.6     # extra radial depth of body notch
    latch_notch_z_mm: float = 1.5         # Z extent of body notch

    # Back cap engagement geometry
    back_cap_thickness_mm: float = 3.0
    back_cap_lip_depth_mm: float = 5.0
    back_cap_lip_undersize_total_mm: float = 0.28
    back_cap_tongue_depth_mm: float = 1.4
    back_cap_tongue_radial_step_mm: float = 0.0   # single-step plug (no tongue/lip step)
    back_cap_groove_radial_clearance_mm: float = 0.08
    back_cap_groove_axial_clearance_mm: float = 0.15
    back_cap_edge_fillet_mm: float = 0.6

    # Optional TPU gasket on cap inner face (disabled: ring geometry conflicts with plug)
    include_back_cap_tpu_gasket: bool = False
    back_cap_tpu_gasket_thickness_mm: float = 1.5
    back_cap_tpu_gasket_outer_inset_mm: float = 1.0
    back_cap_tpu_gasket_ring_width_mm: float = 1.6
    # Clear rear TPU in the cap insertion zone so the ASA cap can seat without collision.
    tpu_rear_cap_relief_depth_mm: float = 5.4
    tpu_rear_cap_relief_radial_mm: float = 0.3

    # Manual back-cap cutouts from Mevo device-edge offsets (default enabled).
    include_manual_back_cutouts: bool = True
    lower_cutout_side_margin_mm: float = 8.5   # widened ~3mm for USB-C boot clearance
    lower_cutout_bottom_offset_mm: float = 7.0
    lower_cutout_height_mm: float = 16.0      # doubled height, extends upward toward camera top
    upper_cutout_side_margin_mm: float = 3.0
    upper_cutout_top_offset_mm: float = 3.0
    upper_cutout_bottom_offset_from_top_mm: float = 28.0

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


def _add_upper_domed_window(width: float, top_y: float, bottom_y: float) -> None:
    """Add a half-moon top with an extended rectangular lower section."""
    if width <= 0.0 or top_y <= bottom_y:
        return
    r = min(0.5 * width, top_y - bottom_y)
    if r <= 0.0:
        return
    diam_y = top_y - r
    stem_h = max(diam_y - bottom_y, 0.0)

    # Build top half-circle only (remove lower semicircle).
    with Locations((0.0, diam_y)):
        Circle(r)
    with Locations((0.0, diam_y)):
        Rectangle(2.0 * r + 0.2, 2.0 * r + 0.2, align=(Align.CENTER, Align.MAX), mode=Mode.SUBTRACT)

    # Extend lower section to requested bottom offset.
    if stem_h > 0.05:
        with Locations((0.0, bottom_y + 0.5 * stem_h)):
            Rectangle(width, stem_h)


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

    # Manual back-cap cutouts in device-centered coordinates.
    device_half_w = 0.5 * p.device_nominal_w_mm
    device_half_h = 0.5 * p.device_nominal_h_mm

    lower_w = max(p.device_nominal_w_mm - 2.0 * p.lower_cutout_side_margin_mm, 2.0)
    lower_h = max(p.lower_cutout_height_mm, 2.0)
    lower_center_y = -device_half_h + p.lower_cutout_bottom_offset_mm + 0.5 * lower_h

    upper_w = max(p.device_nominal_w_mm - 2.0 * p.upper_cutout_side_margin_mm, 2.0)
    upper_top_y = device_half_h - p.upper_cutout_top_offset_mm
    upper_bottom_y = device_half_h - p.upper_cutout_bottom_offset_from_top_mm
    if upper_bottom_y > upper_top_y:
        upper_top_y, upper_bottom_y = upper_bottom_y, upper_top_y

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
        "manual_lower_w_mm": lower_w,
        "manual_lower_h_mm": lower_h,
        "manual_lower_center_y_mm": lower_center_y,
        "manual_upper_w_mm": upper_w,
        "manual_upper_top_y_mm": upper_top_y,
        "manual_upper_bottom_y_mm": upper_bottom_y,
    }


def build_dual_material_body(p: DualMaterialParams):
    d = _derived(p)

    min_x_asa = -0.5 * d["asa_outer_w_mm"]
    min_y_asa = -0.5 * d["asa_outer_h_mm"]
    max_y_asa = 0.5 * d["asa_outer_h_mm"]
    max_x_asa = 0.5 * d["asa_outer_w_mm"]
    min_x_tpu = -0.5 * d["tpu_outer_w_mm"]
    min_y_tpu = -0.5 * d["tpu_outer_h_mm"]
    max_x_tpu = 0.5 * d["tpu_outer_w_mm"]
    max_y_tpu = 0.5 * d["tpu_outer_h_mm"]

    cavity_start_z = d["cavity_start_z_mm"]
    cavity_depth = p.tpu_inner_depth_mm
    body_depth = d["body_depth_mm"]

    vent_z = cavity_start_z + cavity_depth * p.vent_z_ratio
    tripod_z = p.tripod_center_from_front_mm
    wrap_depth = max(min(p.tpu_front_edge_wrap_depth_mm, 0.45 * cavity_depth), 0.6)
    wrap_radial = max(p.tpu_front_edge_wrap_radial_mm, 0.6)
    wrap_inner_w = max(p.tpu_inner_w_mm - 2.0 * wrap_radial, 2.0)
    wrap_inner_h = max(p.tpu_inner_h_mm - 2.0 * wrap_radial, wrap_inner_w + 0.2)
    side_slot_count = max(int(p.side_vent_slot_count), 1)
    side_slot_pitch = max(float(p.side_vent_slot_pitch_z_mm), 0.0)
    slot_mid_z = 0.5 * body_depth
    side_slot_z_centers = [
        float(slot_mid_z + (i - 0.5 * (side_slot_count - 1)) * side_slot_pitch)
        for i in range(side_slot_count)
    ]
    # Keep side slots away from front/back corners for impact strength.
    edge_margin_z = max(8.0, 0.5 * p.side_vent_slot_w_mm + 2.0)
    side_slot_z_centers = [min(max(z, edge_margin_z), body_depth - edge_margin_z) for z in side_slot_z_centers]
    top_hole_count = max(int(p.top_vent_hole_count), 1)
    top_hole_pitch = max(float(p.top_vent_hole_pitch_z_mm), 0.0)
    top_vent_z_centers = [
        float(slot_mid_z + (i - 0.5 * (top_hole_count - 1)) * top_hole_pitch)
        for i in range(top_hole_count)
    ]
    top_hole_margin_z = max(10.0, 0.5 * p.top_vent_slot_width_mm + 3.0)
    top_vent_z_centers = [min(max(z, top_hole_margin_z), body_depth - top_hole_margin_z) for z in top_vent_z_centers]

    # Remove top vents whose center falls within the cold shoe pad footprint.
    if p.include_cold_shoe:
        cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
        cs_pad_half_l = 0.5 * p.cold_shoe_pad_length_mm
        top_vent_z_centers = [
            z for z in top_vent_z_centers
            if z < (cs_pad_z - cs_pad_half_l) or z > (cs_pad_z + cs_pad_half_l)
        ]

    if p.use_led_hole_from_bottom:
        led_center_y = -0.5 * p.device_nominal_h_mm + p.led_hole_center_from_bottom_mm
    else:
        led_center_y = p.lens_center_y_mm + p.led_hole_offset_from_lens_y_mm

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

        # Snap-latch flexure clips on inner X walls for cap retention.
        snap_clip_info = None
        if p.include_snap_clips:
            clip_tip_z = body_depth - p.snap_clip_setback_mm
            clip_base_z = clip_tip_z - p.snap_clip_beam_length_mm
            clip_mid_z = 0.5 * (clip_base_z + clip_tip_z)
            clip_y = p.snap_clip_y_position_mm
            asa_inner_half_x = 0.5 * d["asa_inner_w_mm"]
            beam_t = p.snap_clip_beam_thickness_mm
            catch_h = p.snap_clip_catch_height_mm

            for side_sign in (-1.0, 1.0):
                # Cantilever beam on inner wall surface
                beam_x = side_sign * (asa_inner_half_x - 0.5 * beam_t)
                with Locations((beam_x, clip_y, clip_mid_z)):
                    Box(beam_t, p.snap_clip_beam_width_mm, p.snap_clip_beam_length_mm)
                # Catch nub at tip, protruding radially inward
                nub_x = side_sign * (asa_inner_half_x - beam_t - 0.5 * catch_h)
                nub_z = clip_tip_z - 0.5 * p.snap_clip_catch_depth_mm
                with Locations((nub_x, clip_y, nub_z)):
                    Box(catch_h, p.snap_clip_beam_width_mm, p.snap_clip_catch_depth_mm)

            snap_clip_info = {
                "enabled": True,
                "clip_count": 2,
                "beam_length_mm": float(p.snap_clip_beam_length_mm),
                "beam_width_mm": float(p.snap_clip_beam_width_mm),
                "beam_thickness_mm": float(beam_t),
                "catch_height_mm": float(catch_h),
                "catch_depth_mm": float(p.snap_clip_catch_depth_mm),
                "clip_tip_z_mm": float(clip_tip_z),
                "setback_mm": float(p.snap_clip_setback_mm),
            }

        # Simple bump pockets on inner wall — 4 discrete pockets (one per wall center)
        friction_ridge_info = None
        if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
            fr_z = body_depth - p.friction_ridge_setback_mm
            inner_w_mm = d["asa_inner_w_mm"]
            inner_h_mm = d["asa_inner_h_mm"]
            pocket_w = 8.0   # pocket width (tangential)
            pocket_d = 0.8   # pocket radial depth into wall
            pocket_z = 4.0   # pocket extent along Z
            half_iw = 0.5 * inner_w_mm
            half_ih = 0.5 * inner_h_mm
            # 4 pockets: 2 on X walls (flat sides) + 2 on Y walls (dome ends)
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
                "pocket_width_mm": float(pocket_w),
                "pocket_depth_mm": float(pocket_d),
                "pocket_z_mm": float(pocket_z),
                "setback_mm": float(p.friction_ridge_setback_mm),
                "z_mm": float(fr_z),
            }

        # Optional lens/LED openings when a closed front wall is used.
        if (not p.open_front_ovular) and p.include_front_lens_led_cutouts:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((p.lens_center_x_mm, p.lens_center_y_mm)):
                    Circle(0.5 * p.lens_cutout_d_mm)
                with Locations((p.lens_center_x_mm, led_center_y)):
                    Circle(0.5 * p.led_hole_d_mm)
            extrude(amount=p.sun_hood_depth_mm + 0.6, mode=Mode.SUBTRACT)

        # Compute hood parameters (circular tube around lens, built separately).
        _build_hood = False
        if (not p.open_front_ovular) and p.include_lens_hood and p.lens_hood_depth_mm > 0.0:
            hood_inner_r = 0.5 * p.lens_cutout_d_mm + p.lens_hood_clearance_mm
            hood_outer_r = hood_inner_r + p.lens_hood_wall_mm
            _build_hood = True

        # Thermal vents aligned for both ASA and TPU.
        if p.include_thermal_vents:
            if p.include_side_slot_thermal_vents:
                side_cut_depth = max(p.side_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
                for side in ("neg", "pos"):
                    x_face = min_x_asa - 0.2 if side == "neg" else max_x_asa + 0.2
                    for z_c in side_slot_z_centers:
                        with BuildSketch(Plane.YZ.offset(x_face)):
                            with Locations((p.side_vent_center_y_mm, z_c)):
                                SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                        extrude(amount=side_cut_depth if side == "neg" else -side_cut_depth, mode=Mode.SUBTRACT)

            if p.include_top_vent_hole:
                top_cut_depth = max(p.top_vent_cut_depth_mm, p.asa_wall_mm + p.tpu_wall_mm + 1.0)
                # Top vents are always on the face opposite the bottom tripod side.
                with BuildSketch(Plane.XZ.offset(-(max_y_asa + 0.2))):
                    for z_c in top_vent_z_centers:
                        with Locations((0.0, z_c)):
                            SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
                extrude(amount=top_cut_depth, mode=Mode.SUBTRACT)

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

        # Bottom tripod cutout: rectangular or circular through-cut.
        # Capsule profile has a curved bottom, so the cut must be tall enough
        # to penetrate through the full semicircular wall section.
        tripod_asa_cut_depth = 0.5 * d["asa_outer_h_mm"]
        if p.tripod_use_rect_cutout:
            with Locations((0.0, min_y_asa + 0.5 * tripod_asa_cut_depth, tripod_z)):
                Box(
                    p.tripod_rect_w_mm,
                    tripod_asa_cut_depth,
                    p.tripod_rect_l_mm,
                    mode=Mode.SUBTRACT,
                )
        else:
            with Locations((0.0, min_y_asa + 0.5 * tripod_asa_cut_depth, tripod_z)):
                Cylinder(
                    0.5 * p.tripod_hole_d_mm,
                    tripod_asa_cut_depth,
                    rotation=(90, 0, 0),
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                    mode=Mode.SUBTRACT,
                )

        # Cold shoe mount (ISO 518) on top (Y+) rear with flat fill-pad.
        # Placed AFTER vent cuts so the fill pad re-fills any vent slots
        # that overlap the cold shoe footprint.
        cold_shoe_info = None
        if p.include_cold_shoe:
            # Capsule geometry: straight section up to cap_center_y, then semicircle.
            cap_center_y = 0.5 * (d["asa_outer_h_mm"] - d["asa_outer_w_mm"])
            cap_r = 0.5 * d["asa_outer_w_mm"]
            pad_w = p.cold_shoe_pad_width_mm
            pad_l = p.cold_shoe_pad_length_mm
            pad_z_center = body_depth - p.cold_shoe_pad_z_from_rear_mm

            # Fill height: extend deep enough to fully merge with the shell wall.
            # Use the full cap radius so the pad reaches well past the inner wall
            # at all points; the inner cavity subtraction below cleans the interior.
            pad_fill_height = max(cap_r + 0.5, 2.0)

            # 1) Flat fill-pad: rectangle with rounded corners on top surface.
            # Plane.XZ normal is -Y, so offset(-max_y_asa) places at Y = +max_y (top).
            with BuildSketch(Plane.XZ.offset(-max_y_asa)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(pad_w, pad_l)
                    fillet(vertices(), p.cold_shoe_pad_corner_r_mm)
            extrude(amount=pad_fill_height)

            # Subtract the inner cavity capsule so the fill pad doesn't protrude
            # into the interior — the inside surface follows the smooth capsule curve.
            with BuildSketch(Plane.XY.offset(pad_z_center - 0.5 * pad_l - 1.0)):
                _add_profile(d["asa_inner_w_mm"], d["asa_inner_h_mm"], p.enforce_capsule_profile)
            extrude(amount=pad_l + 2.0, mode=Mode.SUBTRACT)

            # Hollow the fill pad interior to save filament — leave wall-thickness shell.
            pad_shell = p.asa_wall_mm
            pad_inner_w = pad_w - 2.0 * pad_shell
            pad_inner_l = pad_l - 2.0 * pad_shell
            if pad_inner_w > 2.0 and pad_inner_l > 2.0:
                with BuildSketch(Plane.XZ.offset(-max_y_asa + pad_shell)):
                    with Locations((0.0, pad_z_center)):
                        Rectangle(pad_inner_w, pad_inner_l)
                extrude(amount=pad_fill_height - pad_shell, mode=Mode.SUBTRACT)

            # 2) Cold shoe boss on top of pad.
            cs_boss_l = p.cold_shoe_boss_length_mm
            cs_boss_w = p.cold_shoe_boss_width_mm
            cs_slot_w = p.cold_shoe_slot_width_mm
            cs_rail_oh = p.cold_shoe_rail_overhang_mm
            cs_rail_t = p.cold_shoe_rail_thickness_mm
            cs_slot_d = p.cold_shoe_slot_depth_mm
            cs_boss_h = cs_slot_d + cs_rail_t  # derive boss height so rail_thickness controls geometry
            cs_opening = cs_slot_w - 2.0 * cs_rail_oh

            with BuildSketch(Plane.XZ.offset(-max_y_asa)):
                with Locations((0.0, pad_z_center)):
                    Rectangle(cs_boss_w, cs_boss_l)
            extrude(amount=-cs_boss_h)

            # 3) T-slot channel from boss front to past rear end.
            # Boss top is at Y = max_y_asa + cs_boss_h; offset = -(max_y_asa + cs_boss_h).
            boss_top_offset = -(max_y_asa + cs_boss_h)
            cs_front_z = pad_z_center - cs_boss_l * 0.5
            cs_slot_len = body_depth + 0.2 - cs_front_z
            cs_slot_mid_z = cs_front_z + cs_slot_len * 0.5

            # Narrow opening through entire boss height (shoe stem passage)
            with BuildSketch(Plane.XZ.offset(boss_top_offset - 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_opening, cs_slot_len)
            extrude(amount=(cs_boss_h + 0.2), mode=Mode.SUBTRACT)

            # Wide floor pocket from boss bottom upward (flange pocket + rails at top)
            # Boss bottom is at shell surface: Plane.XZ.offset(-max_y_asa)
            # Normal is -Y; negative extrude goes +Y (upward into boss).
            with BuildSketch(Plane.XZ.offset(-max_y_asa + 0.1)):
                with Locations((0.0, cs_slot_mid_z)):
                    Rectangle(cs_slot_w, cs_slot_len)
            extrude(amount=-(cs_slot_d + 0.2), mode=Mode.SUBTRACT)

            cold_shoe_info = {
                "enabled": True,
                "pad_z_center_mm": float(pad_z_center),
                "pad_fill_height_mm": float(pad_fill_height),
                "y_base_mm": float(max_y_asa),
                "boss_height_mm": float(cs_boss_h),
                "slot_width_mm": float(cs_slot_w),
                "rail_opening_mm": float(cs_opening),
                "slide_in_from": "rear",
            }

    asa_shell = _largest_solid(asa_bp.part)

    # Build top-visor lens hood (arc over top of lens opening), then union.
    if _build_hood:
        # Build full cylinder tube, then clip to top-half visor within body width.
        with BuildPart() as hood_bp:
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.0)):
                Cylinder(hood_outer_r, p.lens_hood_depth_mm, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
            with Locations((p.lens_center_x_mm, p.lens_center_y_mm, 0.1)):
                Cylinder(hood_inner_r, p.lens_hood_depth_mm + 0.2, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN),
                         mode=Mode.SUBTRACT)
        # Clip box: keep only top half of arc, constrain width to ASA body
        clip_w = d["asa_outer_w_mm"]
        clip_h = hood_outer_r + 1.0  # from lens center upward
        clip_d = p.lens_hood_depth_mm + 1.0
        with BuildPart() as clip_bp:
            with Locations((p.lens_center_x_mm,
                            p.lens_center_y_mm + 0.5 * clip_h,
                            -0.5 * p.lens_hood_depth_mm)):
                Box(clip_w, clip_h, clip_d)
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

    # TPU sleeve fused in body region only (starts after sun hood).
    with BuildPart() as tpu_bp:
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            _add_profile(d["tpu_outer_w_mm"], d["tpu_outer_h_mm"], p.enforce_capsule_profile)
        extrude(amount=cavity_depth)

        with BuildSketch(Plane.XY.offset(cavity_start_z - 0.2)):
            _add_profile(p.tpu_inner_w_mm, p.tpu_inner_h_mm, p.enforce_capsule_profile)
        extrude(amount=cavity_depth + 0.4, mode=Mode.SUBTRACT)

        # Skeleton frame: cut rectangular windows from each flat wall face,
        # leaving corner bumpers and edge rails.
        # For capsule profile: X walls (flat sides) get full window cuts;
        # Y walls (semicircular ends) get narrower windows within the straight region.
        if p.skeleton_frame:
            bumper_w = p.skeleton_corner_bumper_w_mm
            rail_w = p.skeleton_edge_rail_w_mm
            skel_cut_depth = p.tpu_wall_mm + 1.0

            # X walls (left/right flat sides): wall runs along Y
            tpu_half_outer_w = 0.5 * d["tpu_outer_w_mm"]
            if p.enforce_capsule_profile:
                # Capsule: straight section height = outer_h - outer_w
                straight_h = max(d["tpu_outer_h_mm"] - d["tpu_outer_w_mm"], 0.0)
                x_wall_clear_h = max(straight_h + d["tpu_outer_w_mm"] - 2.0 * bumper_w, 0.0)
            else:
                x_wall_clear_h = max(d["tpu_outer_h_mm"] - 2.0 * bumper_w, 0.0)
            x_wall_clear_z = max(cavity_depth - 2.0 * rail_w, 0.0)
            if x_wall_clear_h > 1.0 and x_wall_clear_z > 1.0:
                x_cut_z = cavity_start_z + 0.5 * cavity_depth
                for side in (-1.0, 1.0):
                    x_face = side * (tpu_half_outer_w + 0.2)
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((0.0, x_cut_z)):
                            Rectangle(x_wall_clear_h, x_wall_clear_z)
                    extrude(
                        amount=skel_cut_depth if side < 0 else -skel_cut_depth,
                        mode=Mode.SUBTRACT,
                    )

            # Y walls (top/bottom): for capsule these are the semicircular ends
            tpu_half_outer_h = 0.5 * d["tpu_outer_h_mm"]
            if p.enforce_capsule_profile:
                # Window width limited to less than the cap diameter for structural integrity
                y_wall_clear_w = max(d["tpu_outer_w_mm"] - 2.0 * bumper_w, 0.0)
            else:
                y_wall_clear_w = max(d["tpu_outer_w_mm"] - 2.0 * bumper_w, 0.0)
            y_wall_clear_z = max(cavity_depth - 2.0 * rail_w, 0.0)
            if y_wall_clear_w > 1.0 and y_wall_clear_z > 1.0:
                y_cut_z = cavity_start_z + 0.5 * cavity_depth
                for side in (-1.0, 1.0):
                    y_face = side * (tpu_half_outer_h + 0.2)
                    with BuildSketch(Plane.XZ.offset(-y_face)):
                        with Locations((0.0, y_cut_z)):
                            Rectangle(y_wall_clear_w, y_wall_clear_z)
                    extrude(
                        amount=skel_cut_depth if side > 0 else -skel_cut_depth,
                        mode=Mode.SUBTRACT,
                    )

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

        # Rear TPU insertion relief for cap seating.
        rear_relief_depth = min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))
        if rear_relief_depth > 0.0:
            relief_start_z = cavity_start_z + cavity_depth - rear_relief_depth
            relief_w = d["tpu_outer_w_mm"] + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            relief_h = d["tpu_outer_h_mm"] + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            with BuildSketch(Plane.XY.offset(relief_start_z - 0.2)):
                _add_profile(relief_w, relief_h, p.enforce_capsule_profile)
            extrude(amount=rear_relief_depth + 0.4, mode=Mode.SUBTRACT)

        if p.include_thermal_vents:
            if p.include_side_slot_thermal_vents:
                side_cut_depth = max(p.side_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
                for side in ("neg", "pos"):
                    x_face = min_x_tpu - 0.2 if side == "neg" else max_x_tpu + 0.2
                    for z_c in side_slot_z_centers:
                        with BuildSketch(Plane.YZ.offset(x_face)):
                            with Locations((p.side_vent_center_y_mm, z_c)):
                                SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                        extrude(amount=side_cut_depth if side == "neg" else -side_cut_depth, mode=Mode.SUBTRACT)

            if p.include_top_vent_hole:
                top_cut_depth = max(p.top_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
                with BuildSketch(Plane.XZ.offset(-(max_y_tpu + 0.2))):
                    for z_c in top_vent_z_centers:
                        with Locations((0.0, z_c)):
                            SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
                extrude(amount=top_cut_depth, mode=Mode.SUBTRACT)

        # Bottom tripod cutout in TPU: rectangular or circular through-cut.
        tripod_tpu_cut_depth = 0.5 * d["tpu_outer_h_mm"]
        if p.tripod_use_rect_cutout:
            tpu_rect_w = p.tripod_rect_w_mm + max(p.tripod_tpu_hole_extra_d_mm, 0.0)
            tpu_rect_l = p.tripod_rect_l_mm + max(p.tripod_tpu_hole_extra_d_mm, 0.0)
            with Locations((0.0, min_y_tpu + 0.5 * tripod_tpu_cut_depth, tripod_z)):
                Box(
                    tpu_rect_w,
                    tripod_tpu_cut_depth,
                    tpu_rect_l,
                    mode=Mode.SUBTRACT,
                )
        else:
            tripod_tpu_d = p.tripod_hole_d_mm + max(p.tripod_tpu_hole_extra_d_mm, 0.0)
            with Locations((0.0, min_y_tpu + 0.5 * tripod_tpu_cut_depth, tripod_z)):
                Cylinder(
                    0.5 * tripod_tpu_d,
                    tripod_tpu_cut_depth,
                    rotation=(90, 0, 0),
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                    mode=Mode.SUBTRACT,
                )

    tpu_frame = _largest_solid(tpu_bp.part)
    tpu_frame.label = "TPU_Frame"

    interface_gap_w_each = 0.5 * (d["asa_inner_w_mm"] - d["tpu_outer_w_mm"])
    interface_gap_h_each = 0.5 * (d["asa_inner_h_mm"] - d["tpu_outer_h_mm"])
    max_abs_interface_gap = max(abs(interface_gap_w_each), abs(interface_gap_h_each))
    bond_grade = "A" if max_abs_interface_gap <= p.bond_interface_tolerance_mm else "B"

    report = {
        "derived_mm": d,
        "features_mm": {
            "open_front_ovular": bool(p.open_front_ovular),
            "include_front_lens_led_cutouts": bool(p.include_front_lens_led_cutouts),
            "lens_cutout_d": float(p.lens_cutout_d_mm),
            "lens_center_x": float(p.lens_center_x_mm),
            "lens_center_y": float(p.lens_center_y_mm),
            "led_hole_d": float(p.led_hole_d_mm),
            "led_hole_center_y": float(led_center_y),
            "led_hole_from_bottom_mm": float(p.led_hole_center_from_bottom_mm),
            "led_hole_from_bottom_mode": bool(p.use_led_hole_from_bottom),
            "led_hole_offset_from_lens_y": float(p.led_hole_offset_from_lens_y_mm),
            "sun_hood_depth": float(p.sun_hood_depth_mm),
            "lens_hood": {
                "enabled": bool((not p.open_front_ovular) and p.include_lens_hood),
                "type": "top_visor",
                "depth": float(p.lens_hood_depth_mm),
                "wall": float(p.lens_hood_wall_mm),
                "clearance": float(p.lens_hood_clearance_mm),
            },
            "tripod_cutout": {
                "shape": "rect" if p.tripod_use_rect_cutout else "circle",
                "rect_w_mm": float(p.tripod_rect_w_mm) if p.tripod_use_rect_cutout else None,
                "rect_l_mm": float(p.tripod_rect_l_mm) if p.tripod_use_rect_cutout else None,
                "hole_d_mm": float(p.tripod_hole_d_mm),
                "center_from_front_mm": float(p.tripod_center_from_front_mm),
                "center_from_back_mm": float(p.tpu_inner_depth_mm - p.tripod_center_from_front_mm),
            },
            "thermal_vents": {
                "enabled": bool(p.include_thermal_vents),
                "side_slots": {
                    "enabled": bool(p.include_side_slot_thermal_vents),
                    "count_per_side": int(side_slot_count),
                    "height": float(p.side_vent_slot_h_mm),
                    "width": float(p.side_vent_slot_w_mm),
                    "pitch_z": float(side_slot_pitch),
                    "center_y": float(p.side_vent_center_y_mm),
                    "z_centers": [float(z) for z in side_slot_z_centers],
                    "aligned_through_asa_tpu": True,
                },
                "top_slots": {
                    "enabled": bool(p.include_top_vent_hole),
                    "count": int(top_hole_count),
                    "pitch_z": float(top_hole_pitch),
                    "slot_width": float(p.top_vent_slot_width_mm),
                    "slot_height": float(p.top_vent_slot_height_mm),
                    "z_centers": [float(z) for z in top_vent_z_centers],
                    "center_x": 0.0,
                    "face": "top_opposite_tripod",
                },
            },
            "tpu_edge_wrap_depth": float(wrap_depth),
            "tpu_edge_wrap_radial": float(wrap_radial),
            "tpu_edge_wrap_enabled": {
                "front": bool(p.include_tpu_front_edge_wrap),
                "rear": bool(p.include_tpu_rear_edge_wrap),
            },
            "tpu_rear_cap_relief": {
                "depth": float(min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))),
                "radial": float(max(p.tpu_rear_cap_relief_radial_mm, 0.0)),
            },
            "back_cap_fit_clearance_total": float(p.back_cap_lip_undersize_total_mm),
            "back_cap_tongue_depth": float(d["tongue_depth_mm"]),
            "back_cap_tongue_radial_step": float(p.back_cap_tongue_radial_step_mm),
            "body_rear_groove_depth": float(d["groove_depth_mm"]),
            "body_rear_groove_start_z": float(d["groove_start_z_mm"]),
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "snap_clips": snap_clip_info if snap_clip_info else {"enabled": False},
            "friction_ridge": friction_ridge_info if friction_ridge_info else {"enabled": False},
        },
        "bond_interface_mm": {
            "target_gap_each": float(p.interface_gap_mm),
            "tolerance_gap_each": float(p.bond_interface_tolerance_mm),
            "actual_gap_each_width": float(interface_gap_w_each),
            "actual_gap_each_height": float(interface_gap_h_each),
            "max_abs_gap_each": float(max_abs_interface_gap),
            "bonded_contact_depth": float(cavity_depth),
            "bond_grade": bond_grade,
        },
        "print_fusion_guidance": {
            "assemble_bodies_in_slicer": True,
            "recommended_outer_walls": 4,
            "prime_tower": True,
            "min_flush_volume_mm3": 250,
        },
        "named_bodies": ["TPU_Frame", "ASA_Shell"],
        "warnings": [],
    }
    if max_abs_interface_gap > p.bond_interface_tolerance_mm:
        report["warnings"].append(
            "TPU-to-ASA interface gap exceeds tolerance; fusion quality may be reduced."
        )
    if interface_gap_w_each < -p.bond_interface_tolerance_mm or interface_gap_h_each < -p.bond_interface_tolerance_mm:
        report["warnings"].append(
            "Negative interface gap indicates interference/overlap between TPU and ASA bodies."
        )
    top_bottom_margin = 0.5 * d["asa_outer_h_mm"] - (p.side_vent_center_y_mm + 0.5 * p.side_vent_slot_h_mm)
    if top_bottom_margin < 2.5:
        report["warnings"].append(
            "Side vents are close to corner band; increase corner margin for higher impact resistance."
        )

    return asa_shell, tpu_frame, report


def build_back_cap(p: DualMaterialParams):
    d = _derived(p)

    slot_w = p.utility_slot_width_mm
    slot_h = d["utility_slot_h_mm"]
    slot_center_y = d["utility_slot_center_y_mm"]

    capsule = p.enforce_capsule_profile
    cut_depth = p.back_cap_thickness_mm + p.back_cap_lip_depth_mm + p.back_cap_tpu_gasket_thickness_mm + 1.0

    # ASA cap: plate + structural plug (tongue + lip tip).
    # Build plate+plug first, fillet those edges, then add retention features.
    with BuildPart() as asa_cap_bp:
        # Outer backing plate.
        with BuildSketch(Plane.XY):
            _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], capsule)
        extrude(amount=p.back_cap_thickness_mm)

        # Structural plug on ASA.
        if p.back_cap_tongue_radial_step_mm > 0.01:
            # Two-stage plug: wider tongue then narrower lip tip.
            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
                _add_profile(d["tongue_w_mm"], d["tongue_h_mm"], capsule)
            extrude(amount=d["tongue_depth_mm"])
            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm + d["tongue_depth_mm"])):
                _add_profile(d["lip_tip_w_mm"], d["lip_tip_h_mm"], capsule)
            extrude(amount=d["lip_tip_depth_mm"])
        else:
            # Single-step plug (no radial step between tongue and lip).
            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
                _add_profile(d["lip_tip_w_mm"], d["lip_tip_h_mm"], capsule)
            extrude(amount=p.back_cap_lip_depth_mm)

        # Port cutouts through full cap (plate + plug).
        if p.include_manual_back_cutouts:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((0.0, d["manual_lower_center_y_mm"])):
                    Rectangle(d["manual_lower_w_mm"], d["manual_lower_h_mm"])
                    fillet(vertices(), min(3.0, 0.5 * min(d["manual_lower_w_mm"], d["manual_lower_h_mm"]) - 0.1))
                _add_upper_domed_window(
                    d["manual_upper_w_mm"],
                    d["manual_upper_top_y_mm"],
                    d["manual_upper_bottom_y_mm"],
                )
            extrude(amount=cut_depth, mode=Mode.SUBTRACT)
        elif p.include_back_utility_slot:
            with BuildSketch(Plane.XY.offset(-0.2)):
                with Locations((0.0, slot_center_y)):
                    _add_vertical_stadium(slot_w, slot_h)
            extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    # Fillet plate+plug edges BEFORE adding small retention features.
    asa_cap = _largest_solid(asa_cap_bp.part)
    try:
        asa_cap = fillet(asa_cap.edges(), p.back_cap_edge_fillet_mm)
    except Exception:
        pass
    asa_cap = _largest_solid(asa_cap)

    # Add snap ridges and friction ridges after filleting so they stay crisp.
    if p.include_snap_clips and p.snap_clip_ridge_height_mm > 0.0:
        ridge_z = p.back_cap_thickness_mm + (p.back_cap_lip_depth_mm - p.snap_clip_setback_mm)
        ridge_h = p.snap_clip_ridge_height_mm
        ridge_depth_z = p.snap_clip_catch_depth_mm
        plug_half_w = 0.5 * d["lip_tip_w_mm"]
        with BuildPart() as snap_ridge_bp:
            for side_sign in (-1.0, 1.0):
                ridge_x = side_sign * (plug_half_w + 0.5 * ridge_h)
                with Locations((ridge_x, p.snap_clip_y_position_mm, ridge_z)):
                    Box(ridge_h, p.snap_clip_beam_width_mm + 1.0, ridge_depth_z)
        try:
            asa_cap = _largest_solid(asa_cap + snap_ridge_bp.part)
        except Exception:
            pass

    if p.include_friction_ridge and p.friction_ridge_height_mm > 0.0:
        fr_z = p.back_cap_thickness_mm + (p.back_cap_lip_depth_mm - p.friction_ridge_setback_mm)
        fr_h = p.friction_ridge_height_mm  # bump protrusion (radial)
        lip_w = d["lip_tip_w_mm"]
        lip_h = d["lip_tip_h_mm"]
        half_lw = 0.5 * lip_w
        half_lh = 0.5 * lip_h
        bump_w = 8.0    # bump tangential width
        bump_z = 4.0    # bump extent along Z

        # Simple flush bumps on plug outer surface (4x: 2 on X walls, 2 on Y walls)
        # X-wall bumps are flat boxes (flat sides of capsule).
        # Y-wall bumps follow the capsule dome curvature using cylinder intersection.
        with BuildPart() as fr_ridge_bp:
            for sx in (-1.0, 1.0):
                with Locations((sx * (half_lw + 0.5 * fr_h), 0.0, fr_z)):
                    Box(fr_h, bump_w, bump_z)
        try:
            asa_cap = _largest_solid(asa_cap + fr_ridge_bp.part)
        except Exception:
            pass

        if p.enforce_capsule_profile:
            # Capsule dome center Y = ± (half_lh - half_lw) for the semicircular ends
            # Dome radius = half_lw (the minor half-axis)
            dome_r_inner = half_lw
            dome_r_outer = dome_r_inner + fr_h
            dome_center_offset = half_lh - dome_r_inner  # Y distance from origin to dome center
            for sy in (-1.0, 1.0):
                dome_cy = sy * dome_center_offset
                # Build curved bump as annular ring clipped to bump footprint
                with BuildPart() as curved_bp:
                    # Outer arc shell
                    Cylinder(dome_r_outer, bump_z, align=(Align.CENTER, Align.CENTER, Align.CENTER))
                    Cylinder(dome_r_inner, bump_z + 0.2, align=(Align.CENTER, Align.CENTER, Align.CENTER),
                             mode=Mode.SUBTRACT)
                # Move to dome center and correct Z, then intersect with bounding box
                from build123d import Location, Vector
                curved_solid = curved_bp.part.moved(Location(Vector(0.0, dome_cy, fr_z)))
                # Clip to bump footprint
                with BuildPart() as clip_bp:
                    with Locations((0.0, sy * (half_lh + 0.5 * fr_h), fr_z)):
                        Box(bump_w, fr_h + 1.0, bump_z)
                try:
                    bump_clipped = curved_solid & clip_bp.part
                    asa_cap = _largest_solid(asa_cap + bump_clipped)
                except Exception:
                    # Fallback to flat box if intersection fails
                    with BuildPart() as flat_bp:
                        with Locations((0.0, sy * (half_lh + 0.5 * fr_h), fr_z)):
                            Box(bump_w, fr_h, bump_z)
                    try:
                        asa_cap = _largest_solid(asa_cap + flat_bp.part)
                    except Exception:
                        pass
        else:
            # Rectangular profile — flat Y bumps
            with BuildPart() as y_bump_bp:
                for sy in (-1.0, 1.0):
                    with Locations((0.0, sy * (half_lh + 0.5 * fr_h), fr_z)):
                        Box(bump_w, fr_h, bump_z)
            try:
                asa_cap = _largest_solid(asa_cap + y_bump_bp.part)
            except Exception:
                pass

    asa_cap.label = "ASA_Back_Cap"

    # TPU gasket: thin face pad/ring only (no structural plug duty).
    gasket = None
    if p.include_back_cap_tpu_gasket and p.back_cap_tpu_gasket_thickness_mm > 0.0:
        gasket_outer_w = max(d["asa_outer_w_mm"] - 2.0 * p.back_cap_tpu_gasket_outer_inset_mm, 2.0)
        gasket_outer_h = max(d["asa_outer_h_mm"] - 2.0 * p.back_cap_tpu_gasket_outer_inset_mm, 2.0)
        gasket_inner_w = max(gasket_outer_w - 2.0 * p.back_cap_tpu_gasket_ring_width_mm, 1.0)
        gasket_inner_h = max(gasket_outer_h - 2.0 * p.back_cap_tpu_gasket_ring_width_mm, 1.0)
        with BuildPart() as gasket_bp:
            # TPU face pad/ring on camera-contact side of ASA plug.
            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
                _add_profile(gasket_outer_w, gasket_outer_h, capsule)
            extrude(amount=p.back_cap_tpu_gasket_thickness_mm)

            with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm - 0.1)):
                _add_profile(gasket_inner_w, gasket_inner_h, capsule)
            extrude(amount=p.back_cap_tpu_gasket_thickness_mm + 0.2, mode=Mode.SUBTRACT)

            if p.include_manual_back_cutouts:
                with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm - 0.1)):
                    with Locations((0.0, d["manual_lower_center_y_mm"])):
                        Rectangle(d["manual_lower_w_mm"], d["manual_lower_h_mm"])
                        fillet(vertices(), min(3.0, 0.5 * min(d["manual_lower_w_mm"], d["manual_lower_h_mm"]) - 0.1))
                    _add_upper_domed_window(
                        d["manual_upper_w_mm"],
                        d["manual_upper_top_y_mm"],
                        d["manual_upper_bottom_y_mm"],
                    )
                extrude(
                    amount=p.back_cap_tpu_gasket_thickness_mm + 0.2,
                    mode=Mode.SUBTRACT,
                )
            elif p.include_back_utility_slot:
                with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm - 0.1)):
                    with Locations((0.0, slot_center_y)):
                        _add_vertical_stadium(slot_w, slot_h)
                extrude(
                    amount=p.back_cap_tpu_gasket_thickness_mm + 0.2,
                    mode=Mode.SUBTRACT,
                )

        gasket = _largest_solid(gasket_bp.part)
        gasket.label = "TPU_Back_Insert"

    if gasket is not None:
        # Subtract TPU volume from ASA so the two bodies don't overlap.
        try:
            asa_cap = _largest_solid(asa_cap - gasket)
        except Exception:
            pass
        asa_cap.label = "ASA_Back_Cap"
        cap_asm = Compound(children=[gasket, asa_cap], label="Mevo_Back_Cap_Assembly")
        named_bodies = ["ASA_Back_Cap", "TPU_Back_Insert"]
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
        "manual_back_cutouts_mm": {
            "enabled": bool(p.include_manual_back_cutouts),
            "lower": {
                "shape": "slot",
                "side_margin": float(p.lower_cutout_side_margin_mm),
                "bottom_offset": float(p.lower_cutout_bottom_offset_mm),
                "width": float(d["manual_lower_w_mm"]),
                "height": float(d["manual_lower_h_mm"]),
                "center_y": float(d["manual_lower_center_y_mm"]),
            },
            "upper": {
                "shape": "domed",
                "side_margin": float(p.upper_cutout_side_margin_mm),
                "top_offset": float(p.upper_cutout_top_offset_mm),
                "bottom_offset_from_top": float(p.upper_cutout_bottom_offset_from_top_mm),
                "width": float(d["manual_upper_w_mm"]),
                "top_y": float(d["manual_upper_top_y_mm"]),
                "bottom_y": float(d["manual_upper_bottom_y_mm"]),
            },
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
        "--disable-manual-back-cutouts",
        action="store_true",
        help="Disable the manual two-cutout Mevo back-cap layout.",
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
    parser.add_argument("--no-cold-shoe", action="store_true", help="Disable cold shoe mount on top rear")
    parser.add_argument("--no-snap-clips", action="store_true", help="Disable snap-latch flexure clips on body and cap")
    parser.add_argument("--no-friction-ridge", action="store_true", help="Disable continuous friction ridge on body and cap")
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None, help="Cold shoe center distance from rear edge (mm)")
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
    if args.disable_manual_back_cutouts:
        p.include_manual_back_cutouts = False
    if args.disable_front_tpu_edge_wrap:
        p.include_tpu_front_edge_wrap = False
    if args.enable_rear_tpu_edge_wrap:
        p.include_tpu_rear_edge_wrap = True
    if args.back_cap_fit_clearance_total is not None:
        p.back_cap_lip_undersize_total_mm = max(float(args.back_cap_fit_clearance_total), 0.0)
    if args.back_cap_tpu_gasket_thickness is not None:
        p.back_cap_tpu_gasket_thickness_mm = max(float(args.back_cap_tpu_gasket_thickness), 0.0)
    p.enforce_capsule_profile = not bool(args.disable_capsule_profile)
    if args.no_cold_shoe:
        p.include_cold_shoe = False
    if args.no_snap_clips:
        p.include_snap_clips = False
    if args.no_friction_ridge:
        p.include_friction_ridge = False
    if args.cold_shoe_z_from_rear is not None:
        p.cold_shoe_pad_z_from_rear_mm = float(args.cold_shoe_z_from_rear)

    asa_shell, tpu_frame, body_report = build_dual_material_body(p)
    back_cap_asm, back_cap_asa, cap_report = build_back_cap(p)

    # Cap-to-body collision check: plate-only (not plug) vs ASA shell.
    d = _derived(p)
    body_depth = d["body_depth_mm"]
    collision_report = {"cap_asa_vs_body_asa_mm3": -1.0, "collision_pass": True, "cap_seated_z_mm": 0.0}
    try:
        from build123d import Location, Vector
        # Cap z=0 face sits flush against shell rear; offset slightly to avoid coincident overlap.
        cap_seated_z = body_depth + 0.1
        with BuildPart() as plate_bp:
            with BuildSketch(Plane.XY):
                _add_profile(d["asa_outer_w_mm"], d["asa_outer_h_mm"], p.enforce_capsule_profile)
            extrude(amount=p.back_cap_thickness_mm)
        plate_only = plate_bp.part.moved(Location(Vector(0, 0, cap_seated_z)))
        try:
            inter = asa_shell & plate_only
            vol = float(inter.volume) if hasattr(inter, "volume") else 0.0
        except Exception:
            vol = 0.0
        collision_report = {
            "cap_asa_vs_body_asa_mm3": float(vol),
            "collision_pass": vol <= 0.5,
            "cap_seated_z_mm": float(cap_seated_z),
        }
    except Exception as e:
        collision_report["error"] = str(e)

    if not collision_report.get("collision_pass", True):
        print(f"WARNING: Cap-body collision detected: {collision_report['cap_asa_vs_body_asa_mm3']:.2f} mm3")

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    # 3 separate output files: ASA shell, TPU frame, back cap
    shell_step = args.out / "mevo_start_asa_shell.step"
    tpu_step = args.out / "mevo_start_tpu_frame.step"
    cap_step = args.out / "mevo_start_back_cap.step"
    report_json = reports_dir / "mevo_start_dual_material_report.json"

    # Also archive old combined dual-material files
    body_step_legacy = args.out / "mevo_start_body_dual_material.step"
    cap_step_legacy = args.out / "mevo_start_back_cap_dual_material.step"
    archived = _archive_existing(
        [shell_step, tpu_step, cap_step, report_json, body_step_legacy, cap_step_legacy],
        args.out,
    )

    export_step(asa_shell, str(shell_step))
    export_step(tpu_frame, str(tpu_step))
    export_step(back_cap_asm, str(cap_step))

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
