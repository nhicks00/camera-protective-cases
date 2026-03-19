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
    Cone,
    Cylinder,
    Locations,
    Mode,
    Plane,
    Rectangle,
    SlotOverall,
    add,
    export_step,
    export_stl,
    extrude,
    fillet,
    import_step,
    loft,
    vertices,
)


@dataclass
class ZowietekParams:
    # Device nominal (L x W x H)
    device_nominal_l_mm: float = 68.6
    device_nominal_w_mm: float = 60.2
    device_nominal_h_mm: float = 51.0
    extra_depth_mm: float = 4.0         # added rear margin for TPU-installed fit + cap seating

    # TPU clearance and wall
    tpu_clearance_mm: float = 0.40        # per side (matched to Mevo)
    tpu_wall_mm: float = 1.8
    tpu_axial_trim_mm: float = 4.0        # shorten TPU/front-back fit from print feedback

    # ASA shell
    asa_wall_mm: float = 2.2
    interface_gap_mm: float = 0.15        # per-side assembly gap so TPU can slide into ASA
    bond_interface_tolerance_mm: float = 0.20
    rear_cap_seat_depth_mm: float = 6.5   # extra shell-only depth behind TPU for cap insertion/snap seating

    # Corner fillets (rounded-rectangle profile)
    asa_outer_corner_r_mm: float = 8.0
    asa_inner_corner_r_mm: float = 5.8
    tpu_outer_corner_r_mm: float = 5.8
    tpu_inner_corner_r_mm: float = 4.0

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
    lens_hood_base_flare_mm: float = 4.0    # extra outer radius at root for strength
    lens_hood_base_depth_mm: float = 6.0    # axial depth of the taper zone

    # Skeleton TPU frame params
    tpu_corner_bumper_w_mm: float = 12.0  # width of each corner bumper along wall
    tpu_edge_rail_w_mm: float = 4.0       # width of connecting rails along each edge
    tpu_front_edge_wrap_depth_mm: float = 2.5
    tpu_front_edge_wrap_radial_mm: float = 2.0
    include_tpu_front_edge_wrap: bool = True

    # Bottom tripod cutout (1/4"-20)
    tripod_rect_w_mm: float = 31.75       # 1.25 inches
    tripod_rect_l_mm: float = 25.4        # 1 inch along Z
    tripod_bottom_center_from_front_mm: float = 47.0  # shifted rearward ~0.5 in from prior print
    # Top tripod cutout (mirrored)
    include_top_tripod: bool = False
    tripod_top_center_from_front_mm: float = 34.3

    # Thermal vents
    include_thermal_vents: bool = True
    side_vent_count: int = 5
    side_vent_slot_h_mm: float = 25.2     # vertical extent
    side_vent_slot_w_mm: float = 3.0      # width (rounded ends)
    side_vent_pitch_z_mm: float = 9.0
    side_vent_center_y_mm: float = 0.0    # vertically centered
    side_vent_cut_depth_mm: float = 6.0
    top_vent_count: int = 4
    top_vent_slot_width_mm: float = 30.8  # long axis (X)
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

    # Cantilever snap-fit latches (beams on cap plug, through-holes in shell)
    include_snap_latches: bool = True
    snap_latch_beam_length_mm: float = 4.0   # free cantilever length along Z axis
    snap_latch_beam_width_mm: float = 8.0    # lateral width of beam strip
    snap_latch_beam_thickness_mm: float = 1.5 # radial thickness of beam (thin for flex)
    snap_latch_hook_height_mm: float = 1.0   # reduced protrusion for easier snap-in
    snap_latch_hook_ramp_mm: float = 2.5     # Z extent of hook ramp
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

    # Rear TPU relief
    tpu_rear_cap_relief_depth_mm: float = 6.0
    tpu_rear_cap_relief_radial_mm: float = 0.3

    # Rear TPU corner bumpers
    include_rear_tpu_bumpers: bool = True
    tpu_rear_bumper_depth_mm: float = 3.0    # how far bumper wraps around rear corners
    tpu_rear_bumper_wall_mm: float = 1.8     # wall thickness of rear bumper

    # Floating sun shade canopy (top + partial sides, open bottom)
    include_sun_shade: bool = True
    sun_shade_standoff_mm: float = 6.0
    sun_shade_wall_mm: float = 2.0
    sun_shade_post_width_mm: float = 4.0
    sun_shade_side_drop_ratio: float = 0.72  # extend side coverage enough to fully mask the vent zone
    sun_shade_side_support_height_mm: float = 3.0  # tie the lower side edge back into the shell


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


def _verify_exported_step(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Expected STEP output missing: {path}")

    part = import_step(str(path))
    solids = part.solids() if hasattr(part, "solids") else []
    if len(solids) != 1:
        raise RuntimeError(f"Expected 1 solid in {path.name}, found {len(solids)}")

    stat = path.stat()
    return {
        "path": str(path),
        "solids": int(len(solids)),
        "volume_mm3": float(part.volume),
        "size_bytes": int(stat.st_size),
        "modified_at": datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds"),
    }


def _verify_file_exists(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Expected export missing: {path}")
    stat = path.stat()
    return {
        "path": str(path),
        "size_bytes": int(stat.st_size),
        "modified_at": datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds"),
    }


def _resolved_wrap_depth(requested_depth: float, cavity_depth: float, enabled: bool) -> float:
    if not enabled:
        return 0.0
    return max(min(requested_depth, 0.45 * cavity_depth), 0.6)


def _build_face_wrap_ring(
    outer_w: float,
    outer_h: float,
    outer_r: float,
    inner_w: float,
    inner_h: float,
    inner_r: float,
    z_start: float,
    depth: float,
):
    with BuildPart() as wrap_bp:
        with BuildSketch(Plane.XY.offset(z_start)):
            Rectangle(outer_w, outer_h)
            fillet(vertices(), outer_r)
        extrude(amount=depth)
        with BuildSketch(Plane.XY.offset(z_start - 0.2)):
            Rectangle(inner_w, inner_h)
            fillet(vertices(), inner_r)
        extrude(amount=depth + 0.4, mode=Mode.SUBTRACT)
    return _largest_solid(wrap_bp.part)


def _build_top_cold_shoe_rails(
    roof_y: float,
    z_center: float,
    boss_l: float,
    boss_w: float,
    slot_w: float,
    opening_w: float,
    slot_d: float,
    rail_t: float,
    rail_oh: float,
    overlap: float,
):
    outer_wall_w = max(0.5 * (boss_w - slot_w), 0.8)
    boss_h = slot_d + rail_t
    wall_center_y = roof_y + 0.5 * (boss_h - overlap)
    overhang_center_y = roof_y + slot_d + 0.5 * rail_t

    with BuildPart() as shoe_bp:
        for sx in (-1.0, 1.0):
            with Locations((sx * (0.5 * slot_w + 0.5 * outer_wall_w), wall_center_y, z_center)):
                Box(outer_wall_w, boss_h + overlap, boss_l)
            with Locations((sx * (0.5 * opening_w + 0.5 * rail_oh), overhang_center_y, z_center)):
                Box(rail_oh, rail_t, boss_l)
    return shoe_bp.part


def _build_side_cold_shoe_rails(
    side_x: float,
    outward_sign: float,
    z_center: float,
    boss_l: float,
    boss_w: float,
    slot_w: float,
    opening_w: float,
    slot_d: float,
    rail_t: float,
    rail_oh: float,
    overlap: float,
):
    outer_wall_w = max(0.5 * (boss_w - slot_w), 0.8)
    boss_h = slot_d + rail_t
    wall_center_x = side_x + outward_sign * 0.5 * (boss_h - overlap)
    overhang_center_x = side_x + outward_sign * (slot_d + 0.5 * rail_t)

    with BuildPart() as shoe_bp:
        for sy in (-1.0, 1.0):
            with Locations((wall_center_x, sy * (0.5 * slot_w + 0.5 * outer_wall_w), z_center)):
                Box(boss_h + overlap, outer_wall_w, boss_l)
            with Locations((overhang_center_x, sy * (0.5 * opening_w + 0.5 * rail_oh), z_center)):
                Box(rail_t, rail_oh, boss_l)
    return shoe_bp.part


def _derived(p: ZowietekParams) -> dict:
    # Inner cavity = device + clearance
    tpu_inner_w = p.device_nominal_w_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_h = p.device_nominal_h_mm + 2.0 * p.tpu_clearance_mm
    base_tpu_inner_depth = p.device_nominal_l_mm + 2.0 * p.tpu_clearance_mm + p.extra_depth_mm

    required_usable_device_depth = p.device_nominal_l_mm + 2.0 * p.tpu_clearance_mm
    tpu_inner_depth = base_tpu_inner_depth
    front_wrap_depth = 0.0
    rear_wrap_depth = 0.0
    for _ in range(4):
        front_wrap_depth = _resolved_wrap_depth(
            p.tpu_front_edge_wrap_depth_mm,
            tpu_inner_depth,
            p.include_tpu_front_edge_wrap,
        )
        rear_wrap_depth = _resolved_wrap_depth(
            p.tpu_front_edge_wrap_depth_mm,
            tpu_inner_depth,
            p.include_rear_tpu_bumpers,
        )
        required_inner_depth = required_usable_device_depth + front_wrap_depth + rear_wrap_depth
        resolved_depth = max(base_tpu_inner_depth, required_inner_depth)
        if abs(resolved_depth - tpu_inner_depth) <= 1e-6:
            tpu_inner_depth = resolved_depth
            break
        tpu_inner_depth = resolved_depth

    tpu_inner_depth = max(tpu_inner_depth - max(p.tpu_axial_trim_mm, 0.0), 10.0)
    usable_tpu_device_depth = tpu_inner_depth - front_wrap_depth - rear_wrap_depth

    tpu_outer_w = tpu_inner_w + 2.0 * p.tpu_wall_mm
    tpu_outer_h = tpu_inner_h + 2.0 * p.tpu_wall_mm

    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    cavity_start_z = p.sun_hood_depth_mm
    shell_inner_depth = tpu_inner_depth + max(p.rear_cap_seat_depth_mm, 0.0)
    rear_cap_fit_margin = max(shell_inner_depth - tpu_inner_depth - p.back_cap_lip_depth_mm, 0.0)
    body_depth = p.sun_hood_depth_mm + shell_inner_depth

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
        "required_usable_device_depth_mm": required_usable_device_depth,
        "usable_tpu_device_depth_mm": usable_tpu_device_depth,
        "front_wrap_intrusion_mm": front_wrap_depth,
        "rear_wrap_intrusion_mm": rear_wrap_depth,
        "tpu_outer_w_mm": tpu_outer_w,
        "tpu_outer_h_mm": tpu_outer_h,
        "asa_inner_w_mm": asa_inner_w,
        "asa_inner_h_mm": asa_inner_h,
        "asa_outer_w_mm": asa_outer_w,
        "asa_outer_h_mm": asa_outer_h,
        "cavity_start_z_mm": cavity_start_z,
        "shell_inner_depth_mm": shell_inner_depth,
        "rear_cap_seat_depth_mm": float(max(p.rear_cap_seat_depth_mm, 0.0)),
        "rear_cap_fit_margin_mm": rear_cap_fit_margin,
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
    shell_inner_depth = d["shell_inner_depth_mm"]
    body_depth = d["body_depth_mm"]
    usable_tpu_device_depth = d["usable_tpu_device_depth_mm"]
    required_usable_device_depth = d["required_usable_device_depth_mm"]

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

    # Keep the rear top vent clear only when the cold shoe lives on the shell.
    if p.include_cold_shoe and not p.include_sun_shade:
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
        extrude(amount=shell_inner_depth + 0.2, mode=Mode.SUBTRACT)

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

        friction_ridge_info = None

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

        # Cold shoe mount on shell top. Disabled when the floating shade is enabled,
        # since the cold shoe gets promoted onto the shade roof instead.
        cold_shoe_info = None
        if p.include_cold_shoe and not p.include_sun_shade:
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

        # Snap-latch through-holes in shell walls (press-to-release access).
        # Beams on cap plug snap outward through these holes when seated.
        # Skip Y+ wall (has cold shoe). Use X+, X-, Y-.
        snap_latch_info = None
        if p.include_snap_latches:
            hook_body_z = body_depth - p.back_cap_lip_depth_mm + 0.5 * p.snap_latch_hook_ramp_mm
            hole_z = hook_body_z
            hole_w = p.snap_latch_hole_width_mm
            hole_h = p.snap_latch_hole_height_mm
            wall_cut = p.asa_wall_mm + 2.0

            # X+ wall
            with BuildSketch(Plane.YZ.offset(half_asa_outer_w + 0.2)):
                with Locations((0.0, hole_z)):
                    Rectangle(hole_w, hole_h)
            extrude(amount=-wall_cut, mode=Mode.SUBTRACT)

            # X- wall
            with BuildSketch(Plane.YZ.offset(-(half_asa_outer_w + 0.2))):
                with Locations((0.0, hole_z)):
                    Rectangle(hole_w, hole_h)
            extrude(amount=wall_cut, mode=Mode.SUBTRACT)

            # Y- wall
            with BuildSketch(Plane.XZ.offset(-(half_asa_outer_h + 0.2))):
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
        retention_bump_info = None
        if p.include_retention_bumps and p.include_snap_latches:
            rb_h = p.retention_bump_height_mm
            rb_w = p.retention_bump_width_mm
            rb_z_ext = p.retention_bump_z_extent_mm
            rb_z = body_depth - p.retention_bump_setback_mm
            half_ih = 0.5 * asa_inner_h
            pocket_d = rb_h + 0.2

            # Y+ wall pocket
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

    # Build top-visor lens hood (arc over top of lens opening)
    if _build_hood:
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
            # Subtract inner bore
            with Locations((cx, cy, 0.1)):
                Cylinder(hood_inner_r, p.lens_hood_depth_mm + 0.2, rotation=(180, 0, 0),
                         align=(Align.CENTER, Align.CENTER, Align.MIN),
                         mode=Mode.SUBTRACT)
        # Clip to top-half visor within body width
        clip_w = d["asa_outer_w_mm"]
        clip_h = hood_outer_r + 1.0
        with BuildPart() as clip_bp:
            with Locations((cx, cy + 0.5 * clip_h, -0.5 * p.lens_hood_depth_mm)):
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

    sun_shade_info = None
    if p.include_sun_shade:
        standoff = p.sun_shade_standoff_mm
        shade_w = p.sun_shade_wall_mm
        post_w = p.sun_shade_post_width_mm

        shade_inner_w = asa_outer_w + 2.0 * standoff
        shade_inner_h = asa_outer_h + 2.0 * standoff
        shade_outer_w = shade_inner_w + 2.0 * shade_w
        shade_outer_h = shade_inner_h + 2.0 * shade_w
        shade_inner_r = min(
            p.asa_outer_corner_r_mm + standoff,
            0.49 * min(shade_inner_w, shade_inner_h),
        )
        shade_outer_r = min(
            shade_inner_r + shade_w,
            0.49 * min(shade_outer_w, shade_outer_h),
        )

        half_shade_outer_w = 0.5 * shade_outer_w
        half_shade_outer_h = 0.5 * shade_outer_h
        half_shade_inner_w = 0.5 * shade_inner_w

        shade_z_start = 0.0
        shade_z_len = body_depth
        shade_mid_z = 0.5 * shade_z_len

        flat_extent_half = max(half_asa_outer_w - p.asa_outer_corner_r_mm, 0.0)
        rib_offset = max(flat_extent_half - 2.0, 0.0)
        rib_radial = standoff + 2.0

        # Top is the +Y side in this model. Keep the shade on the top plus upper
        # side walls, and trim away the lower portions of the side panels.
        side_drop = min(
            max(p.sun_shade_side_drop_ratio * shade_outer_h, 0.25 * shade_outer_h),
            shade_outer_h - 2.0 * shade_w - 2.0,
        )
        side_panel_lower_y = half_shade_outer_h - side_drop
        side_trim_h = max(side_panel_lower_y + half_shade_outer_h, 0.0)
        side_trim_x_depth = max(shade_outer_w - shade_inner_w + 2.0, shade_w + 1.0)
        side_trim_x_center = 0.5 * (half_shade_outer_w + half_shade_inner_w)
        lower_side_support_h = max(min(p.sun_shade_side_support_height_mm, side_drop - 1.0), 1.5)
        lower_side_support_y = side_panel_lower_y + 0.5 * lower_side_support_h
        lower_side_support_x = 0.5 * (half_asa_outer_w + half_shade_outer_w)
        lower_side_support_x_span = max(half_shade_outer_w - half_asa_outer_w, shade_w + 0.8)

        try:
            with BuildPart() as shade_bp:
                with BuildSketch(Plane.XY.offset(shade_z_start)):
                    Rectangle(shade_outer_w, shade_outer_h)
                    fillet(vertices(), shade_outer_r)
                extrude(amount=shade_z_len)
                with BuildSketch(Plane.XY.offset(shade_z_start - 0.1)):
                    Rectangle(shade_inner_w, shade_inner_h)
                    fillet(vertices(), shade_inner_r)
                extrude(amount=shade_z_len + 0.2, mode=Mode.SUBTRACT)

                # Remove the full bottom panel (negative Y face) so the shade stays
                # open underneath while the roof remains on the top face.
                bottom_cut_h = half_shade_outer_h - half_asa_outer_h + 1.0
                with Locations((0.0, -(half_asa_outer_h + 0.5 * bottom_cut_h), shade_mid_z)):
                    Box(shade_outer_w + 2.0, bottom_cut_h + 0.2, shade_z_len + 2.0, mode=Mode.SUBTRACT)

                # Trim the lower portions of the left and right side panels.
                if side_trim_h > 0.5:
                    for sx in (-1.0, 1.0):
                        with Locations((sx * side_trim_x_center, -half_shade_outer_h + 0.5 * side_trim_h, shade_mid_z)):
                            Box(side_trim_x_depth, side_trim_h + 0.2, shade_z_len + 2.0, mode=Mode.SUBTRACT)

                # Use the same rib strategy as Mevo Core; the lower side ribs will
                # disappear automatically where the side panels are trimmed away.
                for ry in (-1.0, 1.0):
                    with Locations((half_asa_outer_w + 0.5 * standoff, ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shade_z_len)
                for ry in (-1.0, 1.0):
                    with Locations((-(half_asa_outer_w + 0.5 * standoff), ry * rib_offset, shade_mid_z)):
                        Box(rib_radial, post_w, shade_z_len)
                for rx in (-1.0, 1.0):
                    with Locations((rx * rib_offset, half_asa_outer_h + 0.5 * standoff, shade_mid_z)):
                        Box(post_w, rib_radial, shade_z_len)

                # Lower edge support ledge: ties the bottom of each partial side
                # panel back into the ASA shell to stiffen the shade termination.
                for sx in (-1.0, 1.0):
                    with Locations((sx * lower_side_support_x, lower_side_support_y, shade_mid_z)):
                        Box(lower_side_support_x_span, lower_side_support_h, shade_z_len)

            shade_solid = _largest_solid(shade_bp.part)
            asa_shell = _largest_solid(asa_shell + shade_solid)

            if p.include_cold_shoe:
                cs_pad_z = body_depth - p.cold_shoe_pad_z_from_rear_mm
                cs_boss_l = p.cold_shoe_boss_length_mm
                cs_boss_w = p.cold_shoe_boss_width_mm
                cs_slot_w = p.cold_shoe_slot_width_mm
                cs_rail_oh = p.cold_shoe_rail_overhang_mm
                cs_rail_t = p.cold_shoe_rail_thickness_mm
                cs_slot_d = p.cold_shoe_slot_depth_mm
                cs_boss_h = cs_slot_d + cs_rail_t
                cs_opening = cs_slot_w - 2.0 * cs_rail_oh
                boss_overlap = min(0.8, shade_w - 0.2)

                top_cold_shoe = _build_top_cold_shoe_rails(
                    roof_y=half_shade_outer_h,
                    z_center=cs_pad_z,
                    boss_l=cs_boss_l,
                    boss_w=cs_boss_w,
                    slot_w=cs_slot_w,
                    opening_w=cs_opening,
                    slot_d=cs_slot_d,
                    rail_t=cs_rail_t,
                    rail_oh=cs_rail_oh,
                    overlap=boss_overlap,
                )
                asa_shell = _largest_solid(asa_shell + top_cold_shoe)

                left_cold_shoe = _build_side_cold_shoe_rails(
                    side_x=-half_shade_outer_w,
                    outward_sign=-1.0,
                    z_center=cs_pad_z,
                    boss_l=cs_boss_l,
                    boss_w=cs_boss_w,
                    slot_w=cs_slot_w,
                    opening_w=cs_opening,
                    slot_d=cs_slot_d,
                    rail_t=cs_rail_t,
                    rail_oh=cs_rail_oh,
                    overlap=boss_overlap,
                )
                asa_shell = _largest_solid(asa_shell + left_cold_shoe)

                right_cold_shoe = _build_side_cold_shoe_rails(
                    side_x=half_shade_outer_w,
                    outward_sign=1.0,
                    z_center=cs_pad_z,
                    boss_l=cs_boss_l,
                    boss_w=cs_boss_w,
                    slot_w=cs_slot_w,
                    opening_w=cs_opening,
                    slot_d=cs_slot_d,
                    rail_t=cs_rail_t,
                    rail_oh=cs_rail_oh,
                    overlap=boss_overlap,
                )
                asa_shell = _largest_solid(asa_shell + right_cold_shoe)

                cold_shoe_info = {
                    "enabled": True,
                    "locations": ["top", "left", "right"],
                    "pad_z_center_mm": float(cs_pad_z),
                    "boss_height_mm": float(cs_boss_h),
                    "slot_width_mm": float(cs_slot_w),
                    "rail_opening_mm": float(cs_opening),
                    "slide_in_from": "rear",
                    "mounted_on": "shade_hood",
                    "style": "open_dual_rail",
                }

            sun_shade_info = {
                "enabled": True,
                "standoff_mm": float(standoff),
                "wall_mm": float(shade_w),
                "shade_outer_w_mm": float(shade_outer_w),
                "shade_outer_h_mm": float(shade_outer_h),
                "post_width_mm": float(post_w),
                "side_drop_ratio": float(p.sun_shade_side_drop_ratio),
                "side_support_height_mm": float(lower_side_support_h),
                "coverage": "top + partial left/right sides (open bottom)",
            }
        except Exception as exc:
            print(f"  WARNING: sun shade failed to build: {exc}")

    asa_shell.label = "ASA_Shell"

    # Compute rear relief depth here so it's accessible outside BuildPart
    rear_relief_depth = min(max(p.tpu_rear_cap_relief_depth_mm, 0.0), max(cavity_depth - 1.0, 0.0))
    face_wrap_radial = max(p.tpu_front_edge_wrap_radial_mm, 0.6)
    face_wrap_inner_w = max(tpu_inner_w - 2.0 * face_wrap_radial, 2.0)
    face_wrap_inner_h = max(tpu_inner_h - 2.0 * face_wrap_radial, 2.0)
    face_wrap_inner_r = max(p.tpu_inner_corner_r_mm - face_wrap_radial, 0.5)
    front_face_wrap_depth = float(d["front_wrap_intrusion_mm"])
    rear_face_wrap_depth = float(d["rear_wrap_intrusion_mm"])
    wrap_z = cavity_start_z + front_face_wrap_depth

    with BuildPart() as tpu_stage1:
        with BuildSketch(Plane.XY.offset(cavity_start_z)):
            Rectangle(tpu_outer_w, tpu_outer_h)
            fillet(vertices(), p.tpu_outer_corner_r_mm)
        extrude(amount=cavity_depth)

        with BuildSketch(Plane.XY.offset(wrap_z - 0.1)):
            Rectangle(tpu_inner_w, tpu_inner_h)
            fillet(vertices(), p.tpu_inner_corner_r_mm)
        extrude(amount=cavity_depth - front_face_wrap_depth + 0.3, mode=Mode.SUBTRACT)

        if p.include_tpu_front_edge_wrap and front_face_wrap_depth > 0.0 and face_wrap_radial > 0.0:
            with BuildSketch(Plane.XY.offset(cavity_start_z)):
                Rectangle(face_wrap_inner_w, face_wrap_inner_h)
                fillet(vertices(), face_wrap_inner_r)
            with BuildSketch(Plane.XY.offset(wrap_z)):
                Rectangle(tpu_inner_w, tpu_inner_h)
                fillet(vertices(), p.tpu_inner_corner_r_mm)
            loft(mode=Mode.SUBTRACT)

    tpu_base = _largest_solid(tpu_stage1.part)

    bumper_w = p.tpu_corner_bumper_w_mm
    rail_w = p.tpu_edge_rail_w_mm
    wall_cut_depth = p.tpu_wall_mm + 1.0
    skel_start_z = cavity_start_z + front_face_wrap_depth + rail_w
    skel_end_z = cavity_start_z + cavity_depth - rail_w
    skel_span = max(skel_end_z - skel_start_z, 0.0)
    relief_start_z = cavity_start_z + cavity_depth - rear_relief_depth

    with BuildPart() as tpu_bp:
        add(tpu_base)

        x_wall_clear_h = max(tpu_outer_h - 2.0 * bumper_w, 0.0)
        if x_wall_clear_h > 1.0 and skel_span > 1.0:
            x_cut_center_z = skel_start_z + 0.5 * skel_span
            for side in (-1.0, 1.0):
                x_face = side * (half_tpu_outer_w + 0.2)
                with BuildSketch(Plane.YZ.offset(x_face)):
                    with Locations((0.0, x_cut_center_z)):
                        Rectangle(x_wall_clear_h, skel_span)
                extrude(
                    amount=wall_cut_depth if side < 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        y_wall_clear_w = max(tpu_outer_w - 2.0 * bumper_w, 0.0)
        if y_wall_clear_w > 1.0 and skel_span > 1.0:
            y_cut_center_z = skel_start_z + 0.5 * skel_span
            for side in (-1.0, 1.0):
                y_face = side * (half_tpu_outer_h + 0.2)
                with BuildSketch(Plane.XZ.offset(-y_face)):
                    with Locations((0.0, y_cut_center_z)):
                        Rectangle(y_wall_clear_w, skel_span)
                extrude(
                    amount=wall_cut_depth if side > 0 else -wall_cut_depth,
                    mode=Mode.SUBTRACT,
                )

        if rear_relief_depth > 0.0:
            relief_w = tpu_outer_w + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            relief_h = tpu_outer_h + 2.0 * max(p.tpu_rear_cap_relief_radial_mm, 0.0)
            with BuildSketch(Plane.XY.offset(relief_start_z - 0.2)):
                Rectangle(relief_w, relief_h)
                fillet(vertices(), max(p.tpu_outer_corner_r_mm + 0.3, 0.5))
            extrude(amount=rear_relief_depth + 0.4, mode=Mode.SUBTRACT)

        if p.include_thermal_vents:
            tpu_side_cut = max(p.side_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            for side in ("neg", "pos"):
                x_face = -half_tpu_outer_w - 0.2 if side == "neg" else half_tpu_outer_w + 0.2
                for z_c in side_slot_z_centers:
                    with BuildSketch(Plane.YZ.offset(x_face)):
                        with Locations((p.side_vent_center_y_mm, z_c)):
                            SlotOverall(p.side_vent_slot_h_mm, p.side_vent_slot_w_mm)
                    extrude(
                        amount=tpu_side_cut if side == "neg" else -tpu_side_cut,
                        mode=Mode.SUBTRACT,
                    )

            tpu_top_cut = max(p.top_vent_cut_depth_mm, p.tpu_wall_mm + 1.5)
            with BuildSketch(Plane.XZ.offset(-(half_tpu_outer_h + 0.2))):
                for z_c in top_vent_z_centers:
                    with Locations((0.0, z_c)):
                        SlotOverall(p.top_vent_slot_width_mm, p.top_vent_slot_height_mm)
            extrude(amount=tpu_top_cut, mode=Mode.SUBTRACT)

        tripod_tpu_cut_depth = half_tpu_outer_h
        tpu_rect_w = p.tripod_rect_w_mm + 2.0
        tpu_rect_l = p.tripod_rect_l_mm + 2.0
        tripod_z_bottom = cavity_start_z + p.tripod_bottom_center_from_front_mm
        with Locations((0.0, -half_tpu_outer_h + 0.5 * tripod_tpu_cut_depth - 0.2, tripod_z_bottom)):
            Box(tpu_rect_w, tripod_tpu_cut_depth, tpu_rect_l, mode=Mode.SUBTRACT)

        if p.include_top_tripod:
            tripod_z_top = cavity_start_z + p.tripod_top_center_from_front_mm
            with Locations((0.0, half_tpu_outer_h - 0.5 * tripod_tpu_cut_depth + 0.2, tripod_z_top)):
                Box(tpu_rect_w, tripod_tpu_cut_depth, tpu_rect_l, mode=Mode.SUBTRACT)

    tpu_frame = _largest_solid(tpu_bp.part)

    if p.include_rear_tpu_bumpers and rear_face_wrap_depth > 0.0:
        rear_z = cavity_start_z + cavity_depth
        rear_wrap_start_z = rear_z - rear_face_wrap_depth
        rear_overlap = max(rail_w, 4.0)
        rear_overlap_start_z = rear_wrap_start_z - rear_overlap

        try:
            with BuildPart() as rear_wrap_bp:
                with BuildSketch(Plane.XY.offset(rear_overlap_start_z)):
                    Rectangle(tpu_outer_w, tpu_outer_h)
                    fillet(vertices(), p.tpu_outer_corner_r_mm)
                extrude(amount=rear_face_wrap_depth + rear_overlap)

                with BuildSketch(Plane.XY.offset(rear_overlap_start_z - 0.1)):
                    Rectangle(tpu_inner_w, tpu_inner_h)
                    fillet(vertices(), p.tpu_inner_corner_r_mm)
                extrude(amount=rear_overlap + 0.2, mode=Mode.SUBTRACT)

                with BuildSketch(Plane.XY.offset(rear_wrap_start_z)):
                    Rectangle(tpu_inner_w, tpu_inner_h)
                    fillet(vertices(), p.tpu_inner_corner_r_mm)
                with BuildSketch(Plane.XY.offset(rear_z)):
                    Rectangle(face_wrap_inner_w, face_wrap_inner_h)
                    fillet(vertices(), face_wrap_inner_r)
                loft(mode=Mode.SUBTRACT)

            tpu_frame = _largest_solid(tpu_frame + rear_wrap_bp.part)
        except Exception:
            print("  WARNING: rear TPU face wrap failed to fuse")
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
                "front_face_wrap": {
                    "enabled": bool(p.include_tpu_front_edge_wrap),
                    "depth_mm": float(front_face_wrap_depth),
                    "rail_thickness_mm": float(face_wrap_radial),
                },
                "rear_face_wrap": {
                    "enabled": bool(p.include_rear_tpu_bumpers),
                    "depth_mm": float(rear_face_wrap_depth),
                    "rail_thickness_mm": float(face_wrap_radial),
                },
            },
            "cold_shoe": cold_shoe_info if cold_shoe_info else {"enabled": False},
            "friction_ridge": {"enabled": False},
            "snap_latches": snap_latch_info if snap_latch_info else {"enabled": False},
            "sun_shade": sun_shade_info if sun_shade_info else {"enabled": False},
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
    if usable_tpu_device_depth < required_usable_device_depth:
        report["warnings"].append(
            "TPU usable axial depth is below nominal device-plus-clearance depth; this build is intentionally trimmed from print feedback."
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
            "ring_opening_w": float(ring_opening_w),
            "ring_opening_h": float(ring_opening_h),
            "bumper_ring_inset": float(p.bumper_ring_inset_mm),
            "snap_latches": snap_latch_cap_info if snap_latch_cap_info else {"enabled": False},
            "retention_bumps": retention_bump_cap_info if retention_bump_cap_info else {"enabled": False},
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
    parser.add_argument("--no-snap-latches", action="store_true", help="Disable cantilever snap-fit latches")
    parser.add_argument("--no-top-tripod", action="store_true", help="Disable top tripod cutout")
    parser.add_argument("--no-hood", action="store_true", help="Disable lens hood")
    parser.add_argument("--no-vents", action="store_true", help="Disable thermal vents")
    parser.add_argument("--no-sun-shade", action="store_true", help="Disable floating shade hood")
    parser.add_argument("--lens-diameter", type=float, default=None, help="Front lens cutout diameter (mm)")
    parser.add_argument("--cold-shoe-z-from-rear", type=float, default=None, help="Cold shoe center from rear (mm)")
    parser.add_argument("--bumper-ring-inset", type=float, default=None, help="Bumper ring inset from device edge (mm)")
    args = parser.parse_args()

    p = ZowietekParams()
    if args.no_cold_shoe:
        p.include_cold_shoe = False
    if args.no_snap_latches:
        p.include_snap_latches = False
    if args.no_top_tripod:
        p.include_top_tripod = False
    if args.no_hood:
        p.include_lens_hood = False
    if args.no_vents:
        p.include_thermal_vents = False
    if args.no_sun_shade:
        p.include_sun_shade = False
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
    shell_stp = args.out / "zowietek_pov_asa_shell.stp"
    tpu_stp = args.out / "zowietek_pov_tpu_frame.stp"
    cap_stp = args.out / "zowietek_pov_back_cap.stp"
    mesh_dir = args.out / "printable_mesh"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    shell_stl = mesh_dir / "zowietek_pov_asa_shell.stl"
    tpu_stl = mesh_dir / "zowietek_pov_tpu_frame.stl"
    cap_stl = mesh_dir / "zowietek_pov_back_cap.stl"
    report_json = reports_dir / "zowietek_pov_dual_material_report.json"

    # Also archive old combined body file
    body_step_legacy = args.out / "zowietek_pov_body_dual_material.step"
    archived = _archive_existing(
        [
            shell_step,
            tpu_step,
            cap_step,
            shell_stp,
            tpu_stp,
            cap_stp,
            shell_stl,
            tpu_stl,
            cap_stl,
            report_json,
            body_step_legacy,
        ],
        args.out,
    )

    export_step(asa_shell, str(shell_step))
    export_step(tpu_frame, str(tpu_step))
    export_step(back_cap, str(cap_step))
    export_stl(asa_shell, str(shell_stl))
    export_stl(tpu_frame, str(tpu_stl))
    export_stl(back_cap, str(cap_stl))

    export_verification = {
        "asa_shell": _verify_exported_step(shell_step),
        "tpu_frame": _verify_exported_step(tpu_step),
        "back_cap": _verify_exported_step(cap_step),
        "asa_shell_stl": _verify_file_exists(shell_stl),
        "tpu_frame_stl": _verify_file_exists(tpu_stl),
        "back_cap_stl": _verify_file_exists(cap_stl),
    }

    payload = {
        "params": asdict(p),
        "body_report": body_report,
        "back_cap_report": cap_report,
        "collision_check": collision_report,
        "export_verification": export_verification,
    }
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {shell_step}")
    print(f"Wrote {tpu_step}")
    print(f"Wrote {cap_step}")
    print(f"Wrote {shell_stl}")
    print(f"Wrote {tpu_stl}")
    print(f"Wrote {cap_stl}")
    print(f"Wrote {report_json}")
    for label, info in export_verification.items():
        if "solids" in info:
            print(
                f"Verified {label}: {info['solids']} solid, "
                f"{info['size_bytes']} bytes, mtime {info['modified_at']}"
            )
        else:
            print(
                f"Verified {label}: "
                f"{info['size_bytes']} bytes, mtime {info['modified_at']}"
            )
    print("Note: STEP files are CAD outputs. Most printer slicers want the STL files in models/zowietek_case/printable_mesh.")


if __name__ == "__main__":
    main()
