#!/usr/bin/env python3
"""Generate a dual-material Mevo case with a separate ASA back cap.

Outputs:
- models/mevo_case/mevo_start_body_dual_material.step  (two solids in one STEP)
  - TPU_Sleeve
  - ASA_Shell
- models/mevo_case/mevo_start_back_cap_asa.step        (pure ASA)
- models/mevo_case/reports/mevo_start_dual_material_report.json

Design intent:
- Body is a single front-closed bucket with integrated sleeve.
- TPU is fused against ASA with zero interface gap.
- Back cap is a separate ASA-only part with USB-C and power access.
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
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
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
    # Mevo envelope (mm)
    device_length_mm: float = 87.0
    device_height_mm: float = 75.5
    device_width_mm: float = 34.0

    # Requested material/fit stack
    device_clearance_mm: float = 0.15
    tpu_thickness_mm: float = 2.0
    asa_wall_mm: float = 3.0
    interface_gap_mm: float = 0.0
    internal_vertical_fillet_mm: float = 3.0

    # Body architecture
    lens_recess_mm: float = 2.0
    rear_clearance_mm: float = 0.15
    lens_opening_d_mm: float = 38.8

    # Venting
    vent_count: int = 3
    vent_h_mm: float = 2.2
    vent_len_mm: float = 12.0
    vent_pitch_mm: float = 4.8
    vent_z_ratio: float = 0.57

    # Bottom tripod opening (body)
    tripod_hole_d_mm: float = 12.7
    tripod_zone_z_ratio: float = 0.78

    # Back cap (pure ASA)
    back_cap_thickness_mm: float = 3.0
    back_cap_plug_depth_mm: float = 1.8
    back_cap_plug_clearance_mm: float = 0.25
    back_cap_edge_fillet_mm: float = 0.5

    # Back cap cutouts (Mevo rear controls)
    power_y_mm: float = 15.0
    power_w_mm: float = 9.0
    power_h_mm: float = 4.0
    usb_y_mm: float = -2.8
    usb_w_mm: float = 12.8
    usb_h_mm: float = 6.4


def _safe_fillet_radius(width: float, height: float, radius: float) -> float:
    return max(0.3, min(float(radius), 0.5 * min(width, height) - 0.05))


def _add_capsule(width: float, height: float) -> None:
    Rectangle(width, height)
    fillet(vertices(), _safe_fillet_radius(width, height, 0.5 * width - 0.05))


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


def _apply_vertical_fillet(shape, radius: float):
    if radius <= 0:
        return shape, 0.0
    try:
        return fillet(shape.edges().filter_by(Axis.Z), radius), float(radius)
    except Exception:
        return shape, 0.0


def build_dual_material_body(p: DualMaterialParams):
    # Inner TPU cavity around the device.
    tpu_inner_w = p.device_width_mm + 2.0 * p.device_clearance_mm
    tpu_inner_h = p.device_height_mm + 2.0 * p.device_clearance_mm

    # TPU outer profile and matching ASA interface profile.
    tpu_outer_w = tpu_inner_w + 2.0 * p.tpu_thickness_mm
    tpu_outer_h = tpu_inner_h + 2.0 * p.tpu_thickness_mm

    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    cavity_depth = p.device_length_mm + p.rear_clearance_mm
    body_depth = p.lens_recess_mm + cavity_depth

    min_y = -0.5 * asa_outer_h
    max_x = 0.5 * asa_outer_w
    min_x = -0.5 * asa_outer_w

    vent_z = p.lens_recess_mm + cavity_depth * p.vent_z_ratio
    tripod_z = p.lens_recess_mm + cavity_depth * p.tripod_zone_z_ratio

    # ASA shell bucket (front-closed, rear-open).
    with BuildPart() as asa_bp:
        with BuildSketch(Plane.XY):
            _add_capsule(asa_outer_w, asa_outer_h)
        extrude(amount=body_depth)

        with BuildSketch(Plane.XY.offset(p.lens_recess_mm)):
            _add_capsule(asa_inner_w, asa_inner_h)
        extrude(amount=cavity_depth + 0.2, mode=Mode.SUBTRACT)

        # Lens aperture through front with protected recess depth.
        with BuildSketch(Plane.XY.offset(-0.2)):
            Circle(0.5 * p.lens_opening_d_mm)
        extrude(amount=p.lens_recess_mm + 0.5, mode=Mode.SUBTRACT)

        # Side vents (sharp rectangular slots).
        for side in (-1.0, 1.0):
            x = side * (max_x - p.asa_wall_mm * 0.55)
            for i in range(p.vent_count):
                y = (i - (p.vent_count - 1) * 0.5) * p.vent_pitch_mm
                with Locations((x, y, vent_z)):
                    Box(
                        p.asa_wall_mm * 2.4,
                        p.vent_h_mm,
                        p.vent_len_mm,
                        mode=Mode.SUBTRACT,
                    )

        # Bottom centered tripod hole.
        hole_depth = p.asa_wall_mm + 2.0
        with BuildSketch(Plane.XZ.offset(min_y + 0.2)):
            with Locations((0.0, tripod_z)):
                Circle(0.5 * p.tripod_hole_d_mm)
        extrude(amount=-hole_depth, mode=Mode.SUBTRACT)

    asa_shell = _largest_solid(asa_bp.part)
    asa_shell, asa_vert_fillet = _apply_vertical_fillet(asa_shell, p.internal_vertical_fillet_mm)
    asa_shell = _largest_solid(asa_shell)
    asa_shell.label = "ASA_Shell"

    # TPU sleeve body fused inside ASA body only.
    with BuildPart() as tpu_bp:
        with BuildSketch(Plane.XY.offset(p.lens_recess_mm)):
            _add_capsule(tpu_outer_w, tpu_outer_h)
        extrude(amount=cavity_depth)

        with BuildSketch(Plane.XY.offset(p.lens_recess_mm - 0.2)):
            _add_capsule(tpu_inner_w, tpu_inner_h)
        extrude(amount=cavity_depth + 0.4, mode=Mode.SUBTRACT)

    tpu_sleeve = _largest_solid(tpu_bp.part)
    tpu_sleeve, tpu_vert_fillet = _apply_vertical_fillet(tpu_sleeve, p.internal_vertical_fillet_mm)
    tpu_sleeve = _largest_solid(tpu_sleeve)
    tpu_sleeve.label = "TPU_Sleeve"

    body_assembly = Compound(children=[tpu_sleeve, asa_shell], label="Mevo_Body_Assembly")

    # Capsule profiles already include large continuous corner curvature.
    # This reports the effective internal corner radius in the profile itself.
    effective_internal_corner_radius = max(0.5 * tpu_inner_w - 0.05, 0.0)

    report = {
        "derived_mm": {
            "tpu_inner_w": float(tpu_inner_w),
            "tpu_inner_h": float(tpu_inner_h),
            "tpu_outer_w": float(tpu_outer_w),
            "tpu_outer_h": float(tpu_outer_h),
            "asa_inner_w": float(asa_inner_w),
            "asa_inner_h": float(asa_inner_h),
            "asa_outer_w": float(asa_outer_w),
            "asa_outer_h": float(asa_outer_h),
            "cavity_depth": float(cavity_depth),
            "body_depth": float(body_depth),
            "lens_recess": float(p.lens_recess_mm),
            "tripod_hole_d": float(p.tripod_hole_d_mm),
            "interface_gap": float(p.interface_gap_mm),
        },
        "fillets_mm": {
            "requested_internal_vertical": float(p.internal_vertical_fillet_mm),
            "applied_asa_vertical": float(asa_vert_fillet),
            "applied_tpu_vertical": float(tpu_vert_fillet),
            "effective_internal_profile_corner_radius": float(effective_internal_corner_radius),
            "meets_internal_fillet_requirement": bool(
                effective_internal_corner_radius >= p.internal_vertical_fillet_mm
            ),
        },
        "named_bodies": ["TPU_Sleeve", "ASA_Shell"],
    }
    return body_assembly, asa_shell, tpu_sleeve, report


def build_back_cap(p: DualMaterialParams):
    tpu_inner_w = p.device_width_mm + 2.0 * p.device_clearance_mm
    tpu_inner_h = p.device_height_mm + 2.0 * p.device_clearance_mm
    tpu_outer_w = tpu_inner_w + 2.0 * p.tpu_thickness_mm
    tpu_outer_h = tpu_inner_h + 2.0 * p.tpu_thickness_mm
    asa_inner_w = tpu_outer_w + 2.0 * p.interface_gap_mm
    asa_inner_h = tpu_outer_h + 2.0 * p.interface_gap_mm
    asa_outer_w = asa_inner_w + 2.0 * p.asa_wall_mm
    asa_outer_h = asa_inner_h + 2.0 * p.asa_wall_mm

    # Plug inserts into the TPU inner opening so the cap can close the rear.
    plug_w = max(tpu_inner_w - 2.0 * p.back_cap_plug_clearance_mm, 2.0)
    plug_h = max(tpu_inner_h - 2.0 * p.back_cap_plug_clearance_mm, 2.0)

    with BuildPart() as cap_bp:
        with BuildSketch(Plane.XY):
            _add_capsule(asa_outer_w, asa_outer_h)
        extrude(amount=p.back_cap_thickness_mm)

        with BuildSketch(Plane.XY.offset(p.back_cap_thickness_mm)):
            _add_capsule(plug_w, plug_h)
        extrude(amount=p.back_cap_plug_depth_mm)

        cut_depth = p.back_cap_thickness_mm + p.back_cap_plug_depth_mm + 2.0
        with BuildSketch(Plane.XY.offset(-1.0)):
            with Locations((0.0, p.power_y_mm)):
                SlotOverall(p.power_w_mm, p.power_h_mm)
            with Locations((0.0, p.usb_y_mm)):
                SlotOverall(p.usb_w_mm, p.usb_h_mm)
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
            "plate_w": float(asa_outer_w),
            "plate_h": float(asa_outer_h),
            "plug_w": float(plug_w),
            "plug_h": float(plug_h),
            "plug_depth": float(p.back_cap_plug_depth_mm),
            "thickness": float(p.back_cap_thickness_mm),
        },
        "cutouts_mm": {
            "power": {
                "y": float(p.power_y_mm),
                "w": float(p.power_w_mm),
                "h": float(p.power_h_mm),
            },
            "usb": {
                "y": float(p.usb_y_mm),
                "w": float(p.usb_w_mm),
                "h": float(p.usb_h_mm),
            },
        },
    }
    return cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate Mevo dual-material body + ASA back cap")
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument("--clearance", type=float, default=None, help="Device clearance inside TPU (mm)")
    parser.add_argument("--interface-gap", type=float, default=None, help="TPU-to-ASA interface gap (mm)")
    parser.add_argument("--asa-wall", type=float, default=None, help="ASA wall thickness (mm)")
    parser.add_argument("--tpu-thickness", type=float, default=None, help="TPU sleeve thickness (mm)")
    parser.add_argument("--fillet", type=float, default=None, help="Internal vertical fillet target (mm)")
    parser.add_argument("--lens-recess", type=float, default=None, help="Front lens recess depth (mm)")
    parser.add_argument("--tripod-hole-d", type=float, default=None, help="Bottom tripod hole diameter (mm)")
    args = parser.parse_args()

    p = DualMaterialParams()
    if args.clearance is not None:
        p.device_clearance_mm = args.clearance
    if args.interface_gap is not None:
        p.interface_gap_mm = args.interface_gap
    if args.asa_wall is not None:
        p.asa_wall_mm = args.asa_wall
    if args.tpu_thickness is not None:
        p.tpu_thickness_mm = args.tpu_thickness
    if args.fillet is not None:
        p.internal_vertical_fillet_mm = args.fillet
    if args.lens_recess is not None:
        p.lens_recess_mm = args.lens_recess
    if args.tripod_hole_d is not None:
        p.tripod_hole_d_mm = args.tripod_hole_d

    body_assembly, _, _, body_report = build_dual_material_body(p)
    back_cap, cap_report = build_back_cap(p)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    body_step = args.out / "mevo_start_body_dual_material.step"
    cap_step = args.out / "mevo_start_back_cap_asa.step"
    report_json = reports_dir / "mevo_start_dual_material_report.json"
    archived = _archive_existing([body_step, cap_step, report_json], args.out)

    export_step(body_assembly, str(body_step))
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
