#!/usr/bin/env python3
"""Generate smooth B-REP front/rear caps for Mevo Start (ASA and TPU profiles).

Outputs (profile=asa):
- mevo_start_front_cap.step
- mevo_start_rear_cap.step
- reports/mevo_start_caps_report.json

Outputs (profile=tpu):
- mevo_start_tpu_front_cap.step
- mevo_start_tpu_rear_cap.step
- reports/mevo_start_tpu_caps_report.json
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
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
class MevoCapParams:
    profile: str = "asa"

    # Nominal Mevo Start envelope (mm)
    device_length_mm: float = 87.0
    device_height_mm: float = 75.5
    device_width_mm: float = 34.0

    # Sleeve geometry that caps mate to
    sleeve_clearance_mm: float = 2.3
    sleeve_wall_mm: float = 3.0

    # Cap geometry
    cap_thickness_mm: float = 3.0
    plug_depth_mm: float = 1.8
    plug_clearance_mm: float = 0.25
    cap_plate_extra_mm: float = 0.0
    edge_fillet_mm: float = 0.5

    # Front cap lens opening
    front_lens_cutout_enabled: bool = True
    lens_center_x_mm: float = 0.0
    lens_center_y_mm: float = 16.5
    lens_cut_radius_mm: float = 19.4
    lens_cut_extra_mm: float = 0.0

    # Rear cap Mevo I/O cutouts (disabled by default until exact mapping is confirmed)
    rear_io_cutouts_enabled: bool = False
    cutout_extra_mm: float = 0.25
    power_y_mm: float = 15.0
    power_w_mm: float = 9.0
    power_h_mm: float = 4.0
    audio_y_mm: float = 6.8
    audio_d_mm: float = 6.7
    usb_y_mm: float = -2.8
    usb_w_mm: float = 12.8
    usb_h_mm: float = 6.4

    # Bottom tripod extension relief on rear cap
    tripod_notch_enabled: bool = True
    tripod_notch_w_mm: float = 16.0
    tripod_notch_h_mm: float = 9.5
    tripod_notch_center_from_bottom_mm: float = 5.2


def _safe_fillet_radius(width: float, height: float, radius: float) -> float:
    return max(0.3, min(float(radius), 0.5 * min(width, height) - 0.05))


def _add_capsule(width: float, height: float) -> None:
    """Vertical pill profile using a rounded rectangle with r ~= width/2."""
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


def _apply_profile(profile: str, p: MevoCapParams) -> None:
    p.profile = profile
    if profile == "tpu":
        # TPU cap profile is thinner/more compliant than ASA.
        p.sleeve_clearance_mm = 0.20
        p.sleeve_wall_mm = 2.10
        p.cap_thickness_mm = 2.40
        p.plug_depth_mm = 3.40
        p.plug_clearance_mm = 0.30
        p.cap_plate_extra_mm = 0.55
        p.edge_fillet_mm = 0.4
        p.lens_cut_extra_mm = 0.15


def _derive_dims(p: MevoCapParams) -> dict:
    inner_w = p.device_width_mm + 2.0 * p.sleeve_clearance_mm
    inner_h = p.device_height_mm + 2.0 * p.sleeve_clearance_mm
    sleeve_outer_w = inner_w + 2.0 * p.sleeve_wall_mm
    sleeve_outer_h = inner_h + 2.0 * p.sleeve_wall_mm

    plate_w = sleeve_outer_w + 2.0 * p.cap_plate_extra_mm
    plate_h = sleeve_outer_h + 2.0 * p.cap_plate_extra_mm
    plug_w = max(inner_w - 2.0 * p.plug_clearance_mm, 2.0)
    plug_h = max(inner_h - 2.0 * p.plug_clearance_mm, 2.0)

    return {
        "inner_w_mm": inner_w,
        "inner_h_mm": inner_h,
        "sleeve_outer_w_mm": sleeve_outer_w,
        "sleeve_outer_h_mm": sleeve_outer_h,
        "plate_w_mm": plate_w,
        "plate_h_mm": plate_h,
        "plug_w_mm": plug_w,
        "plug_h_mm": plug_h,
    }


def _build_cap(
    plate_w: float,
    plate_h: float,
    plug_w: float,
    plug_h: float,
    cap_t: float,
    plug_d: float,
    cutouts: list[dict],
    edge_fillet_mm: float,
):
    with BuildPart() as cap_bp:
        with BuildSketch(Plane.XY):
            _add_capsule(plate_w, plate_h)
        extrude(amount=cap_t)

        with BuildSketch(Plane.XY.offset(cap_t)):
            _add_capsule(plug_w, plug_h)
        extrude(amount=plug_d)

        if cutouts:
            cut_depth = cap_t + plug_d + 2.0
            with BuildSketch(Plane.XY.offset(-1.0)):
                for c in cutouts:
                    with Locations((c["x"], c["y"])):
                        if c["shape"] == "circle":
                            Circle(c["d"] * 0.5)
                        elif c["shape"] == "slot":
                            SlotOverall(c["w"], c["h"])
                        else:
                            Rectangle(c["w"], c["h"])
            extrude(amount=cut_depth, mode=Mode.SUBTRACT)

    part = cap_bp.part
    try:
        part = fillet(part.edges(), edge_fillet_mm)
    except Exception:
        pass
    return _largest_solid(part)


def build_caps(p: MevoCapParams):
    dims = _derive_dims(p)
    plate_w = dims["plate_w_mm"]
    plate_h = dims["plate_h_mm"]
    plug_w = dims["plug_w_mm"]
    plug_h = dims["plug_h_mm"]

    front_cutouts: list[dict] = []
    if p.front_lens_cutout_enabled:
        front_cutouts.append(
            {
                "shape": "circle",
                "x": p.lens_center_x_mm,
                "y": p.lens_center_y_mm,
                "d": 2.0 * (p.lens_cut_radius_mm + p.lens_cut_extra_mm),
            }
        )

    rear_cutouts: list[dict] = []
    if p.rear_io_cutouts_enabled:
        rear_cutouts.extend(
            [
                {
                    "shape": "slot",
                    "x": 0.0,
                    "y": p.power_y_mm,
                    "w": p.power_w_mm + p.cutout_extra_mm,
                    "h": p.power_h_mm + p.cutout_extra_mm,
                },
                {
                    "shape": "circle",
                    "x": 0.0,
                    "y": p.audio_y_mm,
                    "d": p.audio_d_mm + p.cutout_extra_mm,
                },
                {
                    "shape": "slot",
                    "x": 0.0,
                    "y": p.usb_y_mm,
                    "w": p.usb_w_mm + p.cutout_extra_mm,
                    "h": p.usb_h_mm + p.cutout_extra_mm,
                },
            ]
        )

    if p.tripod_notch_enabled:
        rear_cutouts.append(
            {
                "shape": "rect",
                "x": 0.0,
                "y": -0.5 * plate_h + p.tripod_notch_center_from_bottom_mm,
                "w": p.tripod_notch_w_mm,
                "h": p.tripod_notch_h_mm,
            }
        )

    front_cap = _build_cap(
        plate_w=plate_w,
        plate_h=plate_h,
        plug_w=plug_w,
        plug_h=plug_h,
        cap_t=p.cap_thickness_mm,
        plug_d=p.plug_depth_mm,
        cutouts=front_cutouts,
        edge_fillet_mm=p.edge_fillet_mm,
    )
    rear_cap = _build_cap(
        plate_w=plate_w,
        plate_h=plate_h,
        plug_w=plug_w,
        plug_h=plug_h,
        cap_t=p.cap_thickness_mm,
        plug_d=p.plug_depth_mm,
        cutouts=rear_cutouts,
        edge_fillet_mm=p.edge_fillet_mm,
    )

    report = {
        "profile": p.profile,
        "derived": dims,
        "cutouts": {"front": front_cutouts, "rear": rear_cutouts},
    }
    return front_cap, rear_cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate Mevo Start front/rear caps")
    parser.add_argument(
        "--profile",
        choices=["asa", "tpu"],
        default="asa",
        help="Material profile: asa hard-shell caps or tpu caps",
    )
    parser.add_argument("--out", type=Path, default=Path("models/mevo_case"), help="Output directory")
    parser.add_argument("--length-mm", type=float, default=None, help="Device length (mm)")
    parser.add_argument("--height-mm", type=float, default=None, help="Device height (mm)")
    parser.add_argument("--width-mm", type=float, default=None, help="Device width (mm)")
    parser.add_argument(
        "--rear-io-cutouts",
        action="store_true",
        help="Enable rear power/audio/USB cutouts (off by default for Mevo until measured)",
    )
    parser.add_argument(
        "--disable-front-lens-cutout",
        action="store_true",
        help="Disable front lens opening",
    )
    parser.add_argument("--lens-center-y", type=float, default=None, help="Front lens center Y (mm)")
    parser.add_argument("--lens-radius", type=float, default=None, help="Front lens cut radius (mm)")
    args = parser.parse_args()

    params = MevoCapParams()
    _apply_profile(args.profile, params)

    if args.length_mm is not None:
        params.device_length_mm = args.length_mm
    if args.height_mm is not None:
        params.device_height_mm = args.height_mm
    if args.width_mm is not None:
        params.device_width_mm = args.width_mm
    if args.rear_io_cutouts:
        params.rear_io_cutouts_enabled = True
    if args.disable_front_lens_cutout:
        params.front_lens_cutout_enabled = False
    if args.lens_center_y is not None:
        params.lens_center_y_mm = args.lens_center_y
    if args.lens_radius is not None:
        params.lens_cut_radius_mm = args.lens_radius

    front_cap, rear_cap, report = build_caps(params)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    if args.profile == "asa":
        front_step = args.out / "mevo_start_front_cap.step"
        rear_step = args.out / "mevo_start_rear_cap.step"
        report_json = reports_dir / "mevo_start_caps_report.json"
        legacy_report_json = args.out / "mevo_start_caps_report.json"
    else:
        front_step = args.out / "mevo_start_tpu_front_cap.step"
        rear_step = args.out / "mevo_start_tpu_rear_cap.step"
        report_json = reports_dir / "mevo_start_tpu_caps_report.json"
        legacy_report_json = args.out / "mevo_start_tpu_caps_report.json"
    archived = _archive_existing([front_step, rear_step, report_json, legacy_report_json], args.out)

    export_step(front_cap, str(front_step))
    export_step(rear_cap, str(rear_step))

    payload = {"params": asdict(params), "report": report}
    report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {front_step}")
    print(f"Wrote {rear_step}")
    print(f"Wrote {report_json}")


if __name__ == "__main__":
    main()
