#!/usr/bin/env python3
"""Generate MAKI Live rear-cap dual-material assembly (ASA cap + TPU inner gasket).

Outputs:
- models/maki_case/maki_live_rear_cap_dual_material.step
- models/maki_case/maki_live_rear_cap.step
- models/maki_case/reports/maki_live_rear_cap_dual_material_report.json
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
    Compound,
    Locations,
    Mode,
    Plane,
    Rectangle,
    export_step,
    extrude,
    fillet,
)
from generate_maki_live_caps import MakiCapParams, build_caps, _add_rounded_rectangle


@dataclass
class MakiRearCapDualParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")

    # Keep cap generation aligned with current ASA sleeve defaults.
    cutout_extra_mm: float = 1.5
    cap_thickness_mm: float = 3.0
    plug_depth_mm: float = 1.8
    plug_clearance_mm: float = 0.28

    # TPU gasket on the cap inner side (device-contact side).
    include_tpu_gasket: bool = True
    tpu_gasket_thickness_mm: float = 1.2
    tpu_gasket_outer_inset_mm: float = 0.0
    tpu_edge_wrap_depth_mm: float = 1.8
    tpu_edge_wrap_width_mm: float = 1.8


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


def _largest_solid(shape):
    solids = shape.solids() if hasattr(shape, "solids") else []
    if len(solids) <= 1:
        return shape
    return max(solids, key=lambda s: s.volume)


def _build_tpu_gasket(
    plug_w: float,
    plug_h: float,
    plug_r: float,
    cap_thickness: float,
    plug_depth: float,
    cutouts: list[dict],
    p: MakiRearCapDualParams,
):
    if (not p.include_tpu_gasket) or p.tpu_gasket_thickness_mm <= 0.0:
        return None

    # Place gasket at plug tip so it actually contacts camera-side back edges.
    z0 = cap_thickness + plug_depth - p.tpu_gasket_thickness_mm
    gasket_outer_w = max(plug_w - 2.0 * p.tpu_gasket_outer_inset_mm, 2.0)
    gasket_outer_h = max(plug_h - 2.0 * p.tpu_gasket_outer_inset_mm, 2.0)
    gasket_outer_r = max(plug_r - p.tpu_gasket_outer_inset_mm, 0.6)
    lip_depth = max(min(p.tpu_edge_wrap_depth_mm, plug_depth), 0.0)
    lip_w = max(p.tpu_edge_wrap_width_mm, 0.0)
    gasket_inner_w = max(gasket_outer_w - 2.0 * lip_w, 1.0)
    gasket_inner_h = max(gasket_outer_h - 2.0 * lip_w, 1.0)
    gasket_inner_r = max(gasket_outer_r - lip_w, 0.4)
    z_tip = cap_thickness + plug_depth

    with BuildPart() as gasket_bp:
        # Edge-wrap collar near the plug tip to protect camera back-edge corners.
        if lip_depth > 0.0 and lip_w > 0.0:
            with BuildSketch(Plane.XY.offset(z_tip - lip_depth)):
                _add_rounded_rectangle(gasket_outer_w, gasket_outer_h, gasket_outer_r)
            extrude(amount=lip_depth)

            with BuildSketch(Plane.XY.offset(z_tip - lip_depth - 0.1)):
                _add_rounded_rectangle(gasket_inner_w, gasket_inner_h, gasket_inner_r)
            extrude(amount=lip_depth + 0.2, mode=Mode.SUBTRACT)

        # Full face pad so camera back never touches ASA.
        with BuildSketch(Plane.XY.offset(z0)):
            _add_rounded_rectangle(gasket_outer_w, gasket_outer_h, gasket_outer_r)
        extrude(amount=p.tpu_gasket_thickness_mm)

        if cutouts:
            cut_z0 = z_tip - max(lip_depth, p.tpu_gasket_thickness_mm) - 0.1
            cut_h = max(lip_depth, p.tpu_gasket_thickness_mm) + 0.2
            with BuildSketch(Plane.XY.offset(cut_z0)):
                for c in cutouts:
                    with Locations((c["x"], c["y"])):
                        if c["shape"] == "circle":
                            Circle(c["d"] * 0.5)
                        else:
                            Rectangle(c["w"], c["h"])
            extrude(amount=cut_h, mode=Mode.SUBTRACT)

    gasket = _largest_solid(gasket_bp.part)
    try:
        gasket = fillet(gasket.edges(), 0.4)
    except Exception:
        pass
    gasket = _largest_solid(gasket)
    gasket.label = "TPU_Back_Gasket"
    return gasket


def build_rear_cap_dual(p: MakiRearCapDualParams):
    caps_p = MakiCapParams(step_path=p.step_path)
    caps_p.cutout_extra_mm = p.cutout_extra_mm
    caps_p.cap_thickness_mm = p.cap_thickness_mm
    caps_p.plug_depth_mm = p.plug_depth_mm
    caps_p.plug_clearance_mm = p.plug_clearance_mm

    _, rear_cap, caps_report = build_caps(caps_p)
    rear_cap = _largest_solid(rear_cap)
    rear_cap.label = "ASA_Back_Cap"

    derived = caps_report["derived"]
    corner_r = derived["corner_radius_mm"]
    rear_cutouts = caps_report["cutouts"]["rear"]

    gasket = _build_tpu_gasket(
        plug_w=float(derived["plug_w_mm"]),
        plug_h=float(derived["plug_h_mm"]),
        plug_r=float(corner_r["plug"]),
        cap_thickness=float(caps_p.cap_thickness_mm),
        plug_depth=float(caps_p.plug_depth_mm),
        cutouts=rear_cutouts,
        p=p,
    )

    if gasket is not None:
        assembly = Compound(children=[gasket, rear_cap], label="Maki_Rear_Cap_Assembly")
        named_bodies = ["ASA_Back_Cap", "TPU_Back_Gasket"]
    else:
        assembly = rear_cap
        named_bodies = ["ASA_Back_Cap"]

    report = {
        "named_bodies": named_bodies,
        "rear_cap_source": caps_report,
        "tpu_gasket_mm": {
            "enabled": bool(gasket is not None),
            "thickness": float(p.tpu_gasket_thickness_mm),
            "outer_inset": float(p.tpu_gasket_outer_inset_mm),
            "edge_wrap_depth": float(p.tpu_edge_wrap_depth_mm),
            "edge_wrap_width": float(p.tpu_edge_wrap_width_mm),
            "contact_plane_z": float(caps_p.cap_thickness_mm + caps_p.plug_depth_mm),
        },
    }
    return assembly, rear_cap, report


def main():
    parser = argparse.ArgumentParser(description="Generate MAKI rear-cap dual-material assembly")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--cutout-extra", type=float, default=None, help="Extra clearance around extracted rear cutouts (mm)")
    parser.add_argument("--cap-thickness", type=float, default=None, help="ASA rear cap plate thickness (mm)")
    parser.add_argument("--plug-depth", type=float, default=None, help="ASA rear cap plug depth (mm)")
    parser.add_argument("--plug-clearance", type=float, default=None, help="ASA rear cap plug radial clearance (mm)")
    parser.add_argument("--disable-tpu-gasket", action="store_true", help="Disable TPU gasket body on rear cap")
    parser.add_argument("--tpu-gasket-thickness", type=float, default=None, help="TPU gasket thickness on cap interior (mm)")
    parser.add_argument("--tpu-gasket-inset", type=float, default=None, help="TPU gasket outer inset from cap OD (mm)")
    parser.add_argument("--tpu-gasket-edge-wrap-depth", type=float, default=None, help="TPU corner edge-wrap depth from tip plane (mm)")
    parser.add_argument("--tpu-gasket-edge-wrap-width", type=float, default=None, help="TPU edge-wrap width around perimeter (mm)")
    args = parser.parse_args()

    p = MakiRearCapDualParams(step_path=args.step)
    if args.cutout_extra is not None:
        p.cutout_extra_mm = float(args.cutout_extra)
    if args.cap_thickness is not None:
        p.cap_thickness_mm = float(args.cap_thickness)
    if args.plug_depth is not None:
        p.plug_depth_mm = float(args.plug_depth)
    if args.plug_clearance is not None:
        p.plug_clearance_mm = float(args.plug_clearance)
    if args.disable_tpu_gasket:
        p.include_tpu_gasket = False
    if args.tpu_gasket_thickness is not None:
        p.tpu_gasket_thickness_mm = max(float(args.tpu_gasket_thickness), 0.0)
    if args.tpu_gasket_inset is not None:
        p.tpu_gasket_outer_inset_mm = max(float(args.tpu_gasket_inset), 0.0)
    if args.tpu_gasket_edge_wrap_depth is not None:
        p.tpu_edge_wrap_depth_mm = max(float(args.tpu_gasket_edge_wrap_depth), 0.0)
    if args.tpu_gasket_edge_wrap_width is not None:
        p.tpu_edge_wrap_width_mm = max(float(args.tpu_gasket_edge_wrap_width), 0.1)

    args.out.mkdir(parents=True, exist_ok=True)
    reports_dir = args.out / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    out_dual = args.out / "maki_live_rear_cap_dual_material.step"
    out_asa = args.out / "maki_live_rear_cap.step"
    out_json = reports_dir / "maki_live_rear_cap_dual_material_report.json"
    archived = _archive_existing([out_dual, out_asa, out_json], args.out)

    assembly, asa_cap, report = build_rear_cap_dual(p)
    export_step(assembly, str(out_dual))
    export_step(asa_cap, str(out_asa))

    payload = {"params": asdict(p), "report": report}
    payload["params"]["step_path"] = str(p.step_path)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_dual}")
    print(f"Wrote {out_asa}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
