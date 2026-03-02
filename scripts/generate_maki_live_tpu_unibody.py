#!/usr/bin/env python3
"""Generate one-piece TPU unibody for MAKI Live (liner + front cap + rear cap fused).

Outputs:
- models/maki_case/maki_live_tpu_unibody.step
- models/maki_case/maki_live_tpu_unibody_report.json

By default, legacy separate TPU files are archived so only the unibody TPU part
remains visible in models/maki_case top-level.
"""

from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from build123d import Location, Plane, export_step
from generate_maki_live_caps import MakiCapParams, build_caps
from generate_maki_live_tpu_liner import MakiTpuLinerParams, build_liner


@dataclass
class MakiTpuUnibodyParams:
    step_path: Path = Path("refs/BirdDog_MAKI-Live_3D-file.step")
    out_dir: Path = Path("models/maki_case")

    # Boolean fusion overlap into the liner shell at each end.
    front_overlap_mm: float = 0.10
    rear_overlap_mm: float = 0.10

    # Liner tunables
    device_clearance_mm: float = 0.2
    shell_thickness_mm: float = 2.1
    bumper_extra_mm: float = 0.55
    bumper_band_depth_mm: float = 12.0
    end_clearance_mm: float = 0.2
    side_feature_clearance_mm: float = 0.35

    # Cap/cutout tunables
    cutout_extra_mm: float = 0.25
    front_recess_depth_mm: float = 1.2
    front_recess_inset_mm: float = 3.0

    # Folder hygiene
    archive_legacy_separate_tpu_parts: bool = True


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


def build_unibody(p: MakiTpuUnibodyParams):
    liner_p = MakiTpuLinerParams(step_path=p.step_path)
    liner_p.device_clearance_mm = p.device_clearance_mm
    liner_p.shell_thickness_mm = p.shell_thickness_mm
    liner_p.bumper_extra_mm = p.bumper_extra_mm
    liner_p.bumper_band_depth_mm = p.bumper_band_depth_mm
    liner_p.end_clearance_mm = p.end_clearance_mm
    liner_p.side_feature_clearance_mm = p.side_feature_clearance_mm
    liner, liner_report = build_liner(liner_p)

    cap_p = MakiCapParams(step_path=p.step_path)
    # TPU cap defaults from generate_maki_live_caps --profile tpu.
    cap_p.sleeve_clearance_mm = 0.2
    cap_p.sleeve_wall_mm = 2.1
    cap_p.cap_plate_extra_mm = 0.55
    cap_p.cap_thickness_mm = 2.4
    cap_p.plug_depth_mm = 3.4
    cap_p.plug_clearance_mm = 0.30
    cap_p.cutout_extra_mm = p.cutout_extra_mm
    cap_p.front_recess_depth_mm = p.front_recess_depth_mm
    cap_p.front_recess_inset_mm = p.front_recess_inset_mm
    front_cap, rear_cap, caps_report = build_caps(cap_p)

    depth = float(liner_report["derived"]["shell_depth_mm"])
    cap_t = float(cap_p.cap_thickness_mm)

    # Caps from build_caps are centered at XY origin.
    # Mirror over XY so front cap has plug into +Z (inside liner at z=0) and
    # rear cap can be positioned so plug points into -Z at the rear end.
    front_aligned = front_cap.mirror(Plane.XY).move(Location((0.0, 0.0, p.front_overlap_mm)))
    rear_aligned = rear_cap.mirror(Plane.XY).move(
        Location((0.0, 0.0, depth + cap_t - p.rear_overlap_mm))
    )

    unibody = liner.fuse(front_aligned).fuse(rear_aligned)
    unibody = _largest_solid(unibody)

    report = {
        "derived": {
            "shell_depth_mm": depth,
            "cap_thickness_mm": cap_t,
            "fusion_overlap_mm": {
                "front": float(p.front_overlap_mm),
                "rear": float(p.rear_overlap_mm),
            },
        },
        "sources": {
            "liner_report": liner_report,
            "caps_report_tpu_profile": caps_report,
        },
    }
    return unibody, report


def main():
    parser = argparse.ArgumentParser(description="Generate one-piece MAKI Live TPU unibody")
    parser.add_argument("--out", type=Path, default=Path("models/maki_case"), help="Output directory")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("refs/BirdDog_MAKI-Live_3D-file.step"),
        help="STEP model path",
    )
    parser.add_argument("--clearance", type=float, default=None, help="TPU inner clearance to camera body (mm)")
    parser.add_argument("--thickness", type=float, default=None, help="TPU shell thickness (mm)")
    parser.add_argument("--bumper-extra", type=float, default=None, help="Extra TPU bumper radial thickness (mm)")
    parser.add_argument("--band-depth", type=float, default=None, help="TPU bumper band depth from each end (mm)")
    parser.add_argument("--end-clearance", type=float, default=None, help="TPU end clearance at each end (mm)")
    parser.add_argument("--cutout-extra", type=float, default=None, help="Extra clearance around extracted end cutouts (mm)")
    parser.add_argument("--front-recess-depth", type=float, default=None, help="Front bezel recess depth (mm)")
    parser.add_argument("--front-recess-inset", type=float, default=None, help="Front bezel recess inset from edge (mm)")
    parser.add_argument("--front-overlap", type=float, default=None, help="Front cap fusion overlap into liner (mm)")
    parser.add_argument("--rear-overlap", type=float, default=None, help="Rear cap fusion overlap into liner (mm)")
    parser.add_argument(
        "--keep-legacy-separate-tpu-files",
        action="store_true",
        help="Do not archive legacy separate TPU liner/cap outputs",
    )
    args = parser.parse_args()

    params = MakiTpuUnibodyParams(step_path=args.step, out_dir=args.out)
    if args.clearance is not None:
        params.device_clearance_mm = args.clearance
    if args.thickness is not None:
        params.shell_thickness_mm = args.thickness
    if args.bumper_extra is not None:
        params.bumper_extra_mm = args.bumper_extra
    if args.band_depth is not None:
        params.bumper_band_depth_mm = args.band_depth
    if args.end_clearance is not None:
        params.end_clearance_mm = args.end_clearance
    if args.cutout_extra is not None:
        params.cutout_extra_mm = args.cutout_extra
    if args.front_recess_depth is not None:
        params.front_recess_depth_mm = args.front_recess_depth
    if args.front_recess_inset is not None:
        params.front_recess_inset_mm = args.front_recess_inset
    if args.front_overlap is not None:
        params.front_overlap_mm = args.front_overlap
    if args.rear_overlap is not None:
        params.rear_overlap_mm = args.rear_overlap
    if args.keep_legacy_separate_tpu_files:
        params.archive_legacy_separate_tpu_parts = False

    args.out.mkdir(parents=True, exist_ok=True)

    out_step = args.out / "maki_live_tpu_unibody.step"
    out_json = args.out / "maki_live_tpu_unibody_report.json"

    to_archive = [out_step, out_json]
    if params.archive_legacy_separate_tpu_parts:
        to_archive.extend(
            [
                args.out / "maki_live_tpu_liner.step",
                args.out / "maki_live_tpu_liner_report.json",
                args.out / "maki_live_tpu_front_cap.step",
                args.out / "maki_live_tpu_rear_cap.step",
                args.out / "maki_live_tpu_caps_report.json",
            ]
        )
    archived = _archive_existing(to_archive, args.out)

    unibody, report = build_unibody(params)
    export_step(unibody, str(out_step))

    payload = {"params": asdict(params), "report": report}
    payload["params"]["step_path"] = str(params.step_path)
    payload["params"]["out_dir"] = str(params.out_dir)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if archived:
        print(f"Archived {len(archived)} previous file(s) to {args.out / 'archive'}")
    print(f"Wrote {out_step}")
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
