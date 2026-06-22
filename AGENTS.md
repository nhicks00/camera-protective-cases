# Agent Context: Camera Protective Cases (Mevo Start + Mevo Core + BirdDog MAKI Live + Zowietek 4K POV)

This file is the persistent handoff context for future sessions.

## Scope
This repo builds parametric, 3D-printable protective case components for:
- Mevo Start
- Mevo Core
- BirdDog MAKI Live
- Zowietek 4K NDI POV Zoom Camera

Primary goals:
- Tight fit
- Outdoor durability
- Impact resistance
- Clean, smooth CAD geometry (not faceted mesh-style approximations)

## Core Modeling Rules
- Use `build123d` (OpenCascade B-REP).
- Export STEP files for deliverables.
- Keep curves smooth; avoid tiny rectangular facet chains for curved surfaces.
- Keep generated outputs organized by camera and archive prior versions.

## Confirmed Dimension Baseline
Use these as the current default stack unless user explicitly overrides.

Mevo Start:
- Device envelope reference: `87.0 x 67.56 x 34.0 mm` (L x H x W) — height corrected (tripod base excluded)
- TPU inner cavity (active fit): `34.3 x 67.86 x 87.3 mm` (W x H x depth)
- TPU wall: `1.8 mm`
- TPU inner vertical corner fillet: `4.0 mm`
- TPU-to-ASA interface gap: `0.0 mm` (coincident interface)
- ASA shell wall: `2.2 mm`
- ASA outer vertical corner fillet: `6.0 mm`
- Front bucket/sun hood depth: `3.0 mm`
- Front lens cutout: `42.7 mm` (enlarged +12.7 mm / 0.5")
- Front tally LED hole: `3.0 mm`, centered `12.0 mm` above lens center
- Bottom tripod cutout: `31.75 x 50.8 mm` rectangular, center `50.7875 mm` from front face
- ASA back-cap plug lip: `5.0 mm` depth, `0.1 mm` total undersize
- Back utility slot: `15.0 mm` wide, top margin `15.0 mm`, bottom margin `10.0 mm`

BirdDog MAKI Live:
- Device envelope: `120.32 x 56.99 x 56.99 mm` (L x W x H)
- ASA shell wall: `3.0 mm`
- ASA radial clearance to device: `2.3 mm`
- TPU sleeve wall: `2.0 mm`
- TPU radial device clearance: `0.15 mm`
- TPU-to-ASA radial air gap: `0.15 mm` per side (current dual-body default stack)
- ASA cap plug depth: `1.8 mm`
- TPU front face pad thickness: `1.2 mm` (default enabled, STEP-derived cutouts for lens/LED)
- Tripod rect Z shift: `-6.35 mm` (1/4" toward front/lens)

Mevo Core:
- Device envelope: `90.0 x 90.0 x 69.85 mm` (W x H x L), where L = 2.75"
- ASA-only hard cutover: no TPU frame/liner output for Mevo Core
- ASA direct device clearance: `0.6 mm` per side/end
- ASA wall: `3.3 mm`
- ASA outer corner fillet: `24.0 mm`, inner: `22.0 mm`
- Front sun hood depth: `3.0 mm`
- Front lens cutout: `76.2 mm` diameter (3.0"), centered on front face
- Lens hood: full circular tube, `101.6 mm` depth (4.0"), `2.5 mm` wall, `2.5 mm` clearance, with rounded side access notches (`44.45 mm` deep, `50.8 mm` tall, `3.0 mm` corner radius) cut from outside the flared root to avoid access-window slivers; notch cut edges are additionally rounded `0.75 mm` at the terminal perimeter and `0.3 mm` at the root-side small edges
- Bottom tripod cutout: `63.5 x 50.8 mm` rectangular, center `34.925 mm` from front
- Cold shoe mount: ISO 518 on top rear
- Back cap: ASA bumper ring with one large open-center rounded rectangle:
  - Center opening: `74.0 x 74.0 mm`, `8.0 mm` inset from device edge per side
  - Center opening corner radius: `8.0 mm`
- Back cap plug lip: `5.0 mm` depth, `0.28 mm` total undersize
- Retention: 3 snap-latch walls plus Y+ retention bump
- Thermal cutouts: large rectangular panel notches replacing individual slots:
  - one notch per side face spanning the former 6-row side vent bank,
  - one top notch spanning the former 4-row top vent bank, enlarged and Z-aligned to match the side notches,
  - cutout width expands across the flat face while the shade support ribs are pushed into the curved-corner bands with `1.0 mm` clearance from the notch edges.

Zowietek 4K NDI POV Zoom Camera:
- Device envelope: `68.6 x 60.2 x 51.0 mm` (L x W x H), black aluminum, 225g
- TPU clearance: `0.15 mm` per side → inner cavity `60.5 W x 51.3 H x 68.9 L`
- TPU wall: `1.8 mm` (skeleton frame: corner bumpers + edge rails, not solid sleeve)
- TPU corner bumper width: `12.0 mm`, edge rail width: `4.0 mm`
- Interface gap: `0.0 mm` (coincident bond)
- ASA wall: `2.2 mm`
- ASA outer corner fillet: `4.0 mm`, inner: `2.0 mm`
- Front sun hood depth: `3.0 mm`
- Front lens cutout: `37.7 mm` diameter (enlarged +12.7 mm / 0.5"), centered
- Front tally LED: `3.0 mm`, centered `12.0 mm` above lens center
- Lens hood: circular tube around lens opening, `14.0 mm` depth, `2.5 mm` wall, `1.0 mm` clearance
- Tripod mounts: 2x 1/4"-20 UNC (bottom + top), rectangular cutouts `31.75 x 25.4 mm`
- Tripod center from front: `34.3 mm` (both top and bottom)
- Cold shoe mount: ISO 518 on top rear
- Back cap: ASA bumper ring (open center), `3.0 mm` plate + `5.0 mm` plug lip
- Bumper ring inset: `3.0 mm` from device edge per side → opening `54.2 x 45.0 mm`
- Thermal vents: 5 side slots per side + 4 top slots (3 placed after cold shoe filter)
- Retention: 4-point latch pockets/bumps

Slicer baseline (ASA):
- 4 wall loops/perimeters minimum.

## Output Organization Policy
- Mevo Core outputs **2 separate STEP files**: ASA shell and back cap. It is ASA-only; do not regenerate a TPU frame for this case.
- Other active dual-material workflows still output separate ASA shell, TPU frame, and back cap unless explicitly changed.
- Current output directories:
  - `models/mevo_case/` + `reports/` + `archive/`
  - `models/mevo_core_case/` + `reports/` + `archive/`
  - `models/maki_case/` + `reports/` + `archive/`
- Zowietek model outputs were removed because that camera is no longer used.
- Top-level in each case folder should show only latest STEP outputs for that case.
- JSON reports should live in each case's `reports/` subfolder.

## Current Project State

### MAKI Live (BirdDog)
Active hard-shell and TPU workflows:
- ASA sleeve:
  - `models/maki_case/maki_live_case_sleeve.step`
  - Generator: `scripts/generate_maki_live_case.py`
  - Front is integrated into sleeve by default (`front_integrated=True`).
  - Front cutouts are extracted from STEP and cut through the integrated front wall.
  - Back remains open for separate rear cap installation.
  - Tripod detection is STEP-interrogation-first:
    - cylindrical face detection (thread-radius target near 1/4"-20 geometry),
    - downward axis normal filter,
    - centerline/depth region filters,
    - circular-edge fallback only if cylindrical detection fails.
  - Vent behavior uses a large separate tripod-side panel cutout:
    - the former 24-slot, 3-panel vent bank is now one rectangular ASA cutout,
    - this vent-panel cutout is separate from the existing bottom/tripod rectangle, with a `2.0 mm` shell bridge between them,
    - current vent-panel cutout is centered on the tripod side, aligned to the tripod rectangle width, with `1.0 mm` vent margin,
    - logical vent coordinates are still emitted for validation; the physical vent-panel cutout is emitted under `step_side_features.merged_vent_cutouts_applied_entries`,
    - the other 3 ASA shell sides each get one enlarged rectangular surface-panel cutout for heat venting, expanded toward the preserved corner/support bands instead of being limited to the original flat-face span,
    - sun-shade support ribs are moved into those preserved curved-corner bands with `1.0 mm` clearance from the large surface-panel openings, so the openings can maximize heat vent area without exposing the support ribs,
    - sun-shade support ribs are intentionally lightened to `2.5 mm`; the top has 2 diagonal 45-degree corner ribs that overlap the case corner and terminate into the shade inner wall so they do not protrude through the hood, and the side shade-to-case connectors are reduced to 2 ribs total (`Y=+13.96 mm`, left and right sides),
    - Maki shade connector bars are inset `2.0 mm` from each body end (`2.0` to `131.92 mm`) and do not extend into the front/rear shade overhang zones; they are directly fused into the final ASA sleeve body after shade union so they cannot disappear as loose shade sub-geometry or show as protruding end tabs under the overhang,
    - standalone side-trio vent slots are removed from the active ASA shell; only the merged tripod-side panel and the 3 large side/top surface panels remain for ASA heat venting.
  - The large former 24-vent bank is constrained to bottom-connected panels (not direct side walls).
  - Side 3+3 vent derivation remains available in the generator for legacy/fallback use, but it is disabled by default and not part of the active ASA sleeve output.
  - If TPU side vent cutouts are explicitly enabled, the TPU generator uses the same merged tripod-side panel behavior.
  - Vent-bank/tripod side relationship is explicit: merged panel and tripod opening remain on the same expected side (`tripod_expected_side`), with hood on the opposite side.
  - Sunshade side skirt lower corners are rounded by the side-trim profile plus direct lower-edge fillets on the actual skirt edges:
    - outer lower skirt edges use up to `2.0 mm`,
    - inner lower skirt edges use up to `0.75 mm`.
  - Sun shade cover extends `6.35 mm` (`0.25 in`) past both the front and rear shell ends; support ribs are inset `2.0 mm` from each body end and not into either overhang, and each shade end has a low-angle cut-back lofted `1.5 mm` outward shade-perimeter extension that shrinks `0.75 mm` inward toward the case while using the same rounded side-skirt lower trim as the shade cover and only a `0.15 mm` fuse overlap at the shade end.
  - Front camera aperture in the integrated ASA front wall and TPU front face pad uses a single circular `50.8 mm` cutout by default.
  - Through-cut depth increased so vents fully penetrate.
  - Vent row clustering now locks to the STEP-derived rear vent bank (8 rows) and ignores front outlier slots.
  - Tripod armor boss has been removed (params zeroed out).
  - Tripod hole cut orientation is corrected and validated as a true through-cut.
  - Dual-body tripod overlap is now numerically aligned (`dx=0.0`, `dz=0.0` in current reports).
  - Side `3 + 3` vents are rounded-slot cuts (not hard rectangular box cuts).
  - Circular lens hood (tube around lens opening only, not full-face-width crescent); controlled by `lens_hood_*` params. Depth `16 mm`, wall `2.5 mm`, clearance `1.0 mm`.
  - Cold shoe mount (ISO 518 female receptor) on top rear of sleeve:
    - Raised boss on Y+ (top/hood) side, centered 20 mm from rear edge by default.
    - T-slot channel: 18.2 mm floor width, 12.5 mm rail opening, 2.85 mm rail overhang each side.
    - Foot slides in from rear (open at shell_depth end, closed at front).
    - Controlled by `cold_shoe_*` params; disable with `--no-cold-shoe`.
  - Snap-latch flexure clips (2x on inner X walls) for rear cap retention:
    - Cantilever beams: 7×5×1.5 mm, 1.0 mm catch nub at tip.
    - Matching 1.0 mm ridge on rear cap plug for snap engagement.
    - Controlled by `snap_clip_*` params; disable with `--no-snap-clips` / `--no-snap-ridge`.
  - Continuous friction ridge around inner wall perimeter (4 box strips, 0.5 mm inward protrusion, 1.0 mm wide band):
    - Provides distributed holding force for cap retention alongside snap clips.
    - Matching friction ridge on rear cap ASA plug outer surface.
    - Controlled by `friction_ridge_*` params; disable with `--no-friction-ridge`.
  - Vent pass-through validates the active 24 tripanel logical positions; merged physical ASA cutout count is separately reported.
- ASA caps:
  - Active outputs:
    - `models/maki_case/maki_live_back_cap.step`
  - Generator: `scripts/generate_maki_live_caps.py`
  - Rear cap output is the active ASA back cap.
  - Legacy dual rear-cap generator remains available as `scripts/generate_maki_live_rear_cap_dual_material.py`.
  - Rear cutouts are extracted from all STEP solids with tiny-hole filtering to preserve port access cutouts over corner fastener holes.
  - Rear cap port cutouts include default oversize clearance (`cutout_extra_mm=1.5`) for cable boot/plastic strain-relief fit.
  - Snap ridge on ASA plug (1.0 mm height, matching body clips); disable with `--no-snap-ridge`.
  - Continuous friction ridge ring on ASA plug (0.5 mm protrusion, 1.0 mm wide); disable with `--no-friction-ridge`.
- TPU skeleton frame (preferred TPU output):
  - `models/maki_case/maki_live_tpu_sleeve.step`
  - Generator: `scripts/generate_maki_live_tpu_liner.py`
  - Skeleton TPU frame: corner bumpers (12 mm) at each vertical edge connected by edge rails (4 mm) along horizontal edges; wall windows cut through flat sections between bumper/rail zones.
  - Front edge wrap disabled (ASA front wall provides retention); rear edge wrap disabled by default.
  - TPU front face pad is enabled by default (`include_front_face_pad=True`, `1.2 mm` thick):
    - Full-face pad at z=0 fills inner cavity cross-section so camera front contacts TPU, not bare ASA.
    - Lens opening and other front cutouts extracted from STEP and subtracted through pad.
    - In dual-material assembly, TPU is placed front-flush against ASA inner wall when pad is enabled.
  - Rear remains open for insertion; rear-side TPU contact is handled by rear cap TPU gasket.
  - TPU side vent cuts are default-disabled in the active dual-body output; if enabled, side `3 + 3` vents are rounded-slot cuts and the tripod-side vent bank uses the merged panel behavior.
  - Does not use full TPU face caps.
  - ASA vent pass-through is validated at all former logical vent positions, with tripod through-cut also validated; TPU vent validation is skipped when TPU side vent cutouts are disabled.
  - Vent rows are aligned to the ASA sleeve vent coordinates in device frame.
  - Applied empirical vent alignment deltas from tripod-registered overlay:
    - tripanel bank Z shift: `-4.226 mm`
    - side-trio bank Z shift: `-13.397 mm`
  - 24-grid lateral spacing calibration:
    - center column remains fixed
    - left/right columns are inset toward center by `2.6 mm` each
    - resulting tripanel X centers: `-13.4, 0.0, +13.4 mm`
  - Legacy separate TPU liner/caps and unibody files are archived automatically.
- Separate body outputs (ASA + TPU as individual files):
  - `models/maki_case/maki_live_asa_shell.step`
  - `models/maki_case/maki_live_tpu_frame.step`
  - Generator: `scripts/generate_maki_live_dual_material_body.py`
  - TPU is auto-aligned into ASA cavity with report-verified fit stack (default `0.15 mm` radial gap each side, ~`2.1 mm` axial front/back gap).
  - Fit validation pipeline:
    - Script: `scripts/validate_maki_live_fit.py`
    - Output assembly: `models/maki_case/reports/maki_live_fit_validation_assembly.step`
    - Output report: `models/maki_case/reports/maki_live_fit_validation_report.json`
    - Validates real device STEP placement inside body + rear cap and reports pairwise hard-collision volumes.
    - Also validates all major opening geometry against STEP-derived expectations:
      - separated ASA tripanel panel coverage plus large side-panel coverage for the 6 side vent probes,
      - TPU vent positions only when TPU side vent cutouts are enabled,
      - tripod opening alignment for both ASA and TPU,
      - rear-cap cutout center/size matching against STEP-derived rear port cutouts.
    - Current baseline result: no hard collisions and all feature checks passing.

### Mevo Start
Current preferred workflow:
- 3 separate output files:
  - `models/mevo_case/mevo_start_asa_shell.step`
  - `models/mevo_case/mevo_start_tpu_frame.step`
  - `models/mevo_case/mevo_start_back_cap.step` (contains `ASA_Back_Cap` + `TPU_Back_Insert`)
  - Generator: `scripts/generate_mevo_dual_material_case.py`
- Geometry intent:
  - ovular/capsule-profile sleeve geometry (not rounded-rectangle profile),
  - front-integrated body by default (front wall fused to sleeve),
  - TPU body is a skeleton frame (corner bumpers + edge rails, not solid walls), front edge wrap only (rear wrap disabled) to keep insertion path open,
  - front lens/LED cutouts enabled by default in closed-front mode,
  - lens opening defaults updated to `42.2 mm` (enlarged +12.7 mm / 0.5") at `Y=20.0` for improved framing,
  - top-visor lens hood (arc over top of lens opening, clipped to body width) is enabled by default (`depth=16 mm`, `wall=2.5 mm`, `clearance=1.0 mm`),
  - thermal venting: 5x oval slot vents on top (24 mm × 3.5 mm each) plus 7 side slot vents per side (up from 5); top-hole plane orientation is corrected to the hood/top side,
  - separate back-cap assembly: ASA structural plug + TPU gasket face pad,
    - ASA cap carries the full structural plug (3.0 mm plate + 5.0 mm two-stage tongue/lip insertion),
    - TPU is a 1.2 mm gasket ring only — no structural plug duty,
    - snap ridges and friction ridges are on the ASA plug, not TPU,
  - manual two-cutout rear-cap layout is default-enabled (`include_manual_back_cutouts=true`):
    - lower slot from Mevo edge offsets (10 mm side margins, 7 mm bottom offset),
    - upper domed cutout using top offsets (3.0 mm and 28.0 mm from top, 3.0 mm side margins),
  - TPU-aware back-cap fit clearance default: `0.28 mm` total undersize,
  - single-step plug engagement with matching rear body groove seat (ASA, tongue_radial_step=0),
  - bottom tripod opening is a 31.75 × 50.8 mm rectangular cutout, centered 50.79 mm from front, with TPU relief to keep the opening clear.
  - rear TPU insertion relief is enabled (`5.4 mm` default) so the back-cap plug can seat without colliding with sleeve TPU.
  - Cold shoe mount (ISO 518 female receptor) on top rear of capsule:
    - Flat fill-pad bridges curved capsule top to create level mounting surface (28×30 mm, 3 mm corner radius).
    - Raised boss on pad: 22×22×4 mm.
    - T-slot channel: 18.2 mm floor width, 12.5 mm rail opening, 2.85 mm rail overhang each side.
    - Foot slides in from rear (open at rear end, closed at front).
    - Controlled by `cold_shoe_*` params; disable with `--no-cold-shoe`.
  - Snap-latch flexure clips (2x on inner X walls) for back cap retention:
    - Cantilever beams: 7×5×1.5 mm, 1.0 mm catch nub at tip.
    - Matching 1.0 mm ridge on ASA back-cap plug for snap engagement.
    - Controlled by `snap_clip_*` params; disable with `--no-snap-clips`.
  - 4-point detent pocket/bump retention system (2 on X walls + 2 on Y walls):
    - Body has 4 pockets (8×0.8×4 mm) at friction_ridge_setback from rear.
    - Cap plug has 4 matching bumps with 0.4 mm relief channels for deflection during insertion.
    - Controlled by `friction_ridge_*` params; disable with `--no-friction-ridge`.
  - Cap collision check runs automatically during generation:
    - Compares ASA cap plate vs ASA shell (not full assemblies) to avoid false positives from plug-in-cavity geometry.
    - Reports collision volume in `mevo_start_dual_material_report.json`; warns if > 0.5 mm³.
- Mevo back cap utility slot is default-disabled (`include_back_utility_slot=false`) until exact rear port map is confirmed.
- Mevo lens opening uses offset center (`lens_center_y_mm=20.0`) to avoid centered misalignment.
- Optional flags:
  - `--open-front-ovular` for legacy open-front mode,
  - `--disable-front-lens-led-cutouts` to keep front wall solid,
  - `--no-cold-shoe` to disable cold shoe mount,
  - `--cold-shoe-z-from-rear` to adjust cold shoe position (default 15 mm, flush with rear),
  - `--no-snap-clips` to disable snap-latch flexure clips.
- Active review-spec values are recorded in `models/mevo_case/reports/mevo_start_dual_material_report.json`.
- Legacy open-through/cap workflows remain in repo for fallback only and are not the active default path.
- Note: manufacturer STEP-based auto-extraction is currently strongest for MAKI.
  Mevo workflow is dimension/spec-driven unless a full Mevo STEP is provided.

### Mevo Core
- ASA-only workflow with 2 separate output files:
  - `models/mevo_core_case/mevo_core_asa_shell.step`
  - `models/mevo_core_case/mevo_core_back_cap.step`
  - Generator: `scripts/generate_mevo_core_case.py`
- Geometry intent:
  - Square cross-section (90 × 90 mm, rounded-rectangle with aggressive 24 mm outer fillet)
  - Direct ASA camera fit with `0.6 mm` per-side clearance; no TPU assumptions in shell sizing
  - Front-integrated body with centered lens cutout (76.2 mm / 3.0")
  - Main ASA body front face-to-side-wall perimeter uses a targeted `3.0 mm` fillet.
  - Full circular tube lens hood (101.6 mm depth / 4.0”, 2.5 mm wall), NOT visor:
    - hood root has a `1.2 mm` positive-Z overlap collar embedded into the ASA front wall so it fuses seamlessly into the front face instead of relying on a coplanar join.
    - hood base flare is clamped with a `3.0 mm` front-edge inset so the flared root does not hang past the rounded front face perimeter.
    - side lens-access notches use rounded rectangular cuts: `44.45 mm` forward depth, `50.8 mm` height, `25.4 mm` radial bite, `3.0 mm` corner radius; the cutter starts `1.0 mm` outside the widest flared root radius so it clears root artifacts from the access window.
    - the generated notch cut-wall edges are post-rounded with a `0.75 mm` terminal perimeter fillet and `0.3 mm` root-side small-edge fillet to remove pointed cutout edges.
  - Single bottom tripod mount, rectangular cutout (63.5 × 50.8 mm)
  - Cold shoe mount (ISO 518) on top rear
  - External sun shade support ribs are pushed into the curved-corner bands so they clear the large vent panels:
    - side rib centers sit at `|Y|=25.9 mm`, with rib inner edge at `|Y|=23.9 mm`,
    - top rib centers sit at `|X|=25.9 mm`, with rib inner edge at `|X|=23.9 mm`,
    - side/top vent panel half-width remains `22.9 mm`, leaving `1.0 mm` clearance to the nearest rib edge.
  - External sun shade side skirts stop just past the lower support rib: lower rib outside edge `Y=27.9 mm`, skirt bottom cutoff `Y=29.9 mm` (`2.0 mm` overhang past the rib), with `2.0 mm` outside-edge and `0.75 mm` inner-edge lower fillets.
  - External sun shade extends `6.35 mm` behind the rear body face to overhang the back/sides; support ribs stay within the original body depth, the rear end has a cut-back lofted `1.5 mm` outward shade-perimeter extension that shrinks `1.5 mm` inward toward the case with only a `0.15 mm` fuse overlap at the shade end.
  - Other external right-angle details are softened unless they are cold-shoe boss/post geometry:
    - large side/top thermal notches use `2.0 mm` corner radii,
    - sun-shade support ribs use `1.0 mm` rounded-rectangle cross-section corners.
  - Snap-latch release/clearance holes in the shell stay rectangular (`0.0 mm` corner radius) so the back-cap latch bumps have full corner clearance.
  - Back cap = ASA bumper ring with one large open-center rounded-rectangle cutout (`74.0 × 74.0 mm`, `8.0 mm` corner radius)
  - 3 snap-latch walls plus Y+ retention bump
  - Thermal cutouts are large rectangular panel notches, not individual slit vents:
    - one per side face across the former 6-row side vent bank,
    - one top-face notch across the former 4-row top vent bank, enlarged and Z-aligned to match the side notches,
    - each notch uses support-aware sizing so it stays clear of the relocated shade support ribs.
  - Dimension-driven workflow (no manufacturer STEP reference)
- Optional flags: `--no-cold-shoe`, `--no-friction-ridge`, `--no-snap-latches`, `--no-hood`, `--no-vents`, `--no-sun-shade`, `--lens-diameter`, `--hood-depth`

### Zowietek 4K NDI POV Zoom Camera
- 3 separate output files:
  - `models/zowietek_case/zowietek_pov_asa_shell.step`
  - `models/zowietek_case/zowietek_pov_tpu_frame.step`
  - `models/zowietek_case/zowietek_pov_back_cap.step`
  - Generator: `scripts/generate_zowietek_dual_material_case.py`
- Geometry intent:
  - Rounded-rectangle cross-section (box + corner fillets, not capsule)
  - TPU is a skeleton frame (corner bumpers + edge rails, not solid walls)
    - 12 mm corner bumpers at each of 4 vertical edges
    - 4 mm edge rails along horizontal edges connecting corners
    - Wall windows cut through flat wall sections between bumper/rail zones
  - Front-integrated body with lens/LED cutouts and top-visor lens hood
  - Two 1/4”-20 UNC tripod mounts (bottom + top), rectangular cutouts
  - Thermal vents: 5 side slots per side + top slots (cold shoe zone filtered)
  - Cold shoe mount (ISO 518) on top rear
  - Back cap = ASA bumper ring: structural frame with large center opening
    - All rear ports/buttons/LED accessible through open center
    - Plug tongue for body insertion + 4-point cantilever latch retention
  - Dimension-driven workflow (no manufacturer STEP reference)
- Optional flags:
  - `--no-cold-shoe`, `--no-friction-ridge`, `--no-top-tripod`, `--no-hood`, `--no-vents`
  - `--lens-diameter`, `--cold-shoe-z-from-rear`, `--bumper-ring-inset`
- Active review-spec values in `models/zowietek_case/reports/zowietek_pov_dual_material_report.json`

## Terminology Mapping (Important for user shorthand)
User shorthand often means:
- “Maki sleeve” / “Maki ASA” = `maki_live_asa_shell.step`
- “TPU sleeve for Maki” / “TPU frame for Maki” = `maki_live_tpu_frame.step` (skeleton frame)
- “Maki caps” = rear cap by default (`maki_live_back_cap.step`)
- “Mevo rear closure” / “Mevo back cap” = `mevo_start_back_cap.step` (ASA cap + TPU gasket)
- “Mevo Core case” = 3 files in `models/mevo_core_case/`
- “Zowietek case” = 3 files in `models/zowietek_case/`
- “Zowietek bumper ring” = `zowietek_pov_back_cap.step` (ASA back cap with open center)

## Key References
- MAKI references:
  - `refs/BirdDog_MAKI-Live_3D-file.step`
  - `refs/MAKI-Live_drawing.pdf`
- Mevo references:
  - `refs/Mevo_Start_lens_cover_corrected.stl`
  - Additional Mevo legacy refs are present in `refs/`.

## Common Commands
Activate environment:
```bash
source .venv311/bin/activate
```

MAKI:
```bash
python scripts/generate_maki_live_case.py
python scripts/generate_maki_live_tpu_liner.py
python scripts/generate_maki_live_dual_material_body.py
python scripts/generate_maki_live_caps.py
python scripts/validate_maki_live_fit.py
```

Mevo Start:
```bash
python scripts/generate_mevo_dual_material_case.py
```

Mevo Core:
```bash
python scripts/generate_mevo_core_case.py
```

Zowietek:
```bash
python scripts/generate_zowietek_dual_material_case.py
```

Legacy Mevo back plate (only when explicitly requested):
```bash
python scripts/generate_mevo_case.py --include-back-plate
```

## Session Continuity Guidance
When the user gives brief or “random” update requests, assume they refer to this context and default behaviors unless they explicitly override:
- Mevo Core is ASA-only and outputs only ASA shell + back cap.
- Other active dual-material camera workflows output separate ASA shell, TPU frame, and back cap.
- Keep remaining TPU workflows as skeleton frames (corner bumpers + edge rails) unless explicitly changed.
- Maintain archive hygiene policy.
- Preserve smooth B-REP output quality.
- Lens hoods: Mevo Core uses full circular tube; all others use top-visor (clipped arc).

If the user asks for changes that affect fit/alignment:
- Regenerate relevant model(s).
- Check the JSON report in the case `reports/` subfolder.
- Verify outputs are single-solid STEP where applicable.
- Keep latest STEP outputs at top level and auto-archive previous versions.
