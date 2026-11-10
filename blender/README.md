# ARTEMIS in Blender

A rigged, render-ready Blender scene of FRC 4915 Spartronics' 2026 robot **ARTEMIS**.
It is built from the Onshape glTF exports the team made for AdvantageScope.
Every mechanism moves from one set of sliders, in the same units the robot code uses.

![ARTEMIS with intake deployed](previews/hero_intake_deployed.jpg)

| File | What it is |
| --- | --- |
| `ARTEMIS.blend` | The scene (~44 MB, Git LFS). Open with **Blender 5.2 LTS** or newer. GitHub does not store LFS files in forks, so this file is **not committed in `natebrix/2026-Rebuilt`**. Get it from [natebrix/artemis-blender](https://github.com/natebrix/artemis-blender), which is the main home of this project, or build it (see [Rebuilding](#rebuilding), ~35 s). |
| `build_artemis.py` | Rebuilds `ARTEMIS.blend` from `source/`: import, cleanup, rig, materials, studio, demo animation. |
| `materials.py` | Procedural material library and the rules that map each part to a material. |
| `approx_panels.py` | Best-effort panels that the real robot has but the May CAD export lacks: the intake hopper extension and the back walls. See [Approximate parts](#approximate-parts-not-in-the-cad). |
| `fetch_bom_materials.py` | Downloads each part's Onshape material and appearance from the public share into `source/onshape_bom_materials.json`. No API key is needed. |
| `render_previews.py` | Renders a still of any pose or animation frame from the command line. |
| `artemis_joints.json` | Every joint's parent, origin, axis and control property, for simulation or other tools. |
| `source/` | Input GLBs: a copy of `assets/Robot_Artemis` from the `unstable` branch. This is the export of May 19 2026, commit `1368dc9`. The files are byte-identical to those on `unstable`, so git stores them only once. |
| `previews/` | Example renders, including a contact sheet of the demo animation. |

In a repository that stores `ARTEMIS.blend` in LFS, the file may be a ~130-byte text file after cloning. That is the LFS pointer, not the file. Fetch the real file with:

```bash
git lfs install
```

```bash
git lfs pull
```

## Using the rig

* **Units and axes.** The model uses the WPILib robot frame: **+X forward, +Y left, +Z up**, in metres. The origin is on the floor at the centre of the drive base.
* **Controls.** Select the **`ARTEMIS`** empty in the `ARTEMIS › Rig` collection and open *Object Properties › Custom Properties*. Every slider below drives a joint. To keyframe one, hover over it and press `I`.
* **Driving the robot.** Move or rotate the `ARTEMIS` empty itself to drive the robot around. Every part follows.
* **Python not required.** The drivers are plain expressions, so they work with *Auto Run Python Scripts* turned off.

| Property | Units / range | Meaning |
| --- | --- | --- |
| `intake_pivot_deg` | −2 … 132 ° | Matches `PivotSubsystem`. **130 = STOW** (CAD pose, the default), 60 = SAFE, 0 = READY (deployed). Also slides the approximate hopper extension. |
| `turret_deg` | −170 … 225 ° | Matches `TurretSubsystem`. Counter-clockwise (CCW) from robot-forward. |
| `hood_deg` | 0 … 40 ° | Matches `HoodSubsystem`. 0 = down. The hood pivots on the flywheel shaft. |
| `climber_m` | 0 … 0.28 m | Climber extension. The 0.28 m travel is an estimate; see the notes below. |
| `spindexer_deg` | ° | Spindexer rotation, CCW seen from above. |
| `steer_FL/FR/BL/BR_deg` | ° | Swerve module azimuth, CCW. 0 = wheel rolling straight forward. |
| `drive_FL/FR/BL/BR_deg` | ° | Wheel rotation; positive rolls the robot forward. Distance = radians × 0.0508 m. |
| `intake_rollers_deg` | ° | All intake rollers. |
| `shooter_wheels_deg` | ° | Flywheel stack and shooter rollers. |
| `feeder_rollers_deg` | ° | Feeder rollers under the turret. |
| `bumper_blue` | 0 … 1 | 0 = red bumpers, 1 = blue (the default, as in the team photo). |

**Making things spin.** Keyframe the `*_deg` properties with *linear* interpolation. For example, 3,600° over 30 frames is 10 rev/s at 30 fps.

**Roller speed and direction.** Each roller group is an empty named `J_roller_*`, and each has a `ratio` custom property (default 1). Set it negative to reverse that roller, or change it to set its speed relative to the control.

**Demo animation.** The file includes a 10-second action, **`Demo_Mechanisms`** (300 frames at 30 fps). It deploys the intake, aims, raises the hood, steers, spins up and climbs. To pose by hand, unlink it in the Action editor.

![Demo animation frames](previews/demo_contact_sheet.png)

## Rendering

The `Studio` collection holds:

* a floor,
* three area lights,
* the cameras `CAM_Hero`, `CAM_Front`, `CAM_Side`, `CAM_RearHigh` and `CAM_Top`.

To use your own environment, hide or delete the collection.

Render settings:

* The default engine is Cycles at 128 samples with denoising, and the colour transform is AgX.
* EEVEE also works. For the smoked polycarbonate, turn on *Raytracing* in the EEVEE render settings.

To render a still from the command line, run this from this folder with `blender` on your PATH:

```bash
blender -b ARTEMIS.blend --python render_previews.py -- --out shot.png --camera CAM_Hero --samples 128 --pose intake_pivot_deg=0 turret_deg=45 hood_deg=20
```

`--frame N` renders a frame of the demo animation instead of a fixed pose. `--engine workbench --color-by joint` gives a quick clay render that colours each part by the joint it follows.

## Rebuilding

Rebuild after re-exporting the CAD, or after changing the material rules. A build takes about 35 seconds. Use either Blender:

```bash
blender --background --python build_artemis.py -- --out ARTEMIS.blend --report artemis_joints.json
```

or the `bpy` module from PyPI:

```bash
pip install bpy==5.2.4
```

```bash
python build_artemis.py --out ARTEMIS.blend --report artemis_joints.json
```

Options:

| Option | Effect |
| --- | --- |
| `--keep-hardware` | Keep screws, nuts, rivets and washers (+~650k triangles). Useful for close-ups. |
| `--assets DIR` | Read the GLBs from another folder (default: `source/`). |
| `--team-number N` | Text for the bumper numbers (default `4915`; `''` for none). |
| `--no-rollers` | Don't rig spinning rollers. |
| `--no-studio` | Skip the floor, lights and cameras. |
| `--no-demo` | Skip the demo animation. |
| `--no-approx` | Skip the approximate panels in `approx_panels.py`. |

**Re-exporting from Onshape.** The script expects the same layout as the AdvantageScope export:

* `model.glb`: everything static
* `model_0.glb`: intake arm
* `model_1.glb`: turret and shooter
* `model_2.glb`: hood
* `model_3.glb`: spindexer

If the layout changes, edit `COMPONENT_FILES` at the top of `build_artemis.py`.

**Changing materials.** Edit `materials.py`:

* `NAME_RULES` maps part names to material types.
* `PHOTO_OVERRIDES` and `COLOR_OVERRIDES` handle one-off fixes.
* Team-made parts (`Part 12`, `Spindexer Wall`, ...) use the material set in Onshape when the public share has one. Re-run `python fetch_bom_materials.py` after materials change in Onshape.
* Colours such as `BUMPER_BLUE` and `SMOKE_TINT` are at the top of the file.

## How it was built and how accurate it is

* **Geometry.** The geometry comes from the team's AdvantageScope GLBs (Onshape export, May 19 2026). Parts are named as in Onshape and grouped into collections by subsystem. Hardware (791 screws, nuts, rivets and washers) and CAD helper cubes are removed.
* **Moving parts.** The team had already split the intake arm, turret/shooter, hood and spindexer into separate files. The build script also moves the following parts, picked out by name:
  * the turret gear and inner bearing race,
  * the climber's inner tube and hook,
  * each swerve module's turret assembly and wheel.

  Spinning rollers are found automatically. Each wheel is grouped with the shafts, pulleys and hubs on its axis.
* **Pivots and limits.** These come from the robot code on the `unstable` branch, using the `Pose3d` each subsystem logs for AdvantageScope:
  * intake pivot (0.2842, 0, 0.1825),
  * turret (−0.1183, −0.1437, 0.3623),
  * spindexer (0.0472, 0, 0.0619).

  The hood's logged pose uses an offset reference point. The true pivot it implies is the flywheel shaft at (y, z) = (−0.0975, 0.4953), and that is what the rig uses. Swerve axes come from the geometry: the modules sit at (±0.264, ±0.289). Limits are taken from the code's clamps.
* **Materials.** Materials come from four sources:
  * the Onshape *Material* property of team-made parts (smoked and clear polycarbonate, white polypropylene, nylon, Onyx and other prints, birch, EVA),
  * vendor part names,
  * the Onshape colours,
  * the team photo and reveal video (blue bumpers with white "4915" on all four sides, the brass/blue/green flywheel stack, black prints).

### Known limitations

* **Climber travel.** The 0.28 m travel is estimated from the tube lengths, because the climber code has been removed.
* **Hopper extension and back walls are approximate.** They aren't in the May export, so they were modelled from photos and video; see [Approximate parts](#approximate-parts-not-in-the-cad). The way the hopper extension retracts is a guess.
* **Not in the CAD at all.** The sponsor and ARTEMIS decals, wiring and cable sleeves.
* **Guessed materials.** Team-made parts with no material set in Onshape are guessed from their CAD colour. For example, white CAD parts became white HDPE or white prints.
* **Mechanism motion.** All roller ratios are 1. Belts and chains don't animate.
* **Triangle count.** The scene has 2.5 M triangles, which is fine for stills and animation. For real-time use or physics, decimate the meshes or build simple collision shapes.
* **Newer CAD versions.** The model is the May 2026 export. The newer public Onshape share (version `d5fdcfd9…`) adds at least the intake hopper walls. To use it, export the same five files and rebuild.

## Approximate parts (not in the CAD)

The real robot has panels that the May 2026 CAD export doesn't. Without CAD geometry or a newer export, `approx_panels.py` adds them as flat 1/8" smoked-polycarbonate panels. They are sized from the Onshape parts list and placed from the reveal video and the team photo. They live in the `ARTEMIS › Approximate (not in CAD)` collection, so you can hide them or skip them with `--no-approx`.

* **Intake hopper extension:** a front wall and two side walls. These are Parts 34, 32 and 35 in the newer Onshape version. The parts list gives their weights: 1.78 lb for the front wall and 0.955 lb for each side wall, which works out to about 0.21 m² and 0.11 m² at 1/8". The deployed shape matches the video at 0:24 and 0:44.
* **How the extension retracts is a guess.** No photo shows it. The walls can't swing rigidly with the intake arm, because they would pass through the drivebase and the turret. So the model treats them as a sliding extension:
  * The front wall stays upright and slides 24 cm forward on `J_hopper_ext`. It is driven by `intake_pivot_deg` and reaches full extension by 30°, which keeps it ahead of the swinging rollers.
  * The side walls telescope: a Hook modifier moves their front edge.

  A collision check in the stowed, mid-travel and deployed poses, with the turret and hood at several angles, found no overlaps. The only contact is where the deployed front wall meets the intake's side plates.
* **Back walls:** seen at 0:14 in the reveal video. The left half is full height. The right half stops at 0.355 m so the shooter can swing over it. The climber tube sits in the gap between them.

When a newer CAD export is available, delete `approx_panels.py` from the build (or use `--no-approx`) and rebuild.

## Simulation

`artemis_joints.json` lists every joint:

* type (revolute or prismatic),
* parent,
* origin in the robot frame,
* axis,
* the control property and how it converts to an angle.

Each `J_*` empty's children are the rigid parts that move with that joint. That's what a URDF or MJCF exporter needs; export one mesh per joint from Blender and you're most of the way there.
