#!/usr/bin/env python3
"""
build_artemis.py - Build a rigged, render-ready Blender scene of FRC 4915's
2026 robot ARTEMIS from the Onshape glTF exports used for AdvantageScope.

Inputs (default: ./source, a copy of assets/Robot_Artemis from the `unstable`
branch, May 19 2026 export):
    model.glb    static robot (everything that does not move relative to the frame)
    model_0.glb  intake arm   (pivots)
    model_1.glb  turret + shooter (yaws)
    model_2.glb  hood          (pitches, rides on the turret)
    model_3.glb  spindexer     (spins)

Output: a .blend file with
    * the robot in the WPILib robot frame: +X forward, +Y left, +Z up,
      origin on the floor at the centre of the drive base, metres
    * hardware (screws, nuts, rivets, washers, ...) removed (--keep-hardware keeps it)
    * one control object, "ARTEMIS", whose custom properties drive every joint
      (turret, hood, intake pivot, spindexer, climber, 4x swerve steer + drive,
      spinning rollers/flywheels) via drivers, in robot-code units
    * realistic procedural materials (see materials.py)
    * a "Studio" collection with floor, lights and cameras, and a short demo animation

Run with Blender (tested with 5.2 LTS):
    blender --background --python build_artemis.py -- --out ARTEMIS.blend
or with the `bpy` module from PyPI (pip install bpy==5.2.*):
    python build_artemis.py --out ARTEMIS.blend

Run `... -- --help` for all options.
"""

import argparse
import math
import os
import re
import sys
from collections import defaultdict

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import materials  # noqa: E402  (sibling module)

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

# Which GLB holds which moving link. Indices follow the AdvantageScope config
# on the `unstable` branch (May 2026 export).
COMPONENT_FILES = {
    "model_0.glb": "intake_arm",
    "model_1.glb": "turret",
    "model_2.glb": "hood",
    "model_3.glb": "spindexer",
}

# Blender's glTF importer converts glTF Y-up to Blender Z-up. For these
# Onshape exports that leaves the robot with +Y up and its front facing -X.
# This matrix maps those coordinates to the WPILib robot frame:
#   robot = (-x, z, y)
TO_ROBOT = Matrix(((-1, 0, 0, 0),
                   (0, 0, 1, 0),
                   (0, 1, 0, 0),
                   (0, 0, 0, 1)))

# Parts removed unless --keep-hardware.
HARDWARE_RE = re.compile(
    r"\b(BHCS|SHCS|FHCS|LHCS)\b|cap screw|Socket Head Screw|Shaft End Screw|"
    r"Screw\b|Rivet|Locknut|Lock Nut|Hex Nut|Washer|Heat ?Set|HeatSet|"
    r"Retaining Ring|Snap ?Ring|Cotter|Hitch Pin",
    re.I)
# CAD helper geometry, always removed.
HELPER_RE = re.compile(r"^Origin Cube", re.I)

# Turret-ring parts (in the static model) that are fixed to the rotating
# inner race, so they turn with the turret.
TURRET_RING_ROTATING = {
    "Custom 84T 10DP Turret Gear",
    "Inside Bearing Race Clamp Lower",
    "Inside Bearing Race Clamp Upper",
    "Upper Turret Spacer",
    "Lower Turret Spacer",
}

# Climber parts that ride on the inner (moving) tube.
CLIMBER_MOVING_RE = re.compile(
    r"^(Inner Tube|Part 13|Part 14|Crush Block|am-4630 1x1 Clamp Slider Block|"
    r"am-xxxx 5-16OD x \.875 10-32 Round AL Standoff)")
CLIMBER_TRAVEL_M = 0.28  # estimated: inner tube can extend ~28 cm before it leaves the outer tube

# Joints whose pivots come from the robot code (unstable branch,
# subsystems/mechanisms/*: the Pose3d used for AdvantageScope).
#   name: (parent, kind, pivot, axis, prop, offset, sign, (min, max) or None, description)
# A revolute joint's angle (radians) = sign * radians(prop - offset).
CODE_JOINTS = {
    "turret": (None, "revolute", (-0.118295, -0.143695, 0.362276), "Z",
               "turret_deg", 0.0, +1, (-170.0, 225.0),
               "Turret yaw, degrees CCW from robot-forward (TurretSubsystem position). Limits -170..225."),
    "intake_arm": (None, "revolute", (0.2842, 0.0, 0.1825), "Y",
                   "intake_pivot_deg", 130.0, -1, (-2.0, 132.0),
                   "Intake pivot (PivotSubsystem position). 130 = STOW (CAD pose), 60 = SAFE, 0 = READY/deployed."),
    "spindexer": (None, "revolute", (0.0472, -0.0002, 0.0619), "Z",
                  "spindexer_deg", 0.0, +1, None,
                  "Spindexer rotation, degrees CCW seen from above. Key it linearly to spin."),
}
# The hood pivot is found from geometry (see find_hood_pivot); its angle
# follows HoodSubsystem: rotation about robot X by -position.
HOOD_LIMITS = (0.0, 40.0)

# Spinning roller detection.
ROLLER_SEED_RE = re.compile(
    r"Wheel|Flywheel|Roller|Compliant|Squish|Flex|Stealth|Urethane", re.I)
ROLLER_EXCLUDE_RE = re.compile(
    r"Bearing|Belt|Chain|Motor|Planetary|Encoder|Kraken|Gearbox|Gusset|Tube\b|"
    r"Echain|Plate|Retention|Snake|Limelight|Luma|Radio", re.I)
ROLLER_PROPS = [  # (link, path substring) -> control property
    ("intake_arm", "", "intake_rollers_deg"),
    ("turret", "", "shooter_wheels_deg"),
    ("hood", "", "shooter_wheels_deg"),
    ("chassis", "61 - 2026 Turret", "feeder_rollers_deg"),
    ("chassis", "51 - 2026 Indexer", "indexer_rollers_deg"),
    ("chassis", "32 - 2026 Intake", "intake_rollers_deg"),
    ("chassis", "", "misc_rollers_deg"),
]
ROLLER_PROP_DOCS = {
    "intake_rollers_deg": "Intake roller rotation (degrees). Key linearly to spin.",
    "shooter_wheels_deg": "Shooter flywheel/roller rotation (degrees). Key linearly to spin.",
    "feeder_rollers_deg": "Feeder (under-turret) roller rotation (degrees).",
    "indexer_rollers_deg": "Indexer roller rotation (degrees).",
    "misc_rollers_deg": "Other roller rotation (degrees).",
}

# Collections by Onshape top-level subassembly name.
SUBSYSTEMS = [
    ("21 - 2026 Drivebase", "Drivebase"),
    ("32 - 2026 Intake", "Intake"),
    ("51 - 2026 Indexer", "Indexer"),
    ("61 - 2026 Turret", "Turret"),
    ("72 - 2026 Shooter", "Shooter"),
    ("81 - 2026 Climber", "Climber"),
    ("91 - 2026 Bumpers", "Bumpers"),
]

CORNERS = ("FL", "FR", "BL", "BR")

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

SUFFIX_RE = re.compile(r"\.\d{3}$")
INSTANCE_RE = re.compile(r"\s*<\d+>(\.\d{3})?$")


def base_name(name):
    return SUFFIX_RE.sub("", name)


def asm_name(name):
    return INSTANCE_RE.sub("", base_name(name))


def log(*a):
    print("[artemis]", *a, flush=True)


def world_bbox(objs):
    pts = []
    for o in objs:
        mw = o.matrix_world
        pts.extend(mw @ Vector(c) for c in o.bound_box)
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return mn, mx


def bbox_dims(o):
    mn, mx = world_bbox([o])
    return (mn + mx) / 2, mx - mn


AXES = {"X": Vector((1, 0, 0)), "Y": Vector((0, 1, 0)), "Z": Vector((0, 0, 1))}


def link_collection(name, parent):
    col = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if col.name not in parent.children:
        parent.children.link(col)
    return col


def move_to_collection(o, col):
    for c in list(o.users_collection):
        c.objects.unlink(o)
    col.objects.link(o)


# --------------------------------------------------------------------------
# Import
# --------------------------------------------------------------------------

def import_glb(path, link):
    """Import one GLB, bake every part into the robot frame, return the parts."""
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    bpy.context.view_layer.update()
    parts = []
    for o in new:
        if o.type != "MESH":
            continue
        chain, p = [], o.parent
        while p is not None:
            chain.append(asm_name(p.name))
            p = p.parent
        chain.reverse()
        # chain[0] is the top-level assembly; keep the subassembly path below it
        o["onshape_name"] = base_name(o.name)
        o["onshape_path"] = " / ".join(chain[1:])
        o["source"] = os.path.basename(path)
        o["link"] = link
        mw = TO_ROBOT @ o.matrix_world
        if o.data.users > 1:
            o.data = o.data.copy()
        o.parent = None
        o.data.transform(mw)
        o.matrix_world = Matrix.Identity(4)
        parts.append(o)
    for o in new:
        if o.type != "MESH":
            bpy.data.objects.remove(o)
    return parts


def recenter_origin(o):
    """Put the object's origin at the centre of its bounding box."""
    c, _ = bbox_dims(o)
    o.data.transform(Matrix.Translation(-c))
    o.matrix_world = Matrix.Translation(c) @ o.matrix_world


# --------------------------------------------------------------------------
# Classification
# --------------------------------------------------------------------------

def corner_of(center):
    return ("F" if center.x > 0 else "B") + ("L" if center.y > 0 else "R")


def assign_links(parts):
    for o in parts:
        if o["link"] != "chassis":
            continue
        name, path = o["onshape_name"], o["onshape_path"]
        c, _ = bbox_dims(o)
        if "SDS MK5n" in path:
            if name.startswith("SDS MK5 Turret Assembly"):
                o["link"] = "steer_" + corner_of(c)
            elif name.startswith("SDS MK5 Molded Wheel"):
                o["link"] = "wheel_" + corner_of(c)
        elif path.startswith("61 - 2026 Turret") and name in TURRET_RING_ROTATING:
            o["link"] = "turret"
        elif path.startswith("81 - 2026 Climber") and CLIMBER_MOVING_RE.match(name):
            o["link"] = "climber"


def attach_hardware(hardware, parts):
    """Give each kept hardware item the link of the part it sits in."""
    boxes = []
    for p in parts:
        mn, mx = world_bbox([p])
        boxes.append((p, mn, mx, (mx - mn).length))
    for h in hardware:
        c, _ = bbox_dims(h)
        best, best_size = None, 1e9
        for p, mn, mx, size in boxes:
            m = 0.002
            if all(mn[i] - m <= c[i] <= mx[i] + m for i in range(3)) and size < best_size:
                best, best_size = p, size
        if best is not None:
            h["link"] = best["link"]


def symmetric_axis(dims, tol=0.10):
    """For an axis-aligned, rotationally symmetric part return (axis, radial size)."""
    d = list(dims)
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        if d[j] <= 0 or d[k] <= 0:
            continue
        if abs(d[j] - d[k]) / max(d[j], d[k]) < tol and abs(d[i] - d[j]) / max(d[i], d[j]) > tol:
            return i, max(d[j], d[k])
    return None, None


def detect_rollers(parts):
    """Group spinning parts (wheels + their shafts, pulleys, hubs) by axis line.

    Returns a list of dicts: link, axis index, point on axis, members, label, prop.
    """
    eligible = [p for p in parts
                if p["link"] in ("intake_arm", "turret", "hood", "chassis")
                and not ROLLER_EXCLUDE_RE.search(p["onshape_name"])]
    info = {}
    for p in eligible:
        c, d = bbox_dims(p)
        info[p.name] = (c, d)

    # 1. seed lines from obvious rollers
    lines = []  # dict(link, axis, pt, seeds, rmax, tmin, tmax)
    for p in eligible:
        if not ROLLER_SEED_RE.search(p["onshape_name"]):
            continue
        c, d = info[p.name]
        ax, r = symmetric_axis(d)
        if ax is None:
            continue
        for ln in lines:
            if ln["link"] == p["link"] and ln["axis"] == ax:
                off = [abs(c[i] - ln["pt"][i]) for i in range(3) if i != ax]
                if max(off) < 0.004:
                    ln["seeds"].append(p)
                    ln["rmax"] = max(ln["rmax"], r)
                    ln["tmin"] = min(ln["tmin"], c[ax] - d[ax] / 2)
                    ln["tmax"] = max(ln["tmax"], c[ax] + d[ax] / 2)
                    break
        else:
            lines.append(dict(link=p["link"], axis=ax, pt=c.copy(), seeds=[p], rmax=r,
                              tmin=c[ax] - d[ax] / 2, tmax=c[ax] + d[ax] / 2))

    # 2. add co-axial symmetric parts (shafts, pulleys, hubs, spacers, gears)
    taken = set()
    for ln in lines:
        ax = ln["axis"]
        ln["members"] = list(ln["seeds"])
        taken.update(s.name for s in ln["seeds"])
    for p in eligible:
        if p.name in taken:
            continue
        c, d = info[p.name]
        for ln in lines:
            ax = ln["axis"]
            if p["link"] != ln["link"]:
                continue
            j, k = [a for a in range(3) if a != ax]
            if max(abs(c[j] - ln["pt"][j]), abs(c[k] - ln["pt"][k])) > 0.003:
                continue
            if abs(d[j] - d[k]) / max(d[j], d[k], 1e-6) > 0.10:
                continue
            if max(d[j], d[k]) > ln["rmax"] * 1.05:
                continue
            lo, hi = c[ax] - d[ax] / 2, c[ax] + d[ax] / 2
            if hi < ln["tmin"] - 0.12 or lo > ln["tmax"] + 0.12:
                continue
            ln["members"].append(p)
            taken.add(p.name)
            break

    for ln in lines:
        path = ln["seeds"][0]["onshape_path"]
        for link, sub, prop in ROLLER_PROPS:
            if link == ln["link"] and sub in path:
                ln["prop"] = prop
                break
        biggest = max(ln["seeds"], key=lambda s: info[s.name][1].length)
        ln["label"] = re.sub(r"\s*\(.*$", "", biggest["onshape_name"]).strip()
        pt = ln["pt"].copy()
        pt[ln["axis"]] = (ln["tmin"] + ln["tmax"]) / 2
        ln["pt"] = pt
    return lines


# --------------------------------------------------------------------------
# Rig
# --------------------------------------------------------------------------

# World-space rest position of every joint empty. At rest all joint values
# are zero, so each joint's rest matrix is a pure translation to its pivot.
# Parent-inverse matrices are computed from these instead of from evaluated
# matrices (drivers may not have been evaluated yet while building).
REST = {}


def rest_matrix(obj):
    return Matrix.Translation(REST.get(obj.name, Vector()))


def make_empty(name, loc, parent, col, size=0.03, display="PLAIN_AXES"):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = display
    e.empty_display_size = size
    col.objects.link(e)
    if parent is not None:
        e.parent = parent
        e.matrix_parent_inverse = rest_matrix(parent).inverted()
    e.location = loc
    REST[e.name] = Vector(loc)
    return e


def add_prop(root, prop, description, limits=None, soft=None, subtype="NONE", default=0.0):
    root[prop] = float(default)
    ui = root.id_properties_ui(prop)
    kw = dict(description=description, subtype=subtype, precision=3, step=10, default=float(default))
    if limits:
        kw.update(min=limits[0], max=limits[1], soft_min=limits[0], soft_max=limits[1])
    elif soft:
        kw.update(soft_min=soft[0], soft_max=soft[1])
    ui.update(**kw)


def add_driver(obj, path, index, root, prop, expr, extra_vars=()):
    fc = obj.driver_add(path, index)
    drv = fc.driver
    drv.type = "SCRIPTED"
    v = drv.variables.new()
    v.name = "v"
    v.type = "SINGLE_PROP"
    v.targets[0].id = root
    v.targets[0].data_path = f'["{prop}"]'
    for name, target, data_path in extra_vars:
        ev = drv.variables.new()
        ev.name = name
        ev.type = "SINGLE_PROP"
        ev.targets[0].id = target
        ev.targets[0].data_path = data_path
    drv.expression = expr
    drv.use_self = False
    return drv


def parent_keep(child, parent):
    """Parent an unparented part to a joint without moving it (joint at rest)."""
    mw = child.matrix_basis.copy()
    child.parent = parent
    child.matrix_parent_inverse = rest_matrix(parent).inverted()
    child.matrix_basis = mw


# HoodSubsystem logs Pose3d(-0.1181, -0.0972 - 0.0453, 0.4953 + tan(a) * 0.0453)
# rotated by -a about X: a rotation about a reference point 45.3 mm off the
# true pivot plus a compensating shift. The true pivot is therefore
# (y, z) = (-0.0972, 0.4953), which is the flywheel shaft. Snap to the
# detected flywheel axis when one is within 5 mm.
HOOD_PIVOT_YZ = (-0.0972, 0.4953)


def find_hood_pivot(hood_parts, lines):
    mn, mx = world_bbox(hood_parts)
    y, z = HOOD_PIVOT_YZ
    for ln in lines:
        if ln["link"] == "turret" and ln["axis"] == 0:
            if math.hypot(ln["pt"].y - y, ln["pt"].z - z) < 0.005:
                y, z = ln["pt"].y, ln["pt"].z
    return Vector(((mn.x + mx.x) / 2, y, z))


def build_rig(parts, args, rig_col):
    root = bpy.data.objects.new("ARTEMIS", None)
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 0.5
    rig_col.objects.link(root)
    root["robot"] = "FRC 4915 Spartronics - ARTEMIS (2026)"
    root["frame"] = "WPILib robot frame: +X forward, +Y left, +Z up, metres"
    REST[root.name] = Vector()

    by_link = defaultdict(list)
    for p in parts:
        by_link[p["link"]].append(p)

    joints = {}
    joint_specs = []  # for the JSON summary

    def revolute(name, parent, pivot, axis, prop, offset, sign, extra=None, display_size=0.05):
        e = make_empty("J_" + name, pivot, parent, rig_col, size=display_size)
        idx = "XYZ".index(axis)
        e.lock_location = (True, True, True)
        e.lock_rotation = tuple(i != idx for i in range(3))
        e.lock_scale = (True, True, True)
        if offset:
            expr = f"{sign}*radians(v-({offset}))"
        else:
            expr = f"{sign}*radians(v)" if sign != 1 else "radians(v)"
        extra_vars = ()
        if extra == "ratio":
            e["ratio"] = 1.0
            e.id_properties_ui("ratio").update(
                description="Speed ratio relative to the control property (negative = reverse).",
                soft_min=-10, soft_max=10)
            expr = f"({expr})*ratio"
            extra_vars = (("ratio", e, '["ratio"]'),)
        add_driver(e, "rotation_euler", idx, root, prop, expr, extra_vars)
        joints[name] = e
        joint_specs.append(dict(name=name, type="revolute",
                                parent=parent.name if parent else None,
                                origin=list(pivot), axis=axis, control=prop,
                                angle_rad=f"{expr} (v = ARTEMIS['{prop}'])"))
        return e

    # --- code-defined joints -------------------------------------------
    for name, (pname, kind, pivot, axis, prop, offset, sign, limits, doc) in CODE_JOINTS.items():
        if not by_link.get(name):
            log(f"warning: no parts for joint {name}")
            continue
        add_prop(root, prop, doc, limits=limits, default=offset if name == "intake_arm" else 0.0)
        revolute(name, root, Vector(pivot), axis, prop, offset, sign, display_size=0.08)

    lines = detect_rollers(parts) if not args.no_rollers else []

    # --- hood (child of turret) -------------------------------------------
    if by_link.get("hood"):
        pivot = find_hood_pivot(by_link["hood"], lines)
        add_prop(root, "hood_deg",
                 "Hood angle (HoodSubsystem position), degrees. 0 = down (CAD pose), 40 = max.",
                 limits=HOOD_LIMITS)
        revolute("hood", joints["turret"], pivot, "X", "hood_deg", 0.0, -1, display_size=0.06)

    # --- climber (prismatic) ------------------------------------------
    if by_link.get("climber"):
        tube = [p for p in by_link["climber"] if p["onshape_name"] == "Inner Tube"]
        mn, mx = world_bbox(tube or by_link["climber"])
        pivot = Vector(((mn.x + mx.x) / 2, (mn.y + mx.y) / 2, mn.z))
        add_prop(root, "climber_m",
                 f"Climber extension in metres (0 = retracted CAD pose; travel ~{CLIMBER_TRAVEL_M} m is an estimate).",
                 limits=(0.0, CLIMBER_TRAVEL_M), subtype="DISTANCE")
        e = make_empty("J_climber", pivot, root, rig_col, size=0.08, display="SINGLE_ARROW")
        e.lock_location = (True, True, False)
        e.lock_rotation = (True, True, True)
        add_driver(e, "location", 2, root, "climber_m", f"{pivot.z:.6f}+v")
        joints["climber"] = e
        joint_specs.append(dict(name="climber", type="prismatic", parent=root.name,
                                origin=list(pivot), axis="Z", control="climber_m",
                                travel=[0.0, CLIMBER_TRAVEL_M]))

    # --- swerve modules -------------------------------------------------
    for corner in CORNERS:
        steer_parts = by_link.get("steer_" + corner, [])
        wheel_parts = by_link.get("wheel_" + corner, [])
        if not steer_parts or not wheel_parts:
            log(f"warning: swerve {corner} incomplete")
            continue
        wc, _ = bbox_dims(wheel_parts[0])
        smn, smx = world_bbox(steer_parts)
        axis_pt = Vector(((smn.x + smx.x) / 2, (smn.y + smx.y) / 2, wc.z))
        # wheel axle direction from vertex principal axes
        axle = principal_axle(wheel_parts[0])
        heading = math.degrees(math.atan2(-axle.x, axle.y))  # rolling direction angle
        heading = (heading + 90) % 180 - 90
        # rotate module parts so that the rest pose faces forward (heading 0)
        rot = (Matrix.Translation(axis_pt) @ Matrix.Rotation(math.radians(-heading), 4, "Z")
               @ Matrix.Translation(-axis_pt))
        for p in steer_parts + wheel_parts:
            p.matrix_basis = rot @ p.matrix_basis
        bpy.context.view_layer.update()
        wc, _ = bbox_dims(wheel_parts[0])
        add_prop(root, f"steer_{corner}_deg",
                 f"{corner} swerve module azimuth, degrees CCW (0 = wheel rolling forward).",
                 soft=(-180, 180))
        add_prop(root, f"drive_{corner}_deg",
                 f"{corner} wheel rotation, degrees (positive rolls the robot forward when steer = 0). "
                 f"distance = radians * {0.0508:.4f} m.",
                 soft=(-3600, 3600))
        st = revolute("steer_" + corner, root, axis_pt, "Z", f"steer_{corner}_deg", 0.0, 1)
        wh = revolute("wheel_" + corner, st, Vector((axis_pt.x, axis_pt.y, wc.z)), "Y",
                      f"drive_{corner}_deg", 0.0, 1)
        joints["steer_" + corner], joints["wheel_" + corner] = st, wh
        log(f"swerve {corner}: axis at ({axis_pt.x:+.3f},{axis_pt.y:+.3f}), CAD heading {heading:+.1f} deg")

    # --- rollers ----------------------------------------------------------
    used_props = set()
    counters = defaultdict(int)
    for ln in lines:
        link = ln["link"]
        parent = joints.get(link, root)
        counters[link] += 1
        name = f"roller_{link}_{counters[link]:02d}"
        if ln["prop"] not in used_props:
            add_prop(root, ln["prop"], ROLLER_PROP_DOCS[ln["prop"]], soft=(-36000, 36000))
            used_props.add(ln["prop"])
        e = revolute(name, parent, ln["pt"], "XYZ"[ln["axis"]], ln["prop"], 0.0, 1,
                     extra="ratio", display_size=0.03)
        e["label"] = ln["label"]
        for m in ln["members"]:
            m["link"] = name
            m["roller_of"] = link
        log(f"{name}: {len(ln['members'])} parts on {'XYZ'[ln['axis']]}-axis, "
            f"seed '{ln['label']}', control {ln['prop']}")

    # --- parent parts -------------------------------------------------------
    for p in parts:
        j = joints.get(p["link"])
        parent_keep(p, j if j is not None else root)
    root.update_tag()
    bpy.context.view_layer.update()
    return root, joints, joint_specs


def principal_axle(o):
    """Axle direction of a wheel: eigenvector of the vertex covariance whose
    eigenvalue is least like the other two (rotational symmetry axis)."""
    import numpy as np
    n = len(o.data.vertices)
    co = np.empty(n * 3, dtype=np.float64)
    o.data.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    mw = np.array(o.matrix_world)
    co = co @ mw[:3, :3].T
    co -= co.mean(axis=0)
    w, v = np.linalg.eigh(np.cov(co.T))
    gaps = [abs(w[i] - w[(i + 1) % 3]) + abs(w[i] - w[(i + 2) % 3]) for i in range(3)]
    a = Vector(v[:, int(np.argmax(gaps))])
    a.z = 0.0  # swerve wheels: axle is horizontal
    return a.normalized()


# --------------------------------------------------------------------------
# Collections, studio, demo animation
# --------------------------------------------------------------------------

def organise_collections(parts, robot_col):
    cols = {}
    for o in parts:
        top = o["onshape_path"].split(" / ")[0] if o["onshape_path"] else ""
        label = "Frame & Panels"
        for key, lab in SUBSYSTEMS:
            if top.startswith(key):
                label = lab
        if o["source"] == "model_1.glb" or o["source"] == "model_2.glb":
            label = "Shooter"
        if o.get("is_hardware"):
            label = "Hardware"
        if label not in cols:
            cols[label] = link_collection(label, robot_col)
        move_to_collection(o, cols[label])


NUMBER_SIDE_X = -0.20   # centre of the side numbers along the robot (rear half)
NUMBER_BACK_Y = -0.22   # centre of the back number (right half of the back bumper)


def add_bumper_numbers(parts, root, text):
    """White team numbers on the flat face of the side and back bumpers
    (the CAD bumpers have none; the real ones do)."""
    bump = [p for p in parts if p["onshape_path"].startswith("91 - 2026 Bumpers")
            and any(s.material and s.material.name == "Bumper Fabric" for s in p.material_slots)]
    if not bump:
        return
    mn, mx = world_bbox(bump)
    zc = (mn.z + mx.z) / 2
    height = 0.66 * (mx.z - mn.z)  # stays on the flat part of the fabric
    cu = bpy.data.curves.new("BumperNumberText", "FONT")
    cu.body = text
    cu.align_x, cu.align_y = "CENTER", "CENTER"
    cu.offset = 0.035  # embolden the built-in font
    tmp = bpy.data.objects.new("tmp_text", cu)
    bpy.context.scene.collection.objects.link(tmp)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
    bpy.data.objects.remove(tmp)
    bpy.data.curves.remove(cu)
    me.name = "Bumper Number"
    pts = [v.co for v in me.vertices]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), 0))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), 0))
    s = height / (hi.y - lo.y)
    me.transform(Matrix.Scale(s, 4) @ Matrix.Translation(-(lo + hi) / 2))
    me.materials.append(materials.bumper_number_material())
    col = bump[0].users_collection[0]
    gap = 0.0008
    r90 = math.radians(90)
    # As on the real robot (team photo and reveal video): on both sides toward
    # the rear, and on the right half of the back bumper (the back is split in
    # the middle around the climber).
    xs, yb = NUMBER_SIDE_X, NUMBER_BACK_Y
    placements = {
        "L": ((xs, mx.y + gap, zc), (r90, 0.0, math.pi)),
        "R": ((xs, mn.y - gap, zc), (r90, 0.0, 0.0)),
        "B": ((mn.x - gap, yb, zc), (r90, 0.0, -r90)),
    }
    for side, (loc, rot) in placements.items():
        o = bpy.data.objects.new(f"Bumper Number {side}", me)
        col.objects.link(o)
        o.location = loc
        o.rotation_euler = rot
        o.parent = root
        o["onshape_name"] = "Bumper Number"
    log(f"added bumper numbers '{text}' ({height * 1000:.0f} mm tall)")


def build_studio(studio_col):
    scene = bpy.context.scene
    # floor
    bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, 0))
    floor = bpy.context.active_object
    floor.name = "Floor"
    move_to_collection(floor, studio_col)
    floor.data.materials.append(materials.floor_material())

    def area(name, loc, target, power, size):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy = power
        ld.size = size
        lo = bpy.data.objects.new(name, ld)
        studio_col.objects.link(lo)
        lo.location = loc
        look_at(lo, target)
        return lo

    tgt = Vector((0, 0, 0.3))
    area("Key", (2.2, 1.8, 2.4), tgt, 450, 1.2)
    area("Fill", (1.2, -2.6, 1.2), tgt, 90, 2.5)
    area("Rim", (-2.6, 0.4, 2.0), tgt, 400, 1.0)

    cams = {
        "CAM_Hero": ((2.05, 1.55, 1.05), (0.0, 0.0, 0.27), 50),
        "CAM_Front": ((2.6, 0.0, 0.55), (0.0, 0.0, 0.28), 50),
        "CAM_Side": ((0.0, 2.6, 0.55), (0.0, 0.0, 0.28), 50),
        "CAM_RearHigh": ((-1.8, -1.6, 1.7), (0.0, 0.0, 0.25), 50),
        "CAM_Top": ((0.0, 0.0, 3.2), (0.0, 0.0, 0.0), 35),
    }
    for name, (loc, target, lens) in cams.items():
        cd = bpy.data.cameras.new(name)
        cd.lens = lens
        co = bpy.data.objects.new(name, cd)
        studio_col.objects.link(co)
        co.location = loc
        look_at(co, Vector(target))
    scene.camera = bpy.data.objects["CAM_Hero"]

    world = bpy.data.worlds.new("Studio World")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.62, 0.64, 0.68, 1.0)
    bg.inputs["Strength"].default_value = 0.25
    scene.world = world

    scene.render.engine = "CYCLES"
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.fps = 30
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"


def look_at(obj, target):
    d = Vector(target) - obj.location
    obj.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def demo_animation(root):
    """A 10 s demo: deploy intake, spin up, aim, raise hood, climb, steer."""
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 1, 300
    prefs = bpy.context.preferences.edit
    old = prefs.keyframe_new_interpolation_type

    def key(prop, frames):
        if prop not in root.keys():
            return
        for f, v in frames:
            root[prop] = float(v)
            root.keyframe_insert(data_path=f'["{prop}"]', frame=f)

    prefs.keyframe_new_interpolation_type = "BEZIER"
    key("intake_pivot_deg", [(1, 130), (30, 130), (60, 0), (240, 0), (275, 130)])
    key("turret_deg", [(1, 0), (90, 0), (130, 70), (170, -45), (210, 0)])
    key("hood_deg", [(1, 0), (100, 0), (140, 30), (190, 12), (230, 0)])
    key("climber_m", [(1, 0), (230, 0), (260, CLIMBER_TRAVEL_M * 0.9), (290, CLIMBER_TRAVEL_M * 0.9), (300, 0.0)])
    for c, ang in zip(CORNERS, (45, -45, -45, 45)):
        key(f"steer_{c}_deg", [(1, 0), (60, 0), (80, ang), (110, ang), (130, 0)])
    prefs.keyframe_new_interpolation_type = "LINEAR"
    key("intake_rollers_deg", [(1, 0), (40, 0), (240, 360 * 2 * 200 / 30)])
    key("shooter_wheels_deg", [(1, 0), (80, 0), (300, 360 * 4 * 220 / 30)])
    key("feeder_rollers_deg", [(1, 0), (120, 0), (300, 360 * 3 * 180 / 30)])
    key("indexer_rollers_deg", [(1, 0), (120, 0), (300, 360 * 3 * 180 / 30)])
    key("spindexer_deg", [(1, 0), (120, 0), (300, -360 * 1 * 180 / 30)])
    for c in CORNERS:
        key(f"drive_{c}_deg", [(1, 0), (130, 0), (300, 360 * 6)])
    prefs.keyframe_new_interpolation_type = old
    if root.animation_data and root.animation_data.action:
        root.animation_data.action.name = "Demo_Mechanisms"
    scene.frame_set(1)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    if bpy.app.background is False and "--" not in sys.argv:
        argv = []
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--assets", default=os.path.join(HERE, "source"),
                    help="folder with model.glb and model_0..3.glb (default: ./source)")
    ap.add_argument("--out", default=os.path.join(HERE, "ARTEMIS.blend"), help="output .blend")
    ap.add_argument("--keep-hardware", action="store_true", help="keep screws, nuts, rivets, washers")
    ap.add_argument("--no-rollers", action="store_true", help="do not rig spinning rollers")
    ap.add_argument("--no-studio", action="store_true", help="skip floor, lights, cameras")
    ap.add_argument("--no-demo", action="store_true", help="skip the demo animation")
    ap.add_argument("--report", default=None, help="write a JSON joint/part summary here")
    ap.add_argument("--team-number", default="4915",
                    help="bumper numbers to add on the side and back bumpers ('' for none)")
    return ap.parse_args(argv)


def main():
    args = parse_args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    robot_col = link_collection("ARTEMIS", scene.collection)
    rig_col = link_collection("Rig", robot_col)

    parts = import_glb(os.path.join(args.assets, "model.glb"), "chassis")
    for fname, link in COMPONENT_FILES.items():
        path = os.path.join(args.assets, fname)
        if os.path.exists(path):
            parts += import_glb(path, link)
        else:
            log(f"warning: {path} missing; {link} will not be rigged")
    log(f"imported {len(parts)} parts")

    kept, hardware, removed = [], [], 0
    for o in parts:
        n = o["onshape_name"]
        if HELPER_RE.search(n):
            bpy.data.objects.remove(o)
            removed += 1
        elif HARDWARE_RE.search(n):
            if args.keep_hardware:
                o["is_hardware"] = True
                hardware.append(o)
            else:
                bpy.data.objects.remove(o)
                removed += 1
        else:
            kept.append(o)
    log(f"removed {removed} hardware/helper parts, {len(kept)} parts remain")

    assign_links(kept)
    if hardware:
        attach_hardware(hardware, kept)
    parts = kept + hardware
    for o in parts:
        recenter_origin(o)
    bpy.context.view_layer.update()

    root, joints, specs = build_rig(parts, args, rig_col)
    for o in parts:
        o.data.name = o.name
    organise_collections(parts, robot_col)

    materials.assign_all(parts, root)
    if args.team_number:
        add_bumper_numbers(parts, root, args.team_number)

    if not args.no_studio:
        build_studio(link_collection("Studio", scene.collection))
    if not args.no_demo:
        demo_animation(root)
    else:
        scene.frame_set(1)

    bpy.data.orphans_purge(do_recursive=True)
    counts = defaultdict(int)
    for o in parts:
        counts[o.parent.name if o.parent else "-"] += 1
    log("parts per joint: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in parts)
    log(f"{len(parts)} parts, {tris:,} triangles, {len(joints)} joints")

    if args.report:
        import json
        with open(args.report, "w") as f:
            json.dump(dict(joints=specs,
                           controls={k: root.id_properties_ui(k).as_dict() for k in root.keys()
                                     if isinstance(root[k], float)}),
                      f, indent=2)
    out = os.path.abspath(args.out)
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)
    log(f"saved {out}")


if __name__ == "__main__":
    main()
