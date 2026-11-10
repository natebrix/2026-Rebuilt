"""
approx_panels.py - parts of the real robot that are missing from the May 2026
CAD export, approximated as flat 1/8" smoked-polycarbonate panels.

Sources (best effort, no CAD geometry available):
  * Intake hopper extension. On the robot, the deployed intake forms a tall box
    with a front wall and two side walls. They are Parts 34 (front) and 32/35
    (sides) of "32 - 2026 Intake V2" in the newer Onshape version. Their sizes
    come from the BOM masses (1.78 lb and 0.955 lb of polycarbonate at 1/8"
    gives 0.21 m^2 and 0.11 m^2). Their placement and shape come from the
    reveal video (0:24, 0:31, 0:44, 0:45) and the team photo.
    How they retract is not visible anywhere. Swinging them rigidly with the
    intake arm would put them through the drivebase and the turret, so they
    are modelled as a sliding extension: the front wall stays upright and
    slides 24 cm forward as the intake deploys (joint J_hopper_ext, driven
    by intake_pivot_deg), and the side walls telescope with it (a Hook
    modifier moves their front edge).
  * Back walls. Seen in the reveal video at ~0:14: the left half is full height,
    the right half stops below the turret so the shooter can swing over it,
    and the climber tube sits in the gap between them.

All panels go in the "Approximate (not in CAD)" collection and carry an
"approximate" custom property, so they are easy to hide or replace once a
newer CAD export is available.
"""

import bmesh
import bpy
from mathutils import Matrix, Vector

THICKNESS = 0.003175           # 1/8" polycarbonate

# Hopper extension, robot frame, intake DEPLOYED. Side walls sit just outside
# the main hopper walls and the front truss towers (|y| <= 0.362).
EXT_SIDE_Y = 0.3645
EXT_SIDE_XZ = [(0.30, 0.16), (0.60, 0.215), (0.68, 0.50), (0.30, 0.50)]  # rear-bottom first
EXT_SIDE_FRONT = (1, 2)        # indices of the outline points on the sliding front edge
EXT_FRONT_BOTTOM_XZ = (0.60, 0.215)   # just above the front roller
EXT_FRONT_TOP_XZ = (0.68, 0.50)
EXT_TRAVEL = 0.24              # front wall slides this far between STOW and deployed
# Slide fraction vs intake angle: 0 at STOW (130 deg), 1 from 30 deg down to
# READY (0 deg), so the wall stays ahead of the swinging rollers.
EXT_FULL_AT_DEG = 30.0
STOW_DEG = 130.0

# Back walls, robot frame: (y0, y1, z0, z1) on the plane x = BACK_X.
BACK_X = -0.337                # just behind the frame, inside the bumper
BACK_LEFT = (0.045, 0.358, 0.178, 0.533)
BACK_RIGHT = (-0.358, 0.004, 0.178, 0.355)


def _slab(name, outline, normal, thickness=THICKNESS):
    """Closed prism from a planar outline, centred on the outline's plane.
    Vertex i (top face) and i + len(outline) (bottom face) come from outline[i]."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    h = Vector(normal).normalized() * (thickness / 2)
    top = [bm.verts.new(Vector(p) + h) for p in outline]
    bot = [bm.verts.new(Vector(p) - h) for p in outline]
    bm.faces.new(top)
    bm.faces.new(list(reversed(bot)))
    n = len(outline)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new([top[i], top[j], bot[j], bot[i]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    return me


def add(rig, material):
    """Create the panels and attach them to the rig.

    rig: namespace with root, rig_col, robot_col, rest (dict of joint rest
    positions), make_empty, add_driver, parent_keep, recenter.
    Returns (objects, joint_spec).
    """
    col = bpy.data.collections.get("Approximate (not in CAD)") or \
        bpy.data.collections.new("Approximate (not in CAD)")
    if col.name not in rig.robot_col.children:
        rig.robot_col.children.link(col)

    def make(name, outline, normal, path, parent):
        me = _slab(name, outline, normal)
        me.materials.append(material)
        o = bpy.data.objects.new(name, me)
        col.objects.link(o)
        o["onshape_name"] = name
        o["onshape_path"] = path
        o["source"] = "approx_panels.py"
        o["approximate"] = True
        rig.recenter(o)
        bpy.context.view_layer.update()
        rig.parent_keep(o, parent)
        return o

    made = []
    # sliding joint for the hopper extension (rest = STOW, i.e. retracted)
    (xb, zb), (xt, zt) = EXT_FRONT_BOTTOM_XZ, EXT_FRONT_TOP_XZ
    rest_pt = Vector((xb - EXT_TRAVEL, 0.0, zb))
    j = rig.make_empty("J_hopper_ext", rest_pt, rig.root, rig.rig_col, size=0.06,
                       display="SINGLE_ARROW")
    j.lock_location = (False, True, True)
    j.lock_rotation = (True, True, True)
    span = STOW_DEG - EXT_FULL_AT_DEG
    rig.add_driver(j, "location", 0, rig.root, "intake_pivot_deg",
                   f"{rest_pt.x:.6f}+{EXT_TRAVEL}*clamp(({STOW_DEG}-v)/{span},0,1)")
    j["approximate"] = True
    shift = Matrix.Translation((-EXT_TRAVEL, 0, 0))   # deployed -> retracted

    # front wall rides on the joint; it sits between the side walls
    y = EXT_SIDE_Y - THICKNESS
    front = [shift @ Vector(p) for p in ((xb, -y, zb), (xb, y, zb), (xt, y, zt), (xt, -y, zt))]
    nrm = Vector((zt - zb, 0, -(xt - xb))).normalized()
    made.append(make("Intake Hopper Front Wall (approx.)", front, nrm,
                     "32 - 2026 Intake V2", j))

    # side walls: rear edge fixed to the robot, front edge hooked to the joint
    for side, sy in (("Left", EXT_SIDE_Y), ("Right", -EXT_SIDE_Y)):
        pts = []
        for i, (x, z) in enumerate(EXT_SIDE_XZ):
            p = Vector((x, sy, z))
            pts.append(shift @ p if i in EXT_SIDE_FRONT else p)
        o = make(f"Intake Hopper Side Wall {side} (approx.)", pts, (0, 1, 0),
                 "32 - 2026 Intake V2", rig.root)
        n = len(EXT_SIDE_XZ)
        idx = [i for i in EXT_SIDE_FRONT] + [i + n for i in EXT_SIDE_FRONT]
        hook = o.modifiers.new("HopperSlide", "HOOK")
        hook.object = j
        hook.vertex_indices_set(idx)
        # zero effect at rest: parentinv = hook_rest^-1 * object
        hook.matrix_inverse = Matrix.Translation(rig.rest[j.name]).inverted() @ o.matrix_basis
        made.append(o)

    for side, (y0, y1, z0, z1) in (("Left", BACK_LEFT), ("Right", BACK_RIGHT)):
        made.append(make(f"Back Wall {side} (approx.)",
                         [(BACK_X, y0, z0), (BACK_X, y1, z0), (BACK_X, y1, z1), (BACK_X, y0, z1)],
                         (1, 0, 0), "", rig.root))

    spec = dict(name="hopper_ext", type="prismatic", parent=rig.root.name,
                origin=list(rest_pt), axis="X", control="intake_pivot_deg",
                travel=[0.0, EXT_TRAVEL], approximate=True,
                note=f"x = rest + {EXT_TRAVEL} * clamp(({STOW_DEG} - v) / {span}, 0, 1)")
    return made, spec
