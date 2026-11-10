#!/usr/bin/env python3
"""
render_previews.py - render stills of ARTEMIS.blend in a given pose.

    blender -b ARTEMIS.blend --python render_previews.py -- \
        --out previews/hero.png --camera CAM_Hero --engine cycles --samples 64 \
        --pose intake_pivot_deg=0 turret_deg=45 hood_deg=20 climber_m=0.2

or with the bpy module:  python render_previews.py --blend ARTEMIS.blend --out ...

--frame N renders a frame of the demo animation instead of a fixed pose.
--engine is one of: cycles, eevee, workbench (workbench = fast clay check).
"""

import argparse
import os
import sys

import bpy


def parse():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--blend", default=None, help="open this .blend first (bpy-module use)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", default="CAM_Hero")
    ap.add_argument("--engine", default="cycles", choices=["cycles", "eevee", "workbench"])
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--res", default="1600x900")
    ap.add_argument("--frame", type=int, default=None)
    ap.add_argument("--pose", nargs="*", default=[], help="prop=value pairs on the ARTEMIS object")
    ap.add_argument("--color-by", default=None, choices=[None, "joint", "random"],
                    help="workbench only: colour parts by joint or randomly")
    ap.add_argument("--hide", nargs="*", default=[], help="collections to hide")
    ap.add_argument("--cam-from", default=None, help="x,y,z: place a temporary camera here (robot frame)")
    ap.add_argument("--cam-to", default="0,0,0.3", help="x,y,z: point the temporary camera here")
    ap.add_argument("--lens", type=float, default=50.0)
    return ap.parse_args(argv)


def main():
    a = parse()
    if a.blend:
        bpy.ops.wm.open_mainfile(filepath=a.blend)
    sc = bpy.context.scene
    root = bpy.data.objects["ARTEMIS"]
    if a.frame is not None:
        sc.frame_set(a.frame)
    else:
        if root.animation_data:
            root.animation_data.action = None
        for kv in a.pose:
            k, v = kv.split("=")
            root[k] = float(v)
        sc.frame_set(sc.frame_current)
    for name in a.hide:
        col = bpy.data.collections.get(name)
        if col:
            col.hide_render = True
    if a.cam_from:
        from mathutils import Vector
        cd = bpy.data.cameras.new("CAM_Temp")
        cd.lens = a.lens
        cam = bpy.data.objects.new("CAM_Temp", cd)
        sc.collection.objects.link(cam)
        cam.location = Vector([float(v) for v in a.cam_from.split(",")])
        d = Vector([float(v) for v in a.cam_to.split(",")]) - cam.location
        cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        sc.camera = cam
    else:
        sc.camera = bpy.data.objects[a.camera]
    w, h = (int(x) for x in a.res.split("x"))
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.resolution_percentage = 100
    if a.engine == "cycles":
        sc.render.engine = "CYCLES"
        sc.cycles.samples = a.samples
        sc.cycles.use_denoising = True
        sc.cycles.device = "CPU"
    elif a.engine == "eevee":
        sc.render.engine = "BLENDER_EEVEE"
        sc.eevee.taa_render_samples = a.samples
    else:
        sc.render.engine = "BLENDER_WORKBENCH"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.show_cavity = True
        sh.cavity_type = "BOTH"
        if a.color_by == "random":
            sh.color_type = "RANDOM"
        elif a.color_by == "joint":
            sh.color_type = "OBJECT"
            import colorsys
            import zlib
            for o in bpy.data.objects:
                if o.type == "MESH" and o.parent:
                    hue = (zlib.crc32(o.parent.name.encode()) % 997) / 997.0
                    o.color = (*colorsys.hsv_to_rgb(hue, 0.65, 0.95), 1.0)
                    if o.parent.name == "ARTEMIS":
                        o.color = (0.75, 0.75, 0.75, 1.0)
        else:
            sh.color_type = "MATERIAL"
    sc.render.filepath = os.path.abspath(a.out)
    bpy.ops.render.render(write_still=True)
    print("[artemis] wrote", sc.render.filepath, flush=True)


if __name__ == "__main__":
    main()
