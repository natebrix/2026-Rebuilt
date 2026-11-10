"""
materials.py - realistic procedural materials for the ARTEMIS Blender build.

Every imported Onshape material slot is replaced with a physically based
material picked from (1) the part name (vendor parts are named precisely:
"Kraken X60", "2\" Stealth Wheel", "HDPE Skid", ...) and (2) the Onshape
appearance colour of that slot.

To match the real robot, edit COLOR_OVERRIDES / NAME_RULES below and rebuild.
No image textures are used, so the .blend is self-contained.
"""

import colorsys
import re

import bpy

# --------------------------------------------------------------------------
# Tunables
# --------------------------------------------------------------------------

# Onshape appearance colour (hex, sRGB-ish as exported) -> (material class, real colour or None)
# The CAD uses colour-coding (pure red/green/yellow) on team-made parts; map
# those to what the parts really look like. None keeps the CAD colour.
COLOR_OVERRIDES = {
    # filled in from robot photos
}

# (regex on part name, material class). First match wins. Classes are below.
NAME_RULES = [
    (r"Brass", "brass"),
    (r"Belt\b", "rubber"),
    (r"(?<![A-Za-z])Chain\b", "steel"),
    (r"Black Oxide|Black Zinc|Black-Oxide", "black_oxide"),
    (r"Rivet", "aluminum"),
    (r"Spring", "steel"),
    (r"Molded Wheel|Tread", "wheel"),
    (r"Flex Wheel|Squish Wheel|Stealth Wheel|Urethane Wheel|Compliant|Sushi", "urethane"),
    (r"Bearing(?! Retention)(?! Plate)(?! Race)", "bearing"),
    (r"Steel(?! Insert)", "steel"),
    (r"Kraken|MAXPlanetary|MAX 90|Input Coupler", "vendor_metal"),
    (r"HDPE|Skid", "hdpe"),
    (r"Acetal|Delrin", "acetal"),
    (r"Polycarb|Lexan|Window", "polycarb"),
    (r"3D ?print|3DP|Printed|Print\b|Print Adapter|Omni Wheel Spacer|Center Cone|Cable Snake", "print"),
    (r"Battery|roboRIO|PDP|Radio(?! Plate)|Pigeon|Limelight|Luma|Breaker|Encoder|terminal", "electronics"),
    (r"Rounded Hex|Hex Shaft|ThunderHex|Churro|Stub Shaft|Shaft\b|Spline", "aluminum"),
    (r"SDS MK5|WCP-|Aluminum|Tube|Driverail|Bellypan|Gusset|Plate|Gear|Pulley|Sprocket|Hub|Spacer|"
     r"Standoff|Nut Strip|Collar|Clamp|Bracket|Mount|Block|Echain|Tensioner", "cad_metal"),
]

# (substring of the Onshape subassembly path, material class)
SUBSYSTEM_RULES = [
    ("91 - 2026 Bumpers", "bumper"),
    ("PDP 2.0", "electronics"),
]

PRINT_LAYER_MM = 0.25   # visible layer height on printed parts
BUMPER_RED = (0.32, 0.008, 0.01)
BUMPER_BLUE = (0.006, 0.026, 0.20)   # royal blue, from the team photo
SMOKE_TINT = (0.33, 0.325, 0.32)     # per-surface transmission tint of smoked polycarb

# Near-black team-made sheets (CAD colour ~#0d0d0d) at least this big are the
# smoked polycarbonate walls seen in the photo.
SMOKED_MAX_THICKNESS_M = 0.0075
SMOKED_MIN_SIZE_M = 0.15

# Per-part colour overrides from the team photo: (name regex, class, colour list).
# With several colours, instances are coloured in turn, ordered along X.
PHOTO_OVERRIDES = [
    (r'^4" Solid Urethane Wheel', "urethane", [(0.02, 0.22, 0.62), (0.03, 0.40, 0.14)]),
    (r"^4\" Compliant Star", "urethane", [(0.02, 0.10, 0.65)]),
    (r"^Echain Plate", "polycarb", [None]),
    (r"^Center Cone", "print", [(0.02, 0.02, 0.022)]),
]

# --------------------------------------------------------------------------
# Node helpers
# --------------------------------------------------------------------------

_cache = {}


def _new(name):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True  # deprecated (always on) in Blender 5+, needed for 4.x
    except Exception:
        pass
    return m


def _bsdf(m):
    return m.node_tree.nodes.get("Principled BSDF")


def _set(b, **kw):
    names = {
        "base": "Base Color", "metallic": "Metallic", "rough": "Roughness",
        "ior": "IOR", "transmission": "Transmission Weight", "coat": "Coat Weight",
        "coat_rough": "Coat Roughness", "sheen": "Sheen Weight", "sheen_rough": "Sheen Roughness",
        "subsurface": "Subsurface Weight", "aniso": "Anisotropic", "spec": "Specular IOR Level",
    }
    for k, v in kw.items():
        sock = b.inputs.get(names[k])
        if sock is None:
            continue
        if k == "base":
            v = (*v[:3], 1.0)
        sock.default_value = v


def _bump(m, kind, strength, scale, distance=0.0002, detail=4.0):
    """Add a procedural bump (noise / layer lines / weave) to a material."""
    nt = m.node_tree
    b = _bsdf(m)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = strength
    bump.inputs["Distance"].default_value = distance
    if kind == "noise":
        tex = nt.nodes.new("ShaderNodeTexNoise")
        tex.inputs["Scale"].default_value = scale
        tex.inputs["Detail"].default_value = detail
        nt.links.new(tc.outputs["Object"], tex.inputs["Vector"])
        nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
    elif kind == "layers":
        tex = nt.nodes.new("ShaderNodeTexWave")
        tex.wave_type = "BANDS"
        tex.bands_direction = "Z"
        tex.inputs["Scale"].default_value = scale
        tex.inputs["Distortion"].default_value = 0.0
        nt.links.new(tc.outputs["Object"], tex.inputs["Vector"])
        nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
    elif kind == "weave":
        a = nt.nodes.new("ShaderNodeTexWave")
        a.bands_direction = "X"
        a.inputs["Scale"].default_value = scale
        c = nt.nodes.new("ShaderNodeTexWave")
        c.bands_direction = "Y"
        c.inputs["Scale"].default_value = scale
        mul = nt.nodes.new("ShaderNodeMath")
        mul.operation = "MULTIPLY"
        nt.links.new(tc.outputs["Object"], a.inputs["Vector"])
        nt.links.new(tc.outputs["Object"], c.inputs["Vector"])
        nt.links.new(a.outputs["Fac"], mul.inputs[0])
        nt.links.new(c.outputs["Fac"], mul.inputs[1])
        nt.links.new(mul.outputs["Value"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])


def _rough_variation(m, lo, hi, scale=60.0):
    nt = m.node_tree
    b = _bsdf(m)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = scale
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.inputs["To Min"].default_value = lo
    rng.inputs["To Max"].default_value = hi
    nt.links.new(tc.outputs["Object"], tex.inputs["Vector"])
    nt.links.new(tex.outputs["Fac"], rng.inputs["Value"])
    nt.links.new(rng.outputs["Result"], b.inputs["Roughness"])


def _finish(m, color):
    m.diffuse_color = (*color[:3], 1.0)  # solid-view colour
    return m


# --------------------------------------------------------------------------
# Material library
# --------------------------------------------------------------------------

def _q(color):
    return tuple(round(c * 20) / 20 for c in color[:3])


def _color_name(color):
    return f"{_color_word(color)} {_hex(color)}"


def _color_word(color):
    r, g, b = color[:3]
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    if s < 0.15:
        return "Black" if v < 0.12 else "Dark Grey" if v < 0.3 else "Grey" if v < 0.65 else "White"
    names = [(0.03, "Red"), (0.09, "Orange"), (0.18, "Yellow"), (0.45, "Green"),
             (0.55, "Cyan"), (0.75, "Blue"), (0.92, "Purple"), (1.01, "Red")]
    for lim, n in names:
        if h < lim:
            return n
    return "Color"


COLORLESS = {"aluminum", "steel", "black_oxide", "brass", "polycarb", "fabric", "plywood",
             "polycarb_smoked"}


def make(cls, color=(0.5, 0.5, 0.5)):
    key = (cls, None if cls in COLORLESS else _q(color))
    if key in _cache:
        return _cache[key]
    builder = BUILDERS[cls]
    m = builder(color)
    _cache[key] = m
    return m


def aluminum(_c=None):
    m = _new("Aluminum - Machined")
    _set(_bsdf(m), base=(0.80, 0.81, 0.82), metallic=1.0, rough=0.3, aniso=0.3)
    _rough_variation(m, 0.24, 0.36)
    return _finish(m, (0.80, 0.81, 0.82))


def anodized(c):
    c = tuple(max(0.02, min(1.0, x)) for x in c[:3])
    m = _new(f"Aluminum - Anodized {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=1.0, rough=0.38)
    _rough_variation(m, 0.33, 0.43)
    return _finish(m, c)


def powdercoat(c):
    m = _new(f"Powder Coat {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.42, coat=0.25, coat_rough=0.3)
    _bump(m, "noise", 0.08, 900.0, 0.0001)
    return _finish(m, c)


def steel(_c=None):
    m = _new("Steel")
    _set(_bsdf(m), base=(0.56, 0.57, 0.58), metallic=1.0, rough=0.26)
    _rough_variation(m, 0.2, 0.32)
    return _finish(m, (0.56, 0.57, 0.58))


def black_oxide(_c=None):
    m = _new("Steel - Black Oxide")
    _set(_bsdf(m), base=(0.045, 0.045, 0.05), metallic=1.0, rough=0.42)
    return _finish(m, (0.045, 0.045, 0.05))


def brass(_c=None):
    m = _new("Brass")
    _set(_bsdf(m), base=(0.86, 0.64, 0.34), metallic=1.0, rough=0.24)
    _rough_variation(m, 0.18, 0.3)
    return _finish(m, (0.86, 0.64, 0.34))


def plastic(c):
    m = _new(f"Plastic - {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.45)
    return _finish(m, c)


def print3d(c):
    m = _new(f"3D Print - {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.55, subsurface=0.05)
    period_m = PRINT_LAYER_MM / 1000.0
    _bump(m, "layers", 0.15, 0.314 / period_m, 0.00008)
    return _finish(m, c)


def rubber(c=(0.02, 0.02, 0.02)):
    c = c if max(c[:3]) > 0.08 else (0.018, 0.018, 0.018)
    m = _new(f"Rubber - {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.82, spec=0.3)
    _bump(m, "noise", 0.2, 1500.0, 0.0001)
    return _finish(m, c)


def urethane(c):
    m = _new(f"Urethane - {_color_name(c)}")
    dark = max(c[:3]) < 0.1
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.62 if dark else 0.5, spec=0.35,
         subsurface=0.0 if dark else 0.12)
    return _finish(m, c)


def hdpe(c=(0.86, 0.86, 0.84)):
    c = (0.86, 0.86, 0.84) if max(c[:3]) > 0.5 else (0.035, 0.035, 0.035)
    m = _new(f"HDPE - {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.55, subsurface=0.1)
    _bump(m, "noise", 0.05, 600.0, 0.0001)
    return _finish(m, c)


def acetal(c):
    c = c if max(c[:3]) > 0.5 else (0.03, 0.03, 0.03)
    m = _new(f"Acetal - {_color_name(c)}")
    _set(_bsdf(m), base=c, metallic=0.0, rough=0.3)
    return _finish(m, c)


def polycarb(c=(0.9, 0.92, 0.95)):
    m = _new("Polycarbonate - Clear")
    _set(_bsdf(m), base=(0.92, 0.94, 0.96), metallic=0.0, rough=0.04, ior=1.585, transmission=1.0)
    try:
        m.surface_render_method = "DITHERED"
        m.use_raytrace_refraction = True
    except AttributeError:
        pass
    return _finish(m, (0.8, 0.85, 0.9))


def polycarb_smoked(_c=None):
    """Smoked/tinted polycarbonate panels (side walls, intake walls)."""
    m = _new("Polycarbonate - Smoked")
    _set(_bsdf(m), base=SMOKE_TINT, metallic=0.0, rough=0.03, ior=1.585, transmission=1.0)
    try:
        m.surface_render_method = "DITHERED"
        m.use_raytrace_refraction = True
    except AttributeError:
        pass
    return _finish(m, (0.12, 0.12, 0.12))


def fabric(_c=None):
    """Bumper cordura; red/blue switch driven by ARTEMIS['bumper_blue']."""
    m = _new("Bumper Fabric")
    nt = m.node_tree
    b = _bsdf(m)
    mix = nt.nodes.new("ShaderNodeMix")
    mix.name = "AllianceMix"
    mix.data_type = "RGBA"
    # ShaderNodeMix has float/vector/colour sockets that share names; pick the colour ones
    col_in = [sk for sk in mix.inputs if sk.type == "RGBA"]
    col_in[0].default_value = (*BUMPER_RED, 1)
    col_in[1].default_value = (*BUMPER_BLUE, 1)
    col_out = [sk for sk in mix.outputs if sk.type == "RGBA"][0]
    nt.links.new(col_out, b.inputs["Base Color"])
    _set(b, metallic=0.0, rough=0.9, sheen=0.25, sheen_rough=0.5, spec=0.2)
    _bump(m, "weave", 0.3, 400.0, 0.0004)
    return _finish(m, BUMPER_RED)


def plywood(_c=None):
    c = (0.62, 0.45, 0.27)
    m = _new("Plywood")
    nt = m.node_tree
    b = _bsdf(m)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.bands_direction = "X"
    wave.inputs["Scale"].default_value = 60.0
    wave.inputs["Distortion"].default_value = 6.0
    wave.inputs["Detail"].default_value = 3.0
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.50, 0.34, 0.19, 1)
    ramp.color_ramp.elements[1].color = (0.70, 0.53, 0.33, 1)
    nt.links.new(tc.outputs["Object"], wave.inputs["Vector"])
    nt.links.new(wave.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    _set(b, metallic=0.0, rough=0.7)
    return _finish(m, c)


def electronics(c):
    return plastic(c)


def bumper_number_material():
    m = _new("Bumper Numbers - White")
    _set(_bsdf(m), base=(0.82, 0.82, 0.82), metallic=0.0, rough=0.75, sheen=0.2)
    return _finish(m, (0.82, 0.82, 0.82))


def floor_material():
    m = _new("Studio Floor")
    _set(_bsdf(m), base=(0.32, 0.32, 0.34), metallic=0.0, rough=0.55)
    return _finish(m, (0.5, 0.5, 0.52))


BUILDERS = {
    "aluminum": aluminum, "anodized": anodized, "powdercoat": powdercoat, "steel": steel,
    "black_oxide": black_oxide, "brass": brass, "plastic": plastic, "print": print3d,
    "rubber": rubber, "urethane": urethane, "hdpe": hdpe, "acetal": acetal,
    "polycarb": polycarb, "fabric": fabric, "electronics": electronics, "plywood": plywood,
    "polycarb_smoked": polycarb_smoked,
}

# --------------------------------------------------------------------------
# Classification
# --------------------------------------------------------------------------


def _hex(c):
    return "#%02x%02x%02x" % tuple(int(round(max(0, min(1, x)) * 255)) for x in c[:3])


def _hsv(c):
    return colorsys.rgb_to_hsv(*c[:3])


def _is_plate(o):
    d = sorted(o.dimensions)
    return d[0] <= 0.0066 and d[1] >= 8 * d[0]


def _is_default_tint(h, s, v):
    """Onshape/vendor 'unpainted' light blue-grey appearances."""
    return 0.5 < h < 0.7 and s < 0.35 and v > 0.7


def resolve(cls, color, o):
    """Turn a rule class + CAD colour into a concrete (builder class, colour)."""
    h, s, v = _hsv(color)
    if cls in ("cad_metal", "guess") and _is_default_tint(h, s, v):
        return "aluminum", color
    if cls in ("cad_metal", "guess") and s < 0.05 and v > 0.93:
        # pure white team parts are plastic sheet (HDPE) or white prints
        return ("hdpe", color) if _is_plate(o) else ("print", color)
    if cls == "cad_metal":
        if s < 0.15:
            if v > 0.5:
                return "aluminum", color
            return "anodized", (0.04, 0.04, 0.045) if v < 0.2 else color
        return "anodized", color
    if cls == "vendor_metal":
        if s < 0.15 and v > 0.42:
            return "aluminum", color
        if s < 0.15:
            return "anodized", color
        return "plastic", color
    if cls == "wheel":
        return ("rubber", color) if v < 0.25 else ("plastic", color)
    if cls == "bearing":
        return ("rubber", color) if v < 0.15 else ("steel", color)
    if cls == "bumper":
        if 0.04 < h < 0.15 and s > 0.3 and v < 0.8:   # tan: plywood backing
            return "plywood", color
        if s > 0.4 and (h < 0.04 or h > 0.92 or 0.55 < h < 0.75):  # red/blue cloth
            return "fabric", color
        return resolve("guess", color, o)
    if cls == "guess":
        if _is_plate(o):
            return resolve("cad_metal", color, o)
        if s < 0.15 and v > 0.42:
            return "aluminum", color
        return "print", color
    return cls, color


def _sheet_thickness(o):
    """Mean wall thickness of a closed part: 2 * volume / surface area."""
    import numpy as np
    me = o.data
    me.calc_loop_triangles()
    nv, nt = len(me.vertices), len(me.loop_triangles)
    if nt == 0:
        return 1.0
    co = np.empty(nv * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    tri = np.empty(nt * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", tri)
    a, b, c = (co[tri.reshape(-1, 3)[:, i]] for i in range(3))
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1).sum()
    vol = abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0
    return 2.0 * vol / area if area > 0 else 1.0


def _is_smoked_sheet(o, color):
    h, s, v = _hsv(color)
    if v > 0.1 or s > 0.3:
        return False
    if max(o.dimensions) < SMOKED_MIN_SIZE_M:
        return False
    if "_t" not in o:
        o["_t"] = _sheet_thickness(o)
    return o["_t"] <= SMOKED_MAX_THICKNESS_M


_override_index = {}


def classify(o, color):
    name = o.get("onshape_name", o.name)
    hx = _hex(color)
    for pat, cls, colours in PHOTO_OVERRIDES:
        if re.search(pat, name):
            i = _override_index.get(o.name, 0)
            c = colours[i % len(colours)]
            return cls, (c if c is not None else color)
    if hx in COLOR_OVERRIDES:
        cls, real = COLOR_OVERRIDES[hx]
        return cls, real or color
    path = o.get("onshape_path", "")
    for sub, cls in SUBSYSTEM_RULES:
        if sub in path:
            return resolve(cls, color, o)
    for pat, cls in NAME_RULES:
        if re.search(pat, name, re.I):
            r = resolve(cls, color, o)
            if r[0] in ("anodized", "print") and _is_smoked_sheet(o, color):
                return "polycarb_smoked", color
            return r
    r = resolve("guess", color, o)
    if r[0] in ("anodized", "print", "hdpe") and _is_smoked_sheet(o, color):
        return "polycarb_smoked", color
    return r


def _cad_color(mat):
    if mat is None or not mat.node_tree:
        return (0.6, 0.6, 0.6)
    b = _bsdf(mat)
    if b is None:
        return tuple(mat.diffuse_color[:3])
    return tuple(b.inputs["Base Color"].default_value[:3])


def assign_all(parts, root):
    stats = {}
    old_mats = set()
    for pat, _cls, colours in PHOTO_OVERRIDES:
        if len(colours) > 1:
            hits = sorted((o for o in parts if re.search(pat, o.get("onshape_name", ""))),
                          key=lambda o: o.matrix_world.translation.x)
            for i, o in enumerate(hits):
                _override_index[o.name] = i
    for o in parts:
        if not o.material_slots:
            o.data.materials.append(None)
        for slot in o.material_slots:
            old = slot.material
            old_mats.add(old)
            color = _cad_color(old)
            cls, real = classify(o, color)
            new = make(cls, real)
            slot.link = "DATA"
            slot.material = new
            stats[new.name] = stats.get(new.name, 0) + 1
            o["cad_color"] = _hex(color)
    for m in old_mats:
        if m is not None and m.users == 0:
            bpy.data.materials.remove(m)

    # alliance colour switch for the bumpers
    root["bumper_blue"] = 1.0
    root.id_properties_ui("bumper_blue").update(
        min=0.0, max=1.0, soft_min=0.0, soft_max=1.0,
        description="Bumper colour: 0 = red alliance, 1 = blue alliance (default, as in the team photo).")
    for (cls, _), m in _cache.items():
        if cls != "fabric":
            continue
        fc = m.node_tree.driver_add('nodes["AllianceMix"].inputs[0].default_value')
        d = fc.driver
        d.type = "AVERAGE"
        var = d.variables.new()
        var.type = "SINGLE_PROP"
        var.targets[0].id = root
        var.targets[0].data_path = '["bumper_blue"]'
    print("[artemis] materials:", ", ".join(f"{k}={v}" for k, v in sorted(stats.items())), flush=True)
