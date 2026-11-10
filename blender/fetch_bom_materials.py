#!/usr/bin/env python3
"""
fetch_bom_materials.py - pull each part's Onshape *Material* and *Appearance*
from the robot's link-shared Onshape document (no API key needed for a
document shared with "anyone with the link") and save them for materials.py.

    python fetch_bom_materials.py            # writes source/onshape_bom_materials.json

materials.py uses these for team-made parts ("Part 12", "Spindexer Wall", ...),
whose real material can't be told from the part name. A part is matched by its
top-level subassembly (without the "NN - " prefix) and its name.
"""

import argparse
import json
import os
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = "16efabe4144263150bdc02ed"        # 4915 2026 Public Share: ARTEMIS
VERSION = "d5fdcfd93aef43e72bcdf30d"
ELEMENT = "09aa24f6cfc5e3210ecfc998"    # Assembly 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--doc", default=DOC)
    ap.add_argument("--version", default=VERSION)
    ap.add_argument("--element", default=ELEMENT)
    ap.add_argument("--out", default=os.path.join(HERE, "source", "onshape_bom_materials.json"))
    a = ap.parse_args()
    url = (f"https://cad.onshape.com/api/v10/assemblies/d/{a.doc}/v/{a.version}/e/{a.element}"
           "/bom?indented=true&multiLevel=true&generateIfAbsent=true")
    with urllib.request.urlopen(url, timeout=120) as r:
        bom = json.load(r)
    names = {h["id"]: h["name"] for h in bom["headers"]}

    def get(row, header):
        for k, v in row["headerIdToValue"].items():
            if names.get(k) == header:
                return v
        return None

    tops, rows = {}, []
    for row in bom["rows"]:
        item = str(get(row, "Item"))
        name = get(row, "Name")
        name = name.get("value") if isinstance(name, dict) else name
        if "." not in item:
            tops[item] = name
            continue
        mat = get(row, "Material")
        mat = mat.get("displayName") if isinstance(mat, dict) else mat
        app = get(row, "Appearance")
        col = app.get("color") if isinstance(app, dict) else None
        rows.append(dict(
            top=tops.get(item.split(".")[0], ""), item=item, name=name, material=mat,
            color="#%02x%02x%02x" % (col["red"], col["green"], col["blue"]) if col else None))
    out = dict(source=f"https://cad.onshape.com/documents/{a.doc}/v/{a.version}/e/{a.element}",
               parts=rows)
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
    print(f"wrote {len(rows)} BOM rows to {a.out}")


if __name__ == "__main__":
    main()
