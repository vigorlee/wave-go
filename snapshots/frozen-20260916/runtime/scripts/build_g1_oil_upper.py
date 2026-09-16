#!/usr/bin/env python3
"""Build a policy-compatible G1 lower body with fixed Oli HU_D04_01 visuals."""

from __future__ import annotations

import copy
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


G1_UPPER_MESHES = {
    "torso_link_23dof_rev_1_0", "logo_link", "head_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link",
    "left_shoulder_yaw_link", "left_elbow_link",
    "left_wrist_roll_rubber_hand", "right_shoulder_pitch_link",
    "right_shoulder_roll_link", "right_shoulder_yaw_link",
    "right_elbow_link", "right_wrist_roll_rubber_hand",
}


def child(parent: ET.Element, tag: str, name: str | None = None) -> ET.Element:
    for item in parent.findall(tag):
        if name is None or item.get("name") == name:
            return item
    raise RuntimeError(f"missing <{tag}> {name or ''}".strip())


def main() -> int:
    if len(sys.argv) != 6:
        raise SystemExit(
            "usage: build_g1_oil_upper.py G1_XML OLI_XML OLI_MESH_DIR OUT_XML OUT_MESH_DIR"
        )
    g1_xml, oli_xml, oli_mesh_dir, out_xml, out_mesh_dir = map(Path, sys.argv[1:])

    g1_tree = ET.parse(g1_xml)
    g1_root = g1_tree.getroot()
    oli_root = ET.parse(oli_xml).getroot()
    g1_asset = child(g1_root, "asset")
    g1_pelvis = child(child(g1_root, "worldbody"), "body", "pelvis")
    oli_base = child(child(oli_root, "worldbody"), "body", "base_link")
    oli_waist = copy.deepcopy(child(oli_base, "body", "waist_yaw_link"))

    # Remove G1's fixed upper-body render meshes, while leaving its pelvis,
    # inertial parameters, sensors, lower-body joints and contacts untouched.
    for geom in list(g1_pelvis.findall("geom")):
        if geom.get("mesh") in G1_UPPER_MESHES:
            g1_pelvis.remove(geom)

    # Convert Oli's upper tree to fixed visual geometry. This intentionally
    # adds no qpos, actuator, contact, sensor, or dynamic mass to the G1 policy.
    referenced_meshes: set[str] = set()
    for parent in oli_waist.iter():
        for item in list(parent):
            if item.tag in {"joint", "inertial", "site"}:
                parent.remove(item)
            elif item.tag == "geom" and not item.get("mesh"):
                parent.remove(item)
    for body in oli_waist.iter("body"):
        if body.get("name"):
            body.set("name", "oil_" + body.get("name"))
        # MuJoCo 3.3 requires every retained fixed child body to have a valid
        # inertial. Use a numerically valid but dynamically negligible payload;
        # 27 bodies add only 0.027 g in total.
        body.insert(0, ET.Element("inertial", {
            "pos": "0 0 0",
            "mass": "0.000001",
            "diaginertia": "0.000000001 0.000000001 0.000000001",
        }))
    for geom in oli_waist.iter("geom"):
        mesh = geom.get("mesh")
        if mesh:
            referenced_meshes.add(mesh)
            geom.set("mesh", "oil_" + mesh)
        if geom.get("name"):
            geom.set("name", "oil_" + geom.get("name"))
        geom.attrib.pop("class", None)
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("density", "0")
        geom.set("group", "1")

    oli_assets = {
        mesh.get("name"): mesh for asset in oli_root.findall("asset")
        for mesh in asset.findall("mesh")
    }
    out_mesh_dir.mkdir(parents=True, exist_ok=True)
    for name in sorted(referenced_meshes):
        source_asset = oli_assets.get(name)
        if source_asset is None:
            raise RuntimeError(f"Oli mesh asset is missing: {name}")
        source_file = oli_mesh_dir / source_asset.get("file", "")
        if not source_file.is_file():
            raise RuntimeError(f"Oli mesh file is missing: {source_file}")
        target_name = source_file.name
        shutil.copy2(source_file, out_mesh_dir / target_name)
        ET.SubElement(g1_asset, "mesh", {
            "name": "oil_" + name,
            "file": "oil/" + target_name,
        })

    g1_pelvis.insert(0, ET.Comment(
        "Oli HU_D04_01 fixed visual upper body; G1 12-DoF policy contract unchanged."
    ))
    g1_pelvis.insert(1, oli_waist)
    ET.indent(g1_tree, space="  ")
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    g1_tree.write(out_xml, encoding="unicode")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
