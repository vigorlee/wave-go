#!/usr/bin/env python3
"""Deploy the converted Nezha2 model with the MATRiX XGW runtime contract."""

import json
import math
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import trimesh


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "third_party/nezha2_rl_sar/view/mujoco"
SOURCE_XML = SOURCE_ROOT / "nezha2_mujoco.xml"
SOURCE_ASSETS = (
    PROJECT_ROOT
    / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/robots/nezha_description/meshes"
)
MUJOCO_ROOT = PROJECT_ROOT / "matrix/src/robot_mujoco/zsibot_robots/nezha2"
UE_ROOT = (
    PROJECT_ROOT
    / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/nezha2"
)
UE_MODEL_ROOT = UE_ROOT.parent
MATRIX_CONFIG = PROJECT_ROOT / "matrix/config/config.json"
UE_CONFIG = UE_MODEL_ROOT / "config/config.json"
YARD_SOURCE = (
    PROJECT_ROOT
    / "matrix/src/robot_mujoco/zsibot_robots/custom/scene_terrain_yard.xml"
)

POLICY_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FL_foot_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FR_foot_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_foot_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RR_foot_joint",
]
DEFAULTS = [
    0.0, 0.925, -1.85, 0.0,
    0.0, 0.925, -1.85, 0.0,
    0.0, -0.925, 1.85, 0.0,
    0.0, -0.925, 1.85, 0.0,
]
SDK_LEGS = ["FR", "FL", "RR", "RL"]


def quaternion(axis: list[float], angle: float) -> str:
    half = angle / 2.0
    scale = math.sin(half)
    values = [math.cos(half), *(component * scale for component in axis)]
    return " ".join(f"{value:.12g}" for value in values)


def simplify_stl(path: Path, target_triangles: int) -> None:
    # Binary STL repeats vertex coordinates for every face.  Processing must
    # merge them before quadric simplification, otherwise the simplifier sees
    # a cloud of disconnected triangles and merely removes visible patches.
    mesh = trimesh.load_mesh(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"STL did not load as one mesh: {path}")
    if len(mesh.faces) <= target_triangles:
        return
    try:
        simplified = mesh.simplify_quadric_decimation(face_count=target_triangles)
    except BaseException as error:
        # A full mesh is slower to load but remains visually correct.  Never
        # fall back to face sampling because it turns closed parts into sparse
        # transparent fragments in the packaged UE renderer.
        print(f"[WARN] keeping full mesh {path.name}: simplification failed: {error}")
        return
    if len(simplified.faces) == 0:
        raise ValueError(f"simplification produced an empty mesh: {path}")
    simplified.remove_unreferenced_vertices()
    try:
        simplified.fix_normals(multibody=True)
    except ModuleNotFoundError:
        # Normal repair uses optional networkx; QEM already preserves the
        # source winding, so export remains valid when it is unavailable.
        pass
    simplified.export(path, file_type="stl")


def replace(parent: ET.Element, tag: str, replacement: ET.Element) -> None:
    old = parent.find(tag)
    if old is not None:
        parent.remove(old)
    parent.append(replacement)


def build_runtime_xml(yaw_degrees: float, *, yard_visual: bool = False) -> ET.ElementTree:
    tree = ET.parse(SOURCE_XML)
    root = tree.getroot()
    root.set("model", "nezha2_description")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("meshdir", "assets")
    for mesh in root.findall("asset/mesh"):
        mesh.set("file", Path(mesh.get("file", "")).name)

    # Collision proxies are intentionally simple boxes/cylinders for stable
    # contact.  Keep them in physics, but do not let MuJoCo/UE render them over
    # the real URDF STL meshes.
    for geom in root.findall(".//default[@class='collision']/geom"):
        geom.attrib.pop("material", None)
        geom.set("rgba", "0 0 0 0")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MJCF has no worldbody")
    trunk = worldbody.find("body[@name='trunk']")
    if trunk is None:
        raise RuntimeError("MJCF has no trunk body")
    # The packaged UE renderer locates the floating robot root by the legacy
    # BASE_LINK name.  A fixed child alias is insufficient: UE selects that
    # empty body and cannot apply the incoming free-joint qpos to it.
    trunk.set("name", "BASE_LINK")
    for element in root.iter():
        for attribute in ("body", "body1", "body2", "target"):
            if element.get(attribute) == "trunk":
                element.set(attribute, "BASE_LINK")
    trunk.set("pos", "0 0 0.47")
    trunk.set("quat", quaternion([0.0, 0.0, 1.0], math.radians(yaw_degrees)))
    if trunk.find("inertial") is None:
        trunk.insert(
            0,
            ET.Element(
                "inertial",
                {
                    "mass": "44.027",
                    "pos": "-0.000711047828587053 0.0112073624474439 -0.00481932179712385",
                    "quat": "1 0 0 0",
                    "diaginertia": "0.47177966 1.99866713 2.26564344",
                },
            ),
        )

    for name in ("BASE_LINK_site", "imu", "livox_imu", "camera_imu"):
        for site in trunk.findall(f"site[@name='{name}']"):
            trunk.remove(site)
    for body in trunk.findall("body[@name='BASE_LINK']"):
        trunk.remove(body)
    trunk.insert(2, ET.Element("site", {"name": "BASE_LINK_site", "pos": "0 0 0"}))
    trunk.insert(3, ET.Element("site", {"name": "imu", "pos": "0 0 0"}))
    trunk.insert(
        4,
        ET.Element(
            "site",
            {"name": "livox_imu", "pos": "0.13011 0.02329 0.17598", "quat": "1 0 0 0"},
        ),
    )
    trunk.insert(5, ET.Element("site", {"name": "camera_imu", "pos": "0.29 0 0.01"}))

    bodies_by_joint: dict[str, tuple[ET.Element, ET.Element]] = {}
    for body in root.findall(".//body"):
        for joint in body.findall("joint"):
            if joint.get("name"):
                bodies_by_joint[joint.get("name", "")] = (body, joint)
    for name, default in zip(POLICY_ORDER, DEFAULTS, strict=True):
        body, joint = bodies_by_joint[name]
        axis = [float(value) for value in joint.get("axis", "0 0 1").split()]
        body.set("quat", quaternion(axis, default))
        joint.set("ref", f"{default:.12g}")
        joint.set("actuatorfrcrange", "-56 56" if name.endswith("foot_joint") else "-225 225" if "calf" in name else "-150 150")

    actuator = ET.Element("actuator")
    for leg in SDK_LEGS:
        for suffix, label in (
            ("hip_joint", "ABAD_LINK"),
            ("thigh_joint", "HIP_LINK"),
            ("calf_joint", "KNEE_LINK"),
            ("foot_joint", "FOOT_LINK"),
        ):
            ET.SubElement(actuator, "motor", {"name": f"{leg}_{label}", "joint": f"{leg}_{suffix}"})
    replace(root, "actuator", actuator)

    sensor = ET.Element("sensor")
    for kind, suffix in (("jointpos", "pos"), ("jointvel", "vel"), ("jointactuatorfrc", "torque")):
        for leg in SDK_LEGS:
            for joint_label in ("hip", "thigh", "calf", "foot"):
                attributes = {
                    "name": f"{leg}_{joint_label}_{suffix}",
                    "joint": f"{leg}_{joint_label}_joint",
                }
                if kind == "jointactuatorfrc":
                    attributes["noise"] = "0.00"
                ET.SubElement(sensor, kind, attributes)
    for prefix, site in (("imu", "imu"), ("livox_imu", "livox_imu"), ("camera_imu", "camera_imu")):
        ET.SubElement(sensor, "framequat", {"name": f"{prefix}_quat", "objtype": "site", "objname": site})
        ET.SubElement(sensor, "gyro", {"name": f"{prefix}_gyro", "site": site})
        ET.SubElement(sensor, "accelerometer", {"name": f"{prefix}_acc", "site": site})
        frame_prefix = "frame" if prefix == "imu" else f"{prefix}_frame"
        ET.SubElement(sensor, "framepos", {"name": f"{frame_prefix}_pos", "objtype": "site", "objname": site})
        ET.SubElement(sensor, "framelinvel", {"name": f"{frame_prefix}_vel", "objtype": "site", "objname": site})
    replace(root, "sensor", sensor)
    if yard_visual:
        asset = root.find("asset")
        if asset is None:
            raise RuntimeError("MJCF has no asset section")
        ET.SubElement(
            asset,
            "mesh",
            {"name": "yard_stairs_visual.STL", "file": "yard_stairs_visual.STL"},
        )
        static_body = ET.SubElement(worldbody, "body", {"name": "yard_stairs_visual"})
        ET.SubElement(
            static_body,
            "geom",
            {
                "name": "yard_stairs_visual_geom",
                "type": "mesh",
                "mesh": "yard_stairs_visual.STL",
                "contype": "0",
                "conaffinity": "0",
                "group": "2",
                "rgba": "0.48 0.52 0.56 1",
            },
        )
    return tree


def write_scene(destination: Path, include_name: str) -> None:
    scene = YARD_SOURCE.read_text(encoding="utf-8")
    scene = scene.replace("Nezha3 YardWorld stair scene", "Nezha2 YardWorld navigation scene")
    scene = scene.replace('include file="current_yard.xml"', f'include file="{include_name}"')
    destination.write_text(scene, encoding="utf-8")


def build_yard_stairs_visual(destination: Path) -> None:
    scene_root = ET.parse(YARD_SOURCE).getroot()
    stairs = []
    for geom in scene_root.findall("worldbody/geom"):
        name = geom.get("name", "")
        if not name.startswith(("box_OBS", "box2_OBS")):
            continue
        size = [float(value) for value in geom.get("size", "").split()]
        position = [float(value) for value in geom.get("pos", "").split()]
        if geom.get("type") != "box" or len(size) != 3 or len(position) != 3:
            raise RuntimeError(f"unsupported YardWorld stair geom: {name}")
        stairs.append(
            trimesh.creation.box(
                extents=[2.0 * value for value in size],
                transform=trimesh.transformations.translation_matrix(position),
            )
        )
    if not stairs:
        raise RuntimeError("YardWorld scene contains no stair boxes")
    combined = trimesh.util.concatenate(stairs)
    combined.export(destination, file_type="stl")


def deploy(root: Path, triangle_target: int) -> None:
    assets = root / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    for source in SOURCE_ASSETS.glob("*.STL"):
        target = assets / source.name
        if target.exists():
            target.unlink()
        shutil.copy2(source, target)
        target.chmod(0o644)
        # Keep small features intact and give the body more triangles than
        # repeated leg parts.  The per-part ceiling is still bounded by the
        # deployment profile supplied by the caller.
        source_triangles = int.from_bytes(target.read_bytes()[80:84], "little")
        part_target = max(12_000, min(triangle_target, source_triangles // 4))
        simplify_stl(target, part_target)

    add_yard_visual = root == UE_ROOT
    if add_yard_visual:
        build_yard_stairs_visual(assets / "yard_stairs_visual.STL")
    normal = build_runtime_xml(0.0)
    yard = build_runtime_xml(90.0, yard_visual=add_yard_visual)
    ET.indent(normal, space="  ")
    ET.indent(yard, space="  ")
    normal.write(root / "nezha2.xml", encoding="utf-8", xml_declaration=False)
    yard.write(root / "nezha2_yard.xml", encoding="utf-8", xml_declaration=False)
    write_scene(root / "scene_terrain_yard.xml", "nezha2_yard.xml")


def validate(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    has_yard_visual = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "yard_stairs_visual"
    ) >= 0
    expected = dict(
        nq=23, nv=22, nu=16, nbody=25 if has_yard_visual else 24, njnt=17, nsensor=63
    )
    actual = {name: int(getattr(model, name)) for name in expected}
    if actual != expected:
        raise RuntimeError(f"MATRiX contract mismatch: expected={expected}, actual={actual}")
    defaults = []
    for name in POLICY_ORDER:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        defaults.append(float(model.qpos0[model.jnt_qposadr[joint_id]]))
    if any(abs(left - right) > 1e-7 for left, right in zip(defaults, DEFAULTS, strict=True)):
        raise RuntimeError(f"default pose mismatch: {defaults}")
    print(f"MATRIX_NEZHA2_OK {actual} qpos0={defaults}")


def remove_generated_stair_carrier_config() -> None:
    """Remove the temporary second actor used by older Nezha2 deployments."""
    for config_path in (MATRIX_CONFIG, UE_CONFIG):
        if not config_path.is_file():
            continue
        config = json.loads(config_path.read_text(encoding="utf-8"))
        actor = config.get("robot2", {})
        if (
            actor.get("robot_type") == "go2w"
            and actor.get("state_port") == 25411
            and actor.get("cmd_port") == 25412
        ):
            del config["robot2"]
            config_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    if not SOURCE_XML.is_file():
        raise FileNotFoundError(f"run scripts/prepare_nezha2_mujoco.py first: {SOURCE_XML}")
    deploy(MUJOCO_ROOT, 180_000)
    deploy(UE_ROOT, 80_000)
    validate(MUJOCO_ROOT / "scene_terrain_yard.xml")
    validate(UE_ROOT / "scene_terrain_yard.xml")
    remove_generated_stair_carrier_config()
    print("MATRIX_NEZHA2_UE_STAIRS_OK static_body=yard_stairs_visual")
    xgw_link = PROJECT_ROOT / "matrix/src/robot_mujoco/zsibot_robots/xgw/nezha2"
    if xgw_link.exists() or xgw_link.is_symlink():
        xgw_link.unlink()
    xgw_link.symlink_to("../nezha2")


if __name__ == "__main__":
    main()
