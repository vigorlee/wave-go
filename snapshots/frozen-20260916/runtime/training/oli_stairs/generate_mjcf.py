"""Generate a reduced PR-space OLI MJCF suitable for GPU MJX training."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
from mujoco import mjx

from training.oli_stairs.constants import (
    ARTIFACT_ROOT,
    AUXILIARY_JOINT_NAMES,
    JOINT_NAMES,
    SOURCE_MODEL,
    STAGE_HEIGHTS,
    StairSpec,
    load_control_parameters,
    stair_spec_for_stage,
)


PHYSICAL_ACTUATOR_JOINT_NAMES = tuple(
    {
        4: "left_A_achilles_joint",
        5: "left_B_achilles_joint",
        10: "right_A_achilles_joint",
        11: "right_B_achilles_joint",
        13: "waist_A_joint",
        14: "waist_B_joint",
    }.get(index, name)
    for index, name in enumerate(JOINT_NAMES)
)

PHYSICAL_TORQUE_LIMITS = (42.0, 42.0, 42.0, 42.0, 42.0, 42.0)


def _remove_auxiliary_parallel_joints(root: ET.Element) -> None:
    auxiliary = set(AUXILIARY_JOINT_NAMES)
    for body in root.iter("body"):
        for joint in list(body.findall("joint")):
            if joint.get("name") in auxiliary:
                body.remove(joint)

    equality = root.find("equality")
    if equality is not None:
        equality.clear()

    sensor = root.find("sensor")
    if sensor is not None:
        for item in list(sensor):
            if item.get("joint") in auxiliary:
                sensor.remove(item)


def _configure_collision_geometries(root: ET.Element) -> None:
    for geom in root.iter("geom"):
        geom.set("margin", "0")
        geom.set("gap", "0")
        geom_type = geom.get("type", "")
        if geom.get("mesh") or geom_type in {"mesh", "cylinder"}:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")


def _joint_ranges(root: ET.Element) -> dict[str, tuple[float, float]]:
    ranges: dict[str, tuple[float, float]] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        value = joint.get("range")
        if name and value:
            lower, upper = (float(item) for item in value.split())
            ranges[name] = (lower, upper)
    return ranges


def _replace_actuators(root: ET.Element) -> None:
    params = load_control_parameters()
    ranges = _joint_ranges(root)
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    else:
        actuator.clear()

    for index, joint_name in enumerate(JOINT_NAMES):
        lower, upper = ranges[joint_name]
        limit = float(params.torque_limit[index])
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"policy_{index:02d}_{joint_name}",
                "joint": joint_name,
                "kp": f"{float(params.kp[index]):.8g}",
                "kv": f"{float(params.kd[index]):.8g}",
                "ctrllimited": "true",
                "ctrlrange": f"{lower:.8g} {upper:.8g}",
                "forcelimited": "true",
                "forcerange": f"{-limit:.8g} {limit:.8g}",
            },
        )


def _replace_full_torque_actuators(root: ET.Element) -> None:
    params = load_control_parameters()
    limits = params.torque_limit.copy()
    for index, limit in zip((4, 5, 10, 11, 13, 14), PHYSICAL_TORQUE_LIMITS):
        limits[index] = limit

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    else:
        actuator.clear()
    for index, joint_name in enumerate(PHYSICAL_ACTUATOR_JOINT_NAMES):
        limit = float(limits[index])
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"policy_{index:02d}_{joint_name}",
                "joint": joint_name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": f"{-limit:.8g} {limit:.8g}",
            },
        )


def _add_training_sites_and_sensors(root: ET.Element) -> None:
    bodies = {body.get("name"): body for body in root.iter("body")}
    for side in ("left", "right"):
        body = bodies[f"{side}_ankle_roll_link"]
        ET.SubElement(
            body,
            "site",
            {
                "name": f"{side}_foot_training_site",
                "pos": "0.018 0 -0.0635",
                "size": "0.008",
                "rgba": "0 0.6 1 1",
            },
        )

    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")
    ET.SubElement(
        sensor,
        "velocimeter",
        {"name": "Base_LocalLinVel", "site": "imu"},
    )
    ET.SubElement(
        sensor,
        "framelinvel",
        {
            "name": "Base_GlobalLinVel",
            "objtype": "site",
            "objname": "imu",
        },
    )
    ET.SubElement(
        sensor,
        "framelinvel",
        {
            "name": "LeftFoot_GlobalLinVel",
            "objtype": "site",
            "objname": "left_foot_training_site",
        },
    )
    ET.SubElement(
        sensor,
        "framelinvel",
        {
            "name": "RightFoot_GlobalLinVel",
            "objtype": "site",
            "objname": "right_foot_training_site",
        },
    )


def _add_stairs(root: ET.Element, spec: StairSpec) -> None:
    if spec.step_height <= 0.0:
        return

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Source MJCF has no worldbody")

    common = {
        "type": "box",
        "friction": "1.2 0.02 0.002",
        "contype": "1",
        "conaffinity": "1",
        "condim": "3",
        "group": "3",
        "margin": "0",
        "gap": "0",
        "rgba": "0.42 0.44 0.48 1",
    }
    for step in range(spec.step_count):
        top = (step + 1) * spec.step_height
        x = spec.start_x + (step + 0.5) * spec.step_depth
        ET.SubElement(
            worldbody,
            "geom",
            {
                **common,
                "name": f"training_stair_{step + 1:02d}",
                "pos": f"{x:.6g} 0 {top / 2.0:.6g}",
                "size": (
                    f"{spec.step_depth / 2.0:.6g} {spec.half_width:.6g} "
                    f"{top / 2.0:.6g}"
                ),
            },
        )

        if spec.riser_ramp_length > 0.0:
            ramp_length = spec.riser_ramp_length
            slope_length = math.hypot(ramp_length, spec.step_height)
            angle = math.atan2(spec.step_height, ramp_length)
            riser_x = spec.start_x + step * spec.step_depth
            low_height = step * spec.step_height
            ET.SubElement(
                worldbody,
                "geom",
                {
                    **common,
                    "name": f"training_stair_ramp_{step + 1:02d}",
                    "pos": (
                        f"{riser_x - ramp_length / 2.0:.6g} 0 "
                        f"{low_height + spec.step_height / 2.0:.6g}"
                    ),
                    "size": (
                        f"{slope_length / 2.0:.6g} {spec.half_width:.6g} 0.008"
                    ),
                    "euler": f"0 {-angle:.8g} 0",
                    "rgba": "0.36 0.38 0.42 1",
                },
            )

    landing_x = spec.end_x + spec.landing_length / 2.0
    ET.SubElement(
        worldbody,
        "geom",
        {
            **common,
            "name": "training_stair_landing",
            "pos": f"{landing_x:.6g} 0 {spec.top_height / 2.0:.6g}",
            "size": (
                f"{spec.landing_length / 2.0:.6g} {spec.half_width:.6g} "
                f"{spec.top_height / 2.0:.6g}"
            ),
        },
    )


def generate_training_model(
    output_path: Path,
    spec: StairSpec,
    source_path: Path = SOURCE_MODEL,
) -> Path:
    """Generate and validate an OLI training model."""
    tree = ET.parse(source_path)
    root = tree.getroot()
    root.set("model", f"HU_D04_01_PR_stairs_{int(spec.step_height * 100):02d}cm")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    mesh_dir = source_path.parent.parent / "meshes/HU_D04_01"
    compiler.set("meshdir", str(mesh_dir.resolve()))

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("integrator", "Euler")
    option.set("iterations", "8")
    option.set("ls_iterations", "4")

    _remove_auxiliary_parallel_joints(root)
    _configure_collision_geometries(root)
    _replace_actuators(root)
    _add_training_sites_and_sensors(root)
    _add_stairs(root, spec)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        dir=output_path.parent,
        prefix=f".{output_path.name}.",
        suffix=".tmp",
        delete=False,
    ) as stream:
        temporary_path = Path(stream.name)
    try:
        tree.write(temporary_path, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(temporary_path))
        if (model.nq, model.nv, model.nu) != (38, 37, 31):
            raise ValueError(
                f"Unexpected generated model sizes: nq={model.nq}, "
                f"nv={model.nv}, nu={model.nu}"
            )
        actuator_joints = tuple(
            model.joint(model.actuator_trnid[index, 0]).name
            for index in range(model.nu)
        )
        if actuator_joints != JOINT_NAMES:
            raise ValueError("Generated actuator order does not match OLI policy order")
        os.replace(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return output_path


def generate_full_training_model(
    output_path: Path,
    spec: StairSpec,
    source_path: Path = SOURCE_MODEL,
) -> Path:
    """Generate the complete constrained OLI model with AB torque actuators."""
    tree = ET.parse(source_path)
    root = tree.getroot()
    root.set("model", f"HU_D04_01_AB_stairs_{int(spec.step_height * 100):02d}cm")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    mesh_dir = source_path.parent.parent / "meshes/HU_D04_01"
    compiler.set("meshdir", str(mesh_dir.resolve()))

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("integrator", "Euler")
    option.set("iterations", "12")
    option.set("ls_iterations", "6")

    _configure_collision_geometries(root)
    _replace_full_torque_actuators(root)
    _add_training_sites_and_sensors(root)
    _add_stairs(root, spec)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        dir=output_path.parent,
        prefix=f".{output_path.name}.",
        suffix=".tmp",
        delete=False,
    ) as stream:
        temporary_path = Path(stream.name)
    try:
        tree.write(temporary_path, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(temporary_path))
        if (model.nq, model.nv, model.nu, model.neq) != (68, 61, 31, 6):
            raise ValueError(
                f"Unexpected full model sizes: nq={model.nq}, nv={model.nv}, "
                f"nu={model.nu}, neq={model.neq}"
            )
        actuator_joints = tuple(
            model.joint(model.actuator_trnid[index, 0]).name
            for index in range(model.nu)
        )
        if actuator_joints != PHYSICAL_ACTUATOR_JOINT_NAMES:
            raise ValueError("Full-model actuator order does not match OLI policy order")
        os.replace(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return output_path


def validate_mjx(path: Path, impl: str = "jax") -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    mjx.put_model(model, impl=impl)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h15")
    parser.add_argument("--step-count", type=int, default=10)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--start-x", type=float, default=0.75)
    parser.add_argument("--half-width", type=float, default=1.50)
    parser.add_argument("--riser-ramp-length", type=float, default=0.0)
    parser.add_argument("--full-model", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--validate-mjx", action="store_true")
    args = parser.parse_args()

    if args.step_count < 1:
        parser.error("--step-count must be positive")
    for name in ("step_depth", "landing_length", "half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.riser_ramp_length < 0.0:
        parser.error("--riser-ramp-length cannot be negative")

    output = args.output or ARTIFACT_ROOT / "models" / f"oli_{args.stage}.xml"
    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        start_x=args.start_x,
        half_width=args.half_width,
        landing_length=args.landing_length,
        riser_ramp_length=args.riser_ramp_length,
    )
    generator = generate_full_training_model if args.full_model else generate_training_model
    path = generator(output, spec)
    if args.validate_mjx:
        validate_mjx(path)
    print(path)


if __name__ == "__main__":
    main()
