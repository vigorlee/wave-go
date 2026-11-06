#!/usr/bin/env python3
"""
validate_xml_contract.py — Static contract validation for generated MuJoCo XML files.

Usage:
    python3 validate_xml_contract.py <generated_xml> <profile>

    profile: xgb | xxg | xgw | zg | lite3 | generic

Exit codes:
    0 = PASS
    1 = FAIL (contract violations found)
    2 = Usage error
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ──────────────────────────────────────────────────────────────────────────────
# Contracts
# ──────────────────────────────────────────────────────────────────────────────

XGB_CONTRACT = {
    "actuators": [
        ("FAR_ABAD_LINK", "FAR_ABAD_JOINT"),
        ("FAR_HIP_LINK",  "FAR_HIP_JOINT"),
        ("FAR_KNEE_LINK", "FAR_KNEE_JOINT"),
        ("FBL_ABAD_LINK", "FBL_ABAD_JOINT"),
        ("FBL_HIP_LINK",  "FBL_HIP_JOINT"),
        ("FBL_KNEE_LINK", "FBL_KNEE_JOINT"),
        ("RAR_ABAD_LINK", "RAR_ABAD_JOINT"),
        ("RAR_HIP_LINK",  "RAR_HIP_JOINT"),
        ("RAR_KNEE_LINK", "RAR_KNEE_JOINT"),
        ("RBL_ABAD_LINK", "RBL_ABAD_JOINT"),
        ("RBL_HIP_LINK",  "RBL_HIP_JOINT"),
        ("RBL_KNEE_LINK", "RBL_KNEE_JOINT"),
    ],
    "sensors": [
        "FR_hip_pos",   "FR_thigh_pos",   "FR_calf_pos",
        "FL_hip_pos",   "FL_thigh_pos",   "FL_calf_pos",
        "RR_hip_pos",   "RR_thigh_pos",   "RR_calf_pos",
        "RL_hip_pos",   "RL_thigh_pos",   "RL_calf_pos",
        "FR_hip_vel",   "FR_thigh_vel",   "FR_calf_vel",
        "FL_hip_vel",   "FL_thigh_vel",   "FL_calf_vel",
        "RR_hip_vel",   "RR_thigh_vel",   "RR_calf_vel",
        "RL_hip_vel",   "RL_thigh_vel",   "RL_calf_vel",
        "FR_hip_torque","FR_thigh_torque","FR_calf_torque",
        "FL_hip_torque","FL_thigh_torque","FL_calf_torque",
        "RR_hip_torque","RR_thigh_torque","RR_calf_torque",
        "RL_hip_torque","RL_thigh_torque","RL_calf_torque",
        "imu_quat", "imu_gyro", "imu_acc",
        "frame_pos", "frame_vel",
        "livox_imu_quat", "livox_imu_gyro", "livox_imu_acc",
        "livox_imu_frame_pos", "livox_imu_frame_vel",
        "camera_imu_quat", "camera_imu_gyro", "camera_imu_acc",
        "camera_imu_frame_pos", "camera_imu_frame_vel",
    ],
    "hip_joint_range": (-2.0, 3.491),   # (lower, upper) — tolerance ±0.01
    "hip_joint_names": ["FAR_HIP_JOINT", "FBL_HIP_JOINT", "RAR_HIP_JOINT", "RBL_HIP_JOINT"],
    "frictionloss": 0.2,
    "actuatorfrcrange": (-28.0, 28.0),
    "root_body": "base_link",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}

XGW_CONTRACT = {
    "actuators": [
        ("FR_ABAD_LINK", "FAR_ABAD_JOINT"),
        ("FR_HIP_LINK",  "FAR_HIP_JOINT"),
        ("FR_KNEE_LINK", "FAR_KNEE_JOINT"),
        ("FR_FOOT_LINK", "FAR_FOOT_JOINT"),
        ("FL_ABAD_LINK", "FBL_ABAD_JOINT"),
        ("FL_HIP_LINK",  "FBL_HIP_JOINT"),
        ("FL_KNEE_LINK", "FBL_KNEE_JOINT"),
        ("FL_FOOT_LINK", "FBL_FOOT_JOINT"),
        ("RR_ABAD_LINK", "RAR_ABAD_JOINT"),
        ("RR_HIP_LINK",  "RAR_HIP_JOINT"),
        ("RR_KNEE_LINK", "RAR_KNEE_JOINT"),
        ("RR_FOOT_LINK", "RAR_FOOT_JOINT"),
        ("RL_ABAD_LINK", "RBL_ABAD_JOINT"),
        ("RL_HIP_LINK",  "RBL_HIP_JOINT"),
        ("RL_KNEE_LINK", "RBL_KNEE_JOINT"),
        ("RL_FOOT_LINK", "RBL_FOOT_JOINT"),
    ],
    "sensors": [
        "FR_hip_pos",   "FR_thigh_pos",   "FR_calf_pos",   "FR_foot_pos",
        "FL_hip_pos",   "FL_thigh_pos",   "FL_calf_pos",   "FL_foot_pos",
        "RR_hip_pos",   "RR_thigh_pos",   "RR_calf_pos",   "RR_foot_pos",
        "RL_hip_pos",   "RL_thigh_pos",   "RL_calf_pos",   "RL_foot_pos",
        "FR_hip_vel",   "FR_thigh_vel",   "FR_calf_vel",   "FR_foot_vel",
        "FL_hip_vel",   "FL_thigh_vel",   "FL_calf_vel",   "FL_foot_vel",
        "RR_hip_vel",   "RR_thigh_vel",   "RR_calf_vel",   "RR_foot_vel",
        "RL_hip_vel",   "RL_thigh_vel",   "RL_calf_vel",   "RL_foot_vel",
        "FR_hip_torque","FR_thigh_torque","FR_calf_torque","FR_foot_torque",
        "FL_hip_torque","FL_thigh_torque","FL_calf_torque","FL_foot_torque",
        "RR_hip_torque","RR_thigh_torque","RR_calf_torque","RR_foot_torque",
        "RL_hip_torque","RL_thigh_torque","RL_calf_torque","RL_foot_torque",
        "imu_quat", "imu_gyro", "imu_acc",
        "frame_pos", "frame_vel",
        "livox_imu_quat", "livox_imu_gyro", "livox_imu_acc",
        "livox_imu_frame_pos", "livox_imu_frame_vel",
        "camera_imu_quat", "camera_imu_gyro", "camera_imu_acc",
        "camera_imu_frame_pos", "camera_imu_frame_vel",
    ],
    "hip_joint_range": (-1.152, 2.967),
    "hip_joint_names": ["FAR_HIP_JOINT", "FBL_HIP_JOINT", "RAR_HIP_JOINT", "RBL_HIP_JOINT"],
    "frictionloss": None,   # reference XML has inconsistent frictionloss on FOOT joints; skip check
    "actuatorfrcrange": (-28.0, 28.0),
    "root_body": "base_link",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}

XXG_CONTRACT = {
    "actuators": [
        ("FR_ABAD_LINK", "FAR_ABAD_JOINT"),
        ("FR_HIP_LINK",  "FAR_HIP_JOINT"),
        ("FR_KNEE_LINK", "FAR_KNEE_JOINT"),
        ("FL_ABAD_LINK", "FBL_ABAD_JOINT"),
        ("FL_HIP_LINK",  "FBL_HIP_JOINT"),
        ("FL_KNEE_LINK", "FBL_KNEE_JOINT"),
        ("RR_ABAD_LINK", "RAR_ABAD_JOINT"),
        ("RR_HIP_LINK",  "RAR_HIP_JOINT"),
        ("RR_KNEE_LINK", "RAR_KNEE_JOINT"),
        ("RL_ABAD_LINK", "RBL_ABAD_JOINT"),
        ("RL_HIP_LINK",  "RBL_HIP_JOINT"),
        ("RL_KNEE_LINK", "RBL_KNEE_JOINT"),
    ],
    "sensors": XGB_CONTRACT["sensors"],
    "hip_joint_range": (-2.967, 2.967),
    "hip_joint_names": ["FAR_HIP_JOINT", "FBL_HIP_JOINT", "RAR_HIP_JOINT", "RBL_HIP_JOINT"],
    "frictionloss": None,
    "actuatorfrcrange": (-20.0, 20.0),
    "root_body": "BASE_LINK",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}

ZG_CONTRACT = {
    "actuators": [
        ("FAR_ABAD_LINK", "FAR_ABAD_JOINT"),
        ("FAR_HIP_LINK",  "FAR_HIP_JOINT"),
        ("FAR_KNEE_LINK", "FAR_KNEE_JOINT"),
        ("FBL_ABAD_LINK", "FBL_ABAD_JOINT"),
        ("FBL_HIP_LINK",  "FBL_HIP_JOINT"),
        ("FBL_KNEE_LINK", "FBL_KNEE_JOINT"),
        ("RAR_ABAD_LINK", "RAR_ABAD_JOINT"),
        ("RAR_HIP_LINK",  "RAR_HIP_JOINT"),
        ("RAR_KNEE_LINK", "RAR_KNEE_JOINT"),
        ("RBL_ABAD_LINK", "RBL_ABAD_JOINT"),
        ("RBL_HIP_LINK",  "RBL_HIP_JOINT"),
        ("RBL_KNEE_LINK", "RBL_KNEE_JOINT"),
    ],
    "sensors": [
        "FR_hip_pos",   "FR_thigh_pos",   "FR_calf_pos",
        "FL_hip_pos",   "FL_thigh_pos",   "FL_calf_pos",
        "RR_hip_pos",   "RR_thigh_pos",   "RR_calf_pos",
        "RL_hip_pos",   "RL_thigh_pos",   "RL_calf_pos",
        "FR_hip_vel",   "FR_thigh_vel",   "FR_calf_vel",
        "FL_hip_vel",   "FL_thigh_vel",   "FL_calf_vel",
        "RR_hip_vel",   "RR_thigh_vel",   "RR_calf_vel",
        "RL_hip_vel",   "RL_thigh_vel",   "RL_calf_vel",
        "FR_hip_torque","FR_thigh_torque","FR_calf_torque",
        "FL_hip_torque","FL_thigh_torque","FL_calf_torque",
        "RR_hip_torque","RR_thigh_torque","RR_calf_torque",
        "RL_hip_torque","RL_thigh_torque","RL_calf_torque",
        "imu_quat", "imu_gyro", "imu_acc",
        "frame_pos", "frame_vel",
    ],
    "hip_joint_range": None,
    "hip_joint_names": [],
    "frictionloss": None,
    "actuatorfrcrange": (-150.0, 150.0),
    "root_body": "base_link",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}


ZGW_CONTRACT = {
    "actuators": [
        ("FAR_ABAD_LINK", "FAR_ABAD_JOINT"),
        ("FAR_HIP_LINK",  "FAR_HIP_JOINT"),
        ("FAR_KNEE_LINK", "FAR_KNEE_JOINT"),
        ("FAR_FOOT_LINK", "FAR_FOOT_JOINT"),
        ("FBL_ABAD_LINK", "FBL_ABAD_JOINT"),
        ("FBL_HIP_LINK",  "FBL_HIP_JOINT"),
        ("FBL_KNEE_LINK", "FBL_KNEE_JOINT"),
        ("FBL_FOOT_LINK", "FBL_FOOT_JOINT"),
        ("RAR_ABAD_LINK", "RAR_ABAD_JOINT"),
        ("RAR_HIP_LINK",  "RAR_HIP_JOINT"),
        ("RAR_KNEE_LINK", "RAR_KNEE_JOINT"),
        ("RAR_FOOT_LINK", "RAR_FOOT_JOINT"),
        ("RBL_ABAD_LINK", "RBL_ABAD_JOINT"),
        ("RBL_HIP_LINK",  "RBL_HIP_JOINT"),
        ("RBL_KNEE_LINK", "RBL_KNEE_JOINT"),
        ("RBL_FOOT_LINK", "RBL_FOOT_JOINT"),
    ],
    "sensors": [
        "FR_hip_pos",   "FR_thigh_pos",   "FR_calf_pos",   "FR_foot_pos",
        "FL_hip_pos",   "FL_thigh_pos",   "FL_calf_pos",   "FL_foot_pos",
        "RR_hip_pos",   "RR_thigh_pos",   "RR_calf_pos",   "RR_foot_pos",
        "RL_hip_pos",   "RL_thigh_pos",   "RL_calf_pos",   "RL_foot_pos",
        "FR_hip_vel",   "FR_thigh_vel",   "FR_calf_vel",   "FR_foot_vel",
        "FL_hip_vel",   "FL_thigh_vel",   "FL_calf_vel",   "FL_foot_vel",
        "RR_hip_vel",   "RR_thigh_vel",   "RR_calf_vel",   "RR_foot_vel",
        "RL_hip_vel",   "RL_thigh_vel",   "RL_calf_vel",   "RL_foot_vel",
        "FR_hip_torque","FR_thigh_torque","FR_calf_torque","FR_foot_torque",
        "FL_hip_torque","FL_thigh_torque","FL_calf_torque","FL_foot_torque",
        "RR_hip_torque","RR_thigh_torque","RR_calf_torque","RR_foot_torque",
        "RL_hip_torque","RL_thigh_torque","RL_calf_torque","RL_foot_torque",
        "imu_quat", "imu_gyro", "imu_acc",
        "frame_pos", "frame_vel",
    ],
    "hip_joint_range": (-2.618, 2.618),
    "hip_joint_names": ["FAR_HIP_JOINT", "FBL_HIP_JOINT", "RAR_HIP_JOINT", "RBL_HIP_JOINT"],
    "frictionloss": None,
    "actuatorfrcrange": None,
    "root_body": "base_link",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}


LITE3_CONTRACT = {
    "actuators": [
        ("FL_HIP", "FL_HipX_joint"),
        ("FL_THIGH", "FL_HipY_joint"),
        ("FL_SHANK", "FL_Knee_joint"),
        ("FR_HIP", "FR_HipX_joint"),
        ("FR_THIGH", "FR_HipY_joint"),
        ("FR_SHANK", "FR_Knee_joint"),
        ("HL_HIP", "HL_HipX_joint"),
        ("HL_THIGH", "HL_HipY_joint"),
        ("HL_SHANK", "HL_Knee_joint"),
        ("HR_HIP", "HR_HipX_joint"),
        ("HR_THIGH", "HR_HipY_joint"),
        ("HR_SHANK", "HR_Knee_joint"),
    ],
    "sensors": [
        "FL_hip_pos", "FL_thigh_pos", "FL_calf_pos",
        "FR_hip_pos", "FR_thigh_pos", "FR_calf_pos",
        "RL_hip_pos", "RL_thigh_pos", "RL_calf_pos",
        "RR_hip_pos", "RR_thigh_pos", "RR_calf_pos",
        "FL_hip_vel", "FL_thigh_vel", "FL_calf_vel",
        "FR_hip_vel", "FR_thigh_vel", "FR_calf_vel",
        "RL_hip_vel", "RL_thigh_vel", "RL_calf_vel",
        "RR_hip_vel", "RR_thigh_vel", "RR_calf_vel",
        "FL_hip_torque", "FL_thigh_torque", "FL_calf_torque",
        "FR_hip_torque", "FR_thigh_torque", "FR_calf_torque",
        "RL_hip_torque", "RL_thigh_torque", "RL_calf_torque",
        "RR_hip_torque", "RR_thigh_torque", "RR_calf_torque",
        "imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel",
    ],
    "hip_joint_range": (-0.523, 0.523),
    "hip_joint_names": ["FL_HipX_joint", "FR_HipX_joint", "HL_HipX_joint", "HR_HipX_joint"],
    "frictionloss": 0.2,
    "actuatorfrcrange": (-30.0, 30.0),
    "root_body": "base_link",
    "requires_freejoint": True,
    # structural_checks patched in after function definitions
}

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _parse_range(value: str | None) -> tuple[float, float] | None:
    if not value:
        return None
    parts = value.split()
    if len(parts) != 2:
        return None
    try:
        return (float(parts[0]), float(parts[1]))
    except ValueError:
        return None


def _approx(a: float, b: float, tol: float = 0.01) -> bool:
    return abs(a - b) <= tol


# ──────────────────────────────────────────────────────────────────────────────
# Profile-specific structural checks
# ──────────────────────────────────────────────────────────────────────────────

def _xgb_structural(root: ET.Element) -> list[str]:
    """
    xgb foot structure: no separate FOOT body; foot contact sphere lives inside
    each KNEE_LINK body as a geom with size ~0.03.
    """
    violations: list[str] = []
    knee_bodies = ["FAR_KNEE_LINK", "FBL_KNEE_LINK", "RAR_KNEE_LINK", "RBL_KNEE_LINK"]

    # No body named *FOOT* should exist (xgb has fixed/removed FOOT joints)
    foot_bodies = [b.get("name", "") for b in root.iter("body") if "FOOT" in b.get("name", "")]
    if foot_bodies:
        violations.append(
            f"xgb should have no FOOT bodies (found: {foot_bodies}); "
            "FOOT joints must be fixed/removed"
        )

    # Each KNEE body must contain a sphere geom (foot contact sphere, size ~0.03)
    for knee_name in knee_bodies:
        knee_body = root.find(f".//body[@name='{knee_name}']")
        if knee_body is None:
            violations.append(f"xgb structural: body '{knee_name}' not found")
            continue
        # MuJoCo default geom type is "sphere" when type attribute is absent.
        # xgb foot sphere has size="0.03" and no type (or type="sphere").
        has_foot_sphere = any(
            g.get("type", "sphere") in ("sphere",) and _approx(float(g.get("size", "0")), 0.03)
            for g in knee_body.findall("geom")
            if g.get("size") and " " not in g.get("size", "")  # single scalar size = sphere
        )
        if not has_foot_sphere:
            violations.append(
                f"xgb structural: '{knee_name}' missing foot-contact sphere geom (size ~0.03)"
            )

    return violations


def _zg_structural(root: ET.Element) -> list[str]:
    """
    zg foot structure: no separate FOOT body; foot contact sphere lives inside
    each KNEE_LINK body as a geom with size ~0.04.
    """
    violations: list[str] = []
    knee_bodies = ["FAR_KNEE_LINK", "FBL_KNEE_LINK", "RAR_KNEE_LINK", "RBL_KNEE_LINK"]

    foot_bodies = [b.get("name", "") for b in root.iter("body") if "FOOT" in b.get("name", "")]
    if foot_bodies:
        violations.append(
            f"zg should have no FOOT bodies (found: {foot_bodies}); "
            "FOOT joints must be fixed/removed"
        )

    for knee_name in knee_bodies:
        knee_body = root.find(f".//body[@name='{knee_name}']")
        if knee_body is None:
            violations.append(f"zg structural: body '{knee_name}' not found")
            continue
        has_foot_sphere = any(
            g.get("type", "sphere") in ("sphere",)
            and _approx(float(g.get("size", "0")), 0.04)
            for g in knee_body.findall("geom")
            if g.get("size") and " " not in g.get("size", "")
        )
        if not has_foot_sphere:
            violations.append(
                f"zg structural: '{knee_name}' missing foot-contact sphere geom (size ~0.04)"
            )

    expected_ranges = {
        "FAR_HIP_JOINT": (-2.442, 2.791),
        "FBL_HIP_JOINT": (-2.442, 2.791),
        "RAR_HIP_JOINT": (-2.791, 2.442),
        "RBL_HIP_JOINT": (-2.791, 2.442),
    }
    for jname, (exp_lo, exp_hi) in expected_ranges.items():
        j = root.find(f".//joint[@name='{jname}']")
        if j is None:
            violations.append(f"zg structural: joint '{jname}' not found")
            continue
        parsed = _parse_range(j.get('range'))
        if parsed is None:
            violations.append(f"zg structural: joint '{jname}' has no valid range")
            continue
        lo, hi = parsed
        if not (_approx(lo, exp_lo) and _approx(hi, exp_hi)):
            violations.append(
                f"zg structural: joint '{jname}' range expected '{exp_lo} {exp_hi}', got '{lo} {hi}'"
            )

    return violations


def _zgw_structural(root: ET.Element) -> list[str]:
    """
    zgw foot structure: four FOOT_LINK bodies using zg naming (FAR/FBL/RAR/RBL),
    each with exactly one revolute joint (the wheel motor joint).
    """
    violations: list[str] = []
    expected_foot = {
        "FAR_FOOT_LINK": "FAR_FOOT_JOINT",
        "FBL_FOOT_LINK": "FBL_FOOT_JOINT",
        "RAR_FOOT_LINK": "RAR_FOOT_JOINT",
        "RBL_FOOT_LINK": "RBL_FOOT_JOINT",
    }

    for body_name, expected_joint in expected_foot.items():
        foot_body = root.find(f".//body[@name='{body_name}']")
        if foot_body is None:
            violations.append(f"zgw structural: FOOT body '{body_name}' not found")
            continue
        joints = [j.get("name") for j in foot_body.findall("joint")]
        if expected_joint not in joints:
            violations.append(
                f"zgw structural: '{body_name}' missing wheel joint '{expected_joint}' "
                f"(found: {joints})"
            )

    return violations


def _xgw_structural(root: ET.Element) -> list[str]:
    """
    xgw foot structure: four FOOT_LINK bodies, each with exactly one revolute
    joint (the wheel motor joint: FAR/FBL/RAR/RBL_FOOT_JOINT) and mesh geoms.
    """
    violations: list[str] = []
    expected_foot = {
        "FR_FOOT_LINK": "FAR_FOOT_JOINT",
        "FL_FOOT_LINK": "FBL_FOOT_JOINT",
        "RR_FOOT_LINK": "RAR_FOOT_JOINT",
        "RL_FOOT_LINK": "RBL_FOOT_JOINT",
    }

    for body_name, expected_joint in expected_foot.items():
        foot_body = root.find(f".//body[@name='{body_name}']")
        if foot_body is None:
            violations.append(f"xgw structural: FOOT body '{body_name}' not found")
            continue
        joints = [j.get("name") for j in foot_body.findall("joint")]
        if expected_joint not in joints:
            violations.append(
                f"xgw structural: '{body_name}' missing wheel joint '{expected_joint}' "
                f"(found: {joints})"
            )
        geoms = foot_body.findall("geom")
        mesh_geoms = [g for g in geoms if g.get("type", "mesh") in ("mesh", "")]
        if len(mesh_geoms) == 0:
            violations.append(
                f"xgw structural: '{body_name}' has no mesh geoms (wheel geometry missing)"
            )

    return violations


def _lite3_structural(root: ET.Element) -> list[str]:
    """
    lite3 reference structure: one base_link and twelve leg bodies, no FOOT bodies.
    """
    violations: list[str] = []
    expected_bodies = [
        "base_link",
        "FL_HIP", "FL_THIGH", "FL_SHANK",
        "FR_HIP", "FR_THIGH", "FR_SHANK",
        "HL_HIP", "HL_THIGH", "HL_SHANK",
        "HR_HIP", "HR_THIGH", "HR_SHANK",
    ]
    body_names = [b.get("name", "") for b in root.iter("body") if b.get("name")]
    missing = [name for name in expected_bodies if name not in body_names]
    extra = [name for name in body_names if name not in expected_bodies]
    if missing or extra:
        if missing:
            violations.append(f"lite3 structural: missing bodies {missing}")
        if extra:
            violations.append(f"lite3 structural: unexpected bodies {extra}")

    return violations


# Patch structural check functions into contracts (avoids forward-reference at dict literal time)
XGB_CONTRACT["structural_checks"] = _xgb_structural
XGW_CONTRACT["structural_checks"] = _xgw_structural
XXG_CONTRACT["structural_checks"] = _xgb_structural
ZG_CONTRACT["structural_checks"] = _zg_structural
ZGW_CONTRACT["structural_checks"] = _zgw_structural
LITE3_CONTRACT["structural_checks"] = _lite3_structural


# ──────────────────────────────────────────────────────────────────────────────
# Validators
# ──────────────────────────────────────────────────────────────────────────────

def validate_supported(xml_path: Path, contract: dict, profile: str) -> list[str]:
    """Return list of violation strings; empty = PASS."""
    violations: list[str] = []

    try:
        tree = ET.parse(xml_path)
    except ET.ParseError as e:
        return [f"XML parse error: {e}"]

    root = tree.getroot()

    # 1. freejoint
    if contract.get("requires_freejoint"):
        if root.find(".//freejoint") is None:
            violations.append("missing <freejoint> (robot has no floating base)")

    # 2. root body name
    expected_root_body = contract.get("root_body")
    if expected_root_body:
        worldbody = root.find("worldbody")
        if worldbody is not None:
            first_body = worldbody.find("body")
            actual = first_body.get("name") if first_body is not None else None
            if actual != expected_root_body:
                violations.append(
                    f"root body name: expected '{expected_root_body}', got '{actual}'"
                )

    # 3. actuator count and order
    actuator_elem = root.find("actuator")
    actual_motors = list(actuator_elem) if actuator_elem is not None else []
    expected_actuators = contract["actuators"]

    if len(actual_motors) != len(expected_actuators):
        violations.append(
            f"actuator count: expected {len(expected_actuators)}, got {len(actual_motors)}"
        )
    else:
        for i, (motor, (exp_name, exp_joint)) in enumerate(zip(actual_motors, expected_actuators)):
            act_name = motor.get("name")
            act_joint = motor.get("joint")
            if act_name != exp_name or act_joint != exp_joint:
                violations.append(
                    f"actuator[{i}]: expected name='{exp_name}' joint='{exp_joint}', "
                    f"got name='{act_name}' joint='{act_joint}'"
                )

    # 4. sensor names (order-sensitive)
    sensor_elem = root.find("sensor")
    actual_sensors = [s.get("name") for s in (sensor_elem or [])]
    expected_sensors = contract["sensors"]

    if len(actual_sensors) != len(expected_sensors):
        violations.append(
            f"sensor count: expected {len(expected_sensors)}, got {len(actual_sensors)}"
        )
    else:
        for i, (act, exp) in enumerate(zip(actual_sensors, expected_sensors)):
            if act != exp:
                violations.append(f"sensor[{i}]: expected '{exp}', got '{act}'")

    # 5. HIP joint range
    hip_range = contract.get("hip_joint_range")
    hip_names = contract.get("hip_joint_names", [])
    if hip_range:
        for jname in hip_names:
            j = root.find(f".//joint[@name='{jname}']")
            if j is None:
                violations.append(f"joint '{jname}' not found")
                continue
            parsed = _parse_range(j.get("range"))
            if parsed is None:
                violations.append(f"joint '{jname}' has no valid range")
            else:
                lo, hi = parsed
                exp_lo, exp_hi = hip_range
                if not (_approx(lo, exp_lo) and _approx(hi, exp_hi)):
                    violations.append(
                        f"joint '{jname}' range: expected '{exp_lo} {exp_hi}', "
                        f"got '{lo} {hi}'"
                    )

    # 6. frictionloss on all actuated joints
    expected_fl = contract.get("frictionloss")
    if expected_fl is not None:
        for name, _ in expected_actuators:
            # motor name → joint name via actuator list
            joint_name = next(
                (j for n, j in expected_actuators if n == name), None
            )
            if joint_name is None:
                continue
            j = root.find(f".//joint[@name='{joint_name}']")
            if j is None:
                continue
            fl_val = j.get("frictionloss")
            try:
                fl_float = float(fl_val) if fl_val else 0.0
            except ValueError:
                fl_float = 0.0
            if not _approx(fl_float, expected_fl):
                violations.append(
                    f"joint '{joint_name}' frictionloss: expected {expected_fl}, got {fl_val!r}"
                )

    # 7. actuatorfrcrange on all actuated joints
    exp_frc = contract.get("actuatorfrcrange")
    if exp_frc is not None:
        exp_lo, exp_hi = exp_frc
        for name, joint_name in expected_actuators:
            j = root.find(f".//joint[@name='{joint_name}']")
            if j is None:
                continue
            frc = j.get("actuatorfrcrange")
            parsed = _parse_range(frc)
            if parsed is None:
                violations.append(
                    f"joint '{joint_name}' actuatorfrcrange missing or invalid (got {frc!r})"
                )
            else:
                lo, hi = parsed
                if not (_approx(lo, exp_lo) and _approx(hi, exp_hi)):
                    violations.append(
                        f"joint '{joint_name}' actuatorfrcrange: "
                        f"expected '{exp_lo} {exp_hi}', got '{lo} {hi}'"
                    )

    # 8. Profile-specific structural checks
    structural = contract.get("structural_checks")
    if structural and callable(structural):
        violations.extend(structural(root))

    return violations


def validate_generic(xml_path: Path, urdf_path: Path | None = None) -> list[str]:
    """
    Sanity-check a generic robot XML (unknown profile).

    When urdf_path is provided, joints whose URDF <limit effort> > 0 are
    additionally required to have a matching actuatorfrcrange in the XML.
    """
    violations: list[str] = []

    try:
        tree = ET.parse(xml_path)
    except ET.ParseError as e:
        return [f"XML parse error: {e}"]

    root = tree.getroot()

    # Must have freejoint
    if root.find(".//freejoint") is None:
        violations.append("missing <freejoint>")

    # Must have at least one actuator
    actuator_elem = root.find("actuator")
    motors = list(actuator_elem) if actuator_elem is not None else []
    if len(motors) == 0:
        violations.append("no actuators defined")

    # Must have sensor block with IMU sensors
    sensor_elem = root.find("sensor")
    if sensor_elem is None:
        violations.append("missing <sensor> block")
    else:
        sensor_names = {s.get("name") for s in sensor_elem}
        for required in ("imu_quat", "imu_gyro", "imu_acc"):
            if required not in sensor_names:
                violations.append(f"missing required sensor '{required}'")

    # URDF-aware: joints with effort > 0 must have actuatorfrcrange in the XML
    if urdf_path is not None and urdf_path.is_file():
        try:
            urdf_root = ET.parse(urdf_path).getroot()
        except ET.ParseError as e:
            violations.append(f"URDF parse error: {e}")
            return violations

        for ujoint in urdf_root.findall("joint"):
            jname = ujoint.get("name")
            limit_elem = ujoint.find("limit")
            if not jname or limit_elem is None:
                continue
            try:
                effort = float(limit_elem.get("effort", "0"))
            except (ValueError, TypeError):
                continue
            if effort <= 0:
                continue
            # This joint has a non-zero effort limit; require actuatorfrcrange
            xml_joint = root.find(f".//joint[@name='{jname}']")
            if xml_joint is None:
                continue  # joint may have been removed (e.g. fixed joints stripped)
            frc = xml_joint.get("actuatorfrcrange")
            parsed = _parse_range(frc)
            if parsed is None:
                violations.append(
                    f"joint '{jname}' has URDF effort={effort:g} but "
                    f"actuatorfrcrange missing or invalid in XML (got {frc!r})"
                )
            else:
                exp_lo, exp_hi = -effort, effort
                lo, hi = parsed
                if not (_approx(lo, exp_lo) and _approx(hi, exp_hi)):
                    violations.append(
                        f"joint '{jname}' actuatorfrcrange mismatch: "
                        f"URDF effort={effort:g} → expected '{exp_lo:g} {exp_hi:g}', "
                        f"got '{lo} {hi}'"
                    )

    return violations


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

CONTRACTS = {
    "xgb": XGB_CONTRACT,
    "xxg": XXG_CONTRACT,
    "xgw": XGW_CONTRACT,
    "zg": ZG_CONTRACT,
    "zgw": ZGW_CONTRACT,
    "lite3": LITE3_CONTRACT,
}


def _count_actuators(xml_path: Path) -> int:
    try:
        root = ET.parse(xml_path).getroot()
        actuator_elem = root.find("actuator")
        return len(list(actuator_elem)) if actuator_elem is not None else 0
    except ET.ParseError:
        return 0


def main() -> int:
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <generated_xml> <profile> [urdf_path]", file=sys.stderr)
        print("  profile: xgb | xxg | xgw | zg | zgw | lite3 | generic", file=sys.stderr)
        print("  urdf_path: optional; enables effort-aware actuatorfrcrange check for generic", file=sys.stderr)
        return 2

    xml_path = Path(sys.argv[1])
    profile = sys.argv[2].lower()
    urdf_path = Path(sys.argv[3]) if len(sys.argv) > 3 else None

    if not xml_path.is_file():
        print(f"[FAIL] File not found: {xml_path}", file=sys.stderr)
        return 1

    if profile in CONTRACTS:
        violations = validate_supported(xml_path, CONTRACTS[profile], profile)
    elif profile == "generic":
        violations = validate_generic(xml_path, urdf_path)
    else:
        print(f"[FAIL] Unknown profile '{profile}'. Use: xgb | xxg | xgw | zg | zgw | lite3 | generic", file=sys.stderr)
        return 2

    if violations:
        print(f"[FAIL] Contract violations for profile '{profile}' in {xml_path}:")
        for v in violations:
            print(f"  ✗ {v}")
        return 1

    n_actuators = _count_actuators(xml_path)
    urdf_note = ", URDF effort-checked" if (profile == "generic" and urdf_path is not None and urdf_path.is_file()) else ""
    print(f"[PASS] {xml_path} — profile '{profile}' contract satisfied ({n_actuators} actuators{urdf_note})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
