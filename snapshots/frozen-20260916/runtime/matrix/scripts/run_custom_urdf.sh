#!/usr/bin/env bash
set -euo pipefail

ROBOT_ARG="${1:-custom}"
SCENE_ID="${2:-1}"
OFFSCREEN="${3:-0}"
PIXELSTREAM="${4:-0}"
MUJOCORUNNING="${5:-0}"
CUSTOM_URDF="${6:-}"
CUSTOM_NAME="${7:-}"
FORCE_REIMPORT="${SIM_LAUNCHER_FORCE_REIMPORT_CUSTOM_URDF:-0}"
PIPELINE_VERSION=13
MAP_KEY="custom"
MAP_ASSET="/Game/Maps/CustomWorld"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_LAUNCHER_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MATRIX_ROOT="${MATRIX_ROOT:-$SIM_LAUNCHER_ROOT}"
RUN_SIM_SH="$MATRIX_ROOT/scripts/run_sim.sh"

if [[ ! -f "$RUN_SIM_SH" ]]; then
    echo "[ERROR] run_sim.sh not found at: $RUN_SIM_SH" >&2
    exit 1
fi

if [[ -z "$CUSTOM_URDF" ]]; then
    echo "[ERROR] CUSTOM_URDF is empty" >&2
    exit 1
fi

if [[ ! -f "$CUSTOM_URDF" ]]; then
    echo "[ERROR] Custom URDF file not found: $CUSTOM_URDF" >&2
    exit 1
fi

if [[ -z "$CUSTOM_NAME" ]]; then
    CUSTOM_NAME="$(basename "$CUSTOM_URDF")"
    CUSTOM_NAME="${CUSTOM_NAME%.*}"
fi

CUSTOM_NAME="${CUSTOM_NAME//[^A-Za-z0-9_-]/_}"
if [[ -z "$CUSTOM_NAME" ]]; then
    CUSTOM_NAME="custom"
fi

echo "[INFO] ===== Custom URDF wrapper ====="
echo "[INFO] matrix root: $MATRIX_ROOT"
echo "[INFO] robot arg: $ROBOT_ARG"
echo "[INFO] scene id: $SCENE_ID"
echo "[INFO] custom name: $CUSTOM_NAME"
echo "[INFO] custom urdf: $CUSTOM_URDF"
echo "[INFO] force reimport: $FORCE_REIMPORT"

run_env_check() {
    if [[ "${MATRIX_SKIP_ENV_CHECK:-0}" == "1" ]]; then
        echo "[INFO] Environment check skipped by MATRIX_SKIP_ENV_CHECK=1"
        return 0
    fi

    local checker="$MATRIX_ROOT/scripts/check_env.sh"
    if [[ ! -x "$checker" ]]; then
        echo "[WARN] Environment checker not found or not executable: $checker"
        return 0
    fi

    "$checker" custom --custom-urdf "$CUSTOM_URDF"
}

run_env_check

MODEL_DIR="$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
UE_CUSTOM_ROOT="$MODEL_DIR/custom"
UE_CACHE_ROOT="$UE_CUSTOM_ROOT/_cache"
UE_ACTIVE_XML="$UE_CUSTOM_ROOT/current.xml"
UE_ACTIVE_SCENE_XML="$UE_CUSTOM_ROOT/scene.xml"
UE_SCENE_TERRAIN_XML="$UE_CUSTOM_ROOT/scene_terrain_custom.xml"
UE_ACTIVE_ASSETS="$UE_CUSTOM_ROOT/assets"
MUJOCO_CUSTOM_ROOT="$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/custom"
MUJOCO_CACHE_ROOT="$MUJOCO_CUSTOM_ROOT/_cache"
MUJOCO_ACTIVE_XML="$MUJOCO_CUSTOM_ROOT/current.xml"
MUJOCO_ACTIVE_SCENE_XML="$MUJOCO_CUSTOM_ROOT/scene.xml"
MUJOCO_SCENE_TERRAIN_XML="$MUJOCO_CUSTOM_ROOT/scene_terrain_custom.xml"
MUJOCO_ACTIVE_ASSETS="$MUJOCO_CUSTOM_ROOT/assets"
IMPORT_DIR="$MUJOCO_CACHE_ROOT/$CUSTOM_NAME"
UE_IMPORT_DIR="$UE_CACHE_ROOT/$CUSTOM_NAME"

mkdir -p "$UE_CUSTOM_ROOT" "$UE_CACHE_ROOT" "$MUJOCO_CUSTOM_ROOT" "$MUJOCO_CACHE_ROOT" "$IMPORT_DIR" "$UE_IMPORT_DIR"
echo "[INFO] import dir: $IMPORT_DIR"
echo "[INFO] ue import dir: $UE_IMPORT_DIR"

URDF_BASENAME="$(basename "$CUSTOM_URDF")"
URDF_EXT="${URDF_BASENAME##*.}"
ASSET_DIR_NAME="assets"
TARGET_URDF="$IMPORT_DIR/$CUSTOM_NAME.urdf"
TARGET_XML="$IMPORT_DIR/$CUSTOM_NAME.xml"
TARGET_METADATA="$IMPORT_DIR/manifest.json"
UE_TARGET_URDF="$UE_IMPORT_DIR/$CUSTOM_NAME.urdf"
UE_TARGET_XML="$UE_IMPORT_DIR/$CUSTOM_NAME.xml"
UE_TARGET_METADATA="$UE_IMPORT_DIR/manifest.json"
TMP_ASSET_DIR="$IMPORT_DIR/$ASSET_DIR_NAME"
UE_ASSET_DIR="$UE_IMPORT_DIR/$ASSET_DIR_NAME"

detect_reference_profile() {
    local urdf_path="$1"
    local basename_lower
    basename_lower="$(basename "$urdf_path" | tr '[:upper:]' '[:lower:]')"
    case "$basename_lower" in
        # xgb: 12-DOF leg robot (xg_b / xg-b variants)
        xg_b.urdf|xg-b.urdf|xgb.urdf)
            echo "xgb"
            ;;
        # xxg: xgb-family reference robot (URDF name xxg)
        xxg.urdf|xxg.xml|xxg)
            echo "xxg"
            ;;
        # lite3: Unitree Lite3 reference robot
        lite3.urdf|lite3.xml|lite3)
            echo "lite3"
            ;;
        # xgw: 16-DOF wheel-leg robot (xg_wheel / xg-wheel variants)
        xg_wheel.urdf|xg-wheel.urdf|xgw.urdf)
            echo "xgw"
            ;;
        # zg: 12-DOF leg robot (zg / zg.xml variants)
        zg.urdf|zg.xml|zg)
            echo "zg"
            ;;
        # zgw: 16-DOF wheel-leg robot (zg_wheel / zg-wheel variants)
        zg_wheel.urdf|zg-wheel.urdf|zgw.urdf)
            echo "zgw"
            ;;
        # zgws: 16-DOF wheel-spine robot (placeholder — URDF filename TBD)
        zgws.urdf|zg_wheel_spine.urdf|zg-wheel-spine.urdf)
            echo "zgws"
            ;;
        # go2: Unitree Go2 quadruped (placeholder — URDF filename TBD)
        go2.urdf|unitree_go2.urdf)
            echo "go2"
            ;;
        # go2w: Unitree Go2W wheel-leg (placeholder — URDF filename TBD)
        go2w.urdf|unitree_go2w.urdf)
            echo "go2w"
            ;;
        *)
            echo ""
            ;;
    esac
}

get_reference_mujoco_xml() {
    local profile="$1"
    case "$profile" in
        xgb)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xgb/xgb.xml"
            ;;
        xxg)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xxg/xxg.xml"
            ;;
        lite3)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/lite3/lite3.xml"
            ;;
        xgw)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xgw/xgw.xml"
            ;;
        zg)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zg/zg.xml"
            ;;
        zgw)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zgw/zgw.xml"
            ;;
        zgws)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zgws/zgws.xml"
            ;;
        go2)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/go2/go2.xml"
            ;;
        go2w)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/go2w/go2w.xml"
            ;;
        *)
            return 1
            ;;
    esac
}

get_reference_ue_xml() {
    local profile="$1"
    case "$profile" in
        xgb)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb/xgb.xml"
            ;;
        xxg)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xxg/xxg.xml"
            ;;
        lite3)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/lite3/lite3.xml"
            ;;
        xgw)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgw/xgw.xml"
            ;;
        zg)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zg/zg.xml"
            ;;
        zgw)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zgw/zgw.xml"
            ;;
        zgws)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zgws/zgws.xml"
            ;;
        go2)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2/go2.xml"
            ;;
        go2w)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/go2w.xml"
            ;;
        *)
            return 1
            ;;
    esac
}

get_reference_mujoco_assets() {
    local profile="$1"
    case "$profile" in
        xgb)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xgb/assets"
            ;;
        xxg)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xxg/assets"
            ;;
        lite3)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/lite3/assets"
            ;;
        xgw)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/xgw/assets"
            ;;
        zg)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zg/assets"
            ;;
        zgw)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zgw/assets"
            ;;
        zgws)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/zgws/assets"
            ;;
        go2)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/go2/assets"
            ;;
        go2w)
            echo "$MATRIX_ROOT/src/robot_mujoco/zsibot_robots/go2w/assets"
            ;;
        *)
            return 1
            ;;
    esac
}

get_reference_ue_assets() {
    local profile="$1"
    case "$profile" in
        xgb)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb/assets"
            ;;
        xxg)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xxg/assets"
            ;;
        lite3)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/lite3/assets"
            ;;
        xgw)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgw/assets"
            ;;
        zg)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zg/assets"
            ;;
        zgw)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zgw/assets"
            ;;
        zgws)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/zgws/assets"
            ;;
        go2)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2/assets"
            ;;
        go2w)
            echo "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/assets"
            ;;
        *)
            return 1
            ;;
    esac
}

preflight_custom_urdf() {
    local urdf_path="$1"
    local asset_dir="$2"

    python3 - "$urdf_path" "$asset_dir" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path = Path(sys.argv[1])
asset_dir = Path(sys.argv[2])

def family(name: str):
    if name.startswith(("FL_", "FR_", "RL_", "RR_")):
        return "unitree"
    if name.startswith(("FBL_", "FAR_", "RBL_", "RAR_")):
        return "legacy"
    return None

def stl_issue(path: Path, robot_name: str):
    try:
        data = path.read_bytes()
    except OSError as exc:
        return f"cannot read mesh: {exc}"
    if len(data) < 84:
        return f"STL too small ({len(data)} bytes)"
    tri_count = int.from_bytes(data[80:84], "little")
    expected_size = 84 + tri_count * 50
    if len(data) != expected_size:
        return f"STL header/size mismatch (triangles={tri_count}, size={len(data)}, expected={expected_size})"
    if tri_count > 200000:
        return f"STL triangle count {tri_count} exceeds converter limit 200000"
    return None

try:
    root = ET.parse(urdf_path).getroot()
except ET.ParseError as exc:
    print(f"[ERROR] preflight failed: URDF parse error: {exc}", file=sys.stderr)
    sys.exit(1)

mesh_refs = []
for mesh in root.iter("mesh"):
    filename = mesh.get("filename", "")
    if filename:
        mesh_refs.append(filename)

if not mesh_refs:
    print(f"[INFO] preflight passed for {urdf_path.name} (no mesh refs)")
    sys.exit(0)

ref_basenames = sorted({Path(fn).name for fn in mesh_refs})
asset_basenames = sorted({p.name for p in asset_dir.iterdir() if p.is_file()}) if asset_dir.is_dir() else []
robot_name = urdf_path.stem.lower()

ref_families = {family(name) for name in ref_basenames if family(name)}
asset_families = {family(name) for name in asset_basenames if family(name)}

violations = []
for ref_name in ref_basenames:
    mesh_path = asset_dir / ref_name
    if not mesh_path.is_file():
        violations.append(f"missing mesh file: {ref_name} (expected at {mesh_path})")
        continue
    if mesh_path.suffix.lower() == ".stl":
        issue = stl_issue(mesh_path, robot_name)
        if issue:
            violations.append(f"mesh '{ref_name}' cannot be converted by current pipeline: {issue}")

if ref_families and asset_families and ref_families != asset_families:
    violations.append(
        f"mesh naming mismatch: URDF uses {sorted(ref_families)}-style names, "
        f"assets use {sorted(asset_families)}-style names"
    )

if violations:
    print(f"[ERROR] preflight failed for {urdf_path.name}:")
    for violation in violations:
        print(f"  - {violation}")
    sys.exit(1)

print(f"[INFO] preflight passed for {urdf_path.name} ({len(ref_basenames)} mesh refs)")
PY
}

rewrite_mesh_paths() {
    local target_file="$1"
    sed -i "s|filename=\"../meshes/|filename=\"${ASSET_DIR_NAME}/|g" "$target_file" || true
}

normalize_mesh_aliases() {
    local target_file="$1"
    local asset_dir="$2"

    python3 - "$target_file" "$asset_dir" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path = Path(sys.argv[1])
asset_dir = Path(sys.argv[2])

if not urdf_path.is_file() or not asset_dir.is_dir():
    sys.exit(0)

available = {p.name for p in asset_dir.iterdir() if p.is_file()}
aliases = {
    "FL_": "FBL_",
    "FR_": "FAR_",
    "RL_": "RBL_",
    "RR_": "RAR_",
    "FBL_": "FL_",
    "FAR_": "FR_",
    "RBL_": "RL_",
    "RAR_": "RR_",
}

def alias_for(name: str):
    if name in available:
        return None
    for src, dst in aliases.items():
        if name.startswith(src):
            candidate = dst + name[len(src):]
            if candidate in available:
                return candidate
    return None

try:
    tree = ET.parse(urdf_path)
except ET.ParseError:
    sys.exit(0)

changed = False
for mesh in tree.getroot().iter("mesh"):
    filename = mesh.get("filename", "")
    if not filename:
        continue
    path = Path(filename)
    aliased = alias_for(path.name)
    if aliased:
        mesh.set("filename", str(path.with_name(aliased)))
        changed = True

if changed:
    tree.write(str(urdf_path), encoding="unicode", xml_declaration=True)
    print(f"[INFO] normalized mesh aliases in URDF: {urdf_path}")
PY
}

normalize_link_aliases() {
    local target_file="$1"

    python3 - "$target_file" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path = Path(sys.argv[1])
if not urdf_path.is_file():
    sys.exit(0)

aliases = {
    "RAR_FOOT_LINK": "RR_FOOT_LINK",
}

try:
    tree = ET.parse(urdf_path)
except ET.ParseError:
    sys.exit(0)

changed = False
for elem in tree.getroot().iter():
    for attr, value in list(elem.attrib.items()):
        replacement = aliases.get(value)
        if replacement:
            elem.set(attr, replacement)
            changed = True

if changed:
    tree.write(str(urdf_path), encoding="unicode", xml_declaration=True)
    print(f"[INFO] normalized link aliases in URDF: {urdf_path}")
PY
}

write_bbox_stl_proxy_from_binary_stl() {
    local src_stl="$1"
    local dst_stl="$2"

    python3 - "$src_stl" "$dst_stl" <<'PY'
import math
import struct
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])

if not src.is_file():
    print(f"[WARN] proxy source missing: {src}")
    sys.exit(1)

with src.open("rb") as f:
    f.seek(80)
    tri_count_raw = f.read(4)
    if len(tri_count_raw) != 4:
        print(f"[WARN] proxy source truncated: {src}")
        sys.exit(1)
    tri_count = struct.unpack("<I", tri_count_raw)[0]
    payload = f.read(tri_count * 50)
    if len(payload) != tri_count * 50:
        print(f"[WARN] proxy source triangle payload truncated: {src}")
        sys.exit(1)

mn = [math.inf, math.inf, math.inf]
mx = [-math.inf, -math.inf, -math.inf]
for vals in struct.iter_unpack("<12fH", payload):
    verts = vals[3:12]
    for i in range(0, 9, 3):
        x, y, z = verts[i:i + 3]
        if x < mn[0]:
            mn[0] = x
        if y < mn[1]:
            mn[1] = y
        if z < mn[2]:
            mn[2] = z
        if x > mx[0]:
            mx[0] = x
        if y > mx[1]:
            mx[1] = y
        if z > mx[2]:
            mx[2] = z

if not all(math.isfinite(v) for v in (*mn, *mx)):
    print(f"[WARN] failed to derive proxy bounds from: {src}")
    sys.exit(1)

cx = (mn[0] + mx[0]) * 0.5
cy = (mn[1] + mx[1]) * 0.5
cz = (mn[2] + mx[2]) * 0.5
hx = max((mx[0] - mn[0]) * 0.5, 1e-3)
hy = max((mx[1] - mn[1]) * 0.5, 1e-3)
hz = max((mx[2] - mn[2]) * 0.5, 1e-3)

verts = {
    "lbf": (cx - hx, cy - hy, cz - hz),
    "rbf": (cx + hx, cy - hy, cz - hz),
    "rtf": (cx + hx, cy + hy, cz - hz),
    "ltf": (cx - hx, cy + hy, cz - hz),
    "lbb": (cx - hx, cy - hy, cz + hz),
    "rbb": (cx + hx, cy - hy, cz + hz),
    "rtb": (cx + hx, cy + hy, cz + hz),
    "ltb": (cx - hx, cy + hy, cz + hz),
}

faces = [
    ("0 0 -1", ("lbf", "rbf", "rtf")),
    ("0 0 -1", ("lbf", "rtf", "ltf")),
    ("0 0 1", ("lbb", "rtb", "rbb")),
    ("0 0 1", ("lbb", "ltb", "rtb")),
    ("0 -1 0", ("lbf", "lbb", "rbb")),
    ("0 -1 0", ("lbf", "rbb", "rbf")),
    ("0 1 0", ("ltf", "rtf", "rtb")),
    ("0 1 0", ("ltf", "rtb", "ltb")),
    ("-1 0 0", ("lbf", "ltf", "ltb")),
    ("-1 0 0", ("lbf", "ltb", "lbb")),
    ("1 0 0", ("rbf", "rbb", "rtb")),
    ("1 0 0", ("rbf", "rtb", "rtf")),
]

dst.parent.mkdir(parents=True, exist_ok=True)
# Write binary STL — ASCII STL is rejected by MuJoCo's STL loader when the
# header starts with "solid", so we must use binary format here.
with dst.open("wb") as out:
    header = b"zg_base_link_proxy\x00" + b"\x00" * (80 - len(b"zg_base_link_proxy\x00"))
    out.write(header)
    out.write(struct.pack("<I", len(faces)))
    for normal_str, tri in faces:
        nx, ny, nz = (float(v) for v in normal_str.split())
        out.write(struct.pack("<fff", nx, ny, nz))
        for corner in tri:
            x, y, z = verts[corner]
            out.write(struct.pack("<fff", x, y, z))
        out.write(struct.pack("<H", 0))  # attribute byte count

print(
    f"[INFO] generated zg BASE_LINK proxy from bounds of {src}: "
    f"center=({cx:.6f}, {cy:.6f}, {cz:.6f}), "
    f"size=({2*hx:.6f}, {2*hy:.6f}, {2*hz:.6f})"
)
PY
}

normalize_mjcf() {
    local target_xml="$1"
    if [[ ! -f "$target_xml" ]]; then
        return
    fi

    if grep -q 'type=""' "$target_xml"; then
        sed -i 's/type="" rgba/type="sphere" rgba/g' "$target_xml"
        echo "[INFO] normalized empty geom types in xml: $target_xml"
    fi

    sed -i 's/meshdir="meshes"/meshdir="assets"/g' "$target_xml"

    python3 - "$target_xml" <<'PY'
import sys, re
from pathlib import Path
import xml.etree.ElementTree as ET

path = Path(sys.argv[1])
content = path.read_text()

def fix_sphere(match):
    tag = match.group(0)
    if 'size=' not in tag:
        tag = tag.replace('<geom', '<geom size="0.05"', 1)
    return tag

content = re.sub(r'<geom\b[^>]*type="sphere"[^>]*/>', fix_sphere, content)
path.write_text(content)

tree = ET.parse(path)
root = tree.getroot()

asset = root.find("asset")
if asset is not None:
    seen_mesh = set()
    for child in list(asset):
        if child.tag == "mesh":
            mesh_name = child.get("name")
            if not mesh_name:
                continue
            if mesh_name in seen_mesh:
                asset.remove(child)
                continue
            seen_mesh.add(mesh_name)
        elif child.tag == "texture" and child.get("name") == "texplane":
            asset.remove(child)
        elif child.tag == "material" and child.get("name") == "matplane":
            asset.remove(child)

worldbody = root.find("worldbody")
if worldbody is not None:
    for child in list(worldbody):
        if child.tag == "geom" and child.get("type") == "plane":
            worldbody.remove(child)
        elif child.tag == "light":
            worldbody.remove(child)
    seen_world_geoms = set()
    for child in list(worldbody):
        if child.tag != "geom":
            continue
        key = tuple(sorted(child.attrib.items()))
        if key in seen_world_geoms:
            worldbody.remove(child)
        else:
            seen_world_geoms.add(key)

for body in root.iter("body"):
    seen_geoms = set()
    for child in list(body):
        if child.tag != "geom":
            continue
        key = tuple(sorted(child.attrib.items()))
        if key in seen_geoms:
            body.remove(child)
        else:
            seen_geoms.add(key)

tree.write(path, encoding="utf-8", xml_declaration=False)
PY
    echo "[INFO] normalized meshdir and sphere size in xml: $target_xml"
}

write_scene_terrain_custom_template() {
    local target_file="$1"
    mkdir -p "$(dirname "$target_file")"
    cat > "$target_file" <<'EOF'
<mujoco model="custom scene">
  <include file="current.xml" />
  <statistic center="0 0 0.1" extent="0.8" />
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
    <rgba haze="0.15 0.25 0.35 1" />
    <global azimuth="-130" elevation="-20" />
    <map znear="0.01" zfar="100"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />
    <hfield name="perlin_hfield" size="1.0 0.75 0.2 0.2" file="../height_field.png" />
    <hfield name="image_hfield" size="1.0 1.0 0.02 0.1" file="../unitree_hfield.png" />
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true" />
    <geom name="floor" pos="0 0.0 0.01" size="0 0 0.01" type="plane" material="groundplane" />
    <geom name="BP_Scene_Cube_1_C_0" type="box" size="0.5 0.5 0.5" pos="10.0 -6.0 0.5" quat="1.0 0.0 0.0 0.0" rgba="0.8 0.2 0.2 1" solref="0.02 1" solimp="0.9 0.95 0.001"/>
    <geom name="BP_Scene_Cube_2_C_0" type="box" size="0.5 0.5 0.5" pos="10.0 6.0 0.5" quat="1.0 0.0 0.0 0.0" rgba="0.8 0.2 0.2 1" solref="0.02 1" solimp="0.9 0.95 0.001"/>
  </worldbody>
</mujoco>
EOF
}

write_scene_template() {
    local target_file="$1"
    mkdir -p "$(dirname "$target_file")"
    cat > "$target_file" <<'EOF'
<mujoco model="custom scene">
  <include file="current.xml" />

  <statistic center="0 0 0.1" extent="0.8" />

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
    <rgba haze="0.15 0.25 0.35 1" />
    <global azimuth="-130" elevation="-20" />
    <map znear="0.01" zfar="100"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true" />
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" />
  </worldbody>
</mujoco>
EOF
}

sync_runtime_layout() {
    local cache_xml="$1"
    local cache_urdf="$2"
    local cache_assets_dir="$3"
    local active_root="$4"
    local active_xml="$5"
    local active_scene_xml="$6"
    local active_assets_dir="$7"

    if [[ ! -f "$cache_xml" ]]; then
        return
    fi

    cp "$cache_xml" "$active_xml"
    if [[ -n "$REFERENCE_PROFILE" ]]; then
        echo "[INFO] using reference runtime layout for profile '$REFERENCE_PROFILE': $active_xml"
    else
        normalize_mjcf "$active_xml"
        restore_urdf_visual_meshes "$cache_urdf" "$active_xml"
        restore_urdf_fixed_links "$cache_urdf" "$active_xml"
        echo "[INFO] restoring generic runtime layout in: $active_xml"
        restore_generic_runtime_layout "$active_xml" "$cache_urdf"
    fi

    write_scene_template "$active_scene_xml"

    rm -rf "$active_assets_dir"
    if [[ -d "$cache_assets_dir" ]]; then
        mkdir -p "$active_assets_dir"
        cp -r "$cache_assets_dir/." "$active_assets_dir/"
        echo "[INFO] synced assets to: $active_assets_dir"
    fi

    if [[ ! -f "$active_root/scene_terrain_custom.xml" ]]; then
        write_scene_terrain_custom_template "$active_root/scene_terrain_custom.xml"
    fi

    # Ensure height-field PNGs are present in active_root
    # (needed because meshdir="assets" makes MuJoCo resolve "../height_field.png" as
    # active_root/height_field.png).
    local _robots_root
    _robots_root="$(dirname "$active_root")"
    for _png in height_field.png unitree_hfield.png; do
        if [[ ! -f "$active_root/$_png" && -f "$_robots_root/xgb/$_png" ]]; then
            cp "$_robots_root/xgb/$_png" "$active_root/$_png"
            echo "[INFO] copied $_png to $active_root/"
        fi
    done

    # Static contract validation
    local _validate_script
    _validate_script="$(dirname "$(realpath "${BASH_SOURCE[0]}")")/validate_xml_contract.py"
    if [[ -f "$_validate_script" ]]; then
        local _profile="${REFERENCE_PROFILE:-generic}"
        # Build argument array — avoids word-splitting on paths with spaces
        local _validate_args=("$active_xml" "$_profile")
        # For generic profile pass the source URDF so effort-aware actuatorfrcrange check runs
        if [[ -z "$REFERENCE_PROFILE" && -f "$cache_urdf" ]]; then
            _validate_args+=("$cache_urdf")
        fi
        if python3 "$_validate_script" "${_validate_args[@]}"; then
            : # PASS — nothing to do
        else
            echo "[WARN] Contract validation FAILED for $active_xml (profile: $_profile)" >&2
            echo "[WARN] Simulation may behave incorrectly. Check violations above." >&2
        fi
    fi
}

restore_urdf_visual_meshes() {
    local urdf_path="$1"
    local mjcf_path="$2"
    if [[ ! -f "$urdf_path" || ! -f "$mjcf_path" ]]; then
        return
    fi

    python3 - "$urdf_path" "$mjcf_path" <<'PY'
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

urdf_path = Path(sys.argv[1])
mjcf_path = Path(sys.argv[2])

urdf_root = ET.parse(urdf_path).getroot()
mjcf_tree = ET.parse(mjcf_path)
mjcf_root = mjcf_tree.getroot()

asset = mjcf_root.find("asset")
if asset is None:
    asset = ET.SubElement(mjcf_root, "asset")

existing_mesh_names = {mesh.get("name") for mesh in asset.findall("mesh") if mesh.get("name")}

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> str:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return f"{w:.9g} {x:.9g} {y:.9g} {z:.9g}"

def find_body(root: ET.Element, body_name: str):
    for body in root.iter("body"):
        if body.get("name") == body_name:
            return body
    return None

def has_mesh_geom(body: ET.Element, mesh_name: str) -> bool:
    for geom in body.findall("geom"):
        if geom.get("mesh") == mesh_name:
            return True
    return False

def insert_before_child_bodies(body: ET.Element, element: ET.Element) -> None:
    insert_at = len(body)
    for idx, child in enumerate(list(body)):
        if child.tag == "body":
            insert_at = idx
            break
    body.insert(insert_at, element)

for link in urdf_root.findall("link"):
    link_name = link.get("name")
    if not link_name:
        continue

    visual = link.find("visual")
    if visual is None:
        continue

    geometry = visual.find("geometry")
    if geometry is None:
        continue

    mesh = geometry.find("mesh")
    if mesh is None:
        continue

    filename = mesh.get("filename")
    if not filename:
        continue

    mesh_name = Path(filename).stem
    if mesh_name not in existing_mesh_names:
        ET.SubElement(
            asset,
            "mesh",
            attrib={
                "name": mesh_name,
                "content_type": "model/stl",
                "file": Path(filename).name,
            },
        )
        existing_mesh_names.add(mesh_name)

    body = find_body(mjcf_root, link_name)
    if body is None:
        continue
    if has_mesh_geom(body, mesh_name):
        continue

    origin = visual.find("origin")
    pos = "0 0 0"
    quat = "1 0 0 0"
    if origin is not None:
        xyz = origin.get("xyz", "0 0 0").split()
        rpy = origin.get("rpy", "0 0 0").split()
        if len(xyz) == 3:
            pos = " ".join(xyz)
        if len(rpy) == 3:
            try:
                quat = rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            except ValueError:
                quat = "1 0 0 0"

    material = visual.find("material")
    rgba = "0.75294 0.75294 0.75294 1"
    if material is not None:
        color = material.find("color")
        if color is not None and color.get("rgba"):
            rgba = color.get("rgba")

    geom = ET.Element(
        "geom",
        attrib={
            "type": "mesh",
            "mesh": mesh_name,
            "rgba": rgba,
            "class": "visualgeom",
            "contype": "0",
            "conaffinity": "0",
            "density": "0",
            "group": "2",
        },
    )
    if pos != "0 0 0":
        geom.set("pos", pos)
    if quat != "1 0 0 0":
        geom.set("quat", quat)

    insert_before_child_bodies(body, geom)

mjcf_tree.write(mjcf_path, encoding="utf-8", xml_declaration=False)
print(f"[INFO] restored URDF visual meshes in xml: {mjcf_path}")
PY
}

restore_urdf_fixed_links() {
    local urdf_path="$1"
    local mjcf_path="$2"
    if [[ ! -f "$urdf_path" || ! -f "$mjcf_path" ]]; then
        return
    fi

    python3 - "$urdf_path" "$mjcf_path" <<'PY'
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

urdf_path = Path(sys.argv[1])
mjcf_path = Path(sys.argv[2])

urdf_root = ET.parse(urdf_path).getroot()
mjcf_tree = ET.parse(mjcf_path)
mjcf_root = mjcf_tree.getroot()

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> str:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return f"{w:.9g} {x:.9g} {y:.9g} {z:.9g}"

def find_body(root: ET.Element, body_name: str):
    for body in root.iter("body"):
        if body.get("name") == body_name:
            return body
    return None

def insert_before_child_bodies(body: ET.Element, element: ET.Element) -> None:
    insert_at = len(body)
    for idx, child in enumerate(list(body)):
        if child.tag == "body":
            insert_at = idx
            break
    body.insert(insert_at, element)

def find_joint_for_child(child_name: str):
    for joint in urdf_root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link") == child_name:
            return joint
    return None

def parse_xyz(value: str | None) -> str:
    if not value:
        return "0 0 0"
    parts = value.split()
    if len(parts) != 3:
        return "0 0 0"
    return " ".join(parts)

def parse_quat_from_rpy(value: str | None) -> str:
    if not value:
        return "1 0 0 0"
    parts = value.split()
    if len(parts) != 3:
        return "1 0 0 0"
    try:
        return rpy_to_quat(float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return "1 0 0 0"

def add_inertial_from_urdf(body: ET.Element, link: ET.Element) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        return
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None or not mass.get("value"):
        return
    origin = inertial.find("origin")
    pos = parse_xyz(origin.get("xyz") if origin is not None else None)
    quat = parse_quat_from_rpy(origin.get("rpy") if origin is not None else None)
    diag = [
        inertia.get("ixx"),
        inertia.get("iyy"),
        inertia.get("izz"),
    ]
    if any(v is None for v in diag):
        return
    ET.SubElement(
        body,
        "inertial",
        attrib={
            "pos": pos,
            "quat": quat,
            "mass": mass.get("value"),
            "diaginertia": f"{diag[0]} {diag[1]} {diag[2]}",
        },
    )

def add_visual_mesh_geom(body: ET.Element, link: ET.Element) -> None:
    visual = link.find("visual")
    if visual is None:
        return
    mesh = visual.find("geometry/mesh")
    if mesh is None:
        return
    filename = mesh.get("filename")
    if not filename:
        return
    mesh_name = Path(filename).stem
    if any(g.get("mesh") == mesh_name for g in body.findall("geom")):
        return
    origin = visual.find("origin")
    pos = parse_xyz(origin.get("xyz") if origin is not None else None)
    quat = parse_quat_from_rpy(origin.get("rpy") if origin is not None else None)
    material = visual.find("material/color")
    rgba = material.get("rgba") if material is not None and material.get("rgba") else "0.75294 0.75294 0.75294 1"
    geom = ET.Element(
        "geom",
        attrib={
            "name": f"{link.get('name', 'fixed')}_visual",
            "type": "mesh",
            "mesh": mesh_name,
            "material": "default_material",
            "class": "visual",
        },
    )
    if rgba != "0.75294 0.75294 0.75294 1":
        geom.set("rgba", rgba)
    if pos != "0 0 0":
        geom.set("pos", pos)
    if quat != "1 0 0 0":
        geom.set("quat", quat)
    insert_before_child_bodies(body, geom)

def add_fixed_collision_geom(body: ET.Element, link: ET.Element) -> None:
    collisions = link.findall("collision")
    if not collisions:
        return
    # Prefer the fixed-foot sphere collision if present.
    for collision in collisions:
        geometry = collision.find("geometry")
        if geometry is None:
            continue
        sphere = geometry.find("sphere")
        if sphere is None or not sphere.get("radius"):
            continue
        radius = sphere.get("radius")
        origin = collision.find("origin")
        pos = parse_xyz(origin.get("xyz") if origin is not None else None)
        quat = parse_quat_from_rpy(origin.get("rpy") if origin is not None else None)
        # Remove a placeholder sphere inherited on the parent if the joint origin matches it.
        geom = ET.Element(
            "geom",
            attrib={
                "name": f"{link.get('name', 'fixed')}_collision",
                "type": "sphere",
                "size": radius,
                "class": "collision",
            },
        )
        if pos != "0 0 0":
            geom.set("pos", pos)
        if quat != "1 0 0 0":
            geom.set("quat", quat)
        insert_before_child_bodies(body, geom)
        return

for link in urdf_root.findall("link"):
    link_name = link.get("name")
    if not link_name:
        continue
    if find_body(mjcf_root, link_name) is not None:
        continue

    joint = find_joint_for_child(link_name)
    if joint is None or joint.get("type") != "fixed":
        continue

    parent = joint.find("parent")
    parent_link = parent.get("link") if parent is not None else None
    if not parent_link:
        continue
    parent_body = find_body(mjcf_root, parent_link)
    if parent_body is None:
        continue

    origin = joint.find("origin")
    pos = parse_xyz(origin.get("xyz") if origin is not None else None)
    quat = parse_quat_from_rpy(origin.get("rpy") if origin is not None else None)
    child_body = ET.Element(
        "body",
        attrib={
            "name": link_name,
            "pos": pos,
            "quat": quat,
        },
    )
    add_inertial_from_urdf(child_body, link)
    add_visual_mesh_geom(child_body, link)
    add_fixed_collision_geom(child_body, link)
    insert_before_child_bodies(parent_body, child_body)

mjcf_tree.write(mjcf_path, encoding="utf-8", xml_declaration=False)
print(f"[INFO] restored fixed-link bodies from URDF in xml: {mjcf_path}")
PY
}

# DEPRECATED: restore_reference_markers_and_sensors was the old xgb-only post-processing
# pipeline. It is no longer called anywhere. Supported robots (xgb/xgw/zgws/go2/go2w)
# now use the reference-XML direct-copy path (sync_runtime_layout); unknown robots use
# restore_generic_runtime_layout. Do not resurrect this function.
restore_reference_markers_and_sensors() {
    echo '[ERROR] restore_reference_markers_and_sensors is deprecated and must not be called' >&2
    return 1
}

restore_generic_runtime_layout() {
    local mjcf_path="$1"
    local urdf_path="${2:-}"
    if [[ ! -f "$mjcf_path" ]]; then
        return
    fi

    python3 - "$mjcf_path" "$urdf_path" <<'PY'
from pathlib import Path
import math
import re
import sys
import xml.etree.ElementTree as ET

mjcf_path = Path(sys.argv[1])
urdf_path = Path(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else None

tree = ET.parse(mjcf_path)
root = tree.getroot()
urdf_root = ET.parse(urdf_path).getroot() if urdf_path is not None and urdf_path.is_file() else None
urdf_links = {}
urdf_joint_effort = {}  # joint_name -> effort (float), from URDF <limit effort="...">
if urdf_root is not None:
    urdf_links = {
        link.get("name"): link
        for link in urdf_root.findall("link")
        if link.get("name")
    }
    for ujoint in urdf_root.findall("joint"):
        jname = ujoint.get("name")
        limit_elem = ujoint.find("limit")
        if jname and limit_elem is not None:
            try:
                urdf_joint_effort[jname] = float(limit_elem.get("effort", "0"))
            except (ValueError, TypeError):
                pass

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> str:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return f"{w:.9g} {x:.9g} {y:.9g} {z:.9g}"

def parse_xyz(value: str | None) -> str:
    if not value:
        return "0 0 0"
    parts = value.split()
    if len(parts) != 3:
        return "0 0 0"
    return " ".join(parts)

def parse_quat_from_rpy(value: str | None) -> str:
    if not value:
        return "1 0 0 0"
    parts = value.split()
    if len(parts) != 3:
        return "1 0 0 0"
    try:
        return rpy_to_quat(float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return "1 0 0 0"

def xyz_to_tuple(value: str) -> tuple[float, float, float]:
    parts = value.split()
    if len(parts) != 3:
        return (0.0, 0.0, 0.0)
    return tuple(float(part) for part in parts)

def quat_to_tuple(value: str) -> tuple[float, float, float, float]:
    parts = value.split()
    if len(parts) != 4:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(float(part) for part in parts)

def quat_mul(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )

def quat_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)

def rotate_vec(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    qv = (0.0, v[0], v[1], v[2])
    r = quat_mul(quat_mul(q, qv), quat_conj(q))
    return (r[1], r[2], r[3])

def walk_body_poses(body: ET.Element, parent_pos=(0.0, 0.0, 0.0), parent_quat=(1.0, 0.0, 0.0, 0.0)):
    local_pos = xyz_to_tuple(body.get("pos", "0 0 0"))
    local_quat = quat_to_tuple(body.get("quat", "1 0 0 0"))
    rotated = rotate_vec(parent_quat, local_pos)
    world_pos = (
        parent_pos[0] + rotated[0],
        parent_pos[1] + rotated[1],
        parent_pos[2] + rotated[2],
    )
    world_quat = quat_mul(parent_quat, local_quat)
    yield body, world_pos, world_quat
    for child in body.findall("body"):
        yield from walk_body_poses(child, world_pos, world_quat)

def insert_before_child_bodies(body: ET.Element, element: ET.Element) -> None:
    insert_at = len(body)
    for idx, child in enumerate(list(body)):
        if child.tag == "body":
            insert_at = idx
            break
    body.insert(insert_at, element)

def ensure_site(body: ET.Element, attrib: dict) -> None:
    name = attrib.get("name")
    if name is None:
        return
    for elem in body.findall("site"):
        if elem.get("name") == name:
            for key, value in attrib.items():
                elem.set(key, value)
            return
    insert_before_child_bodies(body, ET.Element("site", attrib=attrib))

def replace_inertial_from_urdf(body: ET.Element, link_name: str) -> bool:
    link = urdf_links.get(link_name)
    if link is None:
        return False
    inertial = link.find("inertial")
    if inertial is None:
        return False
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None or not mass.get("value"):
        return False
    for old in list(body.findall("inertial")):
        body.remove(old)
    origin = inertial.find("origin")
    attrs = {
        "mass": mass.get("value"),
        "pos": parse_xyz(origin.get("xyz") if origin is not None else None),
        "quat": parse_quat_from_rpy(origin.get("rpy") if origin is not None else None),
        "diaginertia": " ".join(
            [
                inertia.get("ixx", "0"),
                inertia.get("iyy", "0"),
                inertia.get("izz", "0"),
            ]
        ),
    }
    body.insert(0, ET.Element("inertial", attrib=attrs))
    return True

def is_visual_geom(geom: ET.Element) -> bool:
    if geom.get("class") in {"visual", "visualgeom"}:
        return True
    if geom.get("group") == "2":
        return True
    return geom.get("contype") == "0" and geom.get("conaffinity") == "0"

def strip_geom_attrs(geom: ET.Element, keep: set[str]) -> None:
    for attr in list(geom.attrib):
        if attr not in keep:
            geom.attrib.pop(attr, None)

def derive_motor_name(joint_name: str) -> str:
    if joint_name.endswith("_JOINT"):
        return joint_name[:-6] + "_LINK"
    return joint_name

def derive_sensor_base(joint_name: str) -> str:
    base = joint_name[:-6] if joint_name.endswith("_JOINT") else joint_name
    m = re.match(r"^(FAR|FBL|RAR|RBL|FR|FL|RR|RL)_([A-Z0-9]+)$", base)
    if m:
        leg_prefix = m.group(1)
        suffix = m.group(2)
        leg_map = {
            "FAR": "FR",
            "FR": "FR",
            "FBL": "FL",
            "FL": "FL",
            "RAR": "RR",
            "RR": "RR",
            "RBL": "RL",
            "RL": "RL",
        }
        joint_map = {
            "ABAD": "hip",
            "HIP": "thigh",
            "KNEE": "calf",
            "FOOT": "foot",
        }
        return f"{leg_map[leg_prefix]}_{joint_map.get(suffix, suffix.lower())}"
    return re.sub(r"[^a-z0-9]+", "_", base.lower()).strip("_")

option = root.find("option")
if option is not None:
    root.remove(option)

default_top = root.find("default")
if default_top is None:
    default_top = ET.Element("default")
    root.insert(0, default_top)
else:
    for child in list(default_top):
        default_top.remove(child)
    default_top.attrib.clear()

robot_default = ET.SubElement(default_top, "default", attrib={"class": "robot"})
motor_default = ET.SubElement(robot_default, "default", attrib={"class": "motor"})
ET.SubElement(motor_default, "joint")
ET.SubElement(motor_default, "motor")
visual_default = ET.SubElement(robot_default, "default", attrib={"class": "visual"})
ET.SubElement(
    visual_default,
    "geom",
    attrib={"material": "default_material", "contype": "0", "conaffinity": "0", "group": "2"},
)
collision_default = ET.SubElement(robot_default, "default", attrib={"class": "collision"})
ET.SubElement(
    collision_default,
    "geom",
    attrib={
        "material": "default_material",
        "condim": "3",
        "contype": "0",
        "conaffinity": "1",
        "priority": "1",
        "group": "1",
        "solref": "0.005 1",
        "solimp": "0.99 0.999 1e-05",
        "friction": "1 0.01 0.01",
    },
)

compiler = root.find("compiler")
if compiler is None:
    compiler = ET.Element("compiler")
    insert_at = 1 if root.find("default") is not None else 0
    root.insert(insert_at, compiler)
compiler.set("angle", "radian")
compiler.set("meshdir", "assets")
for attr in list(compiler.attrib):
    if attr not in {"angle", "meshdir"}:
        compiler.attrib.pop(attr, None)

asset = root.find("asset")
if asset is None:
    asset = ET.SubElement(root, "asset")
default_material = None
for material in asset.findall("material"):
    if material.get("name") == "default_material":
        default_material = material
        break
if default_material is None:
    asset.insert(0, ET.Element("material", attrib={"name": "default_material", "rgba": "0.75294 0.75294 0.75294 1"}))

worldbody = root.find("worldbody")
if worldbody is None:
    worldbody = ET.SubElement(root, "worldbody")

root_body = None
for body in worldbody.findall("body"):
    if body.get("name") in {"BASE_LINK", "base_link", "ROOT_LINK", "root"}:
        root_body = body
        break
if root_body is None:
    root_body = ET.SubElement(worldbody, "body", attrib={"name": "BASE_LINK", "pos": "0 0 0", "quat": "1 0 0 0"})

root_link_name = next((name for name in ("BASE_LINK", "base_link", "ROOT_LINK") if name in urdf_links), root_body.get("name", "BASE_LINK"))
root_body.set("name", root_link_name)
root_body.set("childclass", "robot")

freejoints = root_body.findall("freejoint")
if freejoints:
    freejoints[0].set("name", "floating_base")
    for redundant in freejoints[1:]:
        root_body.remove(redundant)
elif not any(child.tag == "joint" for child in root_body):
    root_body.insert(0, ET.Element("freejoint", attrib={"name": "floating_base"}))

restored_inertials = 0
for body in root.iter("body"):
    body_name = body.get("name")
    if body_name and replace_inertial_from_urdf(body, body_name):
        restored_inertials += 1
if restored_inertials:
    print(f"[INFO] restored URDF inertials for {restored_inertials} bodies")

ensure_site(root_body, {"name": "BASE_LINK_site", "pos": "0 0 0", "quat": "1 0 0 0"})
ensure_site(root_body, {"name": "imu", "pos": "0 0 0"})
ensure_site(root_body, {"name": "livox_imu", "pos": "0.13011 0.02329 0.17598", "quat": "1 0 0 0"})
ensure_site(root_body, {"name": "camera_imu", "pos": "0.29 0 0.01"})

for body in root.iter("body"):
    body_name = body.get("name", "body")
    collision_index = 0
    for geom in body.findall("geom"):
        geom_type = geom.get("type", "")
        if geom_type == "mesh":
            if is_visual_geom(geom):
                geom.set("class", "visual")
                geom.set("material", "default_material")
                if "name" not in geom.attrib and geom.get("mesh"):
                    geom.set("name", f"{geom.get('mesh')}_visual")
                strip_geom_attrs(geom, {"name", "pos", "quat", "type", "mesh", "class", "material"})
            else:
                geom.set("class", "collision")
                if "name" not in geom.attrib:
                    geom.set("name", f"{body_name}_collision_{collision_index}" if collision_index else f"{body_name}_collision")
                    collision_index += 1
                strip_geom_attrs(geom, {"name", "pos", "quat", "type", "mesh", "class"})
        elif geom_type in {"box", "sphere", "capsule", "cylinder", "ellipsoid"}:
            # Drop geoms that carry no geometry data — urdf2mjcf sometimes emits a
            # size-less duplicate alongside the real collision geom for fixed links.
            if not geom.get("size") and not geom.get("fromto"):
                body.remove(geom)
                continue
            geom.set("class", "collision")
            if "name" not in geom.attrib:
                geom.set("name", f"{body_name}_collision_{collision_index}" if collision_index else f"{body_name}_collision")
                collision_index += 1
            strip_geom_attrs(geom, {"name", "pos", "quat", "type", "size", "fromto", "mesh", "class"})

# Remove stale class="visualgeom" references: urdf2mjcf marks fixed-link collision spheres
# (e.g. foot-contact spheres merged from FOOT_LINK into KNEE_LINK) with class="visualgeom",
# but no <default class="visualgeom"> exists in the output — MuJoCo rejects the XML.
# These geoms are genuine collision primitives; strip only the invalid class tag and leave
# all other explicit physics attributes (contype, conaffinity, density, group, …) intact.
for _g in root.iter("geom"):
    if _g.get("class") == "visualgeom":
        del _g.attrib["class"]

# Deduplicate geom names: urdf2mjcf can flatten fixed-link meshes onto the base body,
# then restore_urdf_fixed_links re-creates those child bodies with the same mesh name,
# producing two geoms with identical names (e.g. "camera B_visual" on base + child body).
_geom_name_count: dict[str, int] = {}
for _g in root.iter("geom"):
    _n = _g.get("name")
    if _n:
        _geom_name_count[_n] = _geom_name_count.get(_n, 0) + 1
_geom_name_seen: dict[str, int] = {}
for _g in root.iter("geom"):
    _n = _g.get("name")
    if not _n or _geom_name_count.get(_n, 1) <= 1:
        continue
    _idx = _geom_name_seen.get(_n, 0)
    _geom_name_seen[_n] = _idx + 1
    if _idx > 0:
        _g.set("name", f"{_n}_{_idx}")

joint_names = {joint.get("name") for joint in root.iter("joint") if joint.get("name")}
for joint_elem in root.iter("joint"):
    joint_elem.attrib.pop("class", None)
    axis_str = joint_elem.get("axis")
    if axis_str:
        try:
            parts = [float(v) for v in axis_str.split()]
        except ValueError:
            parts = []
        if parts:
            joint_elem.set("axis", " ".join("0" if v == 0.0 else f"{v:g}" for v in parts))
    for attr in ("damping", "armature", "limited", "ctrllimited", "ctrlrange", "gear"):
        joint_elem.attrib.pop(attr, None)
    # Derive actuatorfrcrange from URDF <limit effort=...> instead of deleting it
    jname = joint_elem.get("name", "")
    effort = urdf_joint_effort.get(jname, 0.0)
    if effort > 0:
        joint_elem.set("actuatorfrcrange", f"-{effort:g} {effort:g}")
    else:
        joint_elem.attrib.pop("actuatorfrcrange", None)

actuator = root.find("actuator")
if actuator is None:
    actuator = ET.Element("actuator")
    insert_at = len(root)
    for candidate in ("sensor", "contact"):
        candidate_elem = root.find(candidate)
        if candidate_elem is not None:
            insert_at = min(insert_at, list(root).index(candidate_elem))
    root.insert(insert_at, actuator)

motor_joint_order = []
seen_motor_joints = set()
for motor in list(actuator):
    joint_name = motor.get("joint")
    if not joint_name or joint_name not in joint_names:
        actuator.remove(motor)
        continue
    if joint_name in seen_motor_joints:
        actuator.remove(motor)
        continue
    motor.set("name", derive_motor_name(joint_name))
    motor.attrib.pop("class", None)
    for attr in ("ctrllimited", "ctrlrange", "gear"):
        motor.attrib.pop(attr, None)
    motor_joint_order.append(joint_name)
    seen_motor_joints.add(joint_name)

if not motor_joint_order:
    for joint_elem in root.iter("joint"):
        joint_name = joint_elem.get("name")
        if not joint_name or joint_name in seen_motor_joints:
            continue
        ET.SubElement(actuator, "motor", attrib={"name": derive_motor_name(joint_name), "joint": joint_name})
        motor_joint_order.append(joint_name)
        seen_motor_joints.add(joint_name)

contact = root.find("contact")
if contact is None:
    contact = ET.Element("contact")
    sensor_elem = root.find("sensor")
    insert_at = list(root).index(sensor_elem) if sensor_elem is not None else len(root)
    root.insert(insert_at, contact)
else:
    for child in list(contact):
        contact.remove(child)

body_names = {body.get("name") for body in root.iter("body") if body.get("name")}
exclude_pairs = []
for prefix in ("FAR", "FBL", "RAR", "RBL", "FR", "FL", "RR", "RL"):
    hip_name = f"{prefix}_HIP_LINK"
    knee_name = f"{prefix}_KNEE_LINK"
    foot_name = {
        "FAR": "FR_FOOT_LINK",
        "FBL": "FL_FOOT_LINK",
        "RAR": "RR_FOOT_LINK",
        "RBL": "RL_FOOT_LINK",
    }.get(prefix, f"{prefix}_FOOT_LINK")
    if hip_name in body_names and knee_name in body_names:
        exclude_pairs.append((hip_name, knee_name))
    if knee_name in body_names and foot_name in body_names:
        exclude_pairs.append((knee_name, foot_name))
for body1, body2 in exclude_pairs:
    ET.SubElement(contact, "exclude", attrib={"body1": body1, "body2": body2})

sensor = root.find("sensor")
if sensor is None:
    sensor = ET.SubElement(root, "sensor")
else:
    for child in list(sensor):
        sensor.remove(child)

for joint_name in motor_joint_order:
    ET.SubElement(sensor, "jointpos", attrib={"name": f"{derive_sensor_base(joint_name)}_pos", "joint": joint_name})
for joint_name in motor_joint_order:
    ET.SubElement(sensor, "jointvel", attrib={"name": f"{derive_sensor_base(joint_name)}_vel", "joint": joint_name})
for joint_name in motor_joint_order:
    ET.SubElement(sensor, "jointactuatorfrc", attrib={"name": f"{derive_sensor_base(joint_name)}_torque", "joint": joint_name, "noise": "0.00"})

site_names = {site.get("name") for site in root.iter("site") if site.get("name")}
if "imu" in site_names:
    ET.SubElement(sensor, "framequat", attrib={"name": "imu_quat", "objtype": "site", "objname": "imu", "noise": "0.001"})
    ET.SubElement(sensor, "gyro", attrib={"name": "imu_gyro", "site": "imu"})
    ET.SubElement(sensor, "accelerometer", attrib={"name": "imu_acc", "site": "imu"})
    ET.SubElement(sensor, "framepos", attrib={"name": "frame_pos", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "framelinvel", attrib={"name": "frame_vel", "objtype": "site", "objname": "imu"})
if "livox_imu" in site_names:
    ET.SubElement(sensor, "framequat", attrib={"name": "livox_imu_quat", "objtype": "site", "objname": "livox_imu"})
    ET.SubElement(sensor, "gyro", attrib={"name": "livox_imu_gyro", "site": "livox_imu"})
    ET.SubElement(sensor, "accelerometer", attrib={"name": "livox_imu_acc", "site": "livox_imu"})
    ET.SubElement(sensor, "framepos", attrib={"name": "livox_imu_frame_pos", "objtype": "site", "objname": "livox_imu"})
    ET.SubElement(sensor, "framelinvel", attrib={"name": "livox_imu_frame_vel", "objtype": "site", "objname": "livox_imu"})
if "camera_imu" in site_names:
    ET.SubElement(sensor, "framequat", attrib={"name": "camera_imu_quat", "objtype": "site", "objname": "camera_imu"})
    ET.SubElement(sensor, "gyro", attrib={"name": "camera_imu_gyro", "site": "camera_imu"})
    ET.SubElement(sensor, "accelerometer", attrib={"name": "camera_imu_acc", "site": "camera_imu"})
    ET.SubElement(sensor, "framepos", attrib={"name": "camera_imu_frame_pos", "objtype": "site", "objname": "camera_imu"})
    ET.SubElement(sensor, "framelinvel", attrib={"name": "camera_imu_frame_vel", "objtype": "site", "objname": "camera_imu"})

lowest_bottom = None
for body, body_pos, body_quat in walk_body_poses(root_body):
    if "FOOT" not in body.get("name", ""):
        continue
    for geom in body.findall("geom"):
        if geom.get("class") != "collision" or geom.get("type") != "sphere":
            continue
        size_parts = geom.get("size", "").split()
        if not size_parts:
            continue
        radius = float(size_parts[0])
        geom_pos = xyz_to_tuple(geom.get("pos", "0 0 0"))
        rotated = rotate_vec(body_quat, geom_pos)
        world_center = (
            body_pos[0] + rotated[0],
            body_pos[1] + rotated[1],
            body_pos[2] + rotated[2],
        )
        bottom = world_center[2] - radius
        lowest_bottom = bottom if lowest_bottom is None else min(lowest_bottom, bottom)
if lowest_bottom is not None:
    root_pos = xyz_to_tuple(root_body.get("pos", "0 0 0"))
    new_root_z = root_pos[2] - lowest_bottom
    root_body.set("pos", f"{root_pos[0]:.9g} {root_pos[1]:.9g} {new_root_z:.9g}")
    print(f"[INFO] adjusted root height to keep generic foot spheres above ground: z={new_root_z:.9g}")

tree.write(mjcf_path, encoding="utf-8", xml_declaration=False)
print(f"[INFO] restored generic runtime markers and sensors in xml: {mjcf_path}")
PY
}

write_import_metadata() {
    local metadata_path="$1"
    local source_sha256="$2"
    jq -n \
        --arg schema_version "1" \
        --argjson generator_version "$PIPELINE_VERSION" \
        --arg custom_name "$CUSTOM_NAME" \
        --arg source_path "$CUSTOM_URDF" \
        --arg source_file "$URDF_BASENAME" \
        --arg source_sha256 "$source_sha256" \
        --arg reference_profile "${REFERENCE_PROFILE:-}" \
        --arg map_key "$MAP_KEY" \
        --arg map_asset "$MAP_ASSET" \
        --arg imported_at "$(date -Iseconds)" \
        '{
            schema_version: ($schema_version | tonumber),
            generator_version: $generator_version,
            custom_name: $custom_name,
            source_path: $source_path,
            source_file: $source_file,
            source_sha256: $source_sha256,
            reference_profile: $reference_profile,
            map_key: $map_key,
            map_asset: $map_asset,
            imported_at: $imported_at
        }' > "$metadata_path"
}

read_metadata_value() {
    local metadata_path="$1"
    local key="$2"
    jq -r --arg key "$key" '.[$key] // empty' "$metadata_path"
}

CUSTOM_SRC_DIR="$(cd "$(dirname "$CUSTOM_URDF")" && pwd)"
CUSTOM_ASSET_SRC=""
if [[ -d "$CUSTOM_SRC_DIR/assets" ]]; then
    CUSTOM_ASSET_SRC="$CUSTOM_SRC_DIR/assets"
elif [[ -d "$CUSTOM_SRC_DIR/meshes" ]]; then
    CUSTOM_ASSET_SRC="$CUSTOM_SRC_DIR/meshes"
fi

REFERENCE_PROFILE="$(detect_reference_profile "$CUSTOM_URDF")"
REFERENCE_MUJOCO_XML=""
REFERENCE_UE_XML=""
REFERENCE_MUJOCO_ASSET_SRC=""
REFERENCE_UE_ASSET_SRC=""
if [[ -n "$REFERENCE_PROFILE" ]]; then
    REFERENCE_MUJOCO_XML="$(get_reference_mujoco_xml "$REFERENCE_PROFILE")"
    REFERENCE_UE_XML="$(get_reference_ue_xml "$REFERENCE_PROFILE")"
    REFERENCE_MUJOCO_ASSET_SRC="$(get_reference_mujoco_assets "$REFERENCE_PROFILE")"
    REFERENCE_UE_ASSET_SRC="$(get_reference_ue_assets "$REFERENCE_PROFILE")"
    echo "[INFO] matched controller-supported profile: $REFERENCE_PROFILE"
    echo "[INFO] reference mujoco xml: $REFERENCE_MUJOCO_XML"
    echo "[INFO] reference ue xml: $REFERENCE_UE_XML"
fi

MUJOCO_IMPORT_ASSET_SRC="$CUSTOM_ASSET_SRC"
UE_IMPORT_ASSET_SRC="$CUSTOM_ASSET_SRC"
if [[ -n "$REFERENCE_PROFILE" ]]; then
    if [[ -d "$REFERENCE_MUJOCO_ASSET_SRC" ]]; then
        MUJOCO_IMPORT_ASSET_SRC="$REFERENCE_MUJOCO_ASSET_SRC"
    fi
    if [[ -d "$REFERENCE_UE_ASSET_SRC" ]]; then
        UE_IMPORT_ASSET_SRC="$REFERENCE_UE_ASSET_SRC"
    fi
fi

CURRENT_SOURCE_SHA256="$(sha256sum "$CUSTOM_URDF" | awk '{print $1}')"
echo "[INFO] source sha256: $CURRENT_SOURCE_SHA256"

REUSE_EXISTING_IMPORT=0
if [[ "$FORCE_REIMPORT" != "1" && -f "$TARGET_XML" && -f "$UE_TARGET_XML" ]]; then
    REUSE_EXISTING_IMPORT=1
fi

if [[ "$REUSE_EXISTING_IMPORT" == "1" ]]; then
    if [[ -f "$TARGET_METADATA" ]]; then
        PREVIOUS_SOURCE="$(read_metadata_value "$TARGET_METADATA" "source_path")"
        PREVIOUS_SHA256="$(read_metadata_value "$TARGET_METADATA" "source_sha256")"
        PREVIOUS_PIPELINE_VERSION="$(read_metadata_value "$TARGET_METADATA" "generator_version")"
        if [[ -n "$PREVIOUS_SOURCE" && "$PREVIOUS_SOURCE" != "$CUSTOM_URDF" ]]; then
            echo "[WARN] Existing import '$CUSTOM_NAME' was created from: $PREVIOUS_SOURCE"
            echo "[WARN] Current URDF '$CUSTOM_URDF' will be ignored to avoid overwriting the converted result."
            echo "[WARN] Rename the custom robot or set SIM_LAUNCHER_FORCE_REIMPORT_CUSTOM_URDF=1 to regenerate."
        elif [[ -z "$PREVIOUS_SHA256" ]]; then
            echo "[WARN] Existing import '$CUSTOM_NAME' has no source hash. Regenerating once to seed metadata."
            REUSE_EXISTING_IMPORT=0
        elif [[ -z "$PREVIOUS_PIPELINE_VERSION" || "$PREVIOUS_PIPELINE_VERSION" -lt "$PIPELINE_VERSION" ]]; then
            echo "[WARN] Existing import '$CUSTOM_NAME' was generated by pipeline v${PREVIOUS_PIPELINE_VERSION:-0}."
            echo "[WARN] Current pipeline v${PIPELINE_VERSION} requires regeneration."
            REUSE_EXISTING_IMPORT=0
        elif [[ "$PREVIOUS_SHA256" != "$CURRENT_SOURCE_SHA256" ]]; then
            echo "[WARN] Existing import '$CUSTOM_NAME' source hash changed."
            echo "[WARN] Previous: $PREVIOUS_SHA256"
            echo "[WARN] Current : $CURRENT_SOURCE_SHA256"
            echo "[WARN] Regenerating to avoid reusing stale conversion output."
            REUSE_EXISTING_IMPORT=0
        fi
    fi
fi

if [[ "$REUSE_EXISTING_IMPORT" == "1" ]]; then
    echo "[INFO] Reusing existing converted custom robot: $CUSTOM_NAME"
    echo "[INFO] existing xml: $TARGET_XML"
    echo "[INFO] existing ue xml: $UE_TARGET_XML"
    if [[ ! -f "$UE_TARGET_URDF" && -f "$TARGET_URDF" ]]; then
        cp "$TARGET_URDF" "$UE_TARGET_URDF"
        echo "[INFO] restored UE-side URDF copy: $UE_TARGET_URDF"
    fi

    if [[ -d "$TMP_ASSET_DIR" && ! -d "$UE_ASSET_DIR" ]]; then
        mkdir -p "$UE_ASSET_DIR"
        cp -r "$TMP_ASSET_DIR/." "$UE_ASSET_DIR/"
        echo "[INFO] restored UE-side assets: $UE_ASSET_DIR"
    fi

    sync_runtime_layout "$TARGET_XML" "$TARGET_URDF" "$TMP_ASSET_DIR" "$MUJOCO_CUSTOM_ROOT" "$MUJOCO_ACTIVE_XML" "$MUJOCO_ACTIVE_SCENE_XML" "$MUJOCO_ACTIVE_ASSETS"
    sync_runtime_layout "$UE_TARGET_XML" "$UE_TARGET_URDF" "$UE_ASSET_DIR" "$UE_CUSTOM_ROOT" "$UE_ACTIVE_XML" "$UE_ACTIVE_SCENE_XML" "$UE_ACTIVE_ASSETS"
else
    if [[ -f "$TARGET_METADATA" ]]; then
        PREVIOUS_SOURCE="$(read_metadata_value "$TARGET_METADATA" "source_path")"
        if [[ -n "$PREVIOUS_SOURCE" && "$PREVIOUS_SOURCE" != "$CUSTOM_URDF" ]]; then
            echo "[WARN] Existing import '$CUSTOM_NAME' was created from: $PREVIOUS_SOURCE"
            echo "[WARN] Current URDF '$CUSTOM_URDF' will be re-imported to refresh the converted result."
        fi
    fi

    echo "[INFO] Importing custom robot into isolated directory: $CUSTOM_NAME"
    echo "[INFO] copying URDF to: $TARGET_URDF"
    echo "[INFO] copying UE URDF to: $UE_TARGET_URDF"
    rm -rf "$IMPORT_DIR" "$UE_IMPORT_DIR"
    mkdir -p "$IMPORT_DIR" "$UE_IMPORT_DIR"

    cp "$CUSTOM_URDF" "$TARGET_URDF"
    cp "$CUSTOM_URDF" "$UE_TARGET_URDF"

    if [[ -n "$MUJOCO_IMPORT_ASSET_SRC" ]]; then
        mkdir -p "$TMP_ASSET_DIR" "$UE_ASSET_DIR"
        cp -r "$MUJOCO_IMPORT_ASSET_SRC/." "$TMP_ASSET_DIR/"
        echo "[INFO] copied mujoco assets from: $MUJOCO_IMPORT_ASSET_SRC"
    fi
    if [[ -n "$UE_IMPORT_ASSET_SRC" ]]; then
        mkdir -p "$TMP_ASSET_DIR" "$UE_ASSET_DIR"
        cp -r "$UE_IMPORT_ASSET_SRC/." "$UE_ASSET_DIR/"
        echo "[INFO] copied ue assets from: $UE_IMPORT_ASSET_SRC"
        echo "[INFO] assets dir: $TMP_ASSET_DIR"
    fi

    if [[ -z "$REFERENCE_PROFILE" && "${CUSTOM_NAME,,}" == "lite3" ]]; then
        python3 - "$TMP_ASSET_DIR/torso.STL" "$UE_ASSET_DIR/torso.STL" <<'PY'
import sys
from pathlib import Path

TARGET_TRIANGLES = 180000

def decimate_binary_stl_in_place(path: Path, target_triangles: int = TARGET_TRIANGLES) -> None:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too small to be a binary STL")
    tri_count = int.from_bytes(data[80:84], "little")
    if tri_count <= target_triangles:
        print(f"[INFO] {path.name} already within triangle budget ({tri_count})")
        return
    stride = max(2, (tri_count + target_triangles - 1) // target_triangles)
    sampled = bytearray(data[:80])
    faces = []
    for idx in range(0, tri_count, stride):
        off = 84 + idx * 50
        faces.append(data[off:off + 50])
    sampled.extend(len(faces).to_bytes(4, "little"))
    for face in faces:
        sampled.extend(face)
    path.write_bytes(sampled)
    print(f"[INFO] lite3 torso.STL decimated: {tri_count} -> {len(faces)} triangles")

for arg in sys.argv[1:]:
    decimate_binary_stl_in_place(Path(arg))
PY
    fi

    echo "[INFO] rewriting mesh paths inside URDF"
    rewrite_mesh_paths "$TARGET_URDF"
    rewrite_mesh_paths "$UE_TARGET_URDF"
    normalize_mesh_aliases "$TARGET_URDF" "$TMP_ASSET_DIR"
    normalize_mesh_aliases "$UE_TARGET_URDF" "$UE_ASSET_DIR"
    normalize_link_aliases "$TARGET_URDF"
    normalize_link_aliases "$UE_TARGET_URDF"

    if [[ -n "$REFERENCE_PROFILE" ]]; then
        if [[ ! -f "$REFERENCE_MUJOCO_XML" || ! -f "$REFERENCE_UE_XML" ]]; then
            echo "[ERROR] Missing reference xml for profile '$REFERENCE_PROFILE'" >&2
            exit 1
        fi
        cp "$REFERENCE_MUJOCO_XML" "$TARGET_XML"
        cp "$REFERENCE_UE_XML" "$UE_TARGET_XML"
        echo "[INFO] copied reference xml for profile '$REFERENCE_PROFILE'"
        echo "[INFO] mujoco xml: $REFERENCE_MUJOCO_XML -> $TARGET_XML"
        echo "[INFO] ue xml: $REFERENCE_UE_XML -> $UE_TARGET_XML"
        if [[ "$REFERENCE_PROFILE" == "zg" ]]; then
            ZG_BASE_LINK_SOURCE="$REFERENCE_MUJOCO_ASSET_SRC/BASE_LINK.STL"
            if write_bbox_stl_proxy_from_binary_stl "$ZG_BASE_LINK_SOURCE" "$TMP_ASSET_DIR/BASE_LINK.STL"; then
                cp "$TMP_ASSET_DIR/BASE_LINK.STL" "$UE_ASSET_DIR/BASE_LINK.STL"
                echo "[INFO] applied zg BASE_LINK proxy from staged bbox of: $ZG_BASE_LINK_SOURCE"
            else
                echo "[WARN] zg BASE_LINK proxy generation failed from: $ZG_BASE_LINK_SOURCE"
            fi
        fi
    else
        if ! preflight_custom_urdf "$TARGET_URDF" "$TMP_ASSET_DIR"; then
            exit 1
        fi
        echo "[INFO] converting URDF -> MJCF"
        (
            cd "$IMPORT_DIR"
            python3 - <<PY
import shutil
import tempfile
import xml.etree.ElementTree as ET
import struct
from pathlib import Path
from urdf2mjcf.convert import convert_urdf_to_mjcf

urdf_path = Path("${CUSTOM_NAME}.urdf")
xml_path = Path("${CUSTOM_NAME}.xml")
stage_root = Path(tempfile.mkdtemp(prefix="custom_urdf_stage_", dir="/tmp"))
stage_urdf = stage_root / urdf_path.name
stage_assets = stage_root / "assets"
stage_assets.mkdir(parents=True, exist_ok=True)
custom_asset_src = Path("${MUJOCO_IMPORT_ASSET_SRC}") if "${MUJOCO_IMPORT_ASSET_SRC}" else None

def decimate_binary_stl_in_place(path: Path, target_triangles: int = 180000) -> None:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too small to be a binary STL")
    tri_count = int.from_bytes(data[80:84], "little")
    if tri_count <= target_triangles:
        return
    stride = max(2, (tri_count + target_triangles - 1) // target_triangles)
    sampled = bytearray(data[:80])
    faces = []
    for idx in range(0, tri_count, stride):
        off = 84 + idx * 50
        faces.append(data[off:off + 50])
    sampled.extend(len(faces).to_bytes(4, "little"))
    for face in faces:
        sampled.extend(face)
    path.write_bytes(sampled)

shutil.copy2(urdf_path, stage_urdf)
if custom_asset_src is not None:
    for mesh_file in custom_asset_src.glob("*"):
        if mesh_file.is_file():
            shutil.copy2(mesh_file, stage_assets / mesh_file.name)

if "${CUSTOM_NAME,,}" == "lite3":
    decimate_binary_stl_in_place(stage_assets / "torso.STL")

tree = ET.parse(stage_urdf)
for compiler in tree.iter("compiler"):
    if compiler.get("meshdir") == "assets":
        compiler.set("meshdir", ".")
for mesh in tree.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn:
        mesh.set("filename", str((stage_assets / Path(fn).name).resolve()))
tree.write(str(stage_urdf), encoding="unicode", xml_declaration=True)

try:
    convert_urdf_to_mjcf(str(stage_urdf), str(xml_path), copy_meshes=False)
finally:
    shutil.rmtree(stage_root, ignore_errors=True)
PY
        )

        if [[ ! -f "$TARGET_XML" ]]; then
            echo "[ERROR] MJCF conversion failed: $TARGET_XML not generated" >&2
            exit 1
        fi
        echo "[INFO] generated xml: $TARGET_XML"

        normalize_mjcf "$TARGET_XML"
        restore_urdf_visual_meshes "$TARGET_URDF" "$TARGET_XML"
        restore_urdf_fixed_links "$TARGET_URDF" "$TARGET_XML"
        echo "[INFO] restoring generic runtime layout in: $TARGET_XML"
        restore_generic_runtime_layout "$TARGET_XML" "$TARGET_URDF"

        cp "$TARGET_XML" "$UE_TARGET_XML"
        cp "$TARGET_URDF" "$UE_TARGET_URDF"
        normalize_mjcf "$UE_TARGET_XML"
        restore_urdf_visual_meshes "$UE_TARGET_URDF" "$UE_TARGET_XML"
        restore_urdf_fixed_links "$UE_TARGET_URDF" "$UE_TARGET_XML"
        echo "[INFO] restoring generic runtime layout in: $UE_TARGET_XML"
        restore_generic_runtime_layout "$UE_TARGET_XML" "$UE_TARGET_URDF"
        echo "[INFO] copied xml to UE side: $UE_TARGET_XML"
    fi

    sync_runtime_layout "$TARGET_XML" "$TARGET_URDF" "$TMP_ASSET_DIR" "$MUJOCO_CUSTOM_ROOT" "$MUJOCO_ACTIVE_XML" "$MUJOCO_ACTIVE_SCENE_XML" "$MUJOCO_ACTIVE_ASSETS"
    sync_runtime_layout "$UE_TARGET_XML" "$UE_TARGET_URDF" "$UE_ASSET_DIR" "$UE_CUSTOM_ROOT" "$UE_ACTIVE_XML" "$UE_ACTIVE_SCENE_XML" "$UE_ACTIVE_ASSETS"

    write_import_metadata "$TARGET_METADATA" "$CURRENT_SOURCE_SHA256"
    write_import_metadata "$UE_TARGET_METADATA" "$CURRENT_SOURCE_SHA256"
    echo "[INFO] wrote metadata: $TARGET_METADATA"
    echo "[INFO] wrote metadata: $UE_TARGET_METADATA"
fi

CUSTOM_URDF_RELATIVE="custom/scene_terrain_custom.xml"
if [[ -f "$MATRIX_ROOT/config/config.json" ]]; then
    jq ".robot.robot_type = \"custom\" | .robot.weapon = \"\" | .robot.use_custom_urdf = true | .robot.custom_urdf = \"$CUSTOM_URDF_RELATIVE\" | .robot.custom_name = \"$CUSTOM_NAME\" | .robot.map_key = \"$MAP_KEY\" | .robot.map_asset = \"$MAP_ASSET\"" \
        "$MATRIX_ROOT/config/config.json" > "$MATRIX_ROOT/tmp_config.json" && mv "$MATRIX_ROOT/tmp_config.json" "$MATRIX_ROOT/config/config.json"
    echo "[INFO] updated root config: $MATRIX_ROOT/config/config.json"
fi
if [[ -f "$MODEL_DIR/config/config.json" ]]; then
    jq ".robot.robot_type = \"custom\" | .robot.weapon = \"\" | .robot.use_custom_urdf = true | .robot.custom_urdf = \"$CUSTOM_URDF_RELATIVE\" | .robot.custom_name = \"$CUSTOM_NAME\" | .robot.map_key = \"$MAP_KEY\" | .robot.map_asset = \"$MAP_ASSET\"" \
        "$MODEL_DIR/config/config.json" > "$MATRIX_ROOT/tmp_config.json" && mv "$MATRIX_ROOT/tmp_config.json" "$MODEL_DIR/config/config.json"
    echo "[INFO] updated ue config: $MODEL_DIR/config/config.json"
fi

echo "[INFO] handing off to run_sim.sh with custom URDF: $CUSTOM_URDF_RELATIVE"
# The second run_sim.sh invocation does not need the original URDF path anymore.
# Passing it through can make the broad `pkill -f robot_mujoco` cleanup match this
# launcher command line when the source path itself contains `/robot_mujoco/`.
SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER=1 exec "$RUN_SIM_SH" "custom" "$SCENE_ID" "$OFFSCREEN" "$PIXELSTREAM" "$MUJOCORUNNING" "" "$CUSTOM_NAME"
