#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${ROOT_DIR}/config/go2w_house_dependencies.json"
MATRIX_DIR="${MATRIX_DIR:-${ROOT_DIR}/matrix}"
ROAMERX_DIR="${ROAMERX_DIR:-${ROOT_DIR}/genisom_roamerx_open}"
DREAMWAQ_DIR="${DREAMWAQ_DIR:-${ROOT_DIR}/third_party/DreamWaQ_Go2W}"
COSMOS_ROOT="${COSMOS_ROOT:-${ROOT_DIR}/.external/cosmos}"
COSMOS_FRAMEWORK="${COSMOS_VLN_FRAMEWORK:-${COSMOS_ROOT}/packages/cosmos-framework}"
COSMOS_CHECKPOINT="${COSMOS_VLN_CHECKPOINT:-${COSMOS_ROOT}/Cosmos3-Edge}"
RUNTIME_INSTALL="${GO2W_INSTALL_DIR:-${ROOT_DIR}/install}"

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap_go2w_house.sh COMMAND

Commands:
  check    Read-only audit of the complete HouseWorld runtime
  sources  Clone pinned source repositories and apply runtime patches
  system   Install Ubuntu 22.04, ROS 2 Humble, and MATRiX system packages
  assets   Download verified MATRiX base/shared/HouseWorld release assets
  model    Download the pinned Cosmos3-Edge checkpoint from Hugging Face
  build    Build RoamerX, the Cosmos environment, and the Go2-W RL bridge
  all      Run sources, system, assets, model, build, and check

Environment overrides:
  MATRIX_DIR, ROAMERX_DIR, DREAMWAQ_DIR, COSMOS_ROOT,
  COSMOS_VLN_FRAMEWORK, COSMOS_VLN_CHECKPOINT, GO2W_INSTALL_DIR, UV_BIN, HF_BIN
EOF
}

manifest_value() {
  /usr/bin/python3 - "$MANIFEST" "$1" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
for component in sys.argv[2].split("."):
    value = value[component]
print(value)
PY
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] required command is missing: $1" >&2
    exit 1
  fi
}

clone_pinned() {
  local name="$1"
  local repository="$2"
  local revision="$3"
  local destination="$4"

  if [[ ! -e "$destination" ]]; then
    mkdir -p "$(dirname "$destination")"
    git clone --filter=blob:none --no-checkout "$repository" "$destination"
    git -C "$destination" checkout --detach "$revision"
  elif [[ ! -d "$destination/.git" ]]; then
    echo "[ERROR] ${name} path exists but is not a Git checkout: ${destination}" >&2
    exit 1
  fi

  local actual
  actual="$(git -C "$destination" rev-parse HEAD)"
  if [[ "$actual" != "$revision" ]]; then
    echo "[ERROR] ${name} revision mismatch" >&2
    echo "        expected: ${revision}" >&2
    echo "        actual:   ${actual}" >&2
    echo "        path:     ${destination}" >&2
    exit 1
  fi
  echo "[OK] ${name} revision ${revision}"
}

apply_patch_once() {
  local checkout="$1"
  local patch_file="$2"
  if git -C "$checkout" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    echo "[OK] patch already applied: ${patch_file#${ROOT_DIR}/}"
  elif git -C "$checkout" apply --check "$patch_file"; then
    git -C "$checkout" apply "$patch_file"
    echo "[OK] applied patch: ${patch_file#${ROOT_DIR}/}"
  else
    echo "[ERROR] patch does not apply cleanly: ${patch_file}" >&2
    exit 1
  fi
}

install_sources() {
  require_command git
  clone_pinned \
    "MATRiX" \
    "$(manifest_value sources.matrix.repository)" \
    "$(manifest_value sources.matrix.revision)" \
    "$MATRIX_DIR"
  clone_pinned \
    "RoamerX" \
    "$(manifest_value sources.roamerx.repository)" \
    "$(manifest_value sources.roamerx.revision)" \
    "$ROAMERX_DIR"
  clone_pinned \
    "DreamWaQ Go2-W" \
    "$(manifest_value sources.dreamwaq.repository)" \
    "$(manifest_value sources.dreamwaq.revision)" \
    "$DREAMWAQ_DIR"
  clone_pinned \
    "Cosmos framework" \
    "$(manifest_value sources.cosmos_framework.repository)" \
    "$(manifest_value sources.cosmos_framework.revision)" \
    "$COSMOS_FRAMEWORK"

  apply_patch_once \
    "$MATRIX_DIR" \
    "${ROOT_DIR}/$(manifest_value sources.matrix.patch)"
  apply_patch_once \
    "$ROAMERX_DIR" \
    "${ROOT_DIR}/$(manifest_value sources.roamerx.patch)"
  apply_patch_once \
    "$COSMOS_FRAMEWORK" \
    "${ROOT_DIR}/$(manifest_value sources.cosmos_framework.patch)"
}

check_ubuntu_2204() {
  if [[ ! -r /etc/os-release ]]; then
    echo "[ERROR] cannot identify the operating system" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "[ERROR] Ubuntu 22.04 is required; found ${PRETTY_NAME:-unknown}" >&2
    exit 1
  fi
}

install_system() {
  check_ubuntu_2204
  require_command sudo
  if [[ ! -x "${MATRIX_DIR}/scripts/install_deps.sh" ]]; then
    echo "[ERROR] run '$0 sources' before installing system packages" >&2
    exit 1
  fi
  bash "${MATRIX_DIR}/scripts/install_deps.sh"
  sudo apt-get update
  sudo apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-slam-toolbox
}

verify_matrix_release_archives() {
  require_command sha256sum
  local name
  local expected_size
  local expected_sha256
  while IFS=$'\t' read -r name expected_size expected_sha256; do
    local archive="${MATRIX_DIR}/releases/${name}"
    if [[ ! -f "$archive" ]]; then
      echo "[ERROR] verified MATRiX archive is missing: ${archive}" >&2
      exit 1
    fi

    local actual_size
    actual_size="$(stat -c '%s' "$archive")"
    if [[ "$actual_size" != "$expected_size" ]]; then
      echo "[ERROR] MATRiX archive size mismatch: ${name}" >&2
      echo "        expected: ${expected_size}" >&2
      echo "        actual:   ${actual_size}" >&2
      exit 1
    fi

    local actual_sha256
    actual_sha256="$(sha256sum "$archive")"
    actual_sha256="${actual_sha256%% *}"
    if [[ "$actual_sha256" != "$expected_sha256" ]]; then
      echo "[ERROR] MATRiX archive SHA256 mismatch: ${name}" >&2
      echo "        expected: ${expected_sha256}" >&2
      echo "        actual:   ${actual_sha256}" >&2
      exit 1
    fi
    echo "[OK] verified MATRiX release archive: ${name}"
  done < <(
    /usr/bin/python3 - "$MANIFEST" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1], encoding="utf-8"))
packages = manifest["sources"]["matrix"]["release"]["packages"]
for name in sorted(packages):
    package = packages[name]
    print(f"{name}\t{package['size']}\t{package['sha256']}")
PY
  )
}

install_assets() {
  if [[ ! -x "${MATRIX_DIR}/scripts/release_manager/install_chunks.sh" ]]; then
    echo "[ERROR] run '$0 sources' before downloading MATRiX assets" >&2
    exit 1
  fi
  printf 'HouseWorld\n\n' |
    bash "${MATRIX_DIR}/scripts/release_manager/install_chunks.sh" 0.1.2
  verify_matrix_release_archives
}

install_model() {
  local hf_bin="${HF_BIN:-}"
  if [[ -z "$hf_bin" ]]; then
    hf_bin="$(command -v hf || true)"
  fi
  if [[ -z "$hf_bin" ]]; then
    echo "[ERROR] Hugging Face CLI is missing; install from https://huggingface.co/docs/huggingface_hub/guides/cli" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$COSMOS_CHECKPOINT")"
  "$hf_bin" download \
    "$(manifest_value sources.cosmos_checkpoint.repository)" \
    --revision "$(manifest_value sources.cosmos_checkpoint.revision)" \
    --local-dir "$COSMOS_CHECKPOINT"
}

build_runtime() {
  local uv_bin="${UV_BIN:-}"
  if [[ -z "$uv_bin" ]]; then
    uv_bin="$(command -v uv || true)"
  fi
  if [[ -z "$uv_bin" ]]; then
    echo "[ERROR] uv is missing; install from https://docs.astral.sh/uv/" >&2
    exit 1
  fi
  if [[ ! -d "$ROAMERX_DIR/.git" || ! -d "$COSMOS_FRAMEWORK/.git" ]]; then
    echo "[ERROR] run '$0 sources' before building" >&2
    exit 1
  fi

  (
    cd "$ROAMERX_DIR"
    bash build.sh all
  )
  (
    cd "$COSMOS_FRAMEWORK"
    "$uv_bin" sync --group "$(manifest_value sources.cosmos_framework.cuda_group)"
  )

  local cosmos_python="${COSMOS_FRAMEWORK}/.venv/bin/python"
  local torch_cmake
  torch_cmake="$(
    "$cosmos_python" -c \
      'import torch; print(torch.utils.cmake_prefix_path)'
  )"
  cmake \
    -S "${ROOT_DIR}/controllers/go2w_rl_bridge" \
    -B "${ROOT_DIR}/build/go2w_rl_bridge" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${RUNTIME_INSTALL}" \
    -DCMAKE_PREFIX_PATH="$torch_cmake"
  cmake --build "${ROOT_DIR}/build/go2w_rl_bridge" \
    --parallel "$(nproc)"
  cmake --install "${ROOT_DIR}/build/go2w_rl_bridge"
}

check_file() {
  local path="$1"
  local description="$2"
  if [[ -f "$path" ]]; then
    echo "[OK] ${description}: ${path}"
  else
    echo "[MISSING] ${description}: ${path}"
    missing_count=$((missing_count + 1))
  fi
}

check_package_version() {
  local package="$1"
  local expected_version="$2"
  local description="$3"
  local actual_version
  actual_version="$(dpkg-query -W -f='${Version}' "$package" 2>/dev/null || true)"
  if [[ "$actual_version" == "$expected_version" ]]; then
    echo "[OK] ${description}: ${package} ${actual_version}"
  else
    echo "[MISSING] ${description}: expected ${package} ${expected_version}, found ${actual_version:-not installed}"
    missing_count=$((missing_count + 1))
  fi
}

check_runtime() {
  local missing_count=0
  local os_ok=1
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
      os_ok=0
      echo "[OK] operating system: ${PRETTY_NAME}"
    fi
  fi
  if [[ "$os_ok" -ne 0 ]]; then
    echo "[MISSING] operating system: Ubuntu 22.04 is required"
    missing_count=$((missing_count + 1))
  fi

  check_file "/opt/ros/humble/setup.bash" "ROS 2 Humble"
  check_file "$ROAMERX_DIR/install/setup.bash" "RoamerX workspace"
  check_file \
    "/opt/robot/robot-forward/install/setup.bash" \
    "robot-forward setup"
  check_file \
    "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward" \
    "robot-forward binary"
  check_package_version \
    "robot-forward" \
    "$(manifest_value system_packages.robot_forward)" \
    "robot-forward package"
  check_file \
    "/opt/ros/humble/lib/librmw_zenoh_cpp.so" \
    "rmw_zenoh_cpp"
  check_file \
    "${ROAMERX_DIR}/install/robot_navigo/lib/robot_navigo/vel_cmd_lcm_pub" \
    "robot_navigo velocity bridge"
  check_file "${MATRIX_DIR}/scripts/run_sim.sh" "MATRiX launcher"
  check_file \
    "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_house.xml" \
    "HouseWorld MuJoCo scene"
  check_file \
    "${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks/pakchunk17-Linux.pak" \
    "HouseWorld UE pak"
  check_file \
    "${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks/pakchunk17-Linux.ucas" \
    "HouseWorld UE ucas"
  check_file \
    "${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks/pakchunk17-Linux.utoc" \
    "HouseWorld UE utoc"
  check_file "${RUNTIME_INSTALL}/bin/go2w_rl_bridge" "Go2-W RL bridge"
  check_file \
    "/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node" \
    "slam_toolbox"
  check_file \
    "${COSMOS_FRAMEWORK}/.venv/bin/python" \
    "Cosmos Python environment"
  check_file \
    "${COSMOS_FRAMEWORK}/cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml" \
    "Cosmos3-Edge model config"
  check_file \
    "${COSMOS_CHECKPOINT}/transformer/diffusion_pytorch_model-00001-of-00002.safetensors" \
    "Cosmos3-Edge transformer shard 1"
  check_file \
    "${COSMOS_CHECKPOINT}/transformer/diffusion_pytorch_model-00002-of-00002.safetensors" \
    "Cosmos3-Edge transformer shard 2"
  check_file \
    "${COSMOS_CHECKPOINT}/vae/diffusion_pytorch_model.safetensors" \
    "Cosmos3-Edge VAE"
  check_file \
    "${COSMOS_CHECKPOINT}/vision_encoder/model.safetensors" \
    "Cosmos3-Edge vision encoder"

  local weight
  for weight in \
    actor_dwaq.pt \
    encoder_dwaq.pt \
    latent_mu_dwaq.pt \
    latent_var_dwaq.pt \
    vel_mu_dwaq.pt \
    vel_var_dwaq.pt; do
    check_file \
      "${DREAMWAQ_DIR}/deploy/pre_train/g2wDWAQ/${weight}" \
      "DreamWaQ ${weight}"
  done

  if [[ "$missing_count" -ne 0 ]]; then
    echo "[FAIL] ${missing_count} required runtime item(s) are missing" >&2
    echo "[INFO] install in order: sources -> system -> assets -> model -> build" >&2
    return 1
  fi
  echo "[OK] complete Go2-W HouseWorld runtime is present"
}

command="${1:-check}"
case "$command" in
  check)
    check_runtime
    ;;
  sources)
    install_sources
    ;;
  system)
    install_system
    ;;
  assets)
    install_assets
    ;;
  model)
    install_model
    ;;
  build)
    build_runtime
    ;;
  all)
    install_sources
    install_system
    install_assets
    install_model
    build_runtime
    check_runtime
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
