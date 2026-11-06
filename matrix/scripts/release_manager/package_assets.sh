#!/usr/bin/env bash
set -euo pipefail

# Package runtime assets required by a source checkout to run a released MATRiX build.
# This script only copies files into releases/assets-VERSION.tar.gz; it never removes
# files from the working tree.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

VERSION="${1:-}"
RELEASE_DIR="${PROJECT_ROOT}/releases"
ALLOW_MISSING=0

usage() {
    cat <<'EOF'
Usage:
  scripts/release_manager/package_assets.sh VERSION [options]

Options:
  --release-dir PATH  Output directory (default: matrix/releases)
  --allow-missing     Warn instead of failing when optional runtime groups are missing
  -h, --help          Show this help

The package includes launcher binaries, MuJoCo runtime, MC runtime/config/model
files, UE Engine runtime libraries, and dynamic map payloads. It does not delete
or modify the source runtime files.
EOF
}

if [[ "${VERSION:-}" == "-h" || "${VERSION:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ -z "$VERSION" || "$VERSION" == --* ]]; then
    usage >&2
    error_exit "missing VERSION"
fi
shift || true

while [ $# -gt 0 ]; do
    case "$1" in
        --release-dir)
            RELEASE_DIR="${2:-}"
            shift 2
            ;;
        --allow-missing)
            ALLOW_MISSING=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            error_exit "unknown option: $1"
            ;;
    esac
done

PACKAGE_NAME="assets-${VERSION}.tar.gz"
PACKAGE_PATH="${RELEASE_DIR}/${PACKAGE_NAME}"
TEMP_DIR="${RELEASE_DIR}/.temp_assets_${VERSION}"

FILE_COUNT=0
TOTAL_SIZE=0
WARN_COUNT=0

human_size() {
    local size="$1"
    numfmt --to=iec-i --suffix=B "$size" 2>/dev/null || echo "${size} bytes"
}

file_size() {
    stat -c%s "$1" 2>/dev/null || stat -f%z "$1" 2>/dev/null || echo 0
}

add_size_for_file() {
    local source="$1"
    local size
    size="$(file_size "$source")"
    FILE_COUNT=$((FILE_COUNT + 1))
    TOTAL_SIZE=$((TOTAL_SIZE + size))
}

warn_missing() {
    WARN_COUNT=$((WARN_COUNT + 1))
    log "⚠️  missing: $1"
}

require_runtime_file() {
    local relative_path="$1"
    local source="${PROJECT_ROOT}/${relative_path}"
    if [ ! -f "$source" ]; then
        error_exit "missing required runtime file: ${relative_path}"
    fi
}

copy_file_if_exists() {
    local relative_path="$1"
    local source="${PROJECT_ROOT}/${relative_path}"
    local target="${TEMP_DIR}/${relative_path}"

    if [ ! -f "$source" ] && [ ! -L "$source" ]; then
        return 1
    fi

    mkdir -p "$(dirname "$target")"
    cp -a "$source" "$target"
    add_size_for_file "$source"
    log "  ✓ ${relative_path} ($(human_size "$(file_size "$source")"))"
    return 0
}

copy_dir_if_exists() {
    local relative_path="$1"
    local source="${PROJECT_ROOT}/${relative_path}"
    local target="${TEMP_DIR}/${relative_path}"
    local count
    local size

    if [ ! -d "$source" ]; then
        return 1
    fi

    mkdir -p "$target"
    cp -a "${source}/." "$target/"

    count="$(find "$source" -type f | wc -l)"
    size="$(du -sb "$source" 2>/dev/null | cut -f1 || echo 0)"
    FILE_COUNT=$((FILE_COUNT + count))
    TOTAL_SIZE=$((TOTAL_SIZE + size))
    log "  ✓ ${relative_path}: ${count} files ($(human_size "$size"))"
    return 0
}

copy_glob_group() {
    local label="$1"
    local relative_dir="$2"
    local pattern="$3"
    local source_dir="${PROJECT_ROOT}/${relative_dir}"
    local source_file
    local copied=0

    if [ ! -d "$source_dir" ]; then
        return 1
    fi

    shopt -s nullglob
    for source_file in "${source_dir}"/${pattern}; do
        if [ -f "$source_file" ] || [ -L "$source_file" ]; then
            local relative_path="${relative_dir}/$(basename "$source_file")"
            copy_file_if_exists "$relative_path" >/dev/null
            copied=$((copied + 1))
        fi
    done
    shopt -u nullglob

    if [ "$copied" -gt 0 ]; then
        log "  ✓ ${label}: ${copied} files"
        return 0
    fi
    return 1
}

copy_engine_runtime() {
    local engine_root="${PROJECT_ROOT}/src/UeSim/Linux/Engine"
    local plugin_file
    local binary_dir
    local content_file
    local copied=0

    if [ ! -d "$engine_root" ]; then
        return 1
    fi

    log "收集 UE Engine runtime..."

    copy_file_if_exists "src/UeSim/Linux/Engine/Engine" >/dev/null && copied=$((copied + 1)) || true
    copy_dir_if_exists "src/UeSim/Linux/Engine/Binaries" >/dev/null && copied=$((copied + 1)) || true

    while IFS= read -r -d '' binary_dir; do
        local relative_path="${binary_dir#${PROJECT_ROOT}/}"
        copy_dir_if_exists "$relative_path" >/dev/null && copied=$((copied + 1)) || true
    done < <(find "${engine_root}/Plugins" -type d -name Binaries -print0 2>/dev/null)

    while IFS= read -r -d '' plugin_file; do
        local relative_path="${plugin_file#${PROJECT_ROOT}/}"
        copy_file_if_exists "$relative_path" >/dev/null && copied=$((copied + 1)) || true
    done < <(find "${engine_root}/Plugins" -type f -name '*.uplugin' -print0 2>/dev/null)

    while IFS= read -r -d '' content_file; do
        local relative_path="${content_file#${PROJECT_ROOT}/}"
        copy_file_if_exists "$relative_path" >/dev/null && copied=$((copied + 1)) || true
    done < <(find "${engine_root}/Content" -type f -name '*.bin' -print0 2>/dev/null)

    if [ "$copied" -gt 0 ]; then
        local size
        size="$(du -sb "${TEMP_DIR}/src/UeSim/Linux/Engine" 2>/dev/null | cut -f1 || echo 0)"
        log "  ✓ UE Engine runtime groups: ${copied} ($(human_size "$size"))"
        return 0
    fi
    return 1
}

validate_required_runtime() {
    require_runtime_file "src/UeSim/Linux/Engine/Binaries/Linux/libEOSSDK-Linux-Shipping.so"
    require_runtime_file "src/UeSim/Linux/Engine/Content/Renderer/TessellationTable.bin"
    require_runtime_file "src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux/libopencv_world.so.405"
    require_runtime_file "src/robot_mujoco/simulate/build/robot_mujoco"
    require_runtime_file "src/robot_mc/build/export/mc/bin/mc_ctrl"

    if [ ! -f "${PROJECT_ROOT}/bin/sim_launcher" ] && [ ! -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ]; then
        error_exit "missing launcher runtime: bin/sim_launcher or bin/sim_launcher.bin"
    fi
}

package_assets() {
    mkdir -p "$RELEASE_DIR"
    rm -rf "$TEMP_DIR"
    mkdir -p "$TEMP_DIR"

    validate_required_runtime

    log_section "打包 Assets 文件 (版本: ${VERSION})"

    log_section "[1] 收集 launcher"
    copy_file_if_exists "bin/sim_launcher" || true
    copy_file_if_exists "bin/sim_launcher.bin" || true
    copy_file_if_exists "bin/open_sim_launcher" || true

    log_section "[2] 收集 MuJoCo runtime"
    copy_file_if_exists "src/robot_mujoco/simulate/build/robot_mujoco"
    copy_file_if_exists "src/robot_mujoco/simulate/build/zsibot_mujoco" || true
    copy_file_if_exists "src/robot_mujoco/simulate/build/DynamicMapData.bin" || true
    copy_file_if_exists "src/robot_mujoco/simulate/config.yaml" || true

    log_section "[3] 收集 MC runtime"
    copy_dir_if_exists "src/robot_mc/build/export/mc/bin" || error_exit "missing MC runtime dir"
    copy_dir_if_exists "src/robot_mc/build/export/config" || warn_missing "src/robot_mc/build/export/config"
    copy_file_if_exists "src/robot_mc/build/export/mile_data.txt" || true

    log "收集发布控制模型..."
    copy_dir_if_exists "src/robot_mc/build/export/onnx_model_crypto/xg" || warn_missing "onnx_model_crypto/xg"
    copy_dir_if_exists "src/robot_mc/build/export/onnx_model_crypto/xg_wheel" || warn_missing "onnx_model_crypto/xg_wheel"
    copy_dir_if_exists "src/robot_mc/build/export/onnx_model_crypto/zg_wheels" || warn_missing "onnx_model_crypto/zg_wheels"

    log_section "[4] 收集 UE runtime"
    copy_engine_runtime || error_exit "missing UE Engine runtime files"

    log_section "[5] 收集 dynamic maps"
    copy_dir_if_exists "dynamicmaps" || warn_missing "dynamicmaps"

    if [ "$WARN_COUNT" -gt 0 ] && [ "$ALLOW_MISSING" -ne 1 ]; then
        error_exit "assets package has ${WARN_COUNT} missing optional runtime groups; rerun with --allow-missing to override"
    fi

    log_section "[6] 创建压缩包"
    log "共收集 ${FILE_COUNT} 个文件，总大小: $(human_size "$TOTAL_SIZE")"
    if [ "$FILE_COUNT" -eq 0 ]; then
        error_exit "no files collected"
    fi

    (cd "$TEMP_DIR" && tar -czf "$PACKAGE_PATH" .)

    local package_size
    local sha256
    package_size="$(file_size "$PACKAGE_PATH")"
    sha256="$(sha256sum "$PACKAGE_PATH" | awk '{print $1}')"
    sha256sum "$PACKAGE_PATH" > "${PACKAGE_PATH}.sha256"
    echo "$sha256" > "${RELEASE_DIR}/.assets_sha256_${VERSION}.txt"

    log "✓ ${PACKAGE_NAME}: $(human_size "$package_size")"
    log "✓ SHA256: ${sha256}"

    log_section "[7] 验证 assets 包"
    tar -tzf "$PACKAGE_PATH" >/dev/null
    log "✓ tar.gz 可读取"

    rm -rf "$TEMP_DIR"
    log "✓ 临时文件已清理"
}

package_assets
