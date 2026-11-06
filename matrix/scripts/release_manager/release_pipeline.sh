#!/usr/bin/env bash
set -euo pipefail

# End-to-end release pipeline:
# jszr_mujoco_ue2 source package -> matrix release artifacts -> local install test -> optional upload.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

VERSION=""
SOURCE_DIR="${MATRIX_SOURCE_DIR:-${PROJECT_ROOT}/../jszr_mujoco_ue2}"
MATRIX_DIR="${PROJECT_ROOT}"
RUN_UE_PACKAGE=1
RUN_RELEASE_PACKAGE=1
RUN_COPY=1
RUN_ASSETS_PACKAGE=1
RUN_SPLIT=1
RUN_MANIFEST=1
RUN_INSTALL_DEPS=0
RUN_LOCAL_INSTALL=1
RUN_ENV_CHECK=1
RUN_UPLOAD=0
PUBLISH_MODE="no"
MAKE_LATEST=0
DRY_RUN=0

PACKAGE_ARGS=(--all --incremental --shipping)

usage() {
    cat <<'EOF'
Usage:
  scripts/release_manager/release_pipeline.sh VERSION [options]

Pipeline:
  1. Run jszr_mujoco_ue2/Script/package_with_chunks.sh
  2. Run jszr_mujoco_ue2/Script/package_for_release.sh
  3. Copy dist/release/VERSION artifacts into matrix/releases
  4. Package matrix runtime assets into assets-VERSION.tar.gz
  5. Split files over GitHub's 2GB asset limit
  6. Regenerate checksums and manifest
  7. Install from local artifacts and run runtime environment checks
  8. Optionally upload/publish GitHub Release

Options:
  --source-dir PATH       Source UE project directory (default: ../jszr_mujoco_ue2)
  --chunks ARGS           Chunk packaging args, quoted, e.g. "--base --shared --chunks=24"
  --all                   Package all chunks (default)
  --base                  Add base chunk package arg
  --shared                Add shared chunk package arg
  --custom                Add custom chunk package arg
  --chunk-ids IDS         Package specific chunk ids, e.g. 0,1,24
  --clean                 Use clean UE packaging
  --incremental           Use incremental UE packaging (default)
  --shipping              Use Shipping UE package config (default)
  --development           Use Development UE package config
  --with-debuginfo        Keep debug symbols in UE package
  --install-deps          Run matrix/scripts/install_deps.sh before local install
  --skip-ue-package       Skip package_with_chunks.sh
  --skip-release-package  Skip package_for_release.sh
  --skip-copy             Skip copying artifacts into matrix/releases
  --skip-assets-package   Skip packaging matrix runtime assets
  --skip-split            Skip large-file splitting
  --skip-manifest         Skip checksum and manifest regeneration
  --skip-local-install    Skip local artifact install test
  --skip-env-check        Skip runtime environment check
  --no-test               Same as --skip-local-install --skip-env-check
  --upload                Upload artifacts to GitHub Release and keep draft by default
  --publish               Upload and publish the GitHub Release
  --make-latest           Mark the GitHub Release as latest after upload/publish
  --dry-run               Print commands without executing them
  -h, --help              Show this help

Examples:
  bash scripts/release_manager/release_pipeline.sh 0.1.3 --all --upload --publish --make-latest
  bash scripts/release_manager/release_pipeline.sh 0.1.3 --chunk-ids 0,1,24 --clean --skip-local-install
EOF
}

log_cmd() {
    printf '[CMD]'
    printf ' %q' "$@"
    printf '\n'
}

run_cmd() {
    log_cmd "$@"
    if [ "$DRY_RUN" -eq 0 ]; then
        "$@"
    fi
}

require_file_readable() {
    local path="$1"
    if [ ! -f "$path" ]; then
        error_exit "missing file: $path"
    fi
}

require_dir_existing() {
    local path="$1"
    if [ ! -d "$path" ]; then
        error_exit "missing directory: $path"
    fi
}

reset_package_mode_args() {
    local keep=()
    local arg
    for arg in "${PACKAGE_ARGS[@]}"; do
        case "$arg" in
            --all|--base|--shared|--custom|--chunks=*)
                ;;
            *)
                keep+=("$arg")
                ;;
        esac
    done
    PACKAGE_ARGS=("${keep[@]}")
}

replace_package_build_arg() {
    local new_arg="$1"
    local keep=()
    local arg
    for arg in "${PACKAGE_ARGS[@]}"; do
        case "$arg" in
            --clean|--incremental)
                ;;
            *)
                keep+=("$arg")
                ;;
        esac
    done
    PACKAGE_ARGS=("${keep[@]}" "$new_arg")
}

replace_package_config_arg() {
    local new_arg="$1"
    local keep=()
    local arg
    for arg in "${PACKAGE_ARGS[@]}"; do
        case "$arg" in
            --shipping|--development|--dev)
                ;;
            *)
                keep+=("$arg")
                ;;
        esac
    done
    PACKAGE_ARGS=("${keep[@]}" "$new_arg")
}

parse_args() {
    if [ $# -eq 0 ]; then
        usage
        exit 2
    fi

    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        exit 0
    fi

    if [[ "${1:-}" == --* ]]; then
        usage >&2
        error_exit "missing VERSION"
    fi

    VERSION="$1"
    shift

    while [ $# -gt 0 ]; do
        case "$1" in
            --source-dir)
                SOURCE_DIR="$2"
                shift 2
                ;;
            --chunks)
                reset_package_mode_args
                read -r -a PACKAGE_ARGS <<< "$2"
                shift 2
                ;;
            --all)
                reset_package_mode_args
                PACKAGE_ARGS=(--all "${PACKAGE_ARGS[@]}")
                shift
                ;;
            --base|--shared|--custom)
                if [[ " ${PACKAGE_ARGS[*]} " == *" --all "* ]]; then
                    reset_package_mode_args
                fi
                PACKAGE_ARGS+=("$1")
                shift
                ;;
            --chunk-ids)
                reset_package_mode_args
                PACKAGE_ARGS+=("--chunks=$2")
                shift 2
                ;;
            --clean)
                replace_package_build_arg --clean
                shift
                ;;
            --incremental)
                replace_package_build_arg --incremental
                shift
                ;;
            --shipping)
                replace_package_config_arg --shipping
                shift
                ;;
            --development|--dev)
                replace_package_config_arg --development
                shift
                ;;
            --with-debuginfo)
                PACKAGE_ARGS+=(--with-debuginfo)
                shift
                ;;
            --install-deps)
                RUN_INSTALL_DEPS=1
                shift
                ;;
            --skip-ue-package)
                RUN_UE_PACKAGE=0
                shift
                ;;
            --skip-release-package)
                RUN_RELEASE_PACKAGE=0
                shift
                ;;
            --skip-copy)
                RUN_COPY=0
                shift
                ;;
            --skip-assets-package)
                RUN_ASSETS_PACKAGE=0
                shift
                ;;
            --skip-split)
                RUN_SPLIT=0
                shift
                ;;
            --skip-manifest)
                RUN_MANIFEST=0
                shift
                ;;
            --skip-local-install)
                RUN_LOCAL_INSTALL=0
                shift
                ;;
            --skip-env-check)
                RUN_ENV_CHECK=0
                shift
                ;;
            --no-test)
                RUN_LOCAL_INSTALL=0
                RUN_ENV_CHECK=0
                shift
                ;;
            --upload)
                RUN_UPLOAD=1
                PUBLISH_MODE="no"
                shift
                ;;
            --publish)
                RUN_UPLOAD=1
                PUBLISH_MODE="yes"
                shift
                ;;
            --make-latest)
                MAKE_LATEST=1
                shift
                ;;
            --dry-run)
                DRY_RUN=1
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
}

check_inputs() {
    SOURCE_DIR="$(cd "$SOURCE_DIR" && pwd)"
    MATRIX_DIR="$(cd "$MATRIX_DIR" && pwd)"

    require_dir_existing "$SOURCE_DIR"
    require_dir_existing "$MATRIX_DIR"
    require_file_readable "${SOURCE_DIR}/Script/package_with_chunks.sh"
    require_file_readable "${SOURCE_DIR}/Script/package_for_release.sh"
    require_file_readable "${MATRIX_DIR}/scripts/release_manager/split_large_files.sh"
    require_file_readable "${MATRIX_DIR}/scripts/release_manager/update_checksums_and_manifest.sh"
    require_file_readable "${MATRIX_DIR}/scripts/release_manager/package_assets.sh"
    require_file_readable "${MATRIX_DIR}/scripts/release_manager/install_chunks_local.sh"
    require_file_readable "${MATRIX_DIR}/scripts/check_env.sh"
}

copy_release_artifacts() {
    local source_release_dir="${SOURCE_DIR}/dist/release/${VERSION}"
    local target_release_dir="${MATRIX_DIR}/releases"
    local stale_files=()

    require_dir_existing "$source_release_dir"
    mkdir -p "$target_release_dir"

    shopt -s nullglob
    stale_files=(
        "${target_release_dir}"/*-"${VERSION}".tar.gz
        "${target_release_dir}"/*-"${VERSION}".tar.gz.sha256
        "${target_release_dir}"/*-"${VERSION}".tar.part*
        "${target_release_dir}"/*-"${VERSION}".part*
        "${target_release_dir}"/*-"${VERSION}".tar.merge.sh
        "${target_release_dir}"/*-"${VERSION}".merge.sh
        "${target_release_dir}"/*-"${VERSION}".tar.sha256
        "${target_release_dir}/manifest-${VERSION}.json"
        "${target_release_dir}/checksums-${VERSION}.sha256"
        "${target_release_dir}/RELEASE_NOTES-${VERSION}.md"
    )
    shopt -u nullglob

    if [ "${#stale_files[@]}" -gt 0 ]; then
        log "清理 matrix/releases 中旧的 v${VERSION} 发布产物，避免复用过期分片"
        run_cmd rm -f -- "${stale_files[@]}"
    fi

    log "复制发布产物: ${source_release_dir} -> ${target_release_dir}"
    run_cmd rsync -a \
        --include="*/" \
        --include="*-${VERSION}.tar.gz" \
        --include="manifest-${VERSION}.json" \
        --include="checksums-${VERSION}.sha256" \
        --include="RELEASE_NOTES-${VERSION}.md" \
        --include="README.md" \
        --exclude="*" \
        "${source_release_dir}/" \
        "${target_release_dir}/"
}

verify_release_artifacts() {
    local release_dir="${MATRIX_DIR}/releases"
    local manifest="${release_dir}/manifest-${VERSION}.json"

    require_file_readable "${release_dir}/base-${VERSION}.tar.gz"
    require_file_readable "$manifest"

    if [ ! -f "${release_dir}/shared-${VERSION}.tar.gz" ] && [ ! -f "${release_dir}/shared-${VERSION}.tar.part000" ]; then
        error_exit "missing shared package or split parts for ${VERSION}"
    fi

    if [ ! -f "${release_dir}/assets-${VERSION}.tar.gz" ] && [ ! -f "${release_dir}/assets-${VERSION}.tar.part000" ]; then
        error_exit "missing assets package or split parts for ${VERSION}"
    fi

    if command -v jq >/dev/null 2>&1; then
        jq empty "$manifest" >/dev/null
    fi

    log "✓ 发布产物验证通过: ${release_dir}"
}

make_release_latest() {
    local repo="zsibot/matrix"
    local release_id

    if [ "$DRY_RUN" -eq 1 ]; then
        log_cmd gh api "repos/${repo}/releases/tags/v${VERSION}" --jq '.id'
        log_cmd gh api -X PATCH "repos/${repo}/releases/<release_id>" -f make_latest=true
        return 0
    fi

    release_id="$(gh api "repos/${repo}/releases/tags/v${VERSION}" --jq '.id')"
    run_cmd gh api -X PATCH "repos/${repo}/releases/${release_id}" -f make_latest=true >/dev/null
    gh api "repos/${repo}/releases/latest" --jq '.tag_name + " " + .html_url'
}

main() {
    parse_args "$@"
    check_inputs

    log_section "Release pipeline v${VERSION}"
    log "source: ${SOURCE_DIR}"
    log "matrix: ${MATRIX_DIR}"
    log "package args: ${PACKAGE_ARGS[*]}"

    if [ "$RUN_UE_PACKAGE" -eq 1 ]; then
        log_section "[1] UE chunk package"
        run_cmd bash "${SOURCE_DIR}/Script/package_with_chunks.sh" "$VERSION" "${PACKAGE_ARGS[@]}"
    fi

    if [ "$RUN_RELEASE_PACKAGE" -eq 1 ]; then
        log_section "[2] Build release tarballs"
        run_cmd bash "${SOURCE_DIR}/Script/package_for_release.sh" "$VERSION"
    fi

    if [ "$RUN_COPY" -eq 1 ]; then
        log_section "[3] Copy artifacts to matrix/releases"
        copy_release_artifacts
    fi

    if [ "$RUN_ASSETS_PACKAGE" -eq 1 ]; then
        log_section "[4] Package matrix runtime assets"
        run_cmd bash "${MATRIX_DIR}/scripts/release_manager/package_assets.sh" "$VERSION"
    fi

    if [ "$RUN_SPLIT" -eq 1 ]; then
        log_section "[5] Split large files"
        run_cmd bash "${MATRIX_DIR}/scripts/release_manager/split_large_files.sh" "$VERSION" "${MATRIX_DIR}/releases"
    fi

    if [ "$RUN_MANIFEST" -eq 1 ]; then
        log_section "[6] Regenerate checksums and manifest"
        run_cmd bash "${MATRIX_DIR}/scripts/release_manager/update_checksums_and_manifest.sh" "$VERSION" "${MATRIX_DIR}/releases"
    fi

    if [ "$DRY_RUN" -eq 0 ]; then
        verify_release_artifacts
    fi

    if [ "$RUN_INSTALL_DEPS" -eq 1 ]; then
        log_section "[7] Install system dependencies"
        run_cmd bash "${MATRIX_DIR}/scripts/install_deps.sh"
    fi

    if [ "$RUN_LOCAL_INSTALL" -eq 1 ]; then
        log_section "[8] Local install test"
        run_cmd bash "${MATRIX_DIR}/scripts/release_manager/install_chunks_local.sh" "$VERSION"
    fi

    if [ "$RUN_ENV_CHECK" -eq 1 ]; then
        log_section "[9] Runtime environment check"
        run_cmd bash "${MATRIX_DIR}/scripts/check_env.sh" runtime
    fi

    if [ "$RUN_UPLOAD" -eq 1 ]; then
        log_section "[10] Upload GitHub Release"
        run_cmd env MATRIX_RELEASE_PUBLISH="$PUBLISH_MODE" bash "${MATRIX_DIR}/scripts/release_manager/upload_to_release.sh" "$VERSION"
    fi

    if [ "$MAKE_LATEST" -eq 1 ]; then
        if [ "$RUN_UPLOAD" -ne 1 ]; then
            log "⚠️  --make-latest requested without --upload/--publish; updating existing release"
        fi
        log_section "[11] Mark GitHub Release as latest"
        make_release_latest
    fi

    log_section "Done"
    log "Release pipeline completed for v${VERSION}"
}

main "$@"
