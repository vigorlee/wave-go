#!/bin/bash
set -euo pipefail

# ============================================================================
# 使用 GitHub CLI 上传文件到 Release
# 需要先安装: sudo apt install gh
# 需要先登录: gh auth login
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 加载公共函数库
source "${SCRIPT_DIR}/common.sh"
cd "$PROJECT_ROOT"

VERSION="0.1.2"
REPO="${MATRIX_RELEASE_REPO:-zsibot/matrix}"
PUBLISH_RELEASE="${MATRIX_RELEASE_PUBLISH:-ask}"
RELEASE_DIR="releases"
MAX_SIZE=2147483648  # 2GB in bytes (GitHub Releases limit)

usage() {
    cat <<'EOF'
Usage:
  scripts/release_manager/upload_to_release.sh [VERSION] [options]

Options:
  --repo OWNER/REPO  GitHub repository (default: zsibot/matrix)
  --publish          Publish draft release without prompting
  --draft            Keep draft release without prompting
  --ask              Ask whether to publish draft release (default)
  -h, --help         Show this help
EOF
}

if [ $# -gt 0 ] && [[ "${1:-}" != --* ]]; then
    VERSION="$1"
    shift
fi

while [ $# -gt 0 ]; do
    case "$1" in
        --repo)
            REPO="${2:-}"
            shift 2
            ;;
        --publish)
            PUBLISH_RELEASE="yes"
            shift
            ;;
        --draft|--no-publish)
            PUBLISH_RELEASE="no"
            shift
            ;;
        --ask)
            PUBLISH_RELEASE="ask"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            error_exit "Unknown option: $1"
            ;;
    esac
done

case "$PUBLISH_RELEASE" in
    yes|true|1)
        PUBLISH_RELEASE="yes"
        ;;
    no|false|0)
        PUBLISH_RELEASE="no"
        ;;
    ask|"")
        PUBLISH_RELEASE="ask"
        ;;
    *)
        error_exit "Invalid MATRIX_RELEASE_PUBLISH value: ${PUBLISH_RELEASE} (expected yes, no, or ask)"
        ;;
esac

agent_log() {
    if [ -n "${log_file:-}" ]; then
        echo "$1" >> "$log_file"
    fi
}

# 函数：显示上传进度
show_upload_progress() {
    local current=$1
    local total=$2
    local filename=$3
    local file_size_mb=$4

    local percent=$((current * 100 / total))
    local bar_length=30
    local filled=$((percent * bar_length / 100))
    local empty=$((bar_length - filled))

    printf "\r[进度] ["
    printf "%${filled}s" | tr ' ' '='
    printf "%${empty}s" | tr ' ' '-'
    printf "] %3d%% (%d/%d) - %s (%dMB)" "$percent" "$current" "$total" "$filename" "$file_size_mb"
}

# 函数：上传文件（带进度显示）
upload_file_with_progress() {
    local file="$1"
    local current_num=$2
    local total_num=$3
    local filename=$(basename "$file")
    local file_size=${file_sizes["$file"]:-0}
    local file_size_mb=$((file_size / 1024 / 1024))

    # 显示开始上传
    show_upload_progress "$current_num" "$total_num" "$filename" "$file_size_mb"
    echo ""

    # 记录开始时间
    local start_time=$(date +%s)

    # 执行上传（在后台运行，同时显示进度）
    local upload_pid
    local temp_output=$(mktemp)

    # #region agent log
    local upload_cmd="gh release upload \"v${VERSION}\" \"$file\" --repo \"$REPO\" --clobber"
    agent_log "{\"id\":\"log_$(date +%s)_upload_cmd\",\"timestamp\":$(date +%s)000,\"location\":\"upload_to_release.sh:58\",\"message\":\"Upload command\",\"data\":{\"filename\":\"$filename\",\"command\":\"$upload_cmd\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}"
    # #endregion

    if gh release upload "v${VERSION}" "$file" --repo "$REPO" --clobber > "$temp_output" 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        local speed_mb=$(echo "scale=2; $file_size_mb / $duration" | bc 2>/dev/null || echo "0")

        # #region agent log
        agent_log "{\"id\":\"log_$(date +%s)_upload_success\",\"timestamp\":$(date +%s)000,\"location\":\"upload_to_release.sh:80\",\"message\":\"Upload success\",\"data\":{\"filename\":\"$filename\",\"duration\":$duration,\"speed_mb\":$speed_mb},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"ALL\"}"
        # #endregion

        printf "\r[完成] ✓ %s (%dMB, 耗时: %ds, 速度: %.2fMB/s)\n" "$filename" "$file_size_mb" "$duration" "$speed_mb"
        rm -f "$temp_output"
        return 0
    else
        # #region agent log
        local error_output=$(cat "$temp_output" 2>/dev/null || echo "")
        agent_log "{\"id\":\"log_$(date +%s)_upload_failure\",\"timestamp\":$(date +%s)000,\"location\":\"upload_to_release.sh:92\",\"message\":\"Upload command failed\",\"data\":{\"filename\":\"$filename\",\"error_output\":\"$error_output\",\"file_size\":$file_size,\"actual_size\":${actual_size:-0}},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"ALL\"}"
        # #endregion

        # 检查文件是否实际上传成功（可能是GitHub API的临时错误，但文件实际上传了）
        local uploaded_asset=$(gh release view "v${VERSION}" --repo "$REPO" --json assets --jq ".assets[] | select(.name == \"$filename\")" 2>/dev/null)
        if [ -n "$uploaded_asset" ] && [ "$uploaded_asset" != "null" ]; then
            local uploaded_size=$(echo "$uploaded_asset" | jq -r '.size' 2>/dev/null || echo "0")
            # #region agent log
            agent_log "{\"id\":\"log_$(date +%s)_upload_verify\",\"timestamp\":$(date +%s)000,\"location\":\"upload_to_release.sh:96\",\"message\":\"File actually uploaded despite error\",\"data\":{\"filename\":\"$filename\",\"uploaded_size\":$uploaded_size,\"expected_size\":$file_size},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}"
            # #endregion

            # 检查大小是否匹配（允许1MB的误差）
            local size_diff=$((file_size - uploaded_size))
            if [ "${size_diff#-}" -lt 1048576 ]; then
                local end_time=$(date +%s)
                local duration=$((end_time - start_time))
                local speed_mb=$(echo "scale=2; $file_size_mb / $duration" | bc 2>/dev/null || echo "0")
                printf "\r[完成] ✓ %s (%dMB, 耗时: %ds, 速度: %.2fMB/s) [已存在]\n" "$filename" "$file_size_mb" "$duration" "$speed_mb"
                rm -f "$temp_output"
                return 0
            fi
        fi

        printf "\r[失败] ⚠️  %s 上传失败\n" "$filename"
        cat "$temp_output" >&2
        rm -f "$temp_output"
        return 1
    fi
}

# log_section() 和 error_exit() 已在 common.sh 中定义

# 检查 GitHub CLI
if ! command -v gh &> /dev/null; then
    log "GitHub CLI 未安装"
    log ""
    log "请先安装 GitHub CLI:"
    log "  sudo apt update"
    log "  sudo apt install -y gh"
    log ""
    log "然后登录:"
    log "  gh auth login"
    error_exit "需要先安装并登录 GitHub CLI"
fi

# 检查是否已登录
if ! gh auth status &>/dev/null; then
    log "GitHub CLI 未登录"
    log ""
    log "请先登录:"
    log "  gh auth login"
    error_exit "需要先登录 GitHub CLI"
fi

log "✓ GitHub CLI 已就绪"

log_section "[1] 检查要上传的文件"

# 检查 Release 目录
if [ ! -d "$RELEASE_DIR" ]; then
    error_exit "Release 目录不存在: $RELEASE_DIR"
fi

# 检查基础包是否存在
if [ ! -f "${RELEASE_DIR}/base-${VERSION}.tar.gz" ]; then
    error_exit "基础包不存在: ${RELEASE_DIR}/base-${VERSION}.tar.gz"
fi

# 检查共享包是否存在（可能是分片文件）
if [ ! -f "${RELEASE_DIR}/shared-${VERSION}.tar.gz" ] && [ ! -f "${RELEASE_DIR}/shared-${VERSION}.tar.part000" ]; then
    error_exit "共享资源包不存在: ${RELEASE_DIR}/shared-${VERSION}.tar.gz（也未找到分片文件）"
fi

# 收集所有需要上传的文件
log "扫描需要上传的文件..."
files_to_upload=()
declare -A file_sizes  # 关联数组存储文件大小

# 基础包
if [ -f "${RELEASE_DIR}/base-${VERSION}.tar.gz" ]; then
    files_to_upload+=("${RELEASE_DIR}/base-${VERSION}.tar.gz")
    size=$(stat -c%s "${RELEASE_DIR}/base-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "${RELEASE_DIR}/base-${VERSION}.tar.gz" 2>/dev/null || echo 0)
    file_sizes["${RELEASE_DIR}/base-${VERSION}.tar.gz"]=$size
fi

# 共享包（检查是否需要分片）
if [ -f "${RELEASE_DIR}/shared-${VERSION}.tar.gz" ]; then
    size=$(stat -c%s "${RELEASE_DIR}/shared-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "${RELEASE_DIR}/shared-${VERSION}.tar.gz" 2>/dev/null || echo 0)
    if [ "$size" -gt "$MAX_SIZE" ]; then
        # 超过 2GB，检查是否有分片文件
        if [ -f "${RELEASE_DIR}/shared-${VERSION}.tar.part000" ]; then
            # 添加分片文件
            for part_file in "${RELEASE_DIR}/shared-${VERSION}.tar.part"* "${RELEASE_DIR}/shared-${VERSION}.tar.merge.sh" "${RELEASE_DIR}/shared-${VERSION}.tar.sha256"; do
                if [ -f "$part_file" ]; then
                    files_to_upload+=("$part_file")
                    part_size=$(stat -c%s "$part_file" 2>/dev/null || stat -f%z "$part_file" 2>/dev/null || echo 0)
                    file_sizes["$part_file"]=$part_size
                fi
            done
        else
            # 没有分片文件，跳过（需要先分片）
            log "⚠️  shared-${VERSION}.tar.gz 超过 2GB 但未分片，跳过"
        fi
    else
        # 小于 2GB，直接添加
        files_to_upload+=("${RELEASE_DIR}/shared-${VERSION}.tar.gz")
        file_sizes["${RELEASE_DIR}/shared-${VERSION}.tar.gz"]=$size
    fi
elif [ -f "${RELEASE_DIR}/shared-${VERSION}.tar.part000" ]; then
    # 原文件不存在但分片文件存在（原文件已被分片后删除）
    for part_file in "${RELEASE_DIR}/shared-${VERSION}.tar.part"* "${RELEASE_DIR}/shared-${VERSION}.tar.merge.sh" "${RELEASE_DIR}/shared-${VERSION}.tar.sha256"; do
        if [ -f "$part_file" ]; then
            files_to_upload+=("$part_file")
            part_size=$(stat -c%s "$part_file" 2>/dev/null || stat -f%z "$part_file" 2>/dev/null || echo 0)
            file_sizes["$part_file"]=$part_size
        fi
    done
fi

# 资源文件包（检查是否需要分片）
if [ -f "${RELEASE_DIR}/assets-${VERSION}.tar.gz" ]; then
    size=$(stat -c%s "${RELEASE_DIR}/assets-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "${RELEASE_DIR}/assets-${VERSION}.tar.gz" 2>/dev/null || echo 0)
    if [ "$size" -gt "$MAX_SIZE" ]; then
        # 超过 2GB，检查是否有分片文件
        if [ -f "${RELEASE_DIR}/assets-${VERSION}.tar.part000" ]; then
            # 添加分片文件
            for part_file in "${RELEASE_DIR}/assets-${VERSION}.tar.part"* "${RELEASE_DIR}/assets-${VERSION}.tar.merge.sh" "${RELEASE_DIR}/assets-${VERSION}.tar.sha256"; do
                if [ -f "$part_file" ]; then
                    files_to_upload+=("$part_file")
                    part_size=$(stat -c%s "$part_file" 2>/dev/null || stat -f%z "$part_file" 2>/dev/null || echo 0)
                    file_sizes["$part_file"]=$part_size
                fi
            done
        else
            # 没有分片文件，跳过（需要先分片）
            log "⚠️  assets-${VERSION}.tar.gz 超过 2GB 但未分片，跳过"
        fi
    else
        # 小于 2GB，直接添加
        files_to_upload+=("${RELEASE_DIR}/assets-${VERSION}.tar.gz")
        file_sizes["${RELEASE_DIR}/assets-${VERSION}.tar.gz"]=$size
    fi
elif [ -f "${RELEASE_DIR}/assets-${VERSION}.tar.part000" ]; then
    # 原文件不存在但分片文件存在（原文件已被分片后删除）
    for part_file in "${RELEASE_DIR}/assets-${VERSION}.tar.part"* "${RELEASE_DIR}/assets-${VERSION}.tar.merge.sh" "${RELEASE_DIR}/assets-${VERSION}.tar.sha256"; do
        if [ -f "$part_file" ]; then
            files_to_upload+=("$part_file")
            part_size=$(stat -c%s "$part_file" 2>/dev/null || stat -f%z "$part_file" 2>/dev/null || echo 0)
            file_sizes["$part_file"]=$part_size
        fi
    done
fi

# 地图包
for file in "${RELEASE_DIR}"/*-${VERSION}.tar.gz; do
    if [ -f "$file" ] && [[ "$file" != *"base-${VERSION}.tar.gz" ]] && [[ "$file" != *"shared-${VERSION}.tar.gz" ]] && [[ "$file" != *"assets-${VERSION}.tar.gz" ]] && [[ "$file" != *"lfs-files-${VERSION}.tar.gz" ]]; then
        filename=$(basename "$file")
        base_name="${filename%.tar.gz}"
        size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null || echo 0)

        # 如果文件超过2GB，检查是否有分片文件
        if [ "$size" -gt "$MAX_SIZE" ] && { [ -f "${RELEASE_DIR}/${base_name}.part000" ] || [ -f "${RELEASE_DIR}/${base_name}.tar.part000" ]; }; then
            # 已分割，添加分片文件而不是原始文件
            # 尝试两种命名模式
            if [ -f "${RELEASE_DIR}/${base_name}.part000" ]; then
                pattern="${RELEASE_DIR}/${base_name}"
            else
                pattern="${RELEASE_DIR}/${base_name}.tar"
            fi

            for part_file in "${pattern}.part"* "${pattern}.merge.sh" "${pattern}.sha256"; do
                if [ -f "$part_file" ]; then
                    files_to_upload+=("$part_file")
                    part_size=$(stat -c%s "$part_file" 2>/dev/null || stat -f%z "$part_file" 2>/dev/null || echo 0)
                    file_sizes["$part_file"]=$part_size
                fi
            done
        else
            # 未分割或小于2GB，添加原始文件
            files_to_upload+=("$file")
            file_sizes["$file"]=$size
        fi
    fi
done

# 清单文件
if [ -f "${RELEASE_DIR}/manifest-${VERSION}.json" ]; then
    files_to_upload+=("${RELEASE_DIR}/manifest-${VERSION}.json")
    size=$(stat -c%s "${RELEASE_DIR}/manifest-${VERSION}.json" 2>/dev/null || stat -f%z "${RELEASE_DIR}/manifest-${VERSION}.json" 2>/dev/null || echo 0)
    file_sizes["${RELEASE_DIR}/manifest-${VERSION}.json"]=$size
fi

total_files=${#files_to_upload[@]}
total_size=0
for file in "${files_to_upload[@]}"; do
    total_size=$((total_size + ${file_sizes["$file"]:-0}))
done
total_size_gb=$(echo "scale=2; $total_size / 1024 / 1024 / 1024" | bc)

log "✓ 找到 ${total_files} 个文件需要上传，总大小: ${total_size_gb}GB"

# 检查 Release 是否存在，如果存在则检查已上传的文件
uploaded_files_info=""
if gh release view "v${VERSION}" --repo "$REPO" &>/dev/null; then
    log "检查已上传的文件..."
    uploaded_files_info=$(gh release view "v${VERSION}" --repo "$REPO" --json assets --jq '.assets[] | "\(.name)|\(.size)"' 2>/dev/null || echo "")
    if [ -n "$uploaded_files_info" ]; then
        uploaded_count=$(echo "$uploaded_files_info" | wc -l)
        log "✓ Release 已存在，已上传 ${uploaded_count} 个文件"
    fi
fi

log_section "[2] 上传文件到 GitHub Release v${VERSION}"

# 检查 Release 是否存在，不存在则创建
if ! gh release view "v${VERSION}" --repo "$REPO" &>/dev/null; then
    log "创建 Release v${VERSION}..."
    # 确保基于 upstream/main 分支的最新提交创建 Release
    # 先获取 upstream/main 的最新提交 SHA
    log "获取 upstream/main 分支的最新提交..."
    git fetch upstream main 2>/dev/null || log "⚠️  无法获取 upstream/main，将使用本地 main 分支"

    # 尝试使用 upstream/main，如果不存在则使用 main
    if git rev-parse --verify upstream/main >/dev/null 2>&1; then
        TARGET_COMMIT=$(git rev-parse upstream/main)
        TARGET_BRANCH="upstream/main"
        log "使用 ${TARGET_BRANCH} 的最新提交: ${TARGET_COMMIT:0:8}"
    else
        TARGET_COMMIT=$(git rev-parse main)
        TARGET_BRANCH="main"
        log "使用 ${TARGET_BRANCH} 的最新提交: ${TARGET_COMMIT:0:8}"
    fi

    if [ -f "${RELEASE_DIR}/README.md" ]; then
        gh release create "v${VERSION}" \
            --repo "$REPO" \
            --target "${TARGET_COMMIT}" \
            --title "MATRiX v${VERSION} - Modular Chunk Packages" \
            --notes-file "${RELEASE_DIR}/README.md" \
            --draft
    else
        gh release create "v${VERSION}" \
            --repo "$REPO" \
            --target "${TARGET_COMMIT}" \
            --title "MATRiX v${VERSION} - Modular Chunk Packages" \
            --notes "MATRiX v${VERSION} - Modular Chunk Packages" \
            --draft
    fi
    log "✓ Release 已创建（草稿状态，基于 ${TARGET_BRANCH} 的提交 ${TARGET_COMMIT:0:8}）"
    # 重新获取已上传文件信息（应该为空）
    uploaded_files_info=""
fi

# 函数：刷新已上传文件信息（包含文件名和大小）
refresh_uploaded_files() {
    uploaded_files_info=$(gh release view "v${VERSION}" --repo "$REPO" --json assets --jq '.assets[] | "\(.name)|\(.size)"' 2>/dev/null || echo "")
}

# 函数：从 manifest.json 获取文件的 SHA256
get_sha256_from_manifest() {
    local filename="$1"
    local manifest_file="${RELEASE_DIR}/manifest-${VERSION}.json"

    if [ ! -f "$manifest_file" ] || ! command -v jq &> /dev/null; then
        echo ""
        return
    fi

    # 尝试从 manifest 中获取 SHA256
    local sha256=$(jq -r --arg f "$filename" '
        .packages.base.sha256 // empty |
        if . == empty then
            .packages.shared.sha256 // empty |
            if . == empty then
                (.packages.maps[] | select(.file == $f) | .sha256) // empty
            else . end
        else . end
    ' "$manifest_file" 2>/dev/null || echo "")

    echo "$sha256"
}

# 函数：检查文件是否已上传且与本地一致
check_file_uploaded() {
    local file="$1"
    local filename=$(basename "$file")
    local local_size=${file_sizes["$file"]:-0}

    # 如果已上传文件信息为空，先刷新
    if [ -z "$uploaded_files_info" ]; then
        refresh_uploaded_files || true
    fi

    if [ -z "$uploaded_files_info" ]; then
        return 1  # 未上传
    fi

    # 检查文件名和大小是否匹配
    local found=false
    local remote_size=0
    while IFS='|' read -r name size || [ -n "$name" ]; do
        if [ "$name" == "$filename" ]; then
            found=true
            remote_size=$size
            break
        fi
    done <<< "$uploaded_files_info"

    if [ "$found" = false ]; then
        return 1  # 文件不存在
    fi

    # 大小必须匹配
    if [ "$remote_size" != "$local_size" ]; then
        return 1  # 大小不匹配
    fi

    # 如果 manifest.json 中有 SHA256，也进行校验（可选，更严格）
    # 注意：GitHub Releases API 不直接提供 SHA256，所以这里只检查大小
    # 如果需要更严格的校验，可以下载文件后计算 SHA256，但这会增加时间

    return 0  # 已上传且大小匹配
}

# 计算需要上传的文件总数（排除已上传的）
files_to_upload_count=0
for file in "${files_to_upload[@]}"; do
    if [ -f "$file" ]; then
        if ! check_file_uploaded "$file" 2>/dev/null; then
            files_to_upload_count=$((files_to_upload_count + 1))
        fi
    fi
done

if [ "$files_to_upload_count" -gt 0 ]; then
    log "需要上传 ${files_to_upload_count} 个文件"
else
    log "所有文件已上传，无需上传新文件"
fi

# 初始化上传计数器
current_upload_num=0

# 一次性上传所有文件（包括基础包、共享包、地图包、分片文件、manifest）
log_section "[3] 批量上传所有文件"
log "开始上传 ${files_to_upload_count} 个文件（包括基础包、共享包、资源文件包、地图包、分片文件）..."
echo ""

map_count=0
split_count=0
skipped_count=0
skip_count=0
base_uploaded=false
shared_uploaded=false
assets_uploaded=false

for file in "${files_to_upload[@]}"; do
    if [ ! -f "$file" ]; then
        continue
    fi

    filename=$(basename "$file")
    file_size=${file_sizes["$file"]:-0}
    file_size_mb=$((file_size / 1024 / 1024))

    # 检查是否已上传且完整
    if check_file_uploaded "$file" 2>/dev/null; then
        log "✓ 已上传且完整，跳过: $filename (${file_size_mb}MB)"
        skipped_count=$((skipped_count + 1))
        # 统计已上传的文件类型
        if [[ "$filename" == "base-${VERSION}.tar.gz" ]]; then
            base_uploaded=true
        elif [[ "$filename" == "shared-${VERSION}.tar.gz" ]] || [[ "$filename" == shared-*.tar.part* ]] || [[ "$filename" == shared-*.tar.merge.sh ]] || [[ "$filename" == shared-*.tar.sha256 ]]; then
            shared_uploaded=true
        elif [[ "$filename" == "assets-${VERSION}.tar.gz" ]] || [[ "$filename" == assets-*.tar.part* ]] || [[ "$filename" == assets-*.tar.merge.sh ]] || [[ "$filename" == assets-*.tar.sha256 ]]; then
            assets_uploaded=true
        fi
        continue
    fi

    # 检查文件大小（超过 2GB 的 tar.gz 文件应该已经被分片）
    if [ "$file_size" -gt "$MAX_SIZE" ] && [[ "$filename" == *.tar.gz ]]; then
        log "⚠️  跳过: $filename (${file_size_mb}MB, 超过 2GB 限制，应该使用分片文件)"
        continue
    fi

    # 上传文件
    current_upload_num=$((current_upload_num + 1))
    if upload_file_with_progress "$file" "$current_upload_num" "$files_to_upload_count"; then
        # 统计上传的文件类型
        if [[ "$filename" == "base-${VERSION}.tar.gz" ]]; then
            base_uploaded=true
        elif [[ "$filename" == "shared-${VERSION}.tar.gz" ]] || [[ "$filename" == shared-*.tar.part* ]] || [[ "$filename" == shared-*.tar.merge.sh ]] || [[ "$filename" == shared-*.tar.sha256 ]]; then
            shared_uploaded=true
        elif [[ "$filename" == "assets-${VERSION}.tar.gz" ]] || [[ "$filename" == assets-*.tar.part* ]] || [[ "$filename" == assets-*.tar.merge.sh ]] || [[ "$filename" == assets-*.tar.sha256 ]]; then
            assets_uploaded=true
        elif [[ "$filename" == *-${VERSION}.tar.gz ]]; then
            map_count=$((map_count + 1))
        elif [[ "$filename" == *.part* ]] || [[ "$filename" == *.merge.sh ]] || [[ "$filename" == *.sha256 ]]; then
            split_count=$((split_count + 1))
        fi
        refresh_uploaded_files  # 刷新已上传文件列表
    fi
done

echo ""
log "✓ 上传完成统计:"
if [ "$base_uploaded" = true ]; then
    log "  - 基础包: ✓ 已上传"
else
    log "  - 基础包: ⚠️  未上传"
fi
if [ "$shared_uploaded" = true ]; then
    log "  - 共享资源包: ✓ 已上传（包括分片文件）"
else
    log "  - 共享资源包: ⚠️  未上传"
fi
if [ "$assets_uploaded" = true ]; then
    log "  - 资源文件包: ✓ 已上传（包括分片文件）"
else
    log "  - 资源文件包: ⚠️  未上传"
fi
log "  - 地图包: ${map_count} 个"
log "  - 分片文件: ${split_count} 个"
log "  - 已跳过: ${skipped_count} 个（已上传且与本地一致）"

# 最终验证上传完整性
log_section "[4] 最终验证上传完整性"
log "重新获取已上传文件列表..."
refresh_uploaded_files

if [ -z "$uploaded_files_info" ]; then
    log "⚠️  无法获取已上传文件列表，跳过验证"
else
    missing_count=0
    incomplete_count=0
    uploaded_missing=0

    log "检查所有文件的完整性..."
    for file in "${files_to_upload[@]}"; do
        if [ ! -f "$file" ]; then
            continue
        fi

        filename=$(basename "$file")
        local_size=${file_sizes["$file"]:-0}

        # 检查文件是否已上传
        found=false
        remote_size=0
        while IFS='|' read -r name size; do
            if [ "$name" == "$filename" ]; then
                found=true
                remote_size=$size
                break
            fi
        done <<< "$uploaded_files_info"

        if [ "$found" == false ]; then
            log "⚠️  缺失: $filename"
            ((missing_count++))

            # 尝试上传缺失的文件
            file_size_mb=$((local_size / 1024 / 1024))
            if [ "$local_size" -gt "$MAX_SIZE" ]; then
                log "  跳过（超过 2GB 限制）"
            else
                current_upload_num=$((current_upload_num + 1))
                if upload_file_with_progress "$file" "$current_upload_num" "$files_to_upload_count"; then
                    uploaded_missing=$((uploaded_missing + 1))
                    refresh_uploaded_files  # 刷新已上传文件列表
                fi
            fi
        elif [ "$remote_size" != "$local_size" ]; then
            log "⚠️  文件大小不匹配: $filename (本地: ${local_size}, 远程: ${remote_size})"
            incomplete_count=$((incomplete_count + 1))
            # 重新上传
            file_size_mb=$((local_size / 1024 / 1024))
            current_upload_num=$((current_upload_num + 1))
            if upload_file_with_progress "$file" "$current_upload_num" "$files_to_upload_count"; then
                uploaded_missing=$((uploaded_missing + 1))
                refresh_uploaded_files  # 刷新已上传文件列表
            fi
        fi
    done
    
    if [ "$missing_count" -eq 0 ] && [ "$incomplete_count" -eq 0 ]; then
        log "✓ 所有文件已上传且完整"
    elif [ "$uploaded_missing" -gt 0 ]; then
        log "✓ 已补上传 ${uploaded_missing} 个缺失或不完整的文件"
        if [ "$uploaded_missing" -lt $((missing_count + incomplete_count)) ]; then
            log "⚠️  仍有 $((missing_count + incomplete_count - uploaded_missing)) 个文件缺失或不完整"
        fi
    else
        log "⚠️  仍有 ${missing_count} 个文件缺失，${incomplete_count} 个文件不完整"
    fi
fi

log_section "[5] 完成"
echo ""
echo "✅ 文件上传完成！"
echo ""
echo "📊 上传统计:"
total_uploaded=$(gh release view "v${VERSION}" --repo "$REPO" --json assets -q '.assets | length' 2>/dev/null || echo "0")
echo "  - 总文件数: ${total_uploaded}"
echo "  - 基础包: $(if [ -f "${RELEASE_DIR}/base-${VERSION}.tar.gz" ]; then file_size=$(stat -c%s "${RELEASE_DIR}/base-${VERSION}.tar.gz" 2>/dev/null || echo 0); if [ "$file_size" -le "$MAX_SIZE" ]; then echo "已上传"; else echo "已跳过（超过2GB）"; fi; else echo "不存在"; fi)"
echo "  - 共享资源包: $(if [ -f "${RELEASE_DIR}/shared-${VERSION}.tar.gz" ]; then file_size=$(stat -c%s "${RELEASE_DIR}/shared-${VERSION}.tar.gz" 2>/dev/null || echo 0); if [ "$file_size" -le "$MAX_SIZE" ]; then echo "已上传"; else echo "已跳过（超过2GB）"; fi; else echo "不存在"; fi)"
echo "  - 资源文件包: $(if [ "$assets_uploaded" = true ]; then echo "已上传"; else echo "未上传"; fi)"
echo "  - 地图包: ${map_count} 个已上传"
if [ "$split_count" -gt 0 ]; then
    echo "  - 分割文件: ${split_count} 个大文件已分割上传"
fi
if [ "${skip_count:-0}" -gt 0 ]; then
    echo "  - 跳过: ${skip_count} 个超过 2GB 的文件（未分割）"
fi
echo ""
if [ "${skip_count:-0}" -gt 0 ]; then
    echo "⚠️  注意: 有 ${skip_count} 个文件超过 GitHub Releases 的 2GB 限制"
    echo "   这些文件需要上传到其他存储（如 Google Drive, Baidu Netdisk）"
    echo ""
fi
# 检查 Release 是否为草稿状态
log_section "[6] 检查 Release 状态"
is_draft=$(gh release view "v${VERSION}" --repo "$REPO" --json isDraft -q '.isDraft' 2>/dev/null || echo "false")

if [ "$is_draft" == "true" ]; then
    log "Release 当前是草稿状态"
    if [ "$PUBLISH_RELEASE" = "ask" ]; then
        read -p "是否发布 Release? (从草稿状态发布) [Y/n]: " -n 1 -r
        echo
    elif [ "$PUBLISH_RELEASE" = "yes" ]; then
        REPLY="Y"
        log "MATRIX_RELEASE_PUBLISH=yes，自动发布 Release"
    else
        REPLY="N"
        log "MATRIX_RELEASE_PUBLISH=no，保持草稿状态"
    fi
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        log "发布 Release..."
        # 获取 Release 的 databaseId（REST API 需要数字 ID，不是 GraphQL ID）
        database_id=$(gh api graphql -f query='query($owner: String!, $repo: String!, $tag: String!) { repository(owner: $owner, name: $repo) { release(tagName: $tag) { databaseId } } }' -f owner="${REPO%%/*}" -f repo="${REPO##*/}" -f tag="v${VERSION}" 2>/dev/null | jq -r '.data.repository.release.databaseId' 2>/dev/null)

        if [ -z "$database_id" ] || [ "$database_id" == "null" ]; then
            log "⚠️  无法获取 Release databaseId"
            log "请检查权限或稍后手动发布"
        else
            log "Release databaseId: $database_id"
            if gh api "repos/${REPO}/releases/${database_id}" -X PATCH -f draft=false 2>/dev/null; then
                log "✓ Release 已发布！"
                echo ""
                echo "🔗 Release 链接:"
                echo "  https://github.com/${REPO}/releases/tag/v${VERSION}"
                echo ""

                # 显示 Release 信息
                log "Release 信息:"
                gh release view "v${VERSION}" --repo "$REPO" --json name,isDraft,state,url,assets --jq '{
                    name: .name,
                    draft: .isDraft,
                    state: .state,
                    url: .html_url,
                    assets: (.assets | length)
                }' 2>/dev/null || true
            else
                log "⚠️  发布失败，请检查权限或稍后手动发布"
                echo ""
                echo "🔗 Release 链接（草稿状态）:"
                echo "  https://github.com/${REPO}/releases/tag/v${VERSION}"
            fi
        fi
    else
        log "保持草稿状态"
        echo ""
        echo "🔗 Release 链接（草稿状态）:"
        echo "  https://github.com/${REPO}/releases/tag/v${VERSION}"
        echo ""
        log "稍后可以重新运行此脚本并选择发布，或使用以下命令发布:"
        log "  bash scripts/release_manager/upload_to_release.sh ${VERSION}"
    fi
else
    log "✓ Release 已经是发布状态"
    echo ""
    echo "🔗 Release 链接:"
    echo "  https://github.com/${REPO}/releases/tag/v${VERSION}"
    echo ""

    # 显示 Release 信息
    log "Release 信息:"
    gh release view "v${VERSION}" --repo "$REPO" --json name,isDraft,state,url,assets --jq '{
        name: .name,
        draft: .isDraft,
        state: .state,
        url: .html_url,
        assets: (.assets | length)
    }' 2>/dev/null || true
fi
