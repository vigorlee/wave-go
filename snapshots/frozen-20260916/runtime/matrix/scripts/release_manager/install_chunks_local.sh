#!/bin/bash
set -e

# ============================================================================
# 从本地打包文件安装Chunk包到运行目录
# 直接从 releases/chunks/VERSION 目录解压并组织文件
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 加载公共函数库
source "${SCRIPT_DIR}/common.sh"

VERSION="${1:-0.1.2}"
RELEASE_DIR="${PROJECT_ROOT}/releases"
TARGET_DIR="${PROJECT_ROOT}/src/UeSim/Linux/zsibot_mujoco_ue"
PAK_DIR="${TARGET_DIR}/Content/Paks"

if [ "${MATRIX_SKIP_ENV_CHECK:-0}" != "1" ] && [ -x "${PROJECT_ROOT}/scripts/check_env.sh" ]; then
    "${PROJECT_ROOT}/scripts/check_env.sh" local-install
fi

# 检查发布目录
if [ ! -d "$RELEASE_DIR" ]; then
    error_exit "找不到发布目录: $RELEASE_DIR"
fi

log_section "MATRiX Chunk包本地安装器 v${VERSION}"

# 确保目标目录存在
mkdir -p "$PAK_DIR"
mkdir -p "$TARGET_DIR/Content/model"

log_section "[1] 安装资源文件包 (必需)"
{
    ASSETS_FILE="${RELEASE_DIR}/assets-${VERSION}.tar.gz"
    if [ -f "$ASSETS_FILE" ]; then
        # 检查 assets 包是否为空（只有目录结构）
        file_count=$(tar -tzf "$ASSETS_FILE" 2>/dev/null | grep -v "/$" | wc -l)
        if [ "$file_count" -eq 0 ]; then
            log "⚠️  资源文件包为空（只包含目录结构，没有实际文件）"
            log "   这通常意味着原始文件已被删除，需要重新打包 assets 包"
            log "   跳过安装（如果文件已存在则不影响）"
        else
            log "解压资源文件包（包含 ${file_count} 个文件）..."
            # 资源文件包解压到项目根目录，保持目录结构
            if extract_tar "$ASSETS_FILE" "${PROJECT_ROOT}"; then
                log "✓ 资源文件包安装完成"

                # 验证关键文件是否存在
                if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ] || [ -f "${PROJECT_ROOT}/bin/sim_launcher" ]; then
                    launcher_file="${PROJECT_ROOT}/bin/sim_launcher.bin"
                    if [ ! -f "$launcher_file" ]; then launcher_file="${PROJECT_ROOT}/bin/sim_launcher"; fi

                    launcher_size=$(stat -f%z "$launcher_file" 2>/dev/null || stat -c%s "$launcher_file" 2>/dev/null || echo 0)
                    if [ "$launcher_size" -gt 1000000 ]; then
                        log "✓ 资源文件验证通过: $(basename "$launcher_file") (${launcher_size} 字节)"
                    else
                        # 如果 sim_launcher 是脚本，检查它是否调用了 .bin
                        if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ]; then
                            bin_size=$(stat -f%z "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || stat -c%s "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || echo 0)
                            log "✓ 资源文件验证通过: sim_launcher.bin (${bin_size} 字节)"
                        else
                            log "⚠️  资源文件可能未正确安装: 关键文件过小"
                        fi
                    fi
                else
                    log "⚠️  关键文件缺失: bin/sim_launcher 未找到"
                    log "   请检查 assets 包是否完整"
                fi
            else
                log "⚠️  资源文件包解压失败，跳过"
            fi
        fi
    else
        log "⚠️  资源文件包不存在，跳过（如果文件已存在则不影响）"
    fi
}

log_section "[2] 安装基础包 (必需)"
{
    BASE_FILE="${RELEASE_DIR}/base-${VERSION}.tar.gz"
    if [ ! -f "$BASE_FILE" ]; then
        error_exit "找不到基础包: $BASE_FILE (请先下载到 releases/ 目录)"
    fi

    # 使用公共函数解压
    if extract_tar "$BASE_FILE" "$TARGET_DIR"; then
        # 使用公共函数移动 chunk 文件
        move_chunk_files_to_paks "${TARGET_DIR}/Content/Paks" "$PAK_DIR"

        # 从 UeSim 目录拷贝模型到 robot_mujoco 目录
        copy_models_from_uesim_to_robot_mujoco
    else
        error_exit "基础包解压失败"
    fi

    log "✓ 基础包安装完成"
}

log_section "[3] 安装共享资源包 (推荐)"
{
    SHARED_FILE="${RELEASE_DIR}/shared-${VERSION}.tar.gz"
    if [ -f "$SHARED_FILE" ]; then
        log "解压共享资源包..."
        tar -xzf "$SHARED_FILE" -C "$PAK_DIR"
        log "✓ 共享资源包安装完成"
    else
        log "⚠️  共享资源包不存在，跳过"
    fi
}

log_section "[4] 安装地图包 (可选)"
{
    # 检查并合并分片文件
    for merge_script in "${RELEASE_DIR}"/*.merge.sh; do
        if [ -f "$merge_script" ]; then
            # 获取目标文件名（从 merge script 文件名推断，去掉 .merge.sh 后缀）
            target_file="${merge_script%.merge.sh}"
            # 如果合并后的文件不存在，则执行合并
            if [ ! -f "$target_file" ]; then
                log "发现分片文件，正在合并: $(basename "$target_file")"
                # 检查合并脚本是否有效（不是 "Not Found" 或空文件）
                if [ ! -s "$merge_script" ] || head -1 "$merge_script" | grep -q "Not Found"; then
                    log "⚠️  合并脚本损坏或无效，跳过: $(basename "$merge_script")"
                    log "   提示: 如果完整文件已存在，可以删除分片文件；否则需要重新下载"
                    continue
                fi
                # 赋予执行权限并运行
                chmod +x "$merge_script"
                # 在 RELEASE_DIR 中执行，确保路径正确
                (cd "$RELEASE_DIR" && ./$(basename "$merge_script")) || log "⚠️  合并失败: $(basename "$merge_script")"
            else
                log "分片文件已合并: $(basename "$target_file")"
            fi
        fi
    done

    map_count=0
    map_list=()
    total_maps=$(ls -1 "${RELEASE_DIR}"/*-${VERSION}.tar.gz 2>/dev/null | grep -vE "(base|shared)-" | wc -l)
    current_map=0

    for map_tar in "${RELEASE_DIR}"/*-${VERSION}.tar.gz; do
        # 跳过 base、shared 和 assets
        if [[ "$(basename "$map_tar")" == base-* ]] || [[ "$(basename "$map_tar")" == shared-* ]] || [[ "$(basename "$map_tar")" == assets-* ]]; then
            continue
        fi
        if [ -f "$map_tar" ]; then
            map_name=$(basename "$map_tar" | sed "s/-${VERSION}.tar.gz//")
            ((++current_map))
            map_size=$(du -h "$map_tar" 2>/dev/null | cut -f1)
            log "安装地图包 [${current_map}/${total_maps}]: $map_name (${map_size})"

            # 使用 pv 显示进度（如果可用），否则使用 tar -v 显示解压的文件名
            if command -v pv &> /dev/null; then
                log "  正在解压..."
                if pv -p -t -e -r -b "$map_tar" | tar -xz -C "$PAK_DIR" 2>/dev/null; then
                    map_list+=("$map_name")
                    ((++map_count))
                    log "  ✓ ${map_name} 安装完成"
                else
                    log "  ⚠️  ${map_name} 解压失败，可能文件损坏"
                fi
            else
                log "  正在解压（大文件可能需要几分钟，请稍候）..."
                # 使用后台进程显示进度提示
                (
                    while true; do
                        sleep 3
                        printf "." >&2
                    done
                ) &
                progress_pid=$!

                # 执行解压
                if tar -xzf "$map_tar" -C "$PAK_DIR" 2>/dev/null; then
                    kill $progress_pid 2>/dev/null || true
                    wait $progress_pid 2>/dev/null || true
                    printf "\n" >&2
                    map_list+=("$map_name")
                    ((++map_count))
                    log "  ✓ ${map_name} 安装完成"
                else
                    kill $progress_pid 2>/dev/null || true
                    wait $progress_pid 2>/dev/null || true
                    printf "\n" >&2
                    log "  ⚠️  ${map_name} 解压失败，可能文件损坏"
                fi
            fi
        fi
    done
    if [ $map_count -gt 0 ]; then
        log "✓ 已安装 ${map_count} 个地图包: ${map_list[*]}"
    else
        log "⚠️  未找到地图包，跳过"
    fi
}

log_section "[5] 验证安装"
{
    # 使用公共函数验证安装
    verify_installation "$PAK_DIR"

    # 验证资源文件是否已安装
    if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ] || [ -f "${PROJECT_ROOT}/bin/sim_launcher" ]; then
        launcher_file="${PROJECT_ROOT}/bin/sim_launcher.bin"
        if [ ! -f "$launcher_file" ]; then launcher_file="${PROJECT_ROOT}/bin/sim_launcher"; fi

        launcher_size=$(stat -f%z "$launcher_file" 2>/dev/null || stat -c%s "$launcher_file" 2>/dev/null || echo 0)
        if [ "$launcher_size" -gt 1000000 ]; then
            log "✓ 资源文件验证通过: $(basename "$launcher_file") (${launcher_size} 字节)"
        else
            # 如果 sim_launcher 是脚本，检查它是否调用了 .bin
            if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ]; then
                bin_size=$(stat -f%z "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || stat -c%s "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || echo 0)
                log "✓ 资源文件验证通过: sim_launcher.bin (${bin_size} 字节)"
            else
                log "⚠️  资源文件可能未正确安装: 关键文件过小"
            fi
        fi
    fi
}

log_section "[6] 完成"
{
    echo ""
    echo "✅ Chunk包安装完成！"
    echo ""
    echo "已安装的包:"
    if [ -f "${PROJECT_ROOT}/bin/sim_launcher" ]; then
        echo "  - 资源文件包"
    fi
    echo "  - 基础包 (Chunk 0)"
    if [ -f "${PAK_DIR}/pakchunk1-Linux.pak" ]; then
        echo "  - 共享资源包 (Chunk 1)"
    fi
    map_count=$(ls -1 "${PAK_DIR}"/pakchunk[1-9][0-9]*-Linux.pak 2>/dev/null | wc -l)
    echo "  - 地图包: ${map_count} 个"
    echo ""
    echo "运行目录: ${TARGET_DIR}"
    echo ""
    echo "已安装的地图包列表:"

    for pak_file in "${PAK_DIR}"/pakchunk[1-9][0-9]*-Linux.pak; do
        if [ -f "$pak_file" ]; then
            chunk_id=$(basename "$pak_file" | sed 's/pakchunk\([0-9]*\)-Linux.pak/\1/')

            # 首先使用公共函数获取地图名
            map_name=$(get_map_name_by_chunk_id "$chunk_id")

            # 如果公共函数返回未知地图，尝试通过 manifest 文件查找 (如果存在 jq)
            if [ "$map_name" == "(未知地图)" ] && command -v jq &> /dev/null && [ -f "${RELEASE_DIR}/manifest-${VERSION}.json" ]; then
                # 从 manifest 中查找对应的地图名（通过文件名匹配）
                map_name=$(jq -r ".packages.maps[] | select(.file | contains(\"pakchunk${chunk_id}\")) | .name" "${RELEASE_DIR}/manifest-${VERSION}.json" 2>/dev/null)
                # 如果没找到，尝试通过 tar.gz 文件名匹配
                if [ -z "$map_name" ] || [ "$map_name" == "null" ]; then
                    # 查找包含该 chunk 的地图包文件名
                    for map_tar in "${RELEASE_DIR}"/*-${VERSION}.tar.gz; do
                        if [ -f "$map_tar" ]; then
                            tar_name=$(basename "$map_tar" | sed "s/-${VERSION}.tar.gz//")
                            if [ "$tar_name" != "base" ] && [ "$tar_name" != "shared" ]; then
                                # 检查这个 tar 文件是否包含该 chunk
                                if tar -tzf "$map_tar" 2>/dev/null | grep -q "pakchunk${chunk_id}"; then
                                    map_name="$tar_name"
                                    break
                                fi
                            fi
                        fi
                    done
                fi
            fi

            if [ -z "$map_name" ] || [ "$map_name" == "null" ]; then
                map_name="(未知地图)"
            fi

            echo "  - Chunk ${chunk_id} (${map_name})"
        fi
    done
    # 检查是否有地图包文件（使用 ls 避免 glob 模式在 test 命令中的问题）
    if ! ls "${PAK_DIR}"/pakchunk[1-9][0-9]*-Linux.pak 1>/dev/null 2>&1; then
        echo "  (无)"
    fi
    echo ""
    echo "现在可以运行模拟器了:"
    echo "  cd ${PROJECT_ROOT}"
    echo "  ./bin/sim_launcher 1 0  # XGB机器人，CustomWorld地图"
    echo "  ./bin/sim_launcher 1 1  # XGB机器人，Warehouse地图"
    echo ""
    echo "提示: 如果需要安装更多地图包，可以:"
    echo "  1. 将地图包文件放到 releases/ 目录"
    echo "  2. 重新运行此脚本: bash scripts/release_manager/install_chunks_local.sh ${VERSION}"
}
