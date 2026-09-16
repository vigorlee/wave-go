#!/bin/bash
set -e

# ============================================================================
# 打包 LFS/Assets 文件
# 收集所有运行时必需的大文件（可执行文件、共享库、ONNX模型等）
# 排除：
#   - demo_gif（保留在 Git 仓库）
#   - zsibot_mujoco_ue/Binaries/（会从 base 包中获取）
#   - robot_mujoco/zsibot_robots/（会从 base 包中的 UeSim 目录拷贝）
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

VERSION="${1:-0.1.1}"
RELEASE_DIR="${PROJECT_ROOT}/releases"
PACKAGE_NAME="assets-${VERSION}.tar.gz"
PACKAGE_PATH="${RELEASE_DIR}/${PACKAGE_NAME}"
TEMP_DIR="${RELEASE_DIR}/.temp_assets_${VERSION}"

log_section "打包 Assets 文件 (版本: ${VERSION})"

# 检查版本参数
if [ -z "$1" ]; then
    log "⚠️  未指定版本号，使用默认版本: ${VERSION}"
fi

# 创建临时目录
mkdir -p "${TEMP_DIR}"
mkdir -p "${RELEASE_DIR}"

# 清理临时目录
rm -rf "${TEMP_DIR}"/* 2>/dev/null || true

log_section "[1] 收集需要打包的文件"

file_count=0
total_size=0

# 1. bin/ 目录 - 可执行文件
if [ -d "${PROJECT_ROOT}/bin" ]; then
    log "收集 bin/ 目录..."
    mkdir -p "${TEMP_DIR}/bin"
    if [ -f "${PROJECT_ROOT}/bin/sim_launcher" ]; then
        cp "${PROJECT_ROOT}/bin/sim_launcher" "${TEMP_DIR}/bin/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${PROJECT_ROOT}/bin/sim_launcher" 2>/dev/null || stat -f%z "${PROJECT_ROOT}/bin/sim_launcher" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ bin/sim_launcher ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi
    if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ]; then
        cp "${PROJECT_ROOT}/bin/sim_launcher.bin" "${TEMP_DIR}/bin/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || stat -f%z "${PROJECT_ROOT}/bin/sim_launcher.bin" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ bin/sim_launcher.bin ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi
fi

# 2. src/robot_mc/build/export/onnx_model_crypto/ - ONNX 模型（只发布 run_sim.sh 1-3 需要的控制模型）
if [ -d "${PROJECT_ROOT}/src/robot_mc/build/export/onnx_model_crypto" ]; then
    log "收集 ONNX 模型文件（仅 xg、xg_wheel、zg_wheels）..."
    mkdir -p "${TEMP_DIR}/src/robot_mc/build/export/onnx_model_crypto"
    
    # 遍历所有目录，只收集本次发布需要的控制模型
    for model_dir in "${PROJECT_ROOT}/src/robot_mc/build/export/onnx_model_crypto"/*; do
        if [ -d "$model_dir" ]; then
            model_name=$(basename "$model_dir")
            if [ "$model_name" = "xg" ] || [ "$model_name" = "xg_wheel" ] || [ "$model_name" = "zg_wheels" ]; then
                log "  收集模型: $model_name"
                cp -r "$model_dir" "${TEMP_DIR}/src/robot_mc/build/export/onnx_model_crypto/"
                # 统计文件数
                count=$(find "$model_dir" -type f | wc -l)
                size=$(du -sb "$model_dir" 2>/dev/null | cut -f1 || echo 0)
                file_count=$((file_count + count))
                total_size=$((total_size + size))
                log "    ✓ $model_name: ${count} 个文件 ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
            else
                log "  跳过模型: $model_name (未发布)"
            fi
        fi
    done
fi

# 2b. src/robot_mc/build/export/mc/bin/ - MC 运行时文件
if [ -d "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" ]; then
    log "收集 MC 运行时文件..."
    mkdir -p "${TEMP_DIR}/src/robot_mc/build/export/mc/bin"
    cp -a "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin/." "${TEMP_DIR}/src/robot_mc/build/export/mc/bin/"

    count=$(find "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" -maxdepth 1 -type f | wc -l)
    size=$(du -sb "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" 2>/dev/null | cut -f1 || echo 0)
    file_count=$((file_count + count))
    total_size=$((total_size + size))
    log "  ✓ mc/bin: ${count} 个文件 ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
fi

# 3. src/UeSim/Linux/Engine/ - UE 引擎共享库（OpenCV, ONNX Runtime 等）
# 注意：zsibot_mujoco_ue/Binaries/ 已排除（会从 base 包中获取）
if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine" ]; then
    log "收集 UE 引擎共享库（Engine 目录）..."
    mkdir -p "${TEMP_DIR}/src/UeSim/Linux/Engine"
    
    # 复制 Binaries 目录
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Binaries" ]; then
        mkdir -p "${TEMP_DIR}/src/UeSim/Linux/Engine/Binaries"
        cp -r "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Binaries"/* "${TEMP_DIR}/src/UeSim/Linux/Engine/Binaries/" 2>/dev/null || true
    fi
    
    # 复制 Plugins 目录（复制 Binaries 子目录和 .uplugin 描述文件）
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins" ]; then
        mkdir -p "${TEMP_DIR}/src/UeSim/Linux/Engine/Plugins"
        # 复制 Plugins 下的 Binaries 目录
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins" -type d -name "Binaries" | while IFS= read -r bin_dir; do
            rel_path="${bin_dir#${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins/}"
            target_dir="${TEMP_DIR}/src/UeSim/Linux/Engine/Plugins/${rel_path}"
            mkdir -p "$(dirname "$target_dir")"
            cp -r "$bin_dir" "$target_dir" 2>/dev/null || true
        done
        # 复制插件描述文件，避免下次重打包时丢失 .uplugin
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins" -type f -name "*.uplugin" | while IFS= read -r plugin_file; do
            rel_path="${plugin_file#${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins/}"
            target_file="${TEMP_DIR}/src/UeSim/Linux/Engine/Plugins/${rel_path}"
            mkdir -p "$(dirname "$target_file")"
            cp "$plugin_file" "$target_file" 2>/dev/null || true
        done
    fi
    
    # 复制 Content 目录下的二进制文件（如 TessellationTable.bin）
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content" ]; then
        mkdir -p "${TEMP_DIR}/src/UeSim/Linux/Engine/Content"
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content" -type f -name "*.bin" | while IFS= read -r bin_file; do
            if [ -f "$bin_file" ]; then
                rel_path="${bin_file#${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content/}"
                target_file="${TEMP_DIR}/src/UeSim/Linux/Engine/Content/${rel_path}"
                target_dir=$(dirname "$target_file")
                mkdir -p "$target_dir"
                cp "$bin_file" "$target_file" 2>/dev/null || true
            fi
        done
    fi
    
    # 统计文件（只统计实际复制的文件）
    count=$(find "${TEMP_DIR}/src/UeSim/Linux/Engine" -type f | wc -l)
    size=$(du -sb "${TEMP_DIR}/src/UeSim/Linux/Engine" 2>/dev/null | cut -f1 || echo 0)
    file_count=$((file_count + count))
    total_size=$((total_size + size))
    log "  ✓ UE Engine: ${count} 个文件 ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
fi

# 3b. UE OpenCV runtime library - 需要随 assets 一起打包
# UE 二进制在启动时会从 RPATH 中解析 libopencv_world.so.405。
# 该库通常来自 UnrealEngine 安装目录，而不是项目仓库本身。
UE_OCV_SRC_DIR="${UE_OCV_SRC_DIR:-/home/user/software/UnrealEngine/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux}"
UE_OCV_DST_DIR="${TEMP_DIR}/src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux"
if [ -d "${UE_OCV_SRC_DIR}" ]; then
    log "收集 UE OpenCV 运行库..."
    mkdir -p "${UE_OCV_DST_DIR}"

    if [ -f "${UE_OCV_SRC_DIR}/libopencv_world.so.405" ]; then
        cp -a "${UE_OCV_SRC_DIR}/libopencv_world.so.405" "${UE_OCV_DST_DIR}/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${UE_OCV_SRC_DIR}/libopencv_world.so.405" 2>/dev/null || stat -f%z "${UE_OCV_SRC_DIR}/libopencv_world.so.405" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ libopencv_world.so.405 ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi

    if [ -f "${UE_OCV_SRC_DIR}/libopencv_world.so" ]; then
        cp -a "${UE_OCV_SRC_DIR}/libopencv_world.so" "${UE_OCV_DST_DIR}/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${UE_OCV_SRC_DIR}/libopencv_world.so" 2>/dev/null || stat -f%z "${UE_OCV_SRC_DIR}/libopencv_world.so" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ libopencv_world.so ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi
else
    log "⚠️  UE OpenCV 源目录不存在，跳过: ${UE_OCV_SRC_DIR}"
fi

# 4. src/robot_mujoco/simulate/build/ - MuJoCo 可执行文件和动态地图
if [ -d "${PROJECT_ROOT}/src/robot_mujoco/simulate/build" ]; then
    log "收集 MuJoCo 可执行文件和动态地图..."
    mkdir -p "${TEMP_DIR}/src/robot_mujoco/simulate/build"
    
    # 只复制可执行文件和动态地图数据
    MUJOCO_BIN=""
    if [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/robot_mujoco" ]; then
        MUJOCO_BIN="${PROJECT_ROOT}/src/robot_mujoco/simulate/build/robot_mujoco"
    elif [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/zsibot_mujoco" ]; then
        MUJOCO_BIN="${PROJECT_ROOT}/src/robot_mujoco/simulate/build/zsibot_mujoco"
    fi

    if [ -n "${MUJOCO_BIN}" ]; then
        cp "${MUJOCO_BIN}" "${TEMP_DIR}/src/robot_mujoco/simulate/build/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${MUJOCO_BIN}" 2>/dev/null || stat -f%z "${MUJOCO_BIN}" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ $(basename "${MUJOCO_BIN}") ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi

    if [ -e "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/libstdc++.so.6" ]; then
        cp -a "${PROJECT_ROOT}/src/robot_mujoco/simulate/build"/libstdc++.so.6* "${TEMP_DIR}/src/robot_mujoco/simulate/build/"
        while IFS= read -r lib_file; do
            file_count=$((file_count + 1))
            if [ -L "$lib_file" ]; then
                log "  ✓ $(basename "$lib_file") -> $(readlink "$lib_file")"
                continue
            fi
            size=$(stat -c%s "$lib_file" 2>/dev/null || stat -f%z "$lib_file" 2>/dev/null || echo 0)
            total_size=$((total_size + size))
            log "  ✓ $(basename "$lib_file") ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
        done < <(find "${TEMP_DIR}/src/robot_mujoco/simulate/build" -maxdepth 1 -name 'libstdc++.so.6*' | sort)
    fi
    
    if [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin" ]; then
        cp "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin" "${TEMP_DIR}/src/robot_mujoco/simulate/build/"
        file_count=$((file_count + 1))
        size=$(stat -c%s "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin" 2>/dev/null || stat -f%z "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin" 2>/dev/null || echo 0)
        total_size=$((total_size + size))
        log "  ✓ DynamicMapData.bin ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
    fi
fi

# 5. dynamicmaps/ 目录 - 动态地图二进制文件
if [ -d "${PROJECT_ROOT}/dynamicmaps" ]; then
    log "收集动态地图文件..."
    mkdir -p "${TEMP_DIR}/dynamicmaps"
    cp -r "${PROJECT_ROOT}/dynamicmaps"/* "${TEMP_DIR}/dynamicmaps/" 2>/dev/null || true
    
    count=$(find "${PROJECT_ROOT}/dynamicmaps" -type f | wc -l)
    size=$(du -sb "${PROJECT_ROOT}/dynamicmaps" 2>/dev/null | cut -f1 || echo 0)
    file_count=$((file_count + count))
    total_size=$((total_size + size))
    log "  ✓ dynamicmaps: ${count} 个文件 ($(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "${size} bytes"))"
fi

log ""
log "✓ 共收集 ${file_count} 个文件，总大小: $(numfmt --to=iec-i --suffix=B $total_size 2>/dev/null || echo "${total_size} bytes")"

# 检查是否收集到文件
if [ $file_count -eq 0 ]; then
    error_exit "未找到需要打包的文件！请检查项目目录结构。"
fi

log_section "[2] 创建压缩包"

cd "${TEMP_DIR}"
log "正在压缩..."
if tar -czf "${PACKAGE_PATH}" . 2>/dev/null; then
    PACKAGE_SIZE=$(stat -c%s "${PACKAGE_PATH}" 2>/dev/null || stat -f%z "${PACKAGE_PATH}" 2>/dev/null || echo 0)
    log "✓ 压缩包已创建: ${PACKAGE_NAME}"
    log "  压缩后大小: $(numfmt --to=iec-i --suffix=B $PACKAGE_SIZE 2>/dev/null || echo "${PACKAGE_SIZE} bytes")"
    log "  压缩率: $(awk "BEGIN {printf \"%.1f%%\", (1 - $PACKAGE_SIZE / $total_size) * 100}" 2>/dev/null || echo "N/A")"
else
    error_exit "压缩失败！"
fi

log_section "[3] 计算 SHA256 校验和"

SHA256=$(sha256sum "${PACKAGE_PATH}" | cut -d' ' -f1)
echo "$SHA256" > "${RELEASE_DIR}/.assets_sha256_${VERSION}.txt"
log "✓ SHA256: ${SHA256}"

log_section "[4] 删除原始文件"

deleted_count=0

# 1. 删除 bin/sim_launcher
if [ -f "${PROJECT_ROOT}/bin/sim_launcher" ]; then
    rm -f "${PROJECT_ROOT}/bin/sim_launcher"
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 bin/sim_launcher"
fi

if [ -f "${PROJECT_ROOT}/bin/sim_launcher.bin" ]; then
    rm -f "${PROJECT_ROOT}/bin/sim_launcher.bin"
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 bin/sim_launcher.bin"
fi

# 2. 删除已转入 assets 包的 ONNX 模型文件
if [ -d "${PROJECT_ROOT}/src/robot_mc/build/export/onnx_model_crypto" ]; then
    for model_dir in "${PROJECT_ROOT}/src/robot_mc/build/export/onnx_model_crypto"/*; do
        if [ -d "$model_dir" ]; then
            model_name=$(basename "$model_dir")
            if [ "$model_name" = "xg" ] || [ "$model_name" = "xg_wheel" ] || [ "$model_name" = "zg_wheels" ]; then
                rm -rf "$model_dir"
                deleted_count=$((deleted_count + 1))
                log "  ✓ 已删除 ONNX 模型目录: $model_name"
            fi
        fi
    done
fi

# 2b. 删除 MC 运行时文件
if [ -d "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" ]; then
    find "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" -maxdepth 1 -type f -delete 2>/dev/null || true
    count=$(find "${PROJECT_ROOT}/src/robot_mc/build/export/mc/bin" -maxdepth 1 -type f 2>/dev/null | wc -l)
    if [ "$count" -eq 0 ]; then
        deleted_count=$((deleted_count + 1))
        log "  ✓ 已删除 mc/bin 目录下的文件"
    fi
fi

# 3. 删除 UE Engine 目录下的文件
if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine" ]; then
    # 删除 Binaries 目录下的所有文件
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Binaries" ]; then
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Binaries" -type f -delete 2>/dev/null || true
        # 删除空目录
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Binaries" -type d -empty -delete 2>/dev/null || true
    fi
    
    # 删除 Plugins 目录下 Binaries 子目录中的文件
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins" ]; then
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Plugins" -type d -name "Binaries" -exec rm -rf {} \; 2>/dev/null || true
    fi
    
    # 删除 Content 目录下的 .bin 文件
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content" ]; then
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content" -type f -name "*.bin" -delete 2>/dev/null || true
        # 删除空目录
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Content" -type d -empty -delete 2>/dev/null || true
    fi
    
    # 删除 Config 目录下的文件（如果有）
    if [ -d "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Config" ]; then
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Config" -type f -delete 2>/dev/null || true
        find "${PROJECT_ROOT}/src/UeSim/Linux/Engine/Config" -type d -empty -delete 2>/dev/null || true
    fi
    
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 UE Engine 目录下的文件"
fi

# 4. 删除 MuJoCo 可执行文件和动态地图
if [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/robot_mujoco" ]; then
    rm -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/robot_mujoco"
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 robot_mujoco"
fi

if [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/zsibot_mujoco" ]; then
    rm -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/zsibot_mujoco"
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 zsibot_mujoco"
fi

if [ -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin" ]; then
    rm -f "${PROJECT_ROOT}/src/robot_mujoco/simulate/build/DynamicMapData.bin"
    deleted_count=$((deleted_count + 1))
    log "  ✓ 已删除 DynamicMapData.bin"
fi

# 5. 删除 zsibot_robots 目录下的文件（不打包，但需要删除，因为会从 base 包拷贝）
if [ -d "${PROJECT_ROOT}/src/robot_mujoco/zsibot_robots" ]; then
    find "${PROJECT_ROOT}/src/robot_mujoco/zsibot_robots" -type f ! -name ".gitkeep" -delete 2>/dev/null || true
    # 删除空目录（保留 zsibot_robots 目录本身）
    find "${PROJECT_ROOT}/src/robot_mujoco/zsibot_robots" -type d -empty -delete 2>/dev/null || true
    count=$(find "${PROJECT_ROOT}/src/robot_mujoco/zsibot_robots" -type f ! -name ".gitkeep" 2>/dev/null | wc -l)
    if [ "$count" -eq 0 ]; then
        deleted_count=$((deleted_count + 1))
        log "  ✓ 已删除 zsibot_robots 目录下的文件（会从 base 包拷贝）"
    fi
fi

# 6. 删除 dynamicmaps 目录下的文件
if [ -d "${PROJECT_ROOT}/dynamicmaps" ]; then
    find "${PROJECT_ROOT}/dynamicmaps" -type f ! -name ".gitkeep" -delete 2>/dev/null || true
    count=$(find "${PROJECT_ROOT}/dynamicmaps" -type f ! -name ".gitkeep" 2>/dev/null | wc -l)
    if [ "$count" -eq 0 ]; then
        deleted_count=$((deleted_count + 1))
        log "  ✓ 已删除 dynamicmaps 目录下的文件"
    fi
fi

log "✓ 共删除 ${deleted_count} 个文件/目录"

log_section "[5] 更新 manifest 文件"

MANIFEST_FILE="${RELEASE_DIR}/manifest-${VERSION}.json"
if [ -f "$MANIFEST_FILE" ]; then
    log "更新 manifest 文件中的 assets 包信息..."
    
    # 检查是否有 jq 工具
    if command -v jq &> /dev/null; then
        # 使用 jq 更新 manifest 文件
        # 如果 assets 包已存在，更新它；如果不存在，添加它
        if jq -e '.packages.assets' "$MANIFEST_FILE" >/dev/null 2>&1; then
            # assets 包已存在，更新它
            jq --arg file "${PACKAGE_NAME}" \
               --argjson size "$PACKAGE_SIZE" \
               --arg sha256 "$SHA256" \
               '.packages.assets.file = $file | 
                .packages.assets.size = $size | 
                .packages.assets.sha256 = $sha256 |
                .packages.assets.required = true |
                .packages.assets.description = "资源文件包 - 包含运行时必需的文件（可执行文件、共享库、3D模型等）"' \
               "$MANIFEST_FILE" > "${MANIFEST_FILE}.tmp" && \
            mv "${MANIFEST_FILE}.tmp" "$MANIFEST_FILE"
            log "✓ 已更新 manifest 文件中的 assets 包信息"
        else
            # assets 包不存在，添加它（在 shared 之后，maps 之前）
            jq --arg file "${PACKAGE_NAME}" \
               --argjson size "$PACKAGE_SIZE" \
               --arg sha256 "$SHA256" \
               '.packages.assets = {
                 file: $file,
                 size: $size,
                 sha256: $sha256,
                 required: true,
                 description: "资源文件包 - 包含运行时必需的文件（可执行文件、共享库、3D模型等）"
               }' \
               "$MANIFEST_FILE" > "${MANIFEST_FILE}.tmp" && \
            mv "${MANIFEST_FILE}.tmp" "$MANIFEST_FILE"
            log "✓ 已在 manifest 文件中添加 assets 包信息"
        fi
        
        # 验证更新后的 JSON 格式
        if jq empty "$MANIFEST_FILE" 2>/dev/null; then
            log "✓ manifest 文件格式验证通过"
        else
            log "⚠️  manifest 文件格式验证失败，请手动检查"
        fi
    else
        log "⚠️  jq 未安装，无法自动更新 manifest 文件"
        log "   请手动更新 ${MANIFEST_FILE} 中的 assets 包信息："
        log "   - file: ${PACKAGE_NAME}"
        log "   - size: ${PACKAGE_SIZE}"
        log "   - sha256: ${SHA256}"
        log "   - required: true"
    fi
else
    log "⚠️  manifest 文件不存在: ${MANIFEST_FILE}"
    log "   跳过 manifest 更新（manifest 文件通常由 package_chunks_for_release.sh 生成）"
fi

log_section "[6] 清理临时文件"

rm -rf "${TEMP_DIR}"
log "✓ 临时文件已清理"

log ""
log "=========================================="
log "✓ Assets 文件打包完成！"
log "=========================================="
log "文件: ${PACKAGE_PATH}"
log "大小: $(numfmt --to=iec-i --suffix=B $PACKAGE_SIZE 2>/dev/null || echo "${PACKAGE_SIZE} bytes")"
log "SHA256: ${SHA256}"
log ""
log "注意："
log "  - demo_gif 已排除（保留在 Git 仓库中）"
log "  - 仅发布 xg、xg_wheel、zg_wheels ONNX 模型"
log "  - zsibot_mujoco_ue/Binaries/ 已排除（会从 base 包中获取）"
log "  - robot_mujoco/zsibot_robots/ 已排除（会从 base 包中的 UeSim 目录拷贝）"
if [ -f "$MANIFEST_FILE" ] && command -v jq &> /dev/null; then
    log "  - manifest 文件已更新"
fi
log ""
