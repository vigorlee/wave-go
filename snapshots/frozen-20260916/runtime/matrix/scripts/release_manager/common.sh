#!/bin/bash
# ============================================================================
# 公共函数库 - Release Manager Scripts
# 所有 release_manager 脚本共享的公共函数和变量
# ============================================================================

# 获取脚本目录和项目根目录
# 注意：如果脚本已经设置了这些变量，则不会覆盖
if [ -z "${SCRIPT_DIR:-}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

if [ -z "${PROJECT_ROOT:-}" ]; then
    # release_manager 目录下的脚本需要向上两级到达项目根目录
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

# 日志函数
log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# 日志章节函数
log_section() {
    echo ""
    echo "===== $* ====="
    echo "$(printf '=%.0s' {1..60})"
}

# 错误退出函数
error_exit() {
    log "ERROR: $*"
    exit 1
}

# 解压 tar.gz 文件到指定目录
# 用法: extract_tar <tar_file> <extract_dir>
extract_tar() {
    local tar_file="$1"
    local extract_dir="$2"
    
    if [ ! -f "$tar_file" ]; then
        error_exit "文件不存在: $tar_file"
    fi
    
    log "解压: $(basename "$tar_file")"
    mkdir -p "$extract_dir"
    
    if tar -xzf "$tar_file" -C "$extract_dir" 2>/dev/null; then
        return 0
    else
        log "⚠️  解压失败: $tar_file"
        return 1
    fi
}

# 移动 chunk 文件到 Paks 目录
# 用法: move_chunk_files_to_paks <source_dir> <paks_dir>
move_chunk_files_to_paks() {
    local source_dir="$1"
    local paks_dir="$2"
    
    if [ ! -d "$source_dir" ]; then
        return 0  # 源目录不存在，无需移动
    fi
    
    mkdir -p "$paks_dir"
    
    # 移动所有 .pak, .ucas, .utoc 文件
    find "$source_dir" -maxdepth 1 -type f \( -name "*.pak" -o -name "*.ucas" -o -name "*.utoc" \) 2>/dev/null | while read file; do
        if [ -f "$file" ]; then
            filename=$(basename "$file")
            if [ ! -f "$paks_dir/$filename" ]; then
                mv "$file" "$paks_dir/"
            fi
        fi
    done
}

# 验证安装（检查基础包是否存在）
# 用法: verify_installation <paks_dir>
verify_installation() {
    local paks_dir="$1"
    
    if [ -f "${paks_dir}/pakchunk0-Linux.pak" ]; then
        log "✓ 基础包验证通过"
        local chunk_count=$(ls -1 "${paks_dir}"/pakchunk*.pak 2>/dev/null | wc -l)
        log "✓ 已安装 ${chunk_count} 个chunk文件"
        return 0
    else
        error_exit "基础包验证失败: ${paks_dir}/pakchunk0-Linux.pak 不存在"
    fi
}

# 从 UeSim 目录拷贝模型到 robot_mujoco 目录
# 用法: copy_models_from_uesim_to_robot_mujoco
copy_models_from_uesim_to_robot_mujoco() {
    # 临时禁用set -e，避免错误导致脚本退出
    set +e
    
    # 确保PROJECT_ROOT已设置
    if [ -z "${PROJECT_ROOT:-}" ]; then
        log "⚠️  PROJECT_ROOT 未设置，无法拷贝模型"
        set -e
        return 0
    fi
    
    local uesim_model_dir="${PROJECT_ROOT}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
    local robot_mujoco_dir="${PROJECT_ROOT}/src/robot_mujoco/zsibot_robots"
    
    if [ ! -d "$uesim_model_dir" ]; then
        log "⚠️  UeSim 模型目录不存在: $uesim_model_dir，跳过模型拷贝"
        log "  PROJECT_ROOT: ${PROJECT_ROOT:-未设置}"
        set -e
        return 0
    fi
    
    log "从 UeSim 目录拷贝模型到 robot_mujoco 目录..."
    mkdir -p "$robot_mujoco_dir" || true

    # Only publish the built-in robot models exposed by run_sim.sh 1-5.
    local published_robots=" xgb xgw zgws go2 go2w "
    
    local copied_count=0
    local skipped_count=0
    
    # 遍历 UeSim 模型目录下的所有机器人目录
    if [ -d "$uesim_model_dir" ]; then
        # 使用数组存储机器人目录，避免子shell问题
        local robot_names=()
        
        # 先收集所有机器人目录名
        # 临时禁用nullglob，确保glob扩展正常工作
        local old_nullglob=$(shopt -p nullglob 2>/dev/null || echo "")
        shopt -u nullglob 2>/dev/null || true
        
        for robot_dir in "$uesim_model_dir"/*; do
            # 检查是否是字面量（glob扩展失败的情况）
            if [ "$robot_dir" = "$uesim_model_dir/*" ]; then
                # glob扩展失败，使用find代替
                break
            fi
            if [ -d "$robot_dir" ]; then
                robot_name=$(basename "$robot_dir")
                if [[ "$published_robots" == *" $robot_name "* ]]; then
                    robot_names+=("$robot_name")
                fi
            fi
        done
        
        # 如果数组为空，使用find获取目录列表
        if [ ${#robot_names[@]} -eq 0 ]; then
            while IFS= read -r -d '' robot_dir; do
                robot_name=$(basename "$robot_dir")
                if [[ "$published_robots" == *" $robot_name "* ]]; then
                    robot_names+=("$robot_name")
                fi
            done < <(find "$uesim_model_dir" -maxdepth 1 -type d ! -path "$uesim_model_dir" -print0 2>/dev/null)
        fi
        
        # 恢复nullglob设置
        if [ -n "$old_nullglob" ]; then
            eval "$old_nullglob" 2>/dev/null || true
        fi
        
        if [ ${#robot_names[@]} -eq 0 ]; then
            log "⚠️  未找到机器人目录"
            set -e
            return 0
        fi
        
        # 遍历每个机器人目录，拷贝整个目录
        for robot_name in "${robot_names[@]}"; do
            robot_dir="${uesim_model_dir}/${robot_name}"
            target_robot_dir="${robot_mujoco_dir}/${robot_name}"
            
            if [ ! -d "$robot_dir" ]; then
                continue
            fi
            
            # 创建目标机器人目录
            mkdir -p "$target_robot_dir"
            
            local robot_copied=0
            local robot_skipped=0
            
            # 拷贝整个目录的所有文件（递归）
            while IFS= read -r -d '' source_file; do
                if [ -f "$source_file" ]; then
                    # 获取相对于源机器人目录的路径
                    rel_path="${source_file#$robot_dir/}"
                    target_file="${target_robot_dir}/${rel_path}"
                    target_file_dir=$(dirname "$target_file")
                    
                    # 创建目标目录
                    mkdir -p "$target_file_dir"
                    
                    # 拷贝文件（如果目标文件不存在或源文件更新）
                    if [ ! -f "$target_file" ] || [ "$source_file" -nt "$target_file" ]; then
                        if cp -f "$source_file" "$target_file" 2>/dev/null; then
                            ((copied_count++))
                            ((robot_copied++))
                        fi
                    else
                        ((skipped_count++))
                        ((robot_skipped++))
                    fi
                fi
            done < <(find "$robot_dir" -type f -print0 2>/dev/null)
            
            if [ $robot_copied -gt 0 ] || [ $robot_skipped -gt 0 ]; then
                log "  ${robot_name}: 拷贝 ${robot_copied} 个文件，跳过 ${robot_skipped} 个文件"
            fi
        done
    fi
    
    # 恢复set -e
    set -e
    
    if [ $copied_count -gt 0 ] || [ $skipped_count -gt 0 ]; then
        log "✓ 模型拷贝完成: 总计拷贝 ${copied_count} 个文件，跳过 ${skipped_count} 个文件（已存在）"
    else
        log "⚠️  未找到需要拷贝的模型文件"
    fi
}

# Chunk ID 到地图名的映射
# 用法: get_map_name_by_chunk_id <chunk_id>
get_map_name_by_chunk_id() {
    local chunk_id="$1"
    
    declare -A CHUNK_TO_MAP=(
        ["0"]="EmptyWorld"
        ["1"]="Shared"
        ["11"]="SceneWorld"
        ["12"]="Town10World"
        ["13"]="YardWorld"
        ["14"]="CrowdWorld"
        ["15"]="VeniceWorld"
        ["16"]="RunningWorld"
        ["17"]="HouseWorld"
        ["18"]="IROSFlatWorld"
        ["19"]="IROSSlopedWorld"
        ["20"]="Town10Zombie"
        ["21"]="IROSFlatWorld2025"
        ["22"]="IROSSloppedWorld2025"
        ["23"]="OfficeWorld"
        ["24"]="CustomWorld"
        ["25"]="3DGSWorld"
        ["26"]="MoonWorld"
    )
    
    echo "${CHUNK_TO_MAP[$chunk_id]:-(未知地图)}"
}

# 分割大文件函数（用于超过 2GB 的文件）
# 用法: split_file <input_file> [output_dir]
split_file() {
    local input_file="$1"
    local output_dir="${2:-$(dirname "$input_file")}"
    local base_name=$(basename "$input_file")
    local base_name_no_ext="${base_name%.*}"
    local ext="${base_name##*.}"
    local max_chunk_size=2000000000  # 1.86GB
    
    if [[ ! -f "$input_file" ]]; then
        error_exit "文件不存在: $input_file"
    fi
    
    local file_size=$(stat -c%s "$input_file" 2>/dev/null || stat -f%z "$input_file" 2>/dev/null || echo 0)
    local file_size_gb=$(echo "scale=2; $file_size / 1024 / 1024 / 1024" | bc)
    
    log "文件: $input_file"
    log "大小: ${file_size_gb}GB"
    
    if [[ $file_size -le $max_chunk_size ]]; then
        log "文件小于 2GB，无需分割"
        return 0
    fi
    
    log "开始分割文件..."
    mkdir -p "$output_dir"
    
    # 使用 split 命令分割文件
    # -b: 每个分片的大小（字节）
    # -d: 使用数字后缀
    # -a: 后缀长度
    local chunk_prefix="${output_dir}/${base_name_no_ext}.part"
    split -b "${max_chunk_size}" -d -a 3 "$input_file" "$chunk_prefix"
    
    # 计算分片数量
    local part_count=$(ls -1 "${chunk_prefix}"* 2>/dev/null | wc -l)
    log "✓ 分割完成，共 ${part_count} 个分片"
    
    # 生成合并脚本
    local merge_script="${output_dir}/${base_name_no_ext}.merge.sh"
    cat > "$merge_script" << 'EOFSCRIPT'
#!/bin/bash
# 自动生成的合并脚本
# 用法: ./merge.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_NAME="BASENAME_PLACEHOLDER"
EXT="EXT_PLACEHOLDER"
OUTPUT_FILE="${SCRIPT_DIR}/${BASE_NAME}.${EXT}"
CHECKSUM_FILE="${SCRIPT_DIR}/${BASE_NAME}.${EXT}.sha256"

echo "合并文件: ${BASE_NAME}.${EXT}"
echo "输出: ${OUTPUT_FILE}"

# 删除已存在的输出文件
if [[ -f "$OUTPUT_FILE" ]]; then
    # 检查校验和是否已匹配，如果匹配则无需覆盖
    if [[ -f "$CHECKSUM_FILE" ]]; then
        EXPECTED_SUM=$(cat "$CHECKSUM_FILE" | awk '{print $1}')
        CURRENT_SUM=$(sha256sum "$OUTPUT_FILE" 2>/dev/null | awk '{print $1}' || echo "none")
        if [[ "$EXPECTED_SUM" == "$CURRENT_SUM" ]]; then
            echo "✓ 文件已存在且校验和匹配，无需操作"
            exit 0
        fi
    fi

    read -p "输出文件已存在，是否覆盖? [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "取消合并"
        exit 1
    fi
    rm -f "$OUTPUT_FILE"
fi

# 合并所有分片
echo "正在合并分片..."
cat "${SCRIPT_DIR}/${BASE_NAME}.part"* > "$OUTPUT_FILE"

# 验证校验和
if [[ -f "$CHECKSUM_FILE" ]]; then
    echo "验证校验和..."
    EXPECTED_SUM=$(cat "$CHECKSUM_FILE" | awk '{print $1}')
    CURRENT_SUM=$(sha256sum "$OUTPUT_FILE" | awk '{print $1}')
    
    if [[ "$EXPECTED_SUM" == "$CURRENT_SUM" ]]; then
        echo "✓ 校验和匹配！"
        echo "✓ 合并成功！"
        echo "✓ 可以删除分片文件: rm ${BASE_NAME}.part*"
    else
        echo "❌ 校验和不匹配！"
        echo "  期望: $EXPECTED_SUM"
        echo "  实际: $CURRENT_SUM"
        echo "删除损坏的文件..."
        rm -f "$OUTPUT_FILE"
        exit 1
    fi
else
    # 验证文件大小 (兼容旧逻辑)
    EXPECTED_SIZE="EXPECTED_SIZE_PLACEHOLDER"
    ACTUAL_SIZE=$(stat -c%s "$OUTPUT_FILE" 2>/dev/null || stat -f%z "$OUTPUT_FILE" 2>/dev/null || echo 0)

    if [[ "$ACTUAL_SIZE" == "$EXPECTED_SIZE" ]]; then
        echo "✓ 合并成功！文件大小: $(echo "scale=2; $ACTUAL_SIZE / 1024 / 1024 / 1024" | bc)GB"
        echo "⚠️  注意: 未找到校验和文件，仅验证了文件大小"
        echo "✓ 可以删除分片文件: rm ${BASE_NAME}.part*"
    else
        echo "❌ 文件大小不匹配"
        echo "  期望: ${EXPECTED_SIZE} 字节"
        echo "  实际: ${ACTUAL_SIZE} 字节"
        exit 1
    fi
fi
EOFSCRIPT
    
    # 替换占位符
    sed -i "s/BASENAME_PLACEHOLDER/${base_name_no_ext}/g" "$merge_script"
    sed -i "s/EXT_PLACEHOLDER/${ext}/g" "$merge_script"
    sed -i "s/EXPECTED_SIZE_PLACEHOLDER/${file_size}/g" "$merge_script"
    chmod +x "$merge_script"
    
    log "✓ 合并脚本已生成: $merge_script"
    
    # 生成校验和文件
    local checksum_file="${output_dir}/${base_name_no_ext}.sha256"
    log "计算校验和..."
    sha256sum "$input_file" > "$checksum_file"
    log "✓ 校验和已保存: $checksum_file"
    
    # 列出所有生成的文件
    log ""
    log "生成的文件:"
    ls -lh "${chunk_prefix}"* "$merge_script" "$checksum_file" 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}'
}
