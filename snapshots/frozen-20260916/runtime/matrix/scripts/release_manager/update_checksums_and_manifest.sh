#!/bin/bash
# 更新发布包的校验码和 manifest.json
# 支持分片文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

VERSION="${1:-}"
RELEASE_DIR="${2:-${PROJECT_ROOT}/releases}"

if [ -z "$VERSION" ]; then
    error_exit "请指定版本号: $0 <版本号> [发布目录]"
fi

cd "$RELEASE_DIR" || error_exit "无法进入发布目录: $RELEASE_DIR"

log_section "更新校验码和 manifest - 版本 ${VERSION}"

# 辅助函数：获取文件的 SHA256 校验和
get_sha256() {
    local file="$1"
    local sha256_file="${file}.sha256"
    
    if [ -f "$sha256_file" ]; then
        awk '{print $1}' "$sha256_file"
    else
        local base_name="${file%.tar.gz}"
        local tar_sha256_file="${base_name}.tar.sha256"
        if [ -f "$tar_sha256_file" ]; then
            awk '{print $1}' "$tar_sha256_file"
        elif [ -f "$file" ]; then
            sha256sum "$file" 2>/dev/null | awk '{print $1}'
        else
            echo "null"
        fi
    fi
}

# 辅助函数：检查是否为分片文件
check_is_split() {
    local base_name="$1"
    if ls "${base_name}".tar.part* 1>/dev/null 2>&1 || ls "${base_name}".part* 1>/dev/null 2>&1; then
        echo "true"
    else
        echo "false"
    fi
}

# 辅助函数：获取分片文件列表
get_parts() {
    local base_name="$1"
    local parts=()
    
    # 先检查 tar.part* 格式（排除 .sha256 文件）
    for part_file in "${base_name}".tar.part*; do
        if [ -f "$part_file" ] && [[ ! "$part_file" =~ \.sha256$ ]] && [[ ! "$part_file" =~ \.sha256\.sha256 ]]; then
            parts+=("$(basename "$part_file")")
        fi
    done
    
    # 如果没有找到，检查 .part* 格式（排除 .sha256 文件）
    if [ ${#parts[@]} -eq 0 ]; then
        for part_file in "${base_name}".part*; do
            if [ -f "$part_file" ] && [[ ! "$part_file" =~ \.sha256$ ]] && [[ ! "$part_file" =~ \.sha256\.sha256 ]]; then
                parts+=("$(basename "$part_file")")
            fi
        done
    fi
    
    IFS=$'\n' sorted_parts=($(printf '%s\n' "${parts[@]}" | sort))
    unset IFS
    
    if [ ${#sorted_parts[@]} -eq 0 ]; then
        echo "[]"
    else
        echo -n "["
        local first=true
        for part in "${sorted_parts[@]}"; do
            if [ "$first" = true ]; then
                first=false
            else
                echo -n ","
            fi
            echo -n "\"${part}\""
        done
        echo -n "]"
    fi
}

# 步骤1: 生成校验和
log "生成 SHA256 校验和..."

# 为每个 tar.gz 文件生成单独的 .sha256 文件
for file in *-${VERSION}.tar.gz; do
    if [ -f "$file" ]; then
        sha256_file="${file}.sha256"
        if [ ! -f "$sha256_file" ] || [ "$file" -nt "$sha256_file" ]; then
            log "  计算: $file"
            sha256sum "$file" > "$sha256_file"
        fi
    fi
done

# 为分片文件生成校验和（排除 .sha256 文件，避免递归）
for part_file in *-${VERSION}.tar.part* *-${VERSION}.part*; do
    if [ -f "$part_file" ] && [[ ! "$part_file" =~ \.sha256$ ]] && [[ ! "$part_file" =~ \.sha256\.sha256 ]]; then
        sha256_file="${part_file}.sha256"
        if [ ! -f "$sha256_file" ] || [ "$part_file" -nt "$sha256_file" ]; then
            log "  计算: $part_file"
            sha256sum "$part_file" > "$sha256_file"
        fi
    fi
done

# 为合并脚本生成校验和（如果存在）
for merge_script in *-${VERSION}.tar.merge.sh *-${VERSION}.merge.sh; do
    if [ -f "$merge_script" ]; then
        sha256_file="${merge_script}.sha256"
        if [ ! -f "$sha256_file" ] || [ "$merge_script" -nt "$sha256_file" ]; then
            log "  计算: $merge_script"
            sha256sum "$merge_script" > "$sha256_file"
        fi
    fi
done

log "✓ 校验和已更新"

# 步骤2: 生成 manifest.json
log "生成 manifest.json..."

BASE_SHA256=$(get_sha256 "base-${VERSION}.tar.gz")
BASE_SIZE=$(stat -c%s "base-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "base-${VERSION}.tar.gz" 2>/dev/null || echo 0)

SHARED_SHA256=$(get_sha256 "shared-${VERSION}.tar.gz")
SHARED_SIZE=$(stat -c%s "shared-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "shared-${VERSION}.tar.gz" 2>/dev/null || echo 0)
SHARED_IS_SPLIT=$(check_is_split "shared-${VERSION}")

cat > "manifest-${VERSION}.json" << EOF
{
  "version": "${VERSION}",
  "release_date": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "packages": {
    "base": {
      "file": "base-${VERSION}.tar.gz",
      "required": true,
      "description": "基础包 (Chunk 0) - 包含EmptyWorld和核心蓝图",
      "size": ${BASE_SIZE},
      "sha256": "${BASE_SHA256}"
    },
    "shared": {
      "file": "shared-${VERSION}.tar.gz",
      "required": false,
      "description": "共享资源包 (Chunk 1) - 包含多个地图用到的共享资源",
      "size": ${SHARED_SIZE},
      "sha256": "${SHARED_SHA256}"
EOF

if [ "$SHARED_IS_SPLIT" = "true" ]; then
    SHARED_PARTS=$(get_parts "shared-${VERSION}")
    sed -i '$ s/"$/",/' "manifest-${VERSION}.json"
    cat >> "manifest-${VERSION}.json" << EOF
      "merge_script": "shared-${VERSION}.tar.merge.sh",
      "checksum_file": "shared-${VERSION}.tar.sha256",
      "is_split": true,
      "parts": ${SHARED_PARTS}
EOF
fi

# 添加 assets 包（如果存在）
if [ -f "assets-${VERSION}.tar.gz" ]; then
    ASSETS_SHA256=$(get_sha256 "assets-${VERSION}.tar.gz")
    ASSETS_SIZE=$(stat -c%s "assets-${VERSION}.tar.gz" 2>/dev/null || stat -f%z "assets-${VERSION}.tar.gz" 2>/dev/null || echo 0)
    cat >> "manifest-${VERSION}.json" << EOF
    },
    "assets": {
      "file": "assets-${VERSION}.tar.gz",
      "size": ${ASSETS_SIZE},
      "sha256": "${ASSETS_SHA256}",
      "required": true,
      "description": "资源文件包 - 包含运行时必需的文件（可执行文件、共享库、3D模型等）"
    },
    "maps": [
EOF
else
    cat >> "manifest-${VERSION}.json" << EOF
    },
    "maps": [
EOF
fi

# 添加地图包信息
first=true
for map_tar in *-${VERSION}.tar.gz; do
    if [[ "$map_tar" == base-* ]] || [[ "$map_tar" == shared-* ]] || [[ "$map_tar" == assets-* ]]; then
        continue
    fi
    if [ -f "$map_tar" ]; then
        map_name=$(echo "$map_tar" | sed "s/-${VERSION}.tar.gz//")
        map_size=$(stat -c%s "$map_tar" 2>/dev/null || stat -f%z "$map_tar" 2>/dev/null || echo 0)
        map_sha256=$(get_sha256 "$map_tar")
        map_is_split=$(check_is_split "${map_name}-${VERSION}")
        
        if [ "$first" = true ]; then
            first=false
        else
            sed -i '$ s/}$/},/' "manifest-${VERSION}.json"
        fi
        
        cat >> "manifest-${VERSION}.json" << EOF
      {
        "name": "${map_name}",
        "file": "${map_tar}",
        "required": false,
        "description": "地图包 - ${map_name}",
        "size": ${map_size},
        "sha256": "${map_sha256}"
EOF
        
        if [ "$map_is_split" = "true" ]; then
            map_parts=$(get_parts "${map_name}-${VERSION}")
            if [ -f "${map_name}-${VERSION}.tar.merge.sh" ]; then
                map_merge_script="${map_name}-${VERSION}.tar.merge.sh"
            elif [ -f "${map_name}-${VERSION}.merge.sh" ]; then
                map_merge_script="${map_name}-${VERSION}.merge.sh"
            else
                map_merge_script=""
            fi
            if [ -f "${map_name}-${VERSION}.tar.sha256" ]; then
                map_checksum_file="${map_name}-${VERSION}.tar.sha256"
            elif [ -f "${map_name}-${VERSION}.sha256" ]; then
                map_checksum_file="${map_name}-${VERSION}.sha256"
            else
                map_checksum_file=""
            fi
            
            # Collect optional fields
            MAP_EXTRAS=""
            if [ -n "$map_merge_script" ]; then
                if [ -n "$MAP_EXTRAS" ]; then MAP_EXTRAS="$MAP_EXTRAS,"; fi
                MAP_EXTRAS="${MAP_EXTRAS}\n        "merge_script": "${map_merge_script}""
            fi
            if [ -n "$map_checksum_file" ]; then
                if [ -n "$MAP_EXTRAS" ]; then MAP_EXTRAS="$MAP_EXTRAS,"; fi
                MAP_EXTRAS="${MAP_EXTRAS}\n        "checksum_file": "${map_checksum_file}""
            fi
            
            cat >> "manifest-${VERSION}.json" << EOF
,
        "is_split": true,
        "parts": ${map_parts}${MAP_EXTRAS}
EOF
        fi
        
        cat >> "manifest-${VERSION}.json" << EOF
      }
EOF
    fi
done

cat >> "manifest-${VERSION}.json" << EOF
    ]
  }
}
EOF

log "✓ manifest-${VERSION}.json 已更新"
log "  - 包含 SHA256 校验和"
log "  - 包含分片文件信息（如果存在）"

