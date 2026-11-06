#!/bin/bash
# 检测并拆分大于 2GB 的文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

VERSION="${1:-}"
RELEASE_DIR="${2:-${PROJECT_ROOT}/releases}"

if [ -z "$VERSION" ]; then
    error_exit "请指定版本号: $0 <版本号> [发布目录]"
fi

cd "$RELEASE_DIR" || error_exit "无法进入发布目录: $RELEASE_DIR"

# 2GB 限制（字节）
MAX_SIZE=2147483648

log_section "检测并拆分大于 2GB 的文件 - 版本 ${VERSION}"

SPLIT_COUNT=0
for file in *-${VERSION}.tar.gz; do
    if [ ! -f "$file" ]; then
        continue
    fi

    # 检查是否已经分片
    base_name="${file%.tar.gz}"
    if ls "${base_name}.tar.part"* 1>/dev/null 2>&1 || ls "${base_name}.part"* 1>/dev/null 2>&1; then
        log "  ⏭️  跳过 $file (已分片)"
        continue
    fi

    # 检查文件大小
    file_size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null || echo 0)
    file_size_mb=$((file_size / 1024 / 1024))

    if [ "$file_size" -gt "$MAX_SIZE" ]; then
        log "  拆分: $file (${file_size_mb}MB > 2GB)"
        # 直接调用 common.sh 中的函数
        split_file "$RELEASE_DIR/$file" "$RELEASE_DIR" || error_exit "拆分 $file 失败"

        # 删除原始文件（分片后不再需要）
        # rm -f "$file"
        # log "    ✓ 已删除原始文件: $file"
        SPLIT_COUNT=$((SPLIT_COUNT + 1))
    else
        log "  ✓ $file (${file_size_mb}MB, 无需拆分)"
    fi
done

if [ $SPLIT_COUNT -gt 0 ]; then
    log "✓ 共拆分 $SPLIT_COUNT 个大文件"
else
    log "✓ 没有需要拆分的大文件"
fi

