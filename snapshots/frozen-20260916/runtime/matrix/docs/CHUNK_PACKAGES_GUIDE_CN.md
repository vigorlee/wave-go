# MATRiX Chunk Packages 使用指南

## 📦 什么是Chunk Packages?

MATRiX现在支持模块化打包，将模拟器内容分为：
- **资源文件包**: 包含模拟器启动器和核心二进制文件（必需）
- **基础包**: 必需的核心文件和EmptyWorld地图（必需）
- **共享资源包**: 多个地图共享的资源（推荐安装）
- **地图包**: 各个独立的地图，可按需下载

这种设计让用户可以：
- ✅ 只下载需要的内容，节省存储空间
- ✅ 快速开始（只需下载基础包和资源包）
- ✅ 按需扩展（需要哪个地图再下载）

## 🚀 快速安装

### 自动安装（推荐）

```bash
bash scripts/release_manager/install_chunks.sh 0.1.2
```

### 完整离线包

如果 GitHub 下载不可用，可以下载预打包的完整运行包：

- [matrix_0.1.2.zip（Google Drive）](https://drive.google.com/file/d/1d4q28AgSwmfv7x07oE-YF8xVOdSva9ll/view?usp=drive_link)
- [matrix_0.1.2.zip（百度网盘，提取码：`jbk3`）](https://pan.baidu.com/s/12k5XJwD53ax3we3_1Gulmw?pwd=jbk3)
- [matrix_0.1.2.zip（Artifactory 内部链接）](http://192.168.50.40:8081/artifactory/jszrsim/github/matrix_0.1.2.zip)

SHA256：`452e030471c5fb94240b3bc5fb33b243ca84c6d7b4aa4452a1710f43fd804bfc`

### 手动安装

1. **准备目录**

   在项目根目录进入 `releases` 目录：
   ```bash
   cd releases
   ```

2. **下载包到 releases 目录**

   - **下载资源文件包**（必需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.1.2/assets-0.1.2.tar.gz
     ```

   - **下载基础包**（必需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.1.2/base-0.1.2.tar.gz
     ```

   - **下载共享资源包**（推荐）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.1.2/shared-0.1.2.tar.gz
     ```

   - **下载地图包**（按需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.1.2/SceneWorld-0.1.2.tar.gz
     ```

3. **执行本地安装脚本**

   回到项目根目录并运行安装脚本：
   ```bash
   cd ..
   bash scripts/release_manager/install_chunks_local.sh 0.1.2
   ```

## 📋 包说明

### 资源文件包 (assets-0.1.2.tar.gz) - 必需
- **内容**:
  - `bin/sim_launcher`: 模拟器启动器
  - 核心二进制依赖文件
- **必需**: ✅ 是

### 基础包 (base-0.1.2.tar.gz) - 必需
- **大小**: ~2.3GB
- **内容**:
  - EmptyWorld地图
  - 核心蓝图和系统文件
  - Chunk 0 (pakchunk0)
- **必需**: ✅ 是

### 共享资源包 (shared-0.1.2.tar.gz) - 推荐
- **大小**: ~3.3GB
- **内容**:
  - Fab/Carla共享资源
  - 多个地图共享的蓝图和资源
  - Chunk 1 (pakchunk1)
- **必需**: ⚠️ 否，但多个地图依赖，强烈建议安装

### 地图包 - 可选

| 地图包 | 大小 | Chunk ID | 说明 |
|--------|------|----------|------|
| SceneWorld | ~423MB | 11 | 仓库场景 |
| Town10World | ~1.1GB | 12 | 城镇场景（大） |
| YardWorld | ~695MB | 13 | 庭院场景 |
| CrowdWorld | ~60MB | 14 | 人群场景 |
| VeniceWorld | ~328MB | 15 | 威尼斯场景 |
| RunningWorld | ~36MB | 16 | 跑步场景 |
| HouseWorld | ~265MB | 17 | 房屋场景 |
| IROSFlatWorld | ~300KB | 18 | IROS平地场景 |
| IROSSlopedWorld | ~250MB | 19 | IROS斜坡场景 |
| Town10Zombie | ~628MB | 20 | 僵尸场景（大） |
| IROSFlatWorld2025 | ~148KB | 21 | IROS 2025平地场景 |
| IROSSloppedWorld2025 | ~149KB | 22 | IROS 2025斜坡场景 |
| OfficeWorld | ~418MB | 23 | 办公室场景 |
| CustomWorld | ~22MB | 24 | 自定义场景 |
| 3DGSWorld | ~206MB | 25 | 3D高斯地图 |
| MoonWorld | ~603MB | 26 | 月球环境 |

## 🔍 验证安装

安装后检查：

```bash
# 1. 检查资源文件 (应存在且 >1MB)
ls -lh bin/sim_launcher

# 2. 检查 PAK 文件
cd src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks
ls -lh pakchunk*.pak
```

应该看到：
- `pakchunk0-Linux.pak` - 基础包（必需）
- `pakchunk1-Linux.pak` - 共享资源包（如果已安装）
- `pakchunk11-Linux.pak` 等 - 地图包（如果已安装）

## 🎮 使用

安装完成后，运行模拟器：

```bash
# 已在 matrix 根目录
./bin/sim_launcher
```

## ❓ 常见问题

**Q: 我只想运行EmptyWorld，需要下载哪些包？**
A: 需要资源文件包（assets）和基础包（base）。

**Q: 为什么共享资源包是推荐的？**
A: 因为多个地图都依赖共享资源包中的资源，如果不安装，这些地图可能无法正常加载。

**Q: 我可以只下载部分地图包吗？**
A: 可以！你可以根据需要只下载要使用的地图包。

**Q: 如何更新到新版本？**
A: 下载新版本的包，解压覆盖旧文件即可。建议先备份。

## 📚 更多信息

- [主 README](../README.md) - 项目主文档
- [中文文档](../docs/README_CN.md) - 中文使用指南
