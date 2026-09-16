# 脚本使用指南

MATRiX 提供了多种脚本来帮助您构建、安装和运行仿真器。以下是如何有效使用它们：

## 脚本分类

### 用户脚本（面向最终用户）

| 脚本 | 用途 | 使用方法 |
|------|------|---------|
| `build.sh` | 一键构建和依赖安装 | `./scripts/build.sh` |
| `run_sim.sh` | 启动仿真 (旧版 CLI) | `./scripts/run_sim.sh <机器人类型> <地图ID>` |

*注意：强烈建议直接使用 `./bin/sim_launcher`（支持 GUI 和命令行参数）代替 `run_sim.sh`。*
| `install_chunks.sh` | 从 GitHub Releases 下载并安装分块包 | `bash scripts/release_manager/install_chunks.sh <版本>` |
| `install_chunks_local.sh` | 从本地 releases/ 目录安装分块包 | `bash scripts/release_manager/install_chunks_local.sh <版本>` |

### 开发者脚本（面向贡献者）

| 脚本 | 用途 | 使用方法 |
|------|------|---------|
| `build_mc.sh` | 构建 MC 控制模块 | `./scripts/build_mc.sh` |
| `upload_to_release.sh` | 上传包到 GitHub Releases | `bash scripts/release_manager/upload_to_release.sh <版本>` |
| `split_large_files.sh` | 分割大文件 (>2GB) 用于 GitHub | `bash scripts/release_manager/split_large_files.sh <文件路径>` |

## 高级安装场景

### 离线安装（无网络）

```bash
# 1. 在有网络的机器上，下载包
bash scripts/release_manager/install_chunks.sh 0.1.2

# 2. 将 releases/ 目录复制到离线机器

# 3. 在离线机器上，从本地文件安装
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → 安装资源文件包（必需）和 releases/ 目录中的所有其他包
```

### 稍后添加更多地图

```bash
# 选项 1: 下载并安装新地图
bash scripts/release_manager/install_chunks.sh 0.1.2
# → 选择要下载的额外地图

# 选项 2: 如果文件已在 releases/ 中，直接安装
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → 安装资源文件包（如需要）和 releases/ 中的所有可用地图
```

### 重新安装包

```bash
# 从本地 releases/ 目录快速重新安装
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → 无需下载，快速安装
```

## 脚本选择指南

**何时使用 `install_chunks.sh`：**
- ✅ 首次安装
- ✅ 需要从 GitHub 下载最新版本
- ✅ 希望选择性选择要下载的地图
- ✅ 有网络连接

**何时使用 `install_chunks_local.sh`：**
- ✅ 文件已下载到 `releases/` 目录
- ✅ 离线安装（无网络）
- ✅ 快速重新安装现有包
- ✅ 希望自动安装所有可用地图

## 理解文件位置

```text
matrix/
├── releases/                    # 下载的包（运行 install_chunks.sh 后创建）
│   ├── assets-0.1.2.tar.gz     # 资源文件包（必需）
│   ├── base-0.1.2.tar.gz       # 基础包（必需）
│   ├── shared-0.1.2.tar.gz     # 共享资源（推荐）
│   └── *.tar.gz                # 地图包（可选）
│
└── src/UeSim/Linux/zsibot_mujoco_ue/  # 运行时目录（包安装位置）
    └── Content/Paks/            # 已安装的分块文件 (.pak, .ucas, .utoc)
```

**关键点：**
- `matrix/releases/` = 下载包的存储位置（源文件）
- `src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks/` = 运行时位置（已安装文件）
- `install_chunks.sh` 下载到 `matrix/releases/` 并安装到运行时目录
- `install_chunks_local.sh` 仅从 `matrix/releases/` 安装到运行时目录

> **提示：** 保留 `matrix/releases/` 目录中的文件以供将来使用。您可以删除它们以节省空间，但如果要重新安装，则需要重新下载。