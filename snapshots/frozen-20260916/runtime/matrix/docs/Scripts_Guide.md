# Script Usage Guide

MATRiX provides various scripts to help you build, install, and run the simulator. Here's how to use them effectively:

## Script Categories

### User Scripts (For End Users)

| Script | Purpose | Usage |
|--------|---------|-------|
| `build.sh` | One-click build and dependency installation | `./scripts/build.sh` |
| `run_sim.sh` | Launch simulation (Legacy CLI) | `./scripts/run_sim.sh <robot_type> <map_id>` |

*Note: It is highly recommended to use `./bin/sim_launcher` instead of `run_sim.sh` for both GUI and CLI usage.*
| `install_chunks.sh` | Download and install chunk packages from GitHub | `bash scripts/release_manager/install_chunks.sh <version>` |
| `install_chunks_local.sh` | Install chunk packages from local releases/ directory | `bash scripts/release_manager/install_chunks_local.sh <version>` |

### Developer Scripts (For Contributors)

| Script | Purpose | Usage |
|--------|---------|-------|
| `build_mc.sh` | Build MC control module | `./scripts/build_mc.sh` |
| `upload_to_release.sh` | Upload packages to GitHub Releases | `bash scripts/release_manager/upload_to_release.sh <version>` |
| `split_large_files.sh` | Split large files (>2GB) for GitHub | `bash scripts/release_manager/split_large_files.sh <file_path>` |

## Advanced Installation Scenarios

### Offline Installation (No Internet)

```bash
# 1. On a machine with internet, download packages
bash scripts/release_manager/install_chunks.sh 0.1.2

# 2. Copy the releases/ directory to offline machine

# 3. On offline machine, install from local files
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → Installs assets package (required) and all other packages from releases/ directory
```

### Adding More Maps Later

```bash
# Option 1: Download and install new maps
bash scripts/release_manager/install_chunks.sh 0.1.2
# → Select additional maps to download

# Option 2: If files already in releases/, just install
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → Installs assets package (if needed) and all available maps from releases/
```

### Reinstalling Packages

```bash
# Quick reinstall from local releases/ directory
bash scripts/release_manager/install_chunks_local.sh 0.1.2
# → No download needed, fast installation
```

## Script Selection Guide

**When to use `install_chunks.sh`:**
- ✅ First-time installation
- ✅ Need to download latest version from GitHub
- ✅ Want to selectively choose maps to download
- ✅ Have internet connection

**When to use `install_chunks_local.sh`:**
- ✅ Files already downloaded to `releases/` directory
- ✅ Offline installation (no internet)
- ✅ Quick reinstall of existing packages
- ✅ Want to install all available maps automatically

## Understanding File Locations

```text
matrix/
├── releases/                    # Downloaded packages (created after install_chunks.sh)
│   ├── assets-0.1.2.tar.gz     # Assets package (required)
│   ├── base-0.1.2.tar.gz       # Base package (required)
│   ├── shared-0.1.2.tar.gz     # Shared resources (recommended)
│   └── *.tar.gz                # Map packages (optional)
│
└── src/UeSim/Linux/zsibot_mujoco_ue/  # Runtime directory (where packages are installed)
    └── Content/Paks/            # Installed chunk files (.pak, .ucas, .utoc)
```

**Key Points:**
- `matrix/releases/` = Storage for downloaded packages (source files)
- `src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks/` = Runtime location (installed files)
- `install_chunks.sh` downloads to `matrix/releases/` AND installs to runtime directory
- `install_chunks_local.sh` only installs from `matrix/releases/` to runtime directory

> **Tip:** Keep files in `matrix/releases/` directory for future use. You can delete them to save space, but you'll need to re-download if you want to reinstall.