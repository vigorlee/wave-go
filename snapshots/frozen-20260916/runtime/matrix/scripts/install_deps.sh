#!/bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

echo "Installing system dependencies "

DEPS_DIR="deps"
LOCAL_DEB_TMP_DIR=""
APT_INDEX_UPDATED=0

cleanup_local_deb_tmp_dir() {
    if [ -n "$LOCAL_DEB_TMP_DIR" ]; then
        rm -rf "$LOCAL_DEB_TMP_DIR"
    fi
}

trap cleanup_local_deb_tmp_dir EXIT

if [ ! -d "$DEPS_DIR" ]; then
    echo "ERROR: Dependencies directory not found: $DEPS_DIR"
    exit 1
fi

install_deb_glob() {
    local matched=0
    local pkg
    local apt_pkg
    local tmp_pkg

    for pkg in "$@"; do
        if [ -f "$pkg" ]; then
            if [ -z "$LOCAL_DEB_TMP_DIR" ]; then
                LOCAL_DEB_TMP_DIR="$(mktemp -d /tmp/matrix-local-debs.XXXXXX)"
                chmod 755 "$LOCAL_DEB_TMP_DIR"
            fi

            tmp_pkg="$LOCAL_DEB_TMP_DIR/$(basename "$pkg")"
            cp -f "$pkg" "$tmp_pkg"
            chmod 644 "$tmp_pkg"
            apt_pkg="$tmp_pkg"
            sudo apt install -y --allow-downgrades "$apt_pkg"
            matched=1
        fi
    done

    if [ "$matched" -eq 0 ]; then
        echo "ERROR: No package matched: $*"
        exit 1
    fi
}

apt_has_package() {
    apt-cache show "$1" >/dev/null 2>&1
}

apt_update_once() {
    if [ "$APT_INDEX_UPDATED" -eq 0 ]; then
        sudo apt update
        APT_INDEX_UPDATED=1
    fi
}

remove_partial_robot_forward() {
    local status
    status="$(dpkg-query -W -f='${db:Status-Abbrev}' robot-forward 2>/dev/null || true)"
    if [ -n "$status" ] && [ "$status" != "ii " ]; then
        echo "Removing partially installed robot-forward package state..."
        sudo dpkg --remove robot-forward >/dev/null 2>&1 || true
    fi
}

ensure_ros2_humble_apt_source() {
    if apt_has_package ros-humble-ros-base; then
        return 0
    fi

    echo "ROS 2 Humble apt packages are not available. Configuring ROS 2 apt source..."
    sudo apt install curl gnupg lsb-release ca-certificates software-properties-common -y
    sudo add-apt-repository universe -y

    local ubuntu_codename
    ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
    if [ -z "$ubuntu_codename" ]; then
        echo "ERROR: Cannot detect Ubuntu codename for ROS 2 apt source."
        exit 1
    fi

    local ros_keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
    local ros_repo_url="${ROS_APT_REPO_URL:-http://packages.ros.org/ros2/ubuntu}"
    local tmp_key
    tmp_key="$(mktemp)"

    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$tmp_key"
    sudo gpg --dearmor -o "$ros_keyring" "$tmp_key"
    rm -f "$tmp_key"

    echo "deb [arch=$(dpkg --print-architecture) signed-by=${ros_keyring}] ${ros_repo_url} ${ubuntu_codename} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    apt_update_once

    if ! apt_has_package ros-humble-ros-base; then
        echo "ERROR: ROS 2 Humble packages are still unavailable after adding ${ros_repo_url}."
        echo "       If you are behind a mirror/proxy, set ROS_APT_REPO_URL to a reachable ROS 2 apt mirror and rerun."
        exit 1
    fi
}

remove_partial_robot_forward

sudo apt-get install protobuf-compiler -y
sudo apt-get install libspdlog-dev -y
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev -y
sudo apt-get install libglib2.0-dev mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev libboost-all-dev libeigen3-dev -y
sudo apt install libhdf5-dev -y
sudo apt install libqt5core5a -y
sudo apt install libqt5gui5 -y
sudo apt install libqt5svg5-dev -y
sudo apt install libqt5widgets5 -y
sudo apt install curl -y
sudo apt install cmake-qt-gui -y
sudo apt install g++ gcc -y
sudo apt install libopencv-dev -y
sudo apt install jq -y
sudo apt install libpcl-common1.12 -y
ensure_ros2_humble_apt_source
apt_update_once
sudo apt install ros-humble-desktop ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor -y
sudo apt install qtcreator -y
sudo apt install qtquickcontrols2-5-dev -y
sudo apt install qml-module-qtquick-controls2 -y
sudo apt install libqt5x11extras5-dev -y

install_deb_glob "$DEPS_DIR"/lcm_*.deb
install_deb_glob "$DEPS_DIR"/zsibot_common_*.deb
install_deb_glob "$DEPS_DIR"/ecal_*.deb
install_deb_glob "$DEPS_DIR"/mujoco_*.deb
install_deb_glob "$DEPS_DIR"/onnx_*.deb
install_deb_glob "$DEPS_DIR"/robot-forward_*.deb

sudo apt install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qml-module-qtquick-controls qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts qml-module-qtgraphicaleffects \
    qml-module-qtqml-models2 qml-module-qtqml qml-module-qtquick-window2
sudo apt install fonts-noto-color-emoji -y

echo "System dependencies installed."
