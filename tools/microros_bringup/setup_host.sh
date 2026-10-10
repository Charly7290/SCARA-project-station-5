#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Station 5 (SCARA) -- Micro-ROS bring-up, host side.
#
# Run this on YOUR OWN Linux computer. It stands in for the Raspberry Pi
# that the project architecture calls for -- the Micro-ROS agent and the
# ROS 2 high-level stack don't care what Linux box they run on, they just
# need a USB port to see the Pico on. When you later get an actual
# Raspberry Pi, you copy the resulting ~/ros2_ws over (or just re-run this
# same script on the Pi) -- nothing about the ROS 2 side changes.
#
# What this script does:
#   1. Detects your Ubuntu version and picks the matching ROS 2 distro
#      (22.04 jammy -> humble, 24.04 noble -> jazzy). Installs ROS 2 if
#      it isn't already present.
#   2. Installs Docker if missing (used to run the Micro-ROS agent --
#      avoids building micro-ROS from source).
#   3. Adds your user to the `dialout` group (needed to open
#      /dev/ttyACM0 without sudo) and the `docker` group.
#   4. Creates a NEW, DEDICATED workspace (default: ~/scara_microros_ws --
#      deliberately NOT ~/ros2_ws, so this never touches a workspace you
#      already have) and copies the scara_teleop package into its src/.
#   5. rosdep installs + colcon builds that workspace.
#
# Usage:
#   chmod +x setup_host.sh
#   ./setup_host.sh /path/to/entrega3_patch_bundle/scara_teleop
#
#   # Only if you explicitly want a different workspace path instead of the
#   # default ~/scara_microros_ws (e.g. to fold this into an existing
#   # workspace on purpose):
#   SCARA_WS=~/ros2_ws ./setup_host.sh /path/to/entrega3_patch_bundle/scara_teleop
#
# (If you omit the path, the script still does steps 1-3 and tells you
#  how to finish steps 4-5 by hand.)
# ---------------------------------------------------------------------------
set -euo pipefail

TELEOP_SRC="${1:-}"
WS="${SCARA_WS:-$HOME/scara_microros_ws}"

echo "=== [1/5] Detecting OS / ROS 2 distro ==="
if [ ! -f /etc/os-release ]; then
    echo "Can't find /etc/os-release -- this script assumes Ubuntu. Stopping."
    exit 1
fi
# shellcheck disable=SC1091
. /etc/os-release
echo "Detected: $PRETTY_NAME"

case "${VERSION_CODENAME:-}" in
    jammy)  ROS_DISTRO_TARGET="humble" ;;
    noble)  ROS_DISTRO_TARGET="jazzy" ;;
    *)
        echo "Unrecognized Ubuntu codename '${VERSION_CODENAME:-unknown}'."
        echo "This script only auto-installs ROS 2 on Ubuntu 22.04 (humble) or 24.04 (jazzy)."
        echo "If you're on something else, install ROS 2 yourself first, then re-run this"
        echo "script -- it will detect the existing install and skip straight to step 2."
        ROS_DISTRO_TARGET=""
        ;;
esac

if [ -n "${ROS_DISTRO:-}" ] && [ -d "/opt/ros/${ROS_DISTRO}" ]; then
    echo "ROS 2 '$ROS_DISTRO' is already sourced and installed. Skipping install."
elif [ -n "$ROS_DISTRO_TARGET" ] && [ -d "/opt/ros/$ROS_DISTRO_TARGET" ]; then
    echo "ROS 2 '$ROS_DISTRO_TARGET' is already installed at /opt/ros/$ROS_DISTRO_TARGET. Skipping install."
    ROS_DISTRO="$ROS_DISTRO_TARGET"
elif [ -n "$ROS_DISTRO_TARGET" ]; then
    echo "Installing ROS 2 '$ROS_DISTRO_TARGET' (ros-base)..."
    sudo apt update
    sudo apt install -y locales curl gnupg lsb-release
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y "ros-${ROS_DISTRO_TARGET}-ros-base" \
        python3-colcon-common-extensions python3-rosdep python3-argcomplete python3-pip

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update

    if ! grep -q "source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
        echo "source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash" >> "$HOME/.bashrc"
    fi
    ROS_DISTRO="$ROS_DISTRO_TARGET"
    echo "ROS 2 installed. (Added 'source /opt/ros/$ROS_DISTRO_TARGET/setup.bash' to ~/.bashrc.)"
else
    echo "No ROS 2 install detected and this script doesn't know how to install one"
    echo "for your distro. Install ROS 2 manually, then re-run this script."
    exit 1
fi
# Make sure it's sourced for the rest of THIS script's run too.
# ROS 2's own setup.bash isn't written to tolerate `set -u` (it references
# a couple of internal variables without defaults) -- temporarily relax
# that just for the source, then restore it.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "=== [2/5] Checking Docker (used to run the Micro-ROS agent) ==="
if command -v docker &> /dev/null; then
    echo "Docker already installed."
else
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sudo sh /tmp/get-docker.sh
    rm -f /tmp/get-docker.sh
fi
if ! groups "$USER" | grep -q docker; then
    sudo usermod -aG docker "$USER"
    echo "Added $USER to the 'docker' group -- you'll need to log out/in (or reboot) for this to take effect."
fi

echo "=== [3/5] Serial port permissions (dialout group) ==="
if ! groups "$USER" | grep -q dialout; then
    sudo usermod -aG dialout "$USER"
    echo "Added $USER to the 'dialout' group -- log out/in (or reboot) before plugging in the Pico."
else
    echo "$USER already in 'dialout'."
fi

echo "=== [4/5] ROS 2 workspace + scara_teleop package ==="
mkdir -p "$WS/src"
if [ -n "$TELEOP_SRC" ]; then
    if [ -d "$TELEOP_SRC" ]; then
        rm -rf "$WS/src/scara_teleop"
        cp -r "$TELEOP_SRC" "$WS/src/scara_teleop"
        echo "Copied scara_teleop into $WS/src/scara_teleop"
    else
        echo "WARNING: '$TELEOP_SRC' not found -- skipping copy. Copy it into $WS/src/ yourself:"
        echo "  cp -r /path/to/scara_teleop $WS/src/"
    fi
else
    echo "No source path given -- copy the package in yourself before building:"
    echo "  cp -r /path/to/entrega3_patch_bundle/scara_teleop $WS/src/"
fi

echo "=== [5/5] rosdep install + colcon build ==="
cd "$WS"
if [ -d src/scara_teleop ]; then
    rosdep install --from-paths src --ignore-src -y -r || true
    # pynput is a pip dependency, not always resolvable via rosdep:
    pip3 install --user pynput || pip install --break-system-packages pynput || true
    colcon build --packages-select scara_teleop
    echo ""
    echo "Build done. In any NEW terminal from now on, run:"
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "  source $WS/install/setup.bash"
else
    echo "Skipped build -- scara_teleop isn't in $WS/src yet. Copy it in, then run:"
    echo "  cd $WS && colcon build --packages-select scara_teleop"
fi

echo ""
echo "=== Done. If this was your first run, LOG OUT AND BACK IN NOW ==="
echo "(needed for the new docker/dialout group membership to apply), then"
echo "see BRINGUP_CHECKLIST.md for the actual test sequence with the Pico."
