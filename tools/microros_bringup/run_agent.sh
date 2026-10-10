#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Runs the Micro-ROS agent in Docker, pointed at the Pico's USB-serial port.
#
# Remember: despite the firmware calling this transport "UART"
# (pico_uart_transports.h/.c), it's actually USB CDC (stdio over the Pico's
# USB connection -- see CMakeLists.txt's pico_enable_stdio_usb(main 1)).
# So this always targets a /dev/ttyACM* device, never a physical UART pin.
#
# Usage:
#   chmod +x run_agent.sh
#   ./run_agent.sh            # auto-detects /dev/ttyACM0
#   ./run_agent.sh /dev/ttyACM1   # explicit device, if you have more than one
#
# Pick the ROS_DISTRO image tag below to match whatever setup_host.sh
# installed (check with: echo $ROS_DISTRO).
# ---------------------------------------------------------------------------
set -euo pipefail

ROS_DISTRO_TAG="${ROS_DISTRO:-jazzy}"
DEV="${1:-}"

if [ -z "$DEV" ]; then
    CANDIDATES=(/dev/ttyACM*)
    if [ -e "${CANDIDATES[0]}" ]; then
        DEV="${CANDIDATES[0]}"
        if [ "${#CANDIDATES[@]}" -gt 1 ]; then
            echo "Found multiple serial devices: ${CANDIDATES[*]}"
            echo "Using $DEV -- pass the right one explicitly if that's wrong:"
            echo "  $0 /dev/ttyACM1"
        fi
    else
        echo "No /dev/ttyACM* device found."
        echo "  - Is the Pico plugged in over USB (not just power)?"
        echo "  - Check 'dmesg | tail -20' right after plugging it in."
        echo "  - If it shows up as something other than ttyACM*, pass that path explicitly."
        exit 1
    fi
fi

IMAGE="microros/micro-ros-agent:${ROS_DISTRO_TAG}"

# Only one agent can use the serial port: stop leftover ones first
EXISTING="$(sudo docker ps -q --filter "ancestor=${IMAGE}")"
if [ -n "$EXISTING" ]; then
    echo "Stopping $(echo "$EXISTING" | wc -l) leftover Micro-ROS agent container(s)..."
    sudo docker stop $EXISTING > /dev/null
fi

echo "Using device: $DEV"
echo "Using agent image: ${IMAGE}"
echo "(Ctrl+C to stop the agent -- don't just close the terminal)"
echo ""

# Fixed name: docker refuses to start a second agent
sudo docker run -it --rm --name scara_microros_agent \
    -v /dev:/dev --privileged --net=host \
    "${IMAGE}" \
    serial --dev "$DEV" -b 115200
