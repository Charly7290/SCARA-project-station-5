#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Run this in a SECOND terminal, while run_agent.sh is running in the first
# one and the Pico is plugged in and powered. Verifies the Micro-ROS bridge
# is actually up, then (optionally) sends one real command so you can watch
# the robot react.
#
# Usage:
#   chmod +x test_link.sh
#   ./test_link.sh            # just checks topics/nodes exist
#   ./test_link.sh --send-home   # also publishes HOME once (robot WILL move)
# ---------------------------------------------------------------------------
set -euo pipefail

WS="${SCARA_WS:-$HOME/scara_microros_ws}"
# ROS 2's setup.bash files reference a couple of internal variables without
# defaults, which `set -u` treats as fatal -- relax it just for sourcing.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" 2>/dev/null || true
if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi
set -u

echo "=== ros2 node list ==="
ros2 node list || true
echo ""
echo "=== ros2 topic list ==="
ros2 topic list || true
echo ""

NODE_OK=$(ros2 node list 2>/dev/null | grep -c "pico_node" || true)
TOPIC_OK=$(ros2 topic list 2>/dev/null | grep -c "scara_cmd" || true)

if [ "$NODE_OK" -ge 1 ] && [ "$TOPIC_OK" -ge 1 ]; then
    echo "OK: /pico_node is up and /scara_cmd exists. The bridge is live."
else
    echo "NOT OK: expected /pico_node in the node list and /scara_cmd in the"
    echo "topic list, didn't see both. Checklist:"
    echo "  - Is run_agent.sh still running in the other terminal, with no errors?"
    echo "  - Did it log a new client connecting after you last reset/replugged the Pico?"
    echo "  - Reset the Pico (unplug/replug USB, or its reset button) and check again --"
    echo "    it only shows up here AFTER it successfully pings the agent on boot."
    exit 1
fi

if [ "${1:-}" == "--send-home" ]; then
    echo ""
    echo "Publishing HOME once -- the robot should start its homing sequence now."
    ros2 topic pub --once /scara_cmd std_msgs/msg/String "data: 'HOME'"
    echo "Sent. Watch the robot, not this terminal -- there's no feedback topic"
    echo "back from the firmware yet, so success/failure here is a visual check."
fi
