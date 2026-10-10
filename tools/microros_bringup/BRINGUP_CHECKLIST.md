# Micro-ROS bring-up checklist (your computer, standing in for the Pi)

Goal: confirm the full chain — Pico firmware → USB → Micro-ROS agent → ROS 2
`/scara_cmd` topic — actually works, before worrying about which physical
box the agent runs on. Everything here is Linux-host-agnostic: when you get
a real Raspberry Pi, you run the exact same three scripts on it instead.

I can't run these myself (I don't have access to your computer or the
Pico from here) — run each step and paste back the output if something
doesn't match what's expected, and we'll debug it together.

## 0. One-time setup
```bash
cd /path/to/microros_bringup
chmod +x setup_host.sh run_agent.sh test_link.sh
./setup_host.sh /path/to/entrega3_patch_bundle/scara_teleop
```
If this is the first time your user was added to the `docker`/`dialout`
groups, **log out and back in (or reboot)** before continuing — group
membership changes don't apply to your already-open terminal/session.

Expected at the end: "Build done." with no red `colcon build` errors. If
`colcon build` fails, paste the error — likely a missing rosdep key or a
`pynput` import issue.

## 1. Flash the patched firmware onto the Pico (if you haven't already)
From your Pico repo (with the firmware patch files already merged in,
per our last conversation):
```bash
mkdir -p build && cd build
cmake ..
make -j4
```
Hold the Pico's BOOTSEL button, plug it in over USB, it mounts as a mass
storage device; drag the resulting `main.uf2` onto it. It reboots running
the new firmware automatically.

## 2. Start the agent (terminal 1)
```bash
cd /path/to/microros_bringup
./run_agent.sh
```
Expected: it prints something like
`[1696...] | main.cpp...| Running an agent` and then sits waiting —
no errors, no crash. If you get "no /dev/ttyACM* device found", the Pico
isn't enumerating as USB-serial yet; check `dmesg | tail -20` right after
plugging it in.

## 3. Reset the Pico
Unplug/replug its USB cable, or press its reset button if wired to one.

Expected in terminal 1 (the agent): within a couple of seconds, a new log
line about a client/participant connecting. That's the firmware's
`rmw_uros_ping_agent()` succeeding — before this, the firmware is stuck
retrying the ping and never leaves `STATE_INIT`.

## 4. Verify the bridge (terminal 2)
```bash
cd /path/to/microros_bringup
./test_link.sh
```
Expected: `OK: /pico_node is up and /scara_cmd exists.` If you get the
NOT OK branch instead, go back to step 3 — the agent has to see the Pico's
boot-time ping, so a stale "already running" Pico that booted before the
agent was up won't show up until you reset it again.

## 5. Send one real command and watch the robot
```bash
./test_link.sh --send-home
```
This publishes `HOME` once. Watch the robot itself, not the terminal —
there's no status/feedback topic coming back from the firmware yet, so
"did it work" is a visual check (does it drive toward its limit switches
and complete the two-phase homing sequence we went over last time).

## 6. Try the keyboard teleop node
In a third terminal:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/scara_microros_ws/install/setup.bash
ros2 run scara_teleop keyboard_teleop
```
Press `m` first (enters STATE_MANUAL — jog commands are ignored outside
it), then hold `q`/`a` (J1), `w`/`s` (J2), or `e`/`d` (J3) and watch the
corresponding joint move; release and it should stop. If a joint doesn't
stop within ~400ms of releasing the key, that's the dead-man's-switch
watchdog not working as intended — flag it, that's a real bug to chase,
not expected behavior.

## If something doesn't match
Paste back: which step, the exact command, and the exact terminal output
(not a paraphrase) — especially for `colcon build` failures, agent
connection errors, or `ros2 topic list`/`ros2 node list` output that's
missing `/scara_cmd` or `/pico_node`. That's enough for me to diagnose
without needing to see your machine directly.
