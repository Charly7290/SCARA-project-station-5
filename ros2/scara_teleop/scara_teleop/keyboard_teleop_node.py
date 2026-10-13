#!/usr/bin/env python3
"""
Keyboard teach pendant for the SCARA (Station 5).
Publishes jog commands to scara_cmd while a key is held, STOP on release.

    m      -> MANUAL (needed before jogging)
    q / a  -> J1 CW / CCW
    w / s  -> J2 CW / CCW
    e / d  -> J3 up / down
    h      -> HOME
    Esc    -> quit

Uses pynput because a terminal can't detect key releases.
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from pynput import keyboard
except ImportError as exc:
    raise ImportError(
        "pynput is required for keyboard_teleop (pip install pynput). "
        "On Linux without a desktop session, pynput's evdev backend may "
        "need the running user to be in the 'input' group."
    ) from exc

# Must be faster than the firmware dead-man's timeout (400 ms)
HEARTBEAT_HZ = 10.0

# key -> (move command, stop command); the servo holds its position, no stop needed
JOG_CMDS = {
    'q': ('J1_CW', 'J1_STOP'),
    'a': ('J1_CCW', 'J1_STOP'),
    'w': ('J2_CW', 'J2_STOP'),
    's': ('J2_CCW', 'J2_STOP'),
    'e': ('J3_UP', None),
    'd': ('J3_DOWN', None),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('scara_keyboard_teleop')
        self._pub = self.create_publisher(String, 'scara_cmd', 10)
        self._lock = threading.Lock()
        self._held = set()  # movement keys held now

        self._timer = self.create_timer(1.0 / HEARTBEAT_HZ, self._heartbeat)
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

        self.get_logger().info(
            "Keyboard teach pendant ready. Press 'm' first to enter MANUAL, "
            "then hold q/a=J1, w/s=J2, e/d=J3 to jog (release to stop). "
            "'h' = HOME. Esc = quit."
        )

    def _publish(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)

    @staticmethod
    def _char(key):
        try:
            return key.char.lower()
        except AttributeError:
            return None

    def _on_press(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info('Esc pressed, shutting down teleop.')
            rclpy.shutdown()
            return False  # stops the pynput listener

        c = self._char(key)
        if c is None:
            return

        if c == 'm':
            self._publish('MANUAL')
            return
        if c == 'h':
            with self._lock:
                self._held.clear()
            self._publish('HOME')
            return

        if c in JOG_CMDS:
            move_cmd, _ = JOG_CMDS[c]
            with self._lock:
                is_new = c not in self._held
                self._held.add(c)
            if is_new:
                self._publish(move_cmd)  # the heartbeat repeats it while held

    def _on_release(self, key):
        c = self._char(key)
        if c in JOG_CMDS:
            with self._lock:
                was_held = c in self._held
                self._held.discard(c)
            if was_held:
                _, stop_cmd = JOG_CMDS[c]
                if stop_cmd is not None:
                    self._publish(stop_cmd)

    def _heartbeat(self):
        with self._lock:
            held_now = list(self._held)
        for c in held_now:
            move_cmd, _ = JOG_CMDS[c]
            self._publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
