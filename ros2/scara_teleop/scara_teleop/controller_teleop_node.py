#!/usr/bin/env python3
"""
Controller teach pendant for the SCARA (Station 5).
Reads /joy (joy_node) and publishes commands to scara_cmd.

  LT       -- dead-man's switch: sends MANUAL, release stops J1/J2/J3
  Left X   -- J1 speed
  Right X  -- J2 speed
  RT / RB  -- J3 down / up
  Y        -- HOME
  A        -- vacuum on/off (not affected by LT)

Pad indices are for the GameSir in USB mode; check them with calibrate_joy.py
if the connection mode changes.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class ControllerTeleop(Node):
    def __init__(self):
        super().__init__('scara_controller_teleop')

        # Pad mapping
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('max_speed_percent', 100)
        self.declare_parameter('trigger_threshold', 0.5)
        self.declare_parameter('j1_axis', 0)              # J1 = left stick X
        self.declare_parameter('j2_axis', 3)              # J2 = right stick X
        self.declare_parameter('dead_man_axis', 2)        # LT (analog trigger)
        self.declare_parameter('j3_down_axis', 5)         # RT (analog trigger)
        self.declare_parameter('j3_up_button', 5)         # RB (digital bumper)
        self.declare_parameter('home_button', 2)          # Y
        self.declare_parameter('tool_button', 0)          # A
        self.declare_parameter('invert_left_stick', False)
        self.declare_parameter('invert_right_stick', False)
        # Trigger axis value at rest and fully pressed
        self.declare_parameter('trigger_rest_value', 1.0)
        self.declare_parameter('trigger_full_value', -1.0)

        self._deadzone = float(self.get_parameter('deadzone').value)
        self._max_pct = int(self.get_parameter('max_speed_percent').value)
        self._trig_thresh = float(self.get_parameter('trigger_threshold').value)
        self._lx_axis = int(self.get_parameter('j1_axis').value)
        self._rx_axis = int(self.get_parameter('j2_axis').value)
        self._dead_man_axis = int(self.get_parameter('dead_man_axis').value)
        self._j3_down_axis = int(self.get_parameter('j3_down_axis').value)
        self._j3_up_button = int(self.get_parameter('j3_up_button').value)
        self._home_btn = int(self.get_parameter('home_button').value)
        self._tool_btn = int(self.get_parameter('tool_button').value)
        self._inv_l = -1.0 if bool(self.get_parameter('invert_left_stick').value) else 1.0
        self._inv_r = -1.0 if bool(self.get_parameter('invert_right_stick').value) else 1.0
        self._trig_rest = float(self.get_parameter('trigger_rest_value').value)
        self._trig_full = float(self.get_parameter('trigger_full_value').value)

        # Motion is sent at a fixed rate (sending on every /joy message made the robot lag)
        self.declare_parameter('publish_rate_hz', 20.0)   # keep well above 2.5 Hz (400 ms dead-man timeout)
        self.declare_parameter('joy_timeout_s', 0.5)      # stop sending if /joy goes silent
        self.declare_parameter('j3_up_percent', 100)      # RB speed
        self.declare_parameter('j3_trigger_deadzone', 0.1)
        self._joy_timeout = float(self.get_parameter('joy_timeout_s').value)
        self._j3_up_pct = int(self.get_parameter('j3_up_percent').value)
        self._j3_trig_dz = float(self.get_parameter('j3_trigger_deadzone').value)

        # Button debounce: repeated presses within this time are ignored
        self.declare_parameter('button_debounce_s', 0.3)
        self._debounce_s = float(self.get_parameter('button_debounce_s').value)
        self._last_press_time = {}  # button index -> last accepted press

        self._pub = self.create_publisher(String, 'scara_cmd', 10)
        self._sub = self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value),
            self._on_timer)

        self._dead_man_was_held = False
        self._prev_buttons = []
        self._tool_on = False  # tracked here only, no feedback from the Pico
        self._latest_joy = None
        self._latest_joy_time = None

        self.get_logger().info(
            "Controller teach pendant ready. Hold LT + sticks for J1/J2, "
            "RT/RB for J3. Y = HOME. A = vacuum."
        )

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def _trigger_fraction(self, raw: float) -> float:
        """Trigger reading -> 0.0 (rest) .. 1.0 (fully pressed)."""
        span = self._trig_full - self._trig_rest
        if span == 0:
            return 0.0
        frac = (raw - self._trig_rest) / span
        return max(0.0, min(1.0, frac))

    def _publish(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)

    def _rising_edge(self, buttons, idx: bool) -> bool:
        now = bool(buttons[idx]) if 0 <= idx < len(buttons) else False
        prev = bool(self._prev_buttons[idx]) if (
            self._prev_buttons and idx < len(self._prev_buttons)
        ) else False
        return now and not prev

    def _debounced_press(self, buttons, idx: int) -> bool:
        """Rising edge of a button, with debounce."""
        if not self._rising_edge(buttons, idx):
            return False
        now = self.get_clock().now()
        last = self._last_press_time.get(idx)
        if last is not None and (now - last).nanoseconds * 1e-9 < self._debounce_s:
            return False  # bounce
        self._last_press_time[idx] = now
        return True

    @staticmethod
    def _axis(axes, idx, default=0.0):
        return axes[idx] if 0 <= idx < len(axes) else default

    @staticmethod
    def _button(buttons, idx, default=0):
        return buttons[idx] if 0 <= idx < len(buttons) else default

    def _dead_man_held(self, axes) -> bool:
        return self._trigger_fraction(
            self._axis(axes, self._dead_man_axis, default=self._trig_rest)
        ) > self._trig_thresh

    def _on_joy(self, joy_msg: Joy):
        """Button and LT events (sent immediately). Motion is sent by _on_timer()."""
        axes = joy_msg.axes
        buttons = joy_msg.buttons
        self._latest_joy = joy_msg
        self._latest_joy_time = self.get_clock().now()

        # HOME (works without LT)
        if self._debounced_press(buttons, self._home_btn):
            self._publish('HOME')

        # Vacuum toggle (works without LT)
        if self._debounced_press(buttons, self._tool_btn):
            self._tool_on = not self._tool_on
            self._publish('TOOL_ON' if self._tool_on else 'TOOL_OFF')

        dead_man_held = self._dead_man_held(axes)

        if dead_man_held and not self._dead_man_was_held:
            # LT pressed: enter MANUAL
            self._publish('MANUAL')
        elif not dead_man_held and self._dead_man_was_held:
            # LT released: stop now (vacuum stays as it is)
            self._publish('J1_SPEED:0')
            self._publish('J2_SPEED:0')
            self._publish('J3_SPEED:0')

        self._dead_man_was_held = dead_man_held
        self._prev_buttons = list(buttons)

    def _on_timer(self):
        """Sends J1/J2/J3 speeds from the latest /joy message."""
        if not self._dead_man_was_held or self._latest_joy is None:
            return

        now = self.get_clock().now()
        if (now - self._latest_joy_time).nanoseconds * 1e-9 > self._joy_timeout:
            # /joy stopped: let the firmware dead-man's switch stop the robot
            return

        axes = self._latest_joy.axes
        buttons = self._latest_joy.buttons

        lx = self._apply_deadzone(self._axis(axes, self._lx_axis), self._deadzone) * self._inv_l
        ry = self._apply_deadzone(self._axis(axes, self._rx_axis), self._deadzone) * self._inv_r
        pct1 = int(round(max(-1.0, min(1.0, lx)) * self._max_pct))
        pct2 = int(round(max(-1.0, min(1.0, ry)) * self._max_pct))
        self._publish(f'J1_SPEED:{pct1}')
        self._publish(f'J2_SPEED:{pct2}')

        # J3: RT = down (proportional), RB = up
        rt = self._trigger_fraction(
            self._axis(axes, self._j3_down_axis, default=self._trig_rest))
        down_pct = 0
        if rt > self._j3_trig_dz:
            down_pct = int(round((rt - self._j3_trig_dz) / (1.0 - self._j3_trig_dz) * 100))
        up_pct = self._j3_up_pct if self._button(buttons, self._j3_up_button) else 0
        self._publish(f'J3_SPEED:{up_pct - down_pct}')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerTeleop()
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
