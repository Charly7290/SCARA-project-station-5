#!/usr/bin/env python3
"""
Digital twin bridge for the SCARA (Station 5).
Reads /scara_joints from the Pico and publishes /joint_states for
robot_state_publisher (URDF in scara_description).

  /scara_joints (Float32MultiArray) = [joint 1 (deg), joint 2 (deg), servo (deg), homed, vacuum]
  /joint_states  = joint1 (rad), joint2 (rad), joint_prismatica (m)
  /twin_markers  = vacuum indicator at the tool tip

The servo has no feedback, so J3 follows the commanded angle at servo_max_dps.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

# Vacuum marker colors (r, g, b, alpha): magenta stands out from the green/orange/blue links
VACUUM_ON_RGBA = (1.0, 0.0, 1.0, 1.0)
VACUUM_OFF_RGBA = (0.3, 0.3, 0.3, 0.4)


class JointBridge(Node):
    def __init__(self):
        super().__init__('scara_joint_bridge')

        # Signs and offsets to match the URDF (tune here, no reflash needed)
        self.declare_parameter('sign_j1', -1.0)
        self.declare_parameter('sign_j2', -1.0)
        self.declare_parameter('offset_j1_deg', 0.0)
        self.declare_parameter('offset_j2_deg', 0.0)
        self.declare_parameter('gear_radius_m', 0.020)  # servo pinion, same as the firmware IK
        self.declare_parameter('z_min_m', -0.0628)       # URDF limit of joint_prismatica
        self.declare_parameter('servo_max_dps', 300.0)   # servo speed model, 0 = jump instantly
        self.declare_parameter('timeout_s', 1.0)          # warn if the Pico stops publishing

        # Read once here and again on every 'ros2 param set' (live tuning)
        self._cfg = {n: float(self.get_parameter(n).value) for n in
                     ('sign_j1', 'sign_j2', 'offset_j1_deg', 'offset_j2_deg', 'gear_radius_m',
                      'z_min_m', 'servo_max_dps', 'timeout_s')}
        self.add_on_set_parameters_callback(self._on_params)

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._mk_pub = self.create_publisher(MarkerArray, 'twin_markers', 10)
        self.create_subscription(Float32MultiArray, 'scara_joints', self._on_joints,
                                 qos_profile_sensor_data)  # best effort, matches the Pico
        self.create_timer(0.5, self._check_link)

        self._last_rx = None
        self._homed = None
        self._servo_shown = None  # servo angle shown in RViz (deg)
        self._vacuum = None
        self.get_logger().info('Waiting for /scara_joints from the Pico...')

    def _on_params(self, params):
        for prm in params:
            if prm.name in self._cfg:
                self._cfg[prm.name] = float(prm.value)
                self.get_logger().info(f'{prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    def _servo_model(self, cmd_deg, dt):
        # Move the shown angle towards the commanded one at most servo_max_dps
        if self._servo_shown is None or self._cfg['servo_max_dps'] <= 0.0:
            self._servo_shown = cmd_deg
        else:
            step = self._cfg['servo_max_dps'] * min(dt, 0.2)  # cap dt after a gap in the data
            err = cmd_deg - self._servo_shown
            self._servo_shown += max(-step, min(step, err))
        return self._servo_shown

    def _on_joints(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Bad /scara_joints message ({len(msg.data)} values)')
            return
        j1_deg, j2_deg, servo_deg, homed = msg.data[:4]
        vacuum = (msg.data[4] > 0.5) if len(msg.data) >= 5 else None  # old firmware sends 4

        now = self.get_clock().now()
        dt = 0.05 if self._last_rx is None else (now - self._last_rx).nanoseconds * 1e-9
        self._last_rx = now

        homed = homed > 0.5
        if homed != self._homed:
            if homed:
                self.get_logger().info('Robot homed: twin pose is referenced')
            else:
                self.get_logger().warn('Robot NOT homed: J1/J2 are relative to power-on, run HOME')
            self._homed = homed

        q1 = math.radians(self._cfg['sign_j1'] * j1_deg + self._cfg['offset_j1_deg'])
        q2 = math.radians(self._cfg['sign_j2'] * j2_deg + self._cfg['offset_j2_deg'])
        servo = self._servo_model(float(servo_deg), dt)
        q3 = -math.radians(servo) * self._cfg['gear_radius_m']  # 0 deg = Z top, down is negative
        q3 = max(self._cfg['z_min_m'], min(0.0, q3))

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['joint1', 'joint2', 'joint_prismatica']
        js.position = [q1, q2, q3]
        self._pub.publish(js)

        if vacuum != self._vacuum and vacuum is not None:
            self.get_logger().info('Vacuum ON' if vacuum else 'Vacuum OFF')
        self._vacuum = vacuum
        self._publish_vacuum(vacuum)

    def _publish_vacuum(self, vacuum):
        disc = Marker()
        disc.header.frame_id = 'tool_tip'
        # stamp = 0: RViz uses the latest TF (with the same stamp as the TF it flickers)
        disc.ns, disc.id = 'vacuum', 0
        disc.type = Marker.CYLINDER
        disc.pose.orientation.w = 1.0
        disc.scale.z = 0.004
        text = Marker()
        text.header = disc.header
        text.ns, text.id = 'vacuum', 1
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = -0.02
        text.pose.orientation.w = 1.0
        text.scale.z = 0.015
        text.text = 'VACUUM ON'
        text.color.r, text.color.g, text.color.b, text.color.a = VACUUM_ON_RGBA

        if vacuum is None:  # no vacuum data, hide everything
            disc.action = text.action = Marker.DELETE
        elif vacuum:
            disc.action = text.action = Marker.ADD
            disc.scale.x = disc.scale.y = 0.032  # bigger when ON
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = VACUUM_ON_RGBA
        else:
            disc.action, text.action = Marker.ADD, Marker.DELETE
            disc.scale.x = disc.scale.y = 0.020
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = VACUUM_OFF_RGBA
        self._mk_pub.publish(MarkerArray(markers=[disc, text]))

    def _check_link(self):
        if self._last_rx is None:
            return
        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if age > self._cfg['timeout_s']:
            self.get_logger().warn(f'No data from the Pico for {age:.1f} s (agent/USB?)',
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = JointBridge()
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
