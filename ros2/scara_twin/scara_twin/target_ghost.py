#!/usr/bin/env python3
"""
Ghost of the IK target for the SCARA digital twin.
Reads the targets from /scara_telemetry and publishes /ghost/joint_states for a
second robot_state_publisher (frame_prefix ghost/), drawn transparent in RViz.

  /scara_telemetry (String) = state,pcbs,m1,m2,theta1,theta2,theta_prism,x,y,z
  /ghost/joint_states       = target pose while moving, actual pose otherwise
  /ghost/markers            = target sphere + joint error text
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

STATE_NAMES = {0: 'INIT', 1: 'IDLE', 2: 'MANUAL', 3: 'HOMING', 4: 'GO TO',
               5: 'PICK_CASE', 6: 'PLACE_CASE', 7: 'PICK_PCB', 8: 'INSERT_PCB'}
STATE_HOMING = 3
MOVING_STATES = (4, 5, 6, 7, 8)  # states that use goToPoint (theta1/theta2 are targets)
INVALID_DEG = 180.0              # firmware marker for an unreachable IK point
JOINTS = ['joint1', 'joint2', 'joint_prismatica']


class TargetGhost(Node):
    def __init__(self):
        super().__init__('scara_target_ghost')

        # Same conversion as joint_bridge (launch passes the same values to both)
        self.declare_parameter('sign_j1', 1.0)
        self.declare_parameter('sign_j2', 1.0)
        self.declare_parameter('offset_j1_deg', 0.0)
        self.declare_parameter('offset_j2_deg', 0.0)
        self.declare_parameter('gear_radius_m', 0.020)
        self.declare_parameter('z_min_m', -0.0628)
        self.declare_parameter('timeout_s', 1.0)  # hide the ghost if telemetry stops

        # Read once here and again on every 'ros2 param set' (live tuning)
        self._cfg = {n: float(self.get_parameter(n).value) for n in
                     ('sign_j1', 'sign_j2', 'offset_j1_deg', 'offset_j2_deg', 'gear_radius_m',
                      'z_min_m', 'timeout_s')}
        self.add_on_set_parameters_callback(self._on_params)

        self._pub = self.create_publisher(JointState, 'ghost/joint_states', 10)
        self._mk_pub = self.create_publisher(MarkerArray, 'ghost/markers', 10)
        self.create_subscription(String, 'scara_telemetry', self._on_telemetry,
                                 qos_profile_sensor_data)  # best effort, matches the Pico
        self.create_subscription(JointState, 'joint_states', self._on_actual, 10)
        self.create_timer(0.05, self._update)  # 20 Hz

        self._actual = None      # [q1, q2, q3] from joint_bridge
        self._target = None      # [q1, q2, q3] from telemetry, None = no target
        self._state = None
        self._last_tel = None
        self._shown = False

    def _on_params(self, params):
        for prm in params:
            if prm.name in self._cfg:
                self._cfg[prm.name] = float(prm.value)
                self.get_logger().info(f'{prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    def _to_q(self, j1_deg, j2_deg, servo_deg):
        q1 = math.radians(self._cfg['sign_j1'] * j1_deg + self._cfg['offset_j1_deg'])
        q2 = math.radians(self._cfg['sign_j2'] * j2_deg + self._cfg['offset_j2_deg'])
        q3 = max(self._cfg['z_min_m'], min(0.0, -math.radians(servo_deg) * self._cfg['gear_radius_m']))
        return [q1, q2, q3]

    def _on_actual(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self._actual = [pos[j] for j in JOINTS]
            if self._target is None:  # no target: follow the real robot right away (no lag)
                self._publish_pose(self._actual, msg.header.stamp)

    def _on_telemetry(self, msg: String):
        f = msg.data.split(',')
        try:
            state = int(f[0])
            th1, th2, thp = float(f[4]), float(f[5]), float(f[6])
        except (IndexError, ValueError):
            self.get_logger().warn(f'Bad telemetry: "{msg.data}"', throttle_duration_sec=5.0)
            return
        self._last_tel = self.get_clock().now()
        if state != self._state:
            self.get_logger().info(f'State: {STATE_NAMES.get(state, state)}')
            self._state = state

        valid = all(math.isfinite(v) for v in (th1, th2, thp)) and \
            th1 != INVALID_DEG and th2 != INVALID_DEG
        if state in MOVING_STATES and valid:
            self._target = self._to_q(th1, th2, thp)
        elif state == STATE_HOMING and self._actual is not None:
            self._target = [0.0, 0.0, self._actual[2]]  # HOME = arms straight, URDF zero
        else:
            self._target = None

    def _update(self):
        if self._last_tel is not None:
            age = (self.get_clock().now() - self._last_tel).nanoseconds * 1e-9
            if age > self._cfg['timeout_s']:
                self._target = None

        if self._target is not None:  # without target, _on_actual publishes the pose
            self._publish_pose(self._target, self.get_clock().now().to_msg())
        elif self._shown and self._actual is not None:  # target just ended: back to the robot
            self._publish_pose(self._actual, self.get_clock().now().to_msg())
        self._publish_markers()

    def _publish_pose(self, pose, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = JOINTS
        js.position = list(pose)
        self._pub.publish(js)

    def _publish_markers(self):
        sphere = Marker()
        sphere.header.frame_id = 'ghost/tool_tip'
        # stamp = 0: RViz uses the latest TF (with the same stamp as the TF it flickers)
        sphere.ns, sphere.id = 'ghost', 0
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.4, 0.0, 0.9
        text = Marker()
        text.header = sphere.header
        text.ns, text.id = 'ghost', 1
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = 0.09  # above the ghost tool
        text.pose.orientation.w = 1.0
        text.scale.z = 0.014
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.7, 0.2, 1.0

        if self._target is None:
            if not self._shown:
                return  # already hidden
            sphere.action = text.action = Marker.DELETE
            self._shown = False
        else:
            sphere.action = text.action = Marker.ADD
            text.text = STATE_NAMES.get(self._state, str(self._state))
            if self._actual is not None:
                e = [t - a for t, a in zip(self._target, self._actual)]
                text.text += (f'\nerr J1 {math.degrees(e[0]):+.1f} deg  J2 {math.degrees(e[1]):+.1f} deg'
                              f'  Z {e[2] * 1000:+.1f} mm')
            self._shown = True
        self._mk_pub.publish(MarkerArray(markers=[sphere, text]))


def main(args=None):
    rclpy.init(args=args)
    node = TargetGhost()
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
