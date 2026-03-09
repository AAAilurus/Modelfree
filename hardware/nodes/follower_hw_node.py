#!/usr/bin/env python3
"""
Follower Hardware Node — SO-101 2-DOF

Bi-directional bridge between the follower arm's Feetech STS3215 servos
and ROS2 topics.

  Subscribes:  /so101/arm_position_controller/commands
               (std_msgs/Float64MultiArray — [Shoulder_Pitch, Elbow])

  Publishes:   /so101/joint_states
               (sensor_msgs/JointState)

This mirrors the topic interface used by the Gazebo arm_position_controller
so that all existing model-free / IOC / relay nodes work unchanged.

Parameters:
  serial_port   – serial device for follower arm
  baud_rate     – serial baud rate (default 1 000 000)
  servo_id_j1   – Feetech servo ID for Shoulder_Pitch (default 1)
  servo_id_j2   – Feetech servo ID for Elbow          (default 2)
  joint_name_j1 – ROS joint name for servo 1
  joint_name_j2 – ROS joint name for servo 2
  namespace     – ROS namespace prefix (default 'so101')
  rate_hz       – position-read / publish rate in Hz  (default 50)
  goal_speed    – servo speed limit (0 = max, 1-32767 = limited)
"""

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(__file__))
from feetech_driver import FeetechDriver  # noqa: E402


class FollowerHwNode(Node):
    """
    Drives the physical follower arm and publishes its joint states.

    Received Float64MultiArray commands are written to the servos on the
    same control-loop tick (no internal integrator — pure position control).
    """

    def __init__(self):
        super().__init__('follower_hw_node')

        # ------------------------------------------------------------------
        # ROS2 parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219983-if00',
        )
        self.declare_parameter('baud_rate',     1_000_000)
        self.declare_parameter('servo_id_j1',   1)
        self.declare_parameter('servo_id_j2',   2)
        self.declare_parameter('joint_name_j1', 'Shoulder_Pitch')
        self.declare_parameter('joint_name_j2', 'Elbow')
        self.declare_parameter('namespace',     'so101')
        self.declare_parameter('rate_hz',       50.0)
        self.declare_parameter('goal_speed',    200)   # 0 = max speed

        port       = str(self.get_parameter('serial_port').value)
        baud       = int(self.get_parameter('baud_rate').value)
        self.id1   = int(self.get_parameter('servo_id_j1').value)
        self.id2   = int(self.get_parameter('servo_id_j2').value)
        self.j1    = str(self.get_parameter('joint_name_j1').value)
        self.j2    = str(self.get_parameter('joint_name_j2').value)
        ns         = str(self.get_parameter('namespace').value)
        rate_hz    = float(self.get_parameter('rate_hz').value)
        self.speed = int(self.get_parameter('goal_speed').value)

        # Pending command buffer (updated by subscription callback)
        self._pending_cmd = None

        # ------------------------------------------------------------------
        # Serial driver
        # ------------------------------------------------------------------
        self.driver = FeetechDriver(port, baud)
        try:
            self.driver.open()
            self.get_logger().info(
                f"[follower] Serial opened: {port} @ {baud} baud"
            )
        except Exception as exc:
            self.get_logger().fatal(
                f"[follower] Failed to open serial port {port}: {exc}"
            )
            raise

        # Enable torque on both servos
        for sid, name in [(self.id1, self.j1), (self.id2, self.j2)]:
            if self.driver.ping(sid):
                self.driver.enable_torque(sid)
                self.get_logger().info(
                    f"[follower] Servo ID {sid} ({name}) torque enabled ✓"
                )
            else:
                self.get_logger().warning(
                    f"[follower] Servo ID {sid} ({name}) did NOT respond to ping"
                )

        # ------------------------------------------------------------------
        # ROS publisher & subscriber
        # ------------------------------------------------------------------
        self.pub = self.create_publisher(
            JointState,
            f'/{ns}/joint_states',
            10,
        )
        self.sub = self.create_subscription(
            Float64MultiArray,
            f'/{ns}/arm_position_controller/commands',
            self._cmd_cb,
            10,
        )

        # ------------------------------------------------------------------
        # Control / publish timer
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"[follower] Listening on /{ns}/arm_position_controller/commands"
        )
        self.get_logger().info(
            f"[follower] Publishing /{ns}/joint_states at {rate_hz} Hz"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Float64MultiArray):
        """Store the most recent position command."""
        if len(msg.data) >= 2:
            self._pending_cmd = (float(msg.data[0]), float(msg.data[1]))

    def _tick(self):
        """
        One control tick:
          1. If a pending command exists, send it to the servos.
          2. Read current positions and publish JointState.
        """
        # Send pending command (Shoulder_Pitch, Elbow)
        if self._pending_cmd is not None:
            q1_cmd, q2_cmd = self._pending_cmd
            self.driver.sync_write_positions(
                [(self.id1, q1_cmd), (self.id2, q2_cmd)],
                speed_raw=self.speed,
            )
            self._pending_cmd = None

        # Read present state and publish
        pos1, vel1 = self.driver.read_pos_and_speed(self.id1)
        pos2, vel2 = self.driver.read_pos_and_speed(self.id2)

        if pos1 is None or pos2 is None:
            return

        msg              = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = [self.j1, self.j2]
        msg.position     = [pos1, pos2]
        msg.velocity     = [vel1, vel2]
        msg.effort       = [0.0, 0.0]
        self.pub.publish(msg)

    def destroy_node(self):
        """Disable torque and close serial on shutdown."""
        self.get_logger().info('[follower] Disabling torque and closing serial.')
        for sid in [self.id1, self.id2]:
            try:
                self.driver.disable_torque(sid)
            except Exception:
                pass
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerHwNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
