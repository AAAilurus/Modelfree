#!/usr/bin/env python3
"""
Leader Hardware Node — SO-100 2-DOF

Reads Shoulder_Pitch and Elbow joint angles from the leader arm's
Feetech STS3215 servos over serial and publishes them as:

    /so100/joint_states   (sensor_msgs/JointState)

This mirrors the topic published by the Gazebo joint_state_broadcaster
so that all existing model-free / IOC nodes work unchanged.

Parameters (all ROS2 declare_parameter):
  serial_port   – serial device for leader arm (stable by-id path)
  baud_rate     – serial baud rate (default 1 000 000)
  servo_id_j1   – Feetech servo ID for Shoulder_Pitch (default 1)
  servo_id_j2   – Feetech servo ID for Elbow          (default 2)
  joint_name_j1 – ROS joint name for servo 1          (default 'Shoulder_Pitch')
  joint_name_j2 – ROS joint name for servo 2          (default 'Elbow')
  namespace     – ROS namespace prefix                  (default 'so100')
  rate_hz       – polling rate in Hz                   (default 50)
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# The driver lives alongside this script
import os
sys.path.insert(0, os.path.dirname(__file__))
from feetech_driver import FeetechDriver  # noqa: E402


class LeaderHwNode(Node):
    """Publishes real-time joint states from the physical leader arm."""

    def __init__(self):
        super().__init__('leader_hw_node')

        # ------------------------------------------------------------------
        # ROS2 parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF218344-if00',
        )
        self.declare_parameter('baud_rate',     1_000_000)
        self.declare_parameter('servo_id_j1',   1)          # Shoulder_Pitch
        self.declare_parameter('servo_id_j2',   2)          # Elbow
        self.declare_parameter('joint_name_j1', 'Shoulder_Pitch')
        self.declare_parameter('joint_name_j2', 'Elbow')
        self.declare_parameter('namespace',     'so100')
        self.declare_parameter('rate_hz',       50.0)

        port      = str(self.get_parameter('serial_port').value)
        baud      = int(self.get_parameter('baud_rate').value)
        self.id1  = int(self.get_parameter('servo_id_j1').value)
        self.id2  = int(self.get_parameter('servo_id_j2').value)
        self.j1   = str(self.get_parameter('joint_name_j1').value)
        self.j2   = str(self.get_parameter('joint_name_j2').value)
        ns        = str(self.get_parameter('namespace').value)
        rate_hz   = float(self.get_parameter('rate_hz').value)

        # ------------------------------------------------------------------
        # Serial driver
        # ------------------------------------------------------------------
        self.driver = FeetechDriver(port, baud)
        try:
            self.driver.open()
            self.get_logger().info(
                f"[leader] Serial opened: {port} @ {baud} baud"
            )
        except Exception as exc:
            self.get_logger().fatal(
                f"[leader] Failed to open serial port {port}: {exc}"
            )
            raise

        # Verify servos are responding
        for sid, name in [(self.id1, self.j1), (self.id2, self.j2)]:
            if self.driver.ping(sid):
                self.get_logger().info(
                    f"[leader] Servo ID {sid} ({name}) responded to ping ✓"
                )
            else:
                self.get_logger().warning(
                    f"[leader] Servo ID {sid} ({name}) did NOT respond to ping"
                )

        # Disable torque so the leader arm can be moved freely by hand
        for sid, name in [(self.id1, self.j1), (self.id2, self.j2)]:
            self.driver.disable_torque(sid)
            self.get_logger().info(
                f"[leader] Servo ID {sid} ({name}) torque disabled (free to move by hand)"
            )

        # ------------------------------------------------------------------
        # ROS publisher
        # ------------------------------------------------------------------
        self.pub = self.create_publisher(
            JointState,
            f'/{ns}/joint_states',
            10,
        )

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"[leader] Publishing /{ns}/joint_states at {rate_hz} Hz"
        )

    def _tick(self):
        """Read servo positions and publish JointState."""
        pos1, vel1 = self.driver.read_pos_and_speed(self.id1)
        pos2, vel2 = self.driver.read_pos_and_speed(self.id2)

        if pos1 is None or pos2 is None:
            # Skip this tick if a read failed to avoid publishing stale data
            return

        msg              = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = [self.j1, self.j2]
        msg.position     = [pos1, pos2]
        msg.velocity     = [vel1, vel2]
        msg.effort       = [0.0, 0.0]
        self.pub.publish(msg)

    def destroy_node(self):
        """Clean up: close serial port on shutdown."""
        self.get_logger().info('[leader] Shutting down, closing serial port.')
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderHwNode()
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
