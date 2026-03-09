#!/usr/bin/env python3
"""
Leader-Follower Relay Node

Reads the leader arm's joint states and immediately forwards them as
position commands to the follower arm:

    /so100/joint_states  →  relay  →  /so101/arm_position_controller/commands

This creates a direct kinematic mirroring: moving the leader by hand
causes the follower to track the same angles.

Parameters:
  leader_js_topic    – joint_states topic of the leader (default /so100/joint_states)
  follower_cmd_topic – command topic of the follower
                       (default /so101/arm_position_controller/commands)
  joint_name_j1      – first joint name  (default 'Shoulder_Pitch')
  joint_name_j2      – second joint name (default 'Elbow')
  scale              – optional scaling factor applied to angles (default 1.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RelayNode(Node):
    """
    Mirrors leader joint positions to follower position commands.

    The relay is purely reactive — it publishes one command for every
    joint-state message it receives from the leader.
    """

    def __init__(self):
        super().__init__('relay_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            'leader_js_topic',
            '/so100/joint_states',
        )
        self.declare_parameter(
            'follower_cmd_topic',
            '/so101/arm_position_controller/commands',
        )
        self.declare_parameter('joint_name_j1', 'Shoulder_Pitch')
        self.declare_parameter('joint_name_j2', 'Elbow')
        self.declare_parameter('scale', 1.0)

        leader_topic   = self.get_parameter('leader_js_topic').value
        follower_topic = self.get_parameter('follower_cmd_topic').value
        self.j1        = self.get_parameter('joint_name_j1').value
        self.j2        = self.get_parameter('joint_name_j2').value
        self.scale     = float(self.get_parameter('scale').value)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.sub = self.create_subscription(
            JointState,
            leader_topic,
            self._js_cb,
            10,
        )
        self.pub = self.create_publisher(
            Float64MultiArray,
            follower_topic,
            10,
        )

        self.get_logger().info(
            f"[relay] {leader_topic}  →  {follower_topic}  (scale={self.scale})"
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        """Forward leader positions as follower commands."""
        if self.j1 not in msg.name or self.j2 not in msg.name:
            return  # ignore messages that don't include our joints

        idx1 = msg.name.index(self.j1)
        idx2 = msg.name.index(self.j2)

        q1 = msg.position[idx1] * self.scale
        q2 = msg.position[idx2] * self.scale

        cmd      = Float64MultiArray()
        cmd.data = [q1, q2]
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
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
