#!/usr/bin/env python3
"""
CSV Logger Node

Subscribes to both leader and follower joint states and writes a
timestamped CSV file to the hardware/data/ directory.

Output columns:
    time, leader_joint1, leader_joint2, follower_joint1, follower_joint2

Filename format:
    run_YYYYMMDD_HHMMSS.csv

Parameters:
  leader_js_topic   – (default /so100/joint_states)
  follower_js_topic – (default /so101/joint_states)
  joint_name_j1     – (default 'Shoulder_Pitch')
  joint_name_j2     – (default 'Elbow')
  data_dir          – output directory (default: hardware/data/ relative
                      to this script, or ~/so100_hardware_data as fallback)
  rate_hz           – logging rate in Hz (default 50.0)
"""

import csv
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from sensor_msgs.msg import JointState


def _default_data_dir() -> str:
    """Resolve hardware/data/ relative to this script's location."""
    # When installed to lib/so100_hardware_bringup/, parent dirs differ from
    # the source tree, so also accept an env-var override.
    env = os.environ.get('HARDWARE_DATA_DIR', '')
    if env:
        return env
    # Development: script lives at hardware/nodes/csv_logger_node.py
    script_dir = Path(__file__).resolve().parent          # hardware/nodes/
    candidate  = script_dir.parent / 'data'               # hardware/data/
    if candidate.exists():
        return str(candidate)
    # Fallback: home directory
    return str(Path.home() / 'so100_hardware_data')


class CsvLoggerNode(Node):
    """
    Records leader and follower joint states to a timestamped CSV file.

    The file is opened when the first pair of joint-state messages arrives
    and closed gracefully on shutdown.
    """

    def __init__(self):
        super().__init__('csv_logger_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('leader_js_topic',   '/so100/joint_states')
        self.declare_parameter('follower_js_topic',  '/so101/joint_states')
        self.declare_parameter('joint_name_j1',      'Shoulder_Pitch')
        self.declare_parameter('joint_name_j2',      'Elbow')
        self.declare_parameter('data_dir',           _default_data_dir())
        self.declare_parameter('rate_hz',            50.0)

        leader_topic   = self.get_parameter('leader_js_topic').value
        follower_topic = self.get_parameter('follower_js_topic').value
        self.j1        = self.get_parameter('joint_name_j1').value
        self.j2        = self.get_parameter('joint_name_j2').value
        data_dir       = Path(self.get_parameter('data_dir').value)
        rate_hz        = float(self.get_parameter('rate_hz').value)

        # ------------------------------------------------------------------
        # State buffers
        # ------------------------------------------------------------------
        self._leader_js   = None
        self._follower_js = None

        # ------------------------------------------------------------------
        # CSV file setup
        # ------------------------------------------------------------------
        data_dir.mkdir(parents=True, exist_ok=True)
        timestamp  = time.strftime('%Y%m%d_%H%M%S')
        self._path = data_dir / f'run_{timestamp}.csv'
        self._file = open(self._path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            'time',
            'leader_joint1', 'leader_joint2',
            'follower_joint1', 'follower_joint2',
        ])
        self._t0 = time.time()
        self.get_logger().info(f'[logger] CSV → {self._path}')

        # ------------------------------------------------------------------
        # ROS subscriptions  (BEST_EFFORT to match hardware node QoS)
        # ------------------------------------------------------------------
        js_qos = QoSProfile(
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(JointState, leader_topic,   self._leader_cb,   js_qos)
        self.create_subscription(JointState, follower_topic, self._follower_cb, js_qos)

        # ------------------------------------------------------------------
        # Logging timer
        # ------------------------------------------------------------------
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'[logger] Logging {leader_topic} + {follower_topic} '
            f'at {rate_hz} Hz'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_joints(self, msg: JointState):
        """Extract (q1, q2) from a JointState message by name."""
        if self.j1 not in msg.name or self.j2 not in msg.name:
            return None, None
        i1 = msg.name.index(self.j1)
        i2 = msg.name.index(self.j2)
        q1 = msg.position[i1] if i1 < len(msg.position) else None
        q2 = msg.position[i2] if i2 < len(msg.position) else None
        return q1, q2

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _leader_cb(self, msg: JointState):
        self._leader_js = msg

    def _follower_cb(self, msg: JointState):
        self._follower_js = msg

    def _tick(self):
        """Write one CSV row when both leader and follower data are available."""
        if self._leader_js is None or self._follower_js is None:
            return

        lq1, lq2 = self._get_joints(self._leader_js)
        fq1, fq2 = self._get_joints(self._follower_js)

        if None in (lq1, lq2, fq1, fq2):
            return

        t = time.time() - self._t0
        self._writer.writerow([
            f'{t:.4f}',
            f'{lq1:.6f}', f'{lq2:.6f}',
            f'{fq1:.6f}', f'{fq2:.6f}',
        ])
        # Flush periodically so data is not lost if process is killed
        self._file.flush()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Close the CSV file cleanly on shutdown."""
        if self._file and not self._file.closed:
            self._file.flush()
            self._file.close()
            self.get_logger().info(f'[logger] CSV saved: {self._path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
