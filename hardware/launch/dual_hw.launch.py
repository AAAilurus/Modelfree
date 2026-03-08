"""
dual_hw.launch.py  —  Main hardware entry point

Starts all four hardware components in the correct order:

  1. leader_hw_node    – reads leader arm servos → /so100/joint_states
  2. follower_hw_node  – drives follower arm servos ← /so101/arm_position_controller/commands
  3. relay_node        – /so100/joint_states → /so101/arm_position_controller/commands
  4. csv_logger_node   – logs both joint_states to hardware/data/run_*.csv

Usage:
    ros2 launch so100_hardware_bringup dual_hw.launch.py

Override any argument on the command line, e.g.:
    ros2 launch so100_hardware_bringup dual_hw.launch.py \\
        leader_port:=/dev/ttyUSB0 \\
        follower_port:=/dev/ttyUSB1 \\
        rate_hz:=100.0 \\
        data_dir:=/home/user/Modelfree/hardware/data
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_data_dir() -> str:
    """
    Resolve hardware/data/ relative to this launch file's location.

    When running from the source tree (development), the launch file is at
    hardware/launch/dual_hw.launch.py, so hardware/data/ is one level up.

    When installed (after colcon build), override with the data_dir argument
    or the HARDWARE_DATA_DIR environment variable.
    """
    env = os.environ.get('HARDWARE_DATA_DIR', '')
    if env:
        return env
    launch_dir = Path(__file__).resolve().parent          # hardware/launch/
    candidate  = launch_dir.parent / 'data'               # hardware/data/
    if candidate.exists():
        return str(candidate)
    # After colcon install the launch file moves; fall back to share dir sibling
    share_candidate = launch_dir.parent / 'data'
    return str(share_candidate) if share_candidate.exists() else str(Path.home() / 'so100_hardware_data')


def generate_launch_description():
    default_data = _default_data_dir()

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        # Leader serial device
        DeclareLaunchArgument(
            'leader_port',
            default_value='/dev/serial/by-id/'
                          'usb-1a86_USB_Single_Serial_5AAF218344-if00',
            description='Stable USB serial path for the leader arm',
        ),
        # Follower serial device
        DeclareLaunchArgument(
            'follower_port',
            default_value='/dev/serial/by-id/'
                          'usb-1a86_USB_Single_Serial_5AAF219983-if00',
            description='Stable USB serial path for the follower arm',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Feetech servo baud rate',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='Control and publish rate for all nodes (Hz)',
        ),
        DeclareLaunchArgument(
            'goal_speed',
            default_value='200',
            description='Follower servo speed limit (0=max, 1-32767=limited)',
        ),
        DeclareLaunchArgument(
            'leader_servo_j1',
            default_value='1',
            description='Leader servo ID for Shoulder_Pitch',
        ),
        DeclareLaunchArgument(
            'leader_servo_j2',
            default_value='2',
            description='Leader servo ID for Elbow',
        ),
        DeclareLaunchArgument(
            'follower_servo_j1',
            default_value='1',
            description='Follower servo ID for Shoulder_Pitch',
        ),
        DeclareLaunchArgument(
            'follower_servo_j2',
            default_value='2',
            description='Follower servo ID for Elbow',
        ),
        DeclareLaunchArgument(
            'scale',
            default_value='1.0',
            description='Relay scale factor (1.0 = direct mirror)',
        ),
        DeclareLaunchArgument(
            'data_dir',
            default_value=default_data,
            description='Directory where CSV log files are saved',
        ),
    ]

    # ------------------------------------------------------------------
    # 1. Leader hardware node
    # ------------------------------------------------------------------
    leader_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'leader_hw_node',
        name       = 'leader_hw_node',
        output     = 'screen',
        parameters = [{
            'serial_port': LaunchConfiguration('leader_port'),
            'baud_rate':   LaunchConfiguration('baud_rate'),
            'rate_hz':     LaunchConfiguration('rate_hz'),
            'servo_id_j1': LaunchConfiguration('leader_servo_j1'),
            'servo_id_j2': LaunchConfiguration('leader_servo_j2'),
            'namespace':   'so100',
        }],
    )

    # ------------------------------------------------------------------
    # 2. Follower hardware node
    # ------------------------------------------------------------------
    follower_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'follower_hw_node',
        name       = 'follower_hw_node',
        output     = 'screen',
        parameters = [{
            'serial_port': LaunchConfiguration('follower_port'),
            'baud_rate':   LaunchConfiguration('baud_rate'),
            'rate_hz':     LaunchConfiguration('rate_hz'),
            'goal_speed':  LaunchConfiguration('goal_speed'),
            'servo_id_j1': LaunchConfiguration('follower_servo_j1'),
            'servo_id_j2': LaunchConfiguration('follower_servo_j2'),
            'namespace':   'so101',
        }],
    )

    # ------------------------------------------------------------------
    # 3. Relay node  (leader joints → follower commands)
    # ------------------------------------------------------------------
    relay_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'relay_node',
        name       = 'relay_node',
        output     = 'screen',
        parameters = [{
            'leader_js_topic':    '/so100/joint_states',
            'follower_cmd_topic': '/so101/arm_position_controller/commands',
            'joint_name_j1':      'Shoulder_Pitch',
            'joint_name_j2':      'Elbow',
            'scale':              LaunchConfiguration('scale'),
        }],
    )

    # ------------------------------------------------------------------
    # 4. CSV logger node
    # ------------------------------------------------------------------
    logger_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'csv_logger_node',
        name       = 'csv_logger_node',
        output     = 'screen',
        parameters = [{
            'leader_js_topic':    '/so100/joint_states',
            'follower_js_topic':  '/so101/joint_states',
            'joint_name_j1':      'Shoulder_Pitch',
            'joint_name_j2':      'Elbow',
            'data_dir':           LaunchConfiguration('data_dir'),
            'rate_hz':            LaunchConfiguration('rate_hz'),
        }],
    )

    return LaunchDescription(args + [
        leader_node,
        follower_node,
        relay_node,
        logger_node,
    ])
