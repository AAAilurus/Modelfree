"""
dual_hw_freemodel.launch.py  —  Hardware entry point for the model-free pipeline

Identical to dual_hw.launch.py but with enable_leader_commands set to true so
that fm_leader_hw can physically move the leader arm via position commands.

The relay_node is excluded here because fm_leader_hw / fm_follower_hw /
fm_demo_hw manage the leader → follower command flow themselves.

Usage:
    ros2 launch so100_hardware_bringup dual_hw_freemodel.launch.py

Override any argument on the command line, e.g.:
    ros2 launch so100_hardware_bringup dual_hw_freemodel.launch.py \\
        leader_port:=/dev/ttyUSB0 \\
        follower_port:=/dev/ttyUSB1
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_data_dir() -> str:
    env = os.environ.get('HARDWARE_DATA_DIR', '')
    if env:
        return env
    cwd_candidate = Path.cwd() / 'hardware' / 'data'
    if cwd_candidate.is_dir():
        return str(cwd_candidate)
    launch_dir = Path(__file__).resolve().parent
    dev_candidate = launch_dir.parent / 'data'
    if dev_candidate.is_dir():
        return str(dev_candidate)
    return str(Path.home() / 'so100_hardware_data')


def generate_launch_description():
    default_data = _default_data_dir()

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'leader_port',
            default_value='/dev/serial/by-id/'
                          'usb-1a86_USB_Single_Serial_5AAF218344-if00',
            description='Stable USB serial path for the leader arm',
        ),
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
            'data_dir',
            default_value=default_data,
            description='Directory where CSV log files are saved',
        ),
    ]

    # ------------------------------------------------------------------
    # 1. Leader hardware node  (enable_commands=true for freemodel pipeline)
    # ------------------------------------------------------------------
    leader_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'leader_hw_node',
        name       = 'leader_hw_node',
        output     = 'screen',
        parameters = [{
            'serial_port':     LaunchConfiguration('leader_port'),
            'baud_rate':       LaunchConfiguration('baud_rate'),
            'rate_hz':         LaunchConfiguration('rate_hz'),
            'servo_id_j1':     LaunchConfiguration('leader_servo_j1'),
            'servo_id_j2':     LaunchConfiguration('leader_servo_j2'),
            'namespace':       'so100',
            'enable_commands': True,   # fm_leader_hw physically commands the leader arm
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
    # 3. CSV logger node  (no relay_node — freemodel nodes handle commands)
    # ------------------------------------------------------------------
    logger_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'csv_logger_node',
        name       = 'csv_logger_node',
        output     = 'screen',
        parameters = [{
            'leader_js_topic':   '/so100/joint_states',
            'follower_js_topic': '/so101/joint_states',
            'joint_name_j1':     'Shoulder_Pitch',
            'joint_name_j2':     'Elbow',
            'data_dir':          LaunchConfiguration('data_dir'),
            'rate_hz':           LaunchConfiguration('rate_hz'),
        }],
    )

    return LaunchDescription(args + [
        leader_node,
        follower_node,
        logger_node,
    ])
