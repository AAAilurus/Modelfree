"""
leader_hw.launch.py

Launch ONLY the leader arm hardware node.

The leader arm reads joint angles from its Feetech servos and publishes:
    /so100/joint_states  (sensor_msgs/JointState)

Usage:
    ros2 launch so100_hardware_bringup leader_hw.launch.py

Optional arguments:
    serial_port:=/dev/serial/by-id/<device>
    baud_rate:=1000000
    rate_hz:=50.0
    servo_id_j1:=1
    servo_id_j2:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Launch arguments (all optional — defaults match hardware_params.yaml)
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/'
                          'usb-1a86_USB_Single_Serial_5AAF218344-if00',
            description='Stable USB serial path for the leader arm',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Feetech servo baud rate',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='Joint-state publish rate (Hz)',
        ),
        DeclareLaunchArgument(
            'servo_id_j1',
            default_value='1',
            description='Feetech servo ID for Shoulder_Pitch',
        ),
        DeclareLaunchArgument(
            'servo_id_j2',
            default_value='2',
            description='Feetech servo ID for Elbow',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='so100',
            description='ROS2 namespace for the leader arm',
        ),
    ]

    # ------------------------------------------------------------------
    # Leader hardware node
    # ------------------------------------------------------------------
    leader_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'leader_hw_node',
        name       = 'leader_hw_node',
        output     = 'screen',
        parameters = [{
            'serial_port':   LaunchConfiguration('serial_port'),
            'baud_rate':     LaunchConfiguration('baud_rate'),
            'rate_hz':       LaunchConfiguration('rate_hz'),
            'servo_id_j1':   LaunchConfiguration('servo_id_j1'),
            'servo_id_j2':   LaunchConfiguration('servo_id_j2'),
            'namespace':     LaunchConfiguration('namespace'),
        }],
    )

    return LaunchDescription(args + [leader_node])
