"""
follower_hw.launch.py

Launch ONLY the follower arm hardware node.

The follower arm:
  - Subscribes to /so101/arm_position_controller/commands
    (std_msgs/Float64MultiArray)
  - Sends position commands to its Feetech servos
  - Publishes /so101/joint_states (sensor_msgs/JointState)

Usage:
    ros2 launch so100_hardware_bringup follower_hw.launch.py

Optional arguments:
    serial_port:=/dev/serial/by-id/<device>
    baud_rate:=1000000
    rate_hz:=50.0
    goal_speed:=200
    servo_id_j1:=1
    servo_id_j2:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'serial_port',
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
            description='Control and publish rate (Hz)',
        ),
        DeclareLaunchArgument(
            'goal_speed',
            default_value='200',
            description='Servo goal speed (0 = max speed, 1-32767 = limited)',
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
            default_value='so101',
            description='ROS2 namespace for the follower arm',
        ),
    ]

    # ------------------------------------------------------------------
    # Follower hardware node
    # ------------------------------------------------------------------
    follower_node = Node(
        package    = 'so100_hardware_bringup',
        executable = 'follower_hw_node',
        name       = 'follower_hw_node',
        output     = 'screen',
        parameters = [{
            'serial_port':   LaunchConfiguration('serial_port'),
            'baud_rate':     LaunchConfiguration('baud_rate'),
            'rate_hz':       LaunchConfiguration('rate_hz'),
            'goal_speed':    LaunchConfiguration('goal_speed'),
            'servo_id_j1':   LaunchConfiguration('servo_id_j1'),
            'servo_id_j2':   LaunchConfiguration('servo_id_j2'),
            'namespace':     LaunchConfiguration('namespace'),
        }],
    )

    return LaunchDescription(args + [follower_node])
