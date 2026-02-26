from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def get_robot_description(context, *args, **kwargs):
    pkg_share = FindPackageShare('so100_2dof_bringup').find('so100_2dof_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'so_100_arm_2dof.urdf')
    controller_path = os.path.join(pkg_share, 'config', 'controllers_2dof.yaml')

    with open(urdf_path, 'r') as file:
        urdf_content = file.read()
        gazebo_urdf_content = urdf_content.replace(
            'package://so_100_arm/models/so_100_arm_5dof/meshes',
            'model://so_100_arm_5dof/meshes')
        return {
            'robot_description': ParameterValue(urdf_content, value_type=str),
            'gazebo_description': ParameterValue(gazebo_urdf_content, value_type=str),
            'controller_path': controller_path
        }

def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='5',
        description='DOF configuration'
    )

    pkg_share = FindPackageShare('so100_2dof_bringup').find('so100_2dof_bringup')
    model_path = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 'models')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{model_path}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = model_path

    gz_env = {
        'GZ_SIM_RESOURCE_PATH':      os.environ['GZ_SIM_RESOURCE_PATH'],
        'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib:' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        'GZ_PARTITION':              os.environ.get('GZ_PARTITION', ''),
        'IGN_PARTITION':             os.environ.get('IGN_PARTITION', ''),
        'GZ_IP':                     os.environ.get('GZ_IP', ''),
        'IGN_IP':                    os.environ.get('IGN_IP', ''),
        'GZ_GUI':                    '0',
        'GZ_SIM_HEADLESS_RENDERING': '1',
    }

    def launch_setup(context, *args, **kwargs):
        descriptions = get_robot_description(context)

        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=[
                '-string', descriptions['gazebo_description'].value,
                '-name', 'so_100_arm',
                '-allow_renaming', 'true',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            additional_env=gz_env,
            output='screen'
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/so100/controller_manager'],
            output='screen'
        )

        arm_position_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_position_controller',
                       '--controller-manager', '/so100/controller_manager'],
            output='screen'
        )

        nodes = [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace='so100',
                output='screen',
                parameters=[{'robot_description': descriptions['robot_description']}]
            ),

            ExecuteProcess(
                cmd=['gz', 'sim', '--force-version', '8', '-r', '-s', 'empty.sdf'],
                output='screen',
                additional_env=gz_env
            ),

            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge',
                parameters=[{
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }],
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                additional_env=gz_env,
            ),

            spawn_robot,

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner]
                )
            ),

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_position_controller_spawner]
                )
            ),
        ]
        return nodes

    return LaunchDescription([
        dof_arg,
        OpaqueFunction(function=launch_setup)
    ])
