from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def load_urdf(pkg_name, urdf_rel):
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_rel)
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    gz_content = urdf_content.replace(
        'package://so_100_arm/models/so_100_arm_5dof/meshes',
        'model://so_100_arm_5dof/meshes'
    )
    return ParameterValue(urdf_content, value_type=str), ParameterValue(gz_content, value_type=str), pkg_share

def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        so100_desc, so100_gz, so100_share = load_urdf('so100_2dof_bringup', 'so_100_arm_2dof.urdf')
        so101_desc, so101_gz, so101_share = load_urdf('so101_2dof_bringup', 'so_100_arm_2dof.urdf')

        # Make sure Gazebo can find models/
        model_path_100 = os.path.join(os.path.dirname(os.path.dirname(so100_share)), 'models')
        model_path_101 = os.path.join(os.path.dirname(os.path.dirname(so101_share)), 'models')
        gz_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        extra = ":".join([p for p in [model_path_100, model_path_101] if p])
        gz_resource = (gz_resource + ":" + extra) if gz_resource else extra

        gz_env = {
            'GZ_SIM_RESOURCE_PATH': gz_resource,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib:' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH',''),
            'QT_X11_NO_MITSHM': os.environ.get('QT_X11_NO_MITSHM','1'),
        }

        # GUI ON (do NOT use -s, do NOT set GZ_GUI=0)
        gz = ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen',
            additional_env=gz_env
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            additional_env=gz_env
        )

        rsp_so100 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='so100',
            output='screen',
            parameters=[{'robot_description': so100_desc}]
        )

        rsp_so101 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='so101',
            output='screen',
            parameters=[{'robot_description': so101_desc}]
        )

        spawn_so100 = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_so100',
            output='screen',
            arguments=[
                '-string', so100_gz.value,
                '-name', 'so100_2dof',
                '-allow_renaming', 'true',
                '-x', '0.0', '-y', '0.0', '-z', '0.0'
            ],
            additional_env=gz_env
        )

        spawn_so101 = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_so101',
            output='screen',
            arguments=[
                '-string', so101_gz.value,
                '-name', 'so101_2dof',
                '-allow_renaming', 'true',
                '-x', '0.6', '-y', '0.0', '-z', '0.0'
            ],
            additional_env=gz_env
        )

        jsb_so100 = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/so100/controller_manager']
        )
        apc_so100 = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['arm_position_controller', '--controller-manager', '/so100/controller_manager']
        )

        jsb_so101 = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/so101/controller_manager']
        )
        apc_so101 = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['arm_position_controller', '--controller-manager', '/so101/controller_manager']
        )

        # Spawn order: spawn -> controllers
        return [
            gz, bridge,
            rsp_so100, rsp_so101,
            spawn_so100, spawn_so101,

            RegisterEventHandler(OnProcessExit(target_action=spawn_so100, on_exit=[jsb_so100])),
            RegisterEventHandler(OnProcessExit(target_action=jsb_so100, on_exit=[apc_so100])),

            RegisterEventHandler(OnProcessExit(target_action=spawn_so101, on_exit=[jsb_so101])),
            RegisterEventHandler(OnProcessExit(target_action=jsb_so101, on_exit=[apc_so101])),
        ]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
