import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_sim = get_package_share_directory('uav_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'drone_box', 
                   '-file', os.path.join(pkg_uav_sim, 'models', 'drone_box.sdf')],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/drone_box/link/base_link/wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench'
        ],
        remappings=[
            ('/model/drone_box/link/base_link/wrench', '/drone/thrust')
        ],
        output='screen'
    )

    controller = Node(
        package='uav_sim',
        executable='drone_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
        controller
    ])
