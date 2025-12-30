from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os 


def generate_launch_description():
    pkg_uav_sim = get_package_share_directory("uav_sim")

    world_file = os.path.join(pkg_uav_sim, "worlds", "my_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "drone_box",
            "-file", os.path.join(pkg_uav_sim, "models", "drone_box.sdf")],
        output="screen"
    )

    uav_controller = Node(
        package="uav_sim",
        executable="uav_controller",
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench"
        ],
        # remappings=[
        #     ('/drone/thrust', '/world/empty/wrench')
        # ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
        uav_controller
    ])
