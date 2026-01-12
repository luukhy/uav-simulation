from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os 


def generate_launch_description():
    pkg_uav_sim = get_package_share_directory("uav_sim")

    models_path = os.path.join(pkg_uav_sim, "models")
    world_file = os.path.join(pkg_uav_sim, "worlds", "my_world.sdf")

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_drone_box = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "drone_box",
            "-file", os.path.join(pkg_uav_sim, "models", "drone_box.sdf")],
        output="screen"
    )

    spawn_drone_model = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "bebop",
            "-file", os.path.join(pkg_uav_sim, "models", "parrot_bebop_2", "model.sdf"),
            "-x", "5.0", "-y", "0.0", "-z", "0.5"], 
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
        set_gz_resource_path,
        gazebo,
        spawn_drone_box,
        spawn_drone_model,
        bridge,
        uav_controller
    ])
