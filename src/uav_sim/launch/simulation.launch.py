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
        launch_arguments={'gz_args': f'{world_file}'}.items(),
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

    joy_config_path = os.path.join(
        get_package_share_directory('flight_controller'), 
        'controller_config', 
        'xbox_config.yaml'
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')
        ),
        launch_arguments={
            'config_filepath': joy_config_path,
            'joy_vel': 'cmd_vel' # Ensure topic name matches
        }.items(),
    )

    flight_controller = Node(
        package="flight_controller",
        executable="controller",
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            "/bebop/command/motor_speed/prop_fr@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
            "/bebop/command/motor_speed/prop_fl@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
            "/bebop/command/motor_speed/prop_rr@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
            "/bebop/command/motor_speed/prop_rl@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen"
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        spawn_drone_model,
        joy_node,
        bridge,
        flight_controller
    ])
