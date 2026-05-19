import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_my_robot_py = get_package_share_directory('my_robot_py')
    world_file = os.path.join(pkg_my_robot_py, 'worlds', 'car_world.sdf')

    if not os.path.isfile(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        output='screen'
    )

    square_motion_node = Node(
        package='my_robot_py',
        executable='square_motion',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
        square_motion_node
    ])

