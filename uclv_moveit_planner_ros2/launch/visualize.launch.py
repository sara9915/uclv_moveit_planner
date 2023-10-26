import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch



def generate_launch_description():

    gazebo_ = LaunchConfiguration('gazebo_')

    gazebo_launch_arg = DeclareLaunchArgument(
        name='gazebo_',
        default_value= "false",
        description='Set to true if you want to simulate on gazebo, false otherwise.'
    )


    # Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        condition=IfCondition(gazebo_),
        launch_arguments={'pause': 'true'}.items()
        #launch_arguments={'world': world_path}.items()
    )

    # entity spawn node (to spawn the robot from the /robot_description topic)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    moveit_config = MoveItConfigsBuilder("motoman_sia5f", package_name="yaskawa_moveit_config_ros2").planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]).to_moveit_configs()


    # Run the nodes
    return LaunchDescription([
        generate_demo_launch(moveit_config),
        gazebo_launch_arg,
        launch_gazebo,
        spawn_entity
    ])