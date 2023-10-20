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

    planning_time_ = LaunchConfiguration('planning_time_')
    vel_scaling_ = LaunchConfiguration('vel_scaling_')
    acc_scaling_ = LaunchConfiguration('acc_scaling_')
    planner_type_ = LaunchConfiguration('planner_type_')
    planning_group_ = LaunchConfiguration('planning_group_')

    planning_time_arg = DeclareLaunchArgument(
        name='planning_time_',
        default_value= "10",
        description='Planning time'
    )

    # Run the nodes
    return LaunchDescription([
        planning_time_arg,
        Node(
            package='uclv_moveit_planner_ros2',
            executable='planner_srv',
            name='uclv_planner_srv',
            output='screen',
            parameters=[],
            arguments=[planning_time_],
        )
    ])    
