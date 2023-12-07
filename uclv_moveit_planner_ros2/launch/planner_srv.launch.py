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
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)


def generate_launch_description():

    planning_time = LaunchConfiguration('planning_time')
    vel_scaling = LaunchConfiguration('vel_scaling')
    acc_scaling = LaunchConfiguration('acc_scaling')
    planner_type = LaunchConfiguration('planner_type')
    planning_group = LaunchConfiguration('planning_group')
    simulation = LaunchConfiguration('simulation')

    planning_time_arg = DeclareLaunchArgument(
        name='planning_time',
        default_value= "10.0",
        description='Planning time'
    )

    vel_scaling_arg = DeclareLaunchArgument(
        name='vel_scaling',
        default_value= "0.5",
        description='Velocity scaling'
    )

    acc_scaling_arg = DeclareLaunchArgument(
        name='acc_scaling',
        default_value= "0.5",
        description='Acceleration scaling'
    )

    planner_type_arg = DeclareLaunchArgument(
        name='planner_type',
        default_value= "RRTstarkConfigDefault",
        description='Planner type'
    )

    planning_group_arg = DeclareLaunchArgument(
        name='planning_group',
        default_value= "yaskawa_arm",
        description='Planning group'
    )

    simulation_arg = DeclareBooleanLaunchArg(
        name='simulation',
        default_value= True,
        description='Simulation or real robot'
    )

    # Run the nodes
    return LaunchDescription([
        planning_time_arg,
        vel_scaling_arg,
        acc_scaling_arg,
        planner_type_arg,
        planning_group_arg,
        simulation_arg,
        Node(
            package='uclv_moveit_planner_ros2',
            executable='planner_srv',
            name='uclv_planner_srv',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"planning_time": planning_time},
                {"vel_scaling": vel_scaling},
                {"acc_scaling": acc_scaling},
                {"planner_type": planner_type},
                {"planning_group": planning_group},
            ],
        ),
        Node(
            package='uclv_moveit_planner_ros2',
            executable='execute_traj_as',
            name='execute_traj_action_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"simulation": simulation}
            ],
        )
    ])    
