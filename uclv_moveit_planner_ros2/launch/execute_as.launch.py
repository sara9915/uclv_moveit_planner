from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # simulation = LaunchConfiguration('simulation')

    # simulation_arg = DeclareLaunchArgument(
    #     name='simulation',
    #     default_value= "true",
    #     description='Set it to true if you want to execute the trajectories in simulation or to false if you want to execute them in the real robot'
    # )


    # Run the nodes
    return LaunchDescription([
        # simulation_arg,
        Node(
            package='uclv_moveit_planner_ros2',
            executable='execute_traj_as',
            name='execute_traj_action_server',
            output='screen',
            emulate_tty=False,
            prefix=['xterm -e'],
            # parameters=[
            #     {"simulation": simulation},
            # ],
        )
    ])    
