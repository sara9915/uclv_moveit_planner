# ROS2 MoveIt Planner
The uclv_moveit_planner package provides a ROS 2 node that plans trajectories using MoveIt! and allows to execute them when successfully planned. The repository consists of a package, *uclv_moveit_planner_interface*, that includes the node interfaces (.srv and .action files) and a package, *uclv_moveit_planner_ros2*, that implements service and action to plan and execute trajectory under MoveIt2.

## Installation
Clone the repository into your workspace and build it using colcon build. Note that you need to install MoveIt 2 if you haven’t already (follows [instruction](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) or get it by runnin ```sudo apt install ros-humble-moveit ```).
```bash
cd <your_workspace>/src
git clone https://github.com/sara9915/uclv_moveit_planner.git
cd ..
colcon build --packages-select uclv_moveit_planner_interface uclv_modeit_planner_ros2
```

## Usage
To use this package, make sure you have MoveIt on your machine and that the *robot_description* is available.

- **Planner Service**
  
The uclv_moveit_planner node provides a ROS 2 service for planning trajectories. To start the node, run:
```bash
ros2 run uclv_moveit_planner planner_srv
```
This will start the planner_srv node, which provides the plan_traj service. The service takes a PlanTraj request message, which contains the start and goal states, and returns a PlanTraj response message, which contains the planned trajectory.

You can also set some planning parameters by running the node from the provided launch file. Run
```bash
ros2 launch uclv_moveit_planner planner_srv.launch.py --show-args
```
to visualize all the available arguments and to set them from cli.

An example of the client is also provided in the executable node (see *test_planenr.cpp*).

- **Action Server**

The uclv_moveit_planner node also provides a ROS 2 action server for executing trajectories. To start the node, run:
```bash
ros2 run uclv_moveit_planner execute_traj_action_server
```
This will start the execute_traj_action_server node, which provides the execute_traj action. The action takes a TrajAction goal message, which contains the trajectory to execute, and returns a TrajAction result message, which indicates whether the execution was successful.

The server is realized to work both in simulation and on a real robot. The two modes are distinguished using a parameter *simulation* (true or false). If *simulation* is set to false, the server will publish the joints trajectory received from the goal on a specified topic (for example, /joint_states).

An example of the client is also provided in the executable node (see *test_execution_moveit.cpp*).

## Demo
Some demos are available to test the planner and the execution of the trajectory in simulation on MoveIt2. The demos are developed by using the *yaskawa_moveit_config_ros2* repository and the *uclv_yaskawa_simulation* package (you can install all the required dependencies by using the provided https.rosinstall file). 
