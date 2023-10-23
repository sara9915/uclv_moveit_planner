# uclv_moveit_planner
The uclv_moveit_planner package provides a ROS 2 node that plans trajectories using MoveIt and allows to execute them when successfully planned.

## Installation
Clone the repository into your workspace and build it using colcon build.
```bash
cd <your_workspace>/src
git clone https://github.com/sara9915/uclv_moveit_planner.git
cd ..
colcon build --packages-select uclv_moveit_planner_interface uclv_modeit_planner_ros2
```

## Usage
- **Planner Service**
  
The uclv_moveit_planner node provides a ROS 2 service for planning trajectories. To start the node, run:
```bash
ros2 run uclv_moveit_planner planner_srv
```
This will start the planner_srv node, which provides the plan_traj service. The service takes a PlanTraj request message, which contains the start and goal states, and returns a PlanTraj response message, which contains the planned trajectory.

- Action Server


The uclv_moveit_planner node also provides a ROS 2 action server for executing trajectories. To start the node, run:
```bash
ros2 run uclv_moveit_planner execute_traj_action_server
```
This will start the execute_traj_action_server node, which provides the execute_traj action. The action takes a TrajAction goal message, which contains the trajectory to execute, and returns a TrajAction result message, which indicates whether the execution was successful.

