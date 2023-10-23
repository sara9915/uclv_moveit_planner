cmake_minimum_required(VERSION 3.8)
project(uclv_moveit_planner_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlannerSrv.srv"
  "action/TrajAction.action"
  DEPENDENCIES geometry_msgs moveit_ros_planning_interface# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
