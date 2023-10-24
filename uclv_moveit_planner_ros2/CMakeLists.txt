cmake_minimum_required(VERSION 3.8)
project(uclv_moveit_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uclv_utilities REQUIRED)
find_package(uclv_moveit_planner_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


add_executable(test_planner src/test_planner.cpp)
ament_target_dependencies(test_planner rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
install(TARGETS
  test_planner
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(planner_srv src/planner_srv.cpp)
ament_target_dependencies(planner_srv rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
install(TARGETS
  planner_srv
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(execute_traj_as src/execute_action_server.cpp)
ament_target_dependencies(execute_traj_as rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
install(TARGETS
  execute_traj_as
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(demo src/test_execution_moveit.cpp)
ament_target_dependencies(demo rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
install(TARGETS
  demo
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(pp_traj_server src/pp_traj_server.cpp)
# ament_target_dependencies(pp_traj_server rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
# install(TARGETS
#   pp_traj_server
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(test_pp_client src/test_pp_client.cpp)
# ament_target_dependencies(test_pp_client rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface)
# install(TARGETS
#   test_pp_client
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()
