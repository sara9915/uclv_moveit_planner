
#include "uclv_moveit_planner_ros2/planner.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    uclv::PlannerMoveIt planner(move_group_node, "yaskawa_arm");

    std::cout << BOLDWHITE << "END_EFFECTOR: " << RESET << planner.move_group.getEndEffectorLink() << std::endl;
    std::cout << BOLDWHITE << "\nPLANNING FRAME: " << RESET << planner.move_group.getPlanningFrame() << std::endl;
    std::cout << BOLDWHITE << "\nPOSE REFERENCE FRAME: " << RESET << planner.move_group.getPoseReferenceFrame() << std::endl;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory trajectory;

    std::string planning_frame = "push_extension";

    // Pose target test
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped current_pose_ee;
    geometry_msgs::msg::PoseStamped target_pose_;
    std::vector<geometry_msgs::msg::Pose> target_pose_vector;

    current_pose = planner.move_group.getCurrentPose(planning_frame);
    target_pose_ = current_pose;
    target_pose_.pose.position.y = current_pose.pose.position.y + 0.10;

    current_pose_ee = planner.move_group.getCurrentPose();
    std::cout << BOLDWHITE << "\n\nCURRENT POSE of " << planning_frame << ": " << RESET << std::endl;
    RCLCPP_INFO_STREAM(move_group_node->get_logger(), geometry_msgs::msg::to_yaml(current_pose));
    std::cout << BOLDWHITE << "\n\nCURRENT POSE of " << planner.move_group.getEndEffectorLink() << ": " << RESET << std::endl;
    RCLCPP_INFO_STREAM(move_group_node->get_logger(), geometry_msgs::msg::to_yaml(current_pose_ee));

    if (uclv::askContinue("PLANNING "))
    {
        auto target_pose = planner.set_target_frame(move_group_node, planning_frame, target_pose_);

        planner.move_group.setPoseTarget(target_pose);

        std::cout << BOLDWHITE << "\n\nTARGET POSE of " << planning_frame << ": " << RESET << std::endl;
        RCLCPP_INFO_STREAM(move_group_node->get_logger(), geometry_msgs::msg::to_yaml(target_pose_));
        std::cout << BOLDWHITE << "\n\nTARGET POSE of " << planner.move_group.getEndEffectorLink() << " : " << RESET << std::endl;
        RCLCPP_INFO_STREAM(move_group_node->get_logger(), geometry_msgs::msg::to_yaml(target_pose));

        target_pose_vector.push_back(current_pose_ee.pose);
        target_pose_vector.push_back(target_pose);

        if (planner.cartesian_path_planner(trajectory, target_pose_vector))
        {
            std::cout << BOLDGREEN << "Planning Success" << RESET << std::endl;
            planner.execute(trajectory);
        }
        else
        {
            std::cout << BOLDRED << "Planning Failed" << RESET << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}