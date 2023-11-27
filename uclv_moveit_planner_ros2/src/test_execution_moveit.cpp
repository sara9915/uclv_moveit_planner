#include "rclcpp/rclcpp.hpp"
#include "uclv_moveit_planner_interface/srv/planner_srv.hpp"
#include "uclv_moveit_planner_interface/action/traj_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uclv_utilities/color.h"
#include "uclv_utilities/utilities.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node
{
public:
    using TrajAction_ = uclv_moveit_planner_interface::action::TrajAction;
    using GoalHandleTrajAction = rclcpp_action::ClientGoalHandle<TrajAction_>;

    DemoNode() : Node("demo_node")
    {
        // create node
        rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("tf_node");
        // Create the service client to call the planner service
        this->planner_client_ = create_client<uclv_moveit_planner_interface::srv::PlannerSrv>("planner_service", rmw_qos_profile_services_default);

        // Create the action client to call the trajectory execution action
        this->traj_action_client_ = rclcpp_action::create_client<TrajAction_>(this, "trajectory_execution_as");

        // Wait for the service to be available
        while (!planner_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }

        // Call the planner service
        auto planner_request = std::make_shared<uclv_moveit_planner_interface::srv::PlannerSrv::Request>();
        geometry_msgs::msg::TransformStamped transform;

        bool get_actual_pose = uclv::getTransform(node, "base_link", "push_extension", transform);

        if (get_actual_pose)
        {
            planner_request->planning_type = "cartesian";
            planner_request->target_frame = "push_extension";
            planner_request->pose.pose.position.x = transform.transform.translation.x - 0.1;
            planner_request->pose.pose.position.y = transform.transform.translation.y;
            planner_request->pose.pose.position.z = transform.transform.translation.z;
            planner_request->pose.pose.orientation.x = transform.transform.rotation.x;
            planner_request->pose.pose.orientation.y = transform.transform.rotation.y;
            planner_request->pose.pose.orientation.z = transform.transform.rotation.z;
            planner_request->pose.pose.orientation.w = transform.transform.rotation.w;
            planner_request->start_joints = std::vector<double>();

            // - Translation: [0.756, -0.069, 0.681]
            // - Rotation: in Quaternion [0.490, 0.510, -0.506, -0.495]
            // planner_request->planning_type = "joint";
            // planner_request->target_frame = "push_extension";
            // planner_request->pose.pose.position.x = 0.756;
            // planner_request->pose.pose.position.y = -0.069;
            // planner_request->pose.pose.position.z = 0.681;
            // planner_request->pose.pose.orientation.x = 0.490;
            // planner_request->pose.pose.orientation.y = 0.510;
            // planner_request->pose.pose.orientation.z = -0.506;
            // planner_request->pose.pose.orientation.w = -0.495;
            // planner_request->start_joints = std::vector<double>();

            auto planner_response = planner_client_->async_send_request(planner_request);

            // Wait for the service response
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), planner_response) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "Failed to call planner service");
                return;
            }

            auto planner_response_ = planner_response.get();

            // Get the trajectory from the response
            traj = planner_response_->traj;
            bool success_ = planner_response_->success;

            if (success_)
            {
                std::cout << BOLDGREEN << "Planning successfully created " << RESET << std::endl;
                // Send goal to the action server
                this->timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(500),
                    std::bind(&DemoNode::send_goal, this));
            }
            else
            {
                std::cout << BOLDRED << "Planning failed!" << RESET << std::endl;
                rclcpp::shutdown();
            }
        }
        else
        {
            std::cout << BOLDRED << "Unable to get the actual pose of the robot!" << RESET << std::endl;
            rclcpp::shutdown();
        }

        // Print the joint positions of the trajectory
        // std::cout << "Planner response: " << std::endl;
        // print_joint_points();
    }

    void send_goal()
    {
        using namespace std::placeholders;

        std::cout << BOLDWHITE << "Waiting for action server to start." << RESET << std::endl;

        this->timer_->cancel();

        if (!this->traj_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = TrajAction_::Goal();
        std::vector<moveit_msgs::msg::RobotTrajectory> traj_vec;
        traj_vec.push_back(traj);
        goal_msg.traj = traj_vec;
        goal_msg.topic_robot = "/motoman/joint_ll_control";
        goal_msg.simulation = false;
        goal_msg.rate = 50.0;
        goal_msg.scale_factor = 5.0;
        std::cout << BOLDWHITE << "Sending goal to the action server" << RESET << std::endl;

        auto send_goal_options = rclcpp_action::Client<TrajAction_>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&DemoNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&DemoNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&DemoNode::result_callback, this, _1);
        this->traj_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp::Client<uclv_moveit_planner_interface::srv::PlannerSrv>::SharedPtr planner_client_;
    rclcpp_action::Client<TrajAction_>::SharedPtr traj_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit_msgs::msg::RobotTrajectory traj;

    void print_joint_points()
    {
        for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i)
        {
            auto joint_positions = traj.joint_trajectory.points[i].positions;
            std::cout << "Joint positions at point " << i << ": ";
            for (size_t j = 0; j < joint_positions.size(); ++j)
            {
                std::cout << joint_positions[j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void goal_response_callback(const GoalHandleTrajAction::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleTrajAction::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        if (result.result->success)
            std::cout << BOLDGREEN << "Trajectory execution success!" << RESET << std::endl;
        else
            std::cout << BOLDRED << "Trajectory execution failed!" << RESET << std::endl;

        rclcpp::shutdown();
    }

    void feedback_callback(
        GoalHandleTrajAction::SharedPtr,
        const std::shared_ptr<const TrajAction_::Feedback> feedback)
    {
        std::cout << "Feedback received: " << feedback->progress << std::endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto demo_node = std::make_shared<DemoNode>();
    rclcpp::spin(demo_node);

    rclcpp::shutdown();

    return 0;
}
