#include <rclcpp/rclcpp.hpp>
#include "uclv_moveit_planner_interface/srv/planner_srv.hpp"
#include "uclv_moveit_planner_ros2/planner.h"

namespace uclv
{

    class PlannerService : public rclcpp::Node
    {
    public:
        PlannerService() : Node("planner_service")
        {
            // Store the node as a shared pointer
            node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

            // Create the service
            planner_service_ = this->create_service<uclv_moveit_planner_interface::srv::PlannerSrv>(
                "planner_service",
                std::bind(&PlannerService::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

            // Create the planner object
            const std::string planning_group = "yaskawa_arm";
            planner_ = std::make_unique<PlannerMoveIt>(node_, planning_group);
        }

    private:
        void handle_service_request(const std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv::Request> request,
                                    std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv::Response> response)
        {
            bool success = false;
            moveit_msgs::msg::RobotTrajectory traj_;
            // Call the planner function with the request data
            if (request->planning_type == "joint")
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                success = planner_->joint_path_planner(plan, request->pose.pose);
                traj_ = plan.trajectory_;
            }
            else if (request->planning_type == "cartesian")
            {
                std::vector<geometry_msgs::msg::Pose> target_poses;
                target_poses.push_back(request->pose.pose);
                success = planner_->cartesian_path_planner(traj_,target_poses);
            }
            else
            {
                std::cout << BOLDRED << "Invalid planning type! Allowed types are: 'joint' or 'cartesian' " << RESET << std::endl;
            }

            // Set the response status
            response->success = success;
            response->traj = traj_;
        }

        rclcpp::Service<uclv_moveit_planner_interface::srv::PlannerSrv>::SharedPtr planner_service_;
        std::unique_ptr<PlannerMoveIt> planner_;
        std::shared_ptr<rclcpp::Node> node_;
    };

} // namespace uclv

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uclv::PlannerService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
