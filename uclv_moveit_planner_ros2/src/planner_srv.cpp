#include <rclcpp/rclcpp.hpp>
#include "uclv_moveit_planner_interface/srv/planner_srv.hpp"
#include "uclv_moveit_planner_ros2/planner.h"

namespace uclv
{

    class PlannerService : public rclcpp::Node
    {
    public:
        PlannerService(const rclcpp::NodeOptions &options) : Node("uclv_planner_service", options), node_(std::make_shared<rclcpp::Node>("planner_service")), 
                                                             executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
        {
            executor_->add_node(node_);
            executor_thread_ = std::thread([this]()
                                           { this->executor_->spin(); });
            executor_thread_.detach();
            
            // Create the service
            planner_service_ = this->create_service<uclv_moveit_planner_interface::srv::PlannerSrv>(
                "planner_service",
                std::bind(&PlannerService::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

            planner_moveit_ = std::make_shared<PlannerMoveIt>(node_,"yaskawa_arm");

            std::cout << BOLDGREEN << "Planner service ready!" << RESET << std::endl;
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
                success = planner_moveit_->joint_path_planner(plan, request->pose.pose);
                traj_ = plan.trajectory_;
            }
            else if (request->planning_type == "cartesian")
            {
                std::vector<geometry_msgs::msg::Pose> target_poses;
                target_poses.push_back(request->pose.pose);
                success = planner_moveit_->cartesian_path_planner(traj_, target_poses);
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
        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        std::shared_ptr<PlannerMoveIt> planner_moveit_;
    };

} // namespace uclv

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto planner_srv_node = std::make_shared<uclv::PlannerService>(node_options);
    rclcpp::spin(planner_srv_node);

    rclcpp::shutdown();
    return 0;
}
