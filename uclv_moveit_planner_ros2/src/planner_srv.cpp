#include <rclcpp/rclcpp.hpp>
#include "uclv_moveit_planner_interface/srv/planner_srv.hpp"
#include "uclv_moveit_planner_ros2/planner.h"

namespace uclv
{

    class PlannerService : public rclcpp::Node
    {
        // This is the class that implements the planner service node in ROS2 using the PlannerMoveIt class from uclv_moveit_planner package (see planner.h).
    public:
        /**
         * @brief Constructor of the PlannerService class that initializes the node and the service and creates a thread to execute the service in parallel with the main thread.
         * @param options The options for the ROS2 node.
         */
        PlannerService(const rclcpp::NodeOptions &options) : Node("uclv_planner_service", options)
        {
            // Get the parameters from launch file
            parameters.push_back(this->get_parameter("planning_time"));
            parameters.push_back(this->get_parameter("vel_scaling"));
            parameters.push_back(this->get_parameter("acc_scaling"));
            parameters.push_back(this->get_parameter("planner_type"));
            parameters.push_back(this->get_parameter("planning_group"));

            std::cout << BOLDGREEN << "Planner service initializing..." << RESET << std::endl;
            std::cout << BOLDWHITE << "Selected planning group: " << RESET << parameters.at(4).as_string() << std::endl;
            std::cout << BOLDWHITE << "Selected planning time: " << RESET << parameters.at(0).as_double() << std::endl;
            std::cout << BOLDWHITE << "Selected velocity scaling factor: " << RESET << parameters.at(1).as_double() << std::endl;
            std::cout << BOLDWHITE << "Selected acceleration scaling factor: " << RESET << parameters.at(2).as_double() << std::endl;
            std::cout << BOLDWHITE << "Selected planner type: " << RESET << parameters.at(3).as_string() << std::endl;

            // Create the service in a thread to execute it in parallel with the main thread (this is necessary to avoid blocking the main thread).
            // Also the PlannerMoveIt class is initialized in this thread.
            std::thread([this]()
                        {
                            std::cout << BOLDWHITE << "Initializing Planner MoveIt!" << RESET << std::endl;
                            using namespace std::placeholders;
                            this->get_clock()->sleep_for(std::chrono::seconds(2));
                            // Get parameters from node uclv_moveit_planner

                            planner_moveit_ = std::make_shared<PlannerMoveIt>(this->shared_from_this(), parameters.at(4).as_string(), parameters.at(0).as_double(), parameters.at(1).as_double(), parameters.at(2).as_double(), parameters.at(3).as_string());

                            // Create the service
                            std::cout << BOLDGREEN << "Creating planner service..." << RESET << std::endl;
                            planner_service_ = this->create_service<uclv_moveit_planner_interface::srv::PlannerSrv>(
                                "planner_service",
                                std::bind(&PlannerService::handle_service_request, this, std::placeholders::_1, std::placeholders::_2)); })
                .detach();

            std::cout << BOLDGREEN << "Planner service ready!" << RESET << std::endl;
        }

    private:

        /**
         * @brief Callback function for the planner service.
         * @param request The request data for the service. It contains the target frame and the target pose.
         * @param response The response data for the service. It contains the trajectory and the success status.
         * @return void
         */
        void handle_service_request(const std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv::Request> request,
                                    std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv::Response> response)
        {
            bool success = false;
            moveit_msgs::msg::RobotTrajectory traj_;
            auto target_pose = planner_moveit_->set_target_frame(this->shared_from_this(), request->target_frame, request->pose);
            // Call the planner function with the request data
            if (request->planning_type == "joint")
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                success = planner_moveit_->joint_path_planner(plan, target_pose);
                traj_ = plan.trajectory_;
            }
            else if (request->planning_type == "cartesian")
            {
                std::vector<geometry_msgs::msg::Pose> target_poses;
                target_poses.push_back(target_pose);
                success = planner_moveit_->cartesian_path_planner(traj_, target_poses);
            }
            else
            {
                std::cout << BOLDRED << "Invalid planning type! Allowed types are: 'joint' or 'cartesian' " << RESET << std::endl;
            }

            // Set the response status
            response->success = success;
            response->traj = traj_;
            if (success)
                std::cout << BOLDGREEN << "Planning success!" << RESET << std::endl;
            else
                std::cout << BOLDRED << "Planning failed!" << RESET << std::endl;
        }

        void set_planner_params(const double &planning_time, const double &vel_scaling, const double &acc_scaling, const std::string &planner_type)
        {
            planner_moveit_->update_planning_parameters(planning_time, vel_scaling, acc_scaling, planner_type);
        }

        rclcpp::Service<uclv_moveit_planner_interface::srv::PlannerSrv>::SharedPtr planner_service_;
        std::shared_ptr<PlannerMoveIt> planner_moveit_;
        std::vector<rclcpp::Parameter> parameters;
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


