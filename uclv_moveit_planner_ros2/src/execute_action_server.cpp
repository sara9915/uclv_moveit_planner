#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uclv_moveit_planner_interface/action/traj_action.hpp"
#include "uclv_moveit_planner_ros2/planner.h"

namespace uclv
{
    class ExecuteTrajActionServer : public rclcpp::Node
    {
    public:
        using TrajAction = uclv_moveit_planner_interface::action::TrajAction;
        using GoalHandleTrajAction = rclcpp_action::ServerGoalHandle<TrajAction>;

        ExecuteTrajActionServer(const rclcpp::NodeOptions &options)
            : Node("execute_traj_action_server", options)
        {
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "uclv_planner_srv");
            while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            this->parameters = parameters_client->get_parameters({"planning_time", "vel_scaling", "acc_scaling", "planner_type", "planning_group"});
            this->simulation = this->get_parameter("simulation").as_bool();

            std::thread([this]()
                        {
                            using namespace std::placeholders;
                            this->get_clock()->sleep_for(std::chrono::seconds(2));
                            // Get parameters from node uclv_moveit_planner

                            planner_moveit_ = std::make_shared<PlannerMoveIt>(this->shared_from_this(), parameters.at(4).as_string(), parameters.at(0).as_double(), parameters.at(1).as_double(), parameters.at(2).as_double(), parameters.at(3).as_string());
                            this->action_server_ = rclcpp_action::create_server<TrajAction>(
                                this,
                                "trajectory_execution_as",
                                std::bind(&ExecuteTrajActionServer::handle_goal, this, _1),
                                std::bind(&ExecuteTrajActionServer::handle_cancel, this, _1),
                                std::bind(&ExecuteTrajActionServer::handle_accepted, this, _1)); })
                .detach();
        }

    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp_action::Server<TrajAction>::SharedPtr action_server_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        std::shared_ptr<PlannerMoveIt> planner_moveit_;
        std::vector<rclcpp::Parameter> parameters;
        bool simulation;

        rclcpp_action::GoalResponse
        handle_goal(
            const rclcpp_action::GoalUUID &uuid)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleTrajAction> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTrajAction> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&ExecuteTrajActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleTrajAction> goal_handle)
        {
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TrajAction::Feedback>();
            auto result = std::make_shared<TrajAction::Result>();

            auto & progress = feedback->progress; 

            std::cout << BOLDGREEN << "Trajectory execution started!" << RESET << std::endl;

            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            if (simulation)
            {
                for (int i = 0; i < int(goal->traj.size()); i++)
                {
                    planner_moveit_->execute_sim(goal->traj.at(i));
                    progress = int32_t(i);
                    // std::cout << BOLDMAGENTA << "Publishing feedback" << RESET << std::endl;
                    goal_handle->publish_feedback(feedback);
                }
                result->success = true;
            }
            else
            {
                for (int i = 0; i < int(goal->traj.size()); i++)
                {
                    if (uclv::askContinue("EXECUTION ON REAL ROBOT"))
                    {
                        planner_moveit_->execute_robot(this->shared_from_this(), goal->traj.at(0), goal->scale_factor, goal->rate, goal->topic_robot);
                        feedback->progress = i;
                        // std::cout << BOLDMAGENTA << "Publishing feedback" << RESET << std::endl;
                        goal_handle->publish_feedback(feedback);
                        result->success = true;
                    }
                    else
                    {
                        std::cout << BOLDRED << "Aborted execution!" << RESET << std::endl;
                        break;
                    }
                }
            }

            // Check if goal is done
            if (rclcpp::ok())
            {
                goal_handle->succeed(result);
                if (result->success)
                    std::cout << BOLDGREEN << "Trajectory execution finished successfully!" << RESET << std::endl;
                else
                    std::cout << BOLDRED << "Trajectory execution failed!" << RESET << std::endl;
            }
        }
    }; // class ExecuteTrajActionServer

} // namespace uclv

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    // node_options.allow_undeclared_parameters(true);

    auto execute_traj_as_node = std::make_shared<uclv::ExecuteTrajActionServer>(node_options);
    rclcpp::spin(execute_traj_as_node);
    rclcpp::shutdown();
    return 0;
}
