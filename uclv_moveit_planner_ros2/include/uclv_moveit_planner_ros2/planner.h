#include "uclv_utilities/utilities.hpp"
#include <rclcpp/rclcpp.hpp>
#include <optional>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/geometry_msgs/msg/pose.h"

namespace uclv
{
    class PlannerMoveIt
    {
    private:
    public:
        const moveit::core::JointModelGroup *joint_model_group;
        moveit::core::RobotStatePtr start_state;
        moveit::planning_interface::MoveGroupInterface move_group;

        robot_model_loader::RobotModelLoader robot_model_loader;

        double planning_time = 10.0;
        double vel_scaling = 1;
        double acc_scaling = 1;
        int num_joints = 0;
        std::string planner_type = "RRTStar";

        /***************** METHODS **************/
        // Constructor
        PlannerMoveIt(const rclcpp::Node::SharedPtr node, const std::string &planning_group) : move_group(node, planning_group), robot_model_loader(node)
        {
            joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
            num_joints = joint_model_group->getVariableNames().size();

            const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
            start_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));

            set_planning_parameters();
            std::cout << "\n\n\n\n"
                      << std::endl;
            std::cout << BOLDMAGENTA << "MoveIt Planner successfully created! " << RESET << std::endl;
            std::cout << BOLDWHITE << "Selected planning group: " << RESET << planning_group << std::endl;
        }

        PlannerMoveIt(const rclcpp::Node::SharedPtr node, const std::string &planning_group, const double &planning_time_, const double &vel_scaling_, const double &acc_scaling_, const std::string planner_type_) : move_group(node, planning_group), robot_model_loader(node)
        {
            joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
            num_joints = joint_model_group->getVariableNames().size();

            update_planning_parameters(planning_time_, vel_scaling_, acc_scaling_, planner_type_);

            move_group.setNumPlanningAttempts(10);
            move_group.setReplanAttempts(10);

            const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
            start_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));

            std::cout << "\n\n\n\n"
                      << std::endl;
            std::cout << BOLDMAGENTA << "MoveIt Planner successfully created! " << RESET << std::endl;
            std::cout << BOLDWHITE << "Selected planning group: " << RESET << planning_group << std::endl;
        }

        // nDestructor
        ~PlannerMoveIt() {}

        void update_planning_parameters(const double &planning_time_, const double &vel_scaling_, const double &acc_scaling_, const std::string planner_type_)
        {
            this->planning_time = planning_time_;
            this->planner_type = planner_type_;
            this->acc_scaling = acc_scaling_;
            this->vel_scaling = vel_scaling_;

            set_planning_parameters();
        }

        void set_planning_parameters()
        {
            this->move_group.setMaxVelocityScalingFactor(this->vel_scaling);
            this->move_group.setMaxAccelerationScalingFactor(this->acc_scaling);
            this->move_group.setPlanningTime(this->planning_time);
            this->move_group.setPlannerId(this->planner_type);
            // this->move_group.setPlannerId("");
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::vector<double> &target_joint_group)
        {
            move_group.setStartStateToCurrentState();
            move_group.setJointValueTarget(target_joint_group);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::vector<double> &target_joint_group, const Eigen::VectorXd &start_joint_group)
        {
            start_state->setJointGroupPositions(this->joint_model_group, start_joint_group);
            move_group.setStartState(*start_state);

            move_group.setJointValueTarget(target_joint_group);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const double &vel_scaling, const double &acc_scaling, const double &planning_time, const std::string planner_type, const std::vector<double> &target_joint_group)
        {

            update_planning_parameters(planning_time, vel_scaling, acc_scaling, planner_type);
            move_group.setStartStateToCurrentState();
            move_group.setJointValueTarget(target_joint_group);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const double &vel_scaling, const double &acc_scaling, const double &planning_time, const std::string planner_type, const std::vector<double> &target_joint_group, const Eigen::VectorXd &start_joint_group)
        {
            update_planning_parameters(planning_time, vel_scaling, acc_scaling, planner_type);

            start_state->setJointGroupPositions(this->joint_model_group, start_joint_group);
            move_group.setStartState(*start_state);

            move_group.setJointValueTarget(target_joint_group);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &target_pose)
        {
            start_state->setFromIK(joint_model_group, start_pose);
            move_group.setStartState(*start_state);
            move_group.setPoseTarget(target_pose);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::vector<double> &start_pose, const geometry_msgs::msg::Pose &target_pose)
        {
            start_state->setJointGroupPositions(joint_model_group, start_pose);
            move_group.setStartState(*start_state);
            move_group.setPoseTarget(target_pose);
            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const geometry_msgs::msg::Pose &target_pose)
        {
            move_group.setStartStateToCurrentState();
            move_group.setPoseTarget(target_pose);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool joint_path_planner(moveit::planning_interface::MoveGroupInterface::Plan &plan, const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &target_pose, const double &vel_scaling, const double &acc_scaling, const double &planning_time, const std::string planner_type)
        {
            update_planning_parameters(planning_time, vel_scaling, acc_scaling, planner_type);

            start_state = move_group.getCurrentState();
            start_state->setFromIK(joint_model_group, start_pose);
            move_group.setStartState(*start_state);
            move_group.setPoseTarget(target_pose);

            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        bool cartesian_path_planner(moveit_msgs::msg::RobotTrajectory &trajectory, std::vector<geometry_msgs::msg::Pose> &target_poses)
        {
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(target_poses, eef_step, jump_threshold, trajectory);
            if (fraction == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        bool cartesian_path_planner(moveit_msgs::msg::RobotTrajectory &trajectory, const std::vector<double> &start_joints, std::vector<geometry_msgs::msg::Pose> &target_poses)
        {
            start_state->setJointGroupPositions(joint_model_group, start_joints);
            move_group.setStartState(*start_state);
            move_group.setEndEffectorLink("tool0");
            std::cout << "PLANNING TIME: " << move_group.getPlanningTime() << std::endl;
            std::cout << "PLANNER TYPE: " << move_group.getPlannerId() << std::endl;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(target_poses, eef_step, jump_threshold, trajectory);
            if (fraction == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Set the moving frame, i.e., target pose expressed in target frame
        geometry_msgs::msg::Pose set_target_frame(rclcpp::Node::SharedPtr node_, const std::string target_frame, geometry_msgs::msg::PoseStamped &target_pose)
        {
            // Get the planning frame setted as default by moveit
            auto source_frame = this->move_group.getEndEffectorLink();

            bool getTransform_ = false;

            geometry_msgs::msg::TransformStamped target_T_source_;

            while (!getTransform_)
                getTransform_ = uclv::getTransform(node_, target_frame, source_frame, target_T_source_);

            // uclv::print_geometry_transform(target_T_source_.transform);

            geometry_msgs::msg::Transform base_T_target_ = uclv::geometry_2_transform(target_pose.pose);

            Eigen::Isometry3d target_T_source = uclv::geometry_2_eigen(target_T_source_.transform);
            Eigen::Isometry3d base_T_target = uclv::geometry_2_eigen(base_T_target_);
            Eigen::Isometry3d base_T_source(base_T_target * target_T_source);

            auto pose_return = uclv::eigen_2_geometry(base_T_source);
            auto norm_quat = uclv::normalize_quaternion(pose_return.orientation);
            pose_return.orientation = norm_quat;

            return pose_return;
        }

        void execute(const moveit_msgs::msg::RobotTrajectory &trajectory)
        {
            if (uclv::askContinue("EXECUTION"))
            {
                move_group.execute(trajectory);
            }
            else
                return;
        }

        bool attachCollisionObj(const std::string &obj_id, const std::string &link_name = "ee_fringers")
        {
            return move_group.attachObject(obj_id, link_name);
        }

        bool detachCollisionObj(const std::string &obj_id)
        {
            return move_group.detachObject(obj_id);
        }
    };

}