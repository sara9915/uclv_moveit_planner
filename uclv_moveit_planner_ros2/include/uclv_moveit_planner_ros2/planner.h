#include "uclv_utilities/utilities.hpp"
#include <rclcpp/rclcpp.hpp>
#include <optional>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/geometry_msgs/msg/pose.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "builtin_interfaces/msg/duration.hpp"

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

        void execute_sim(const moveit_msgs::msg::RobotTrajectory &trajectory)
        {
            bool ask = uclv::askContinue("EXECUTION IN SIMULATION");
            if (ask)
            {
                move_group.execute(trajectory);
            }
            else
            {
                std::cout << BOLDRED << "Aborted execution!" << RESET << std::endl;
                return;
            }
        }

        double interpolate_traj(const moveit_msgs::msg::RobotTrajectory &trajectory, const double &scale_factor, Eigen::Matrix<double, 6, -1> &coeff, int j)
        {
            // coeff_q1 coeff_q2 coeff_q3 ... coeff_q7
            //  a5
            //  a4
            //  ...
            //  a0

            // coeff.resize(6, this->num_joints);
            std::vector<sensor_msgs::msg::JointState> joint_cmd_vect;

            double tf = 0;
            auto dur_next = (trajectory.joint_trajectory.points[j + 1].time_from_start).sec + (trajectory.joint_trajectory.points[j + 1].time_from_start).nanosec * pow(10, -9);
            auto dur_prev = (trajectory.joint_trajectory.points[j].time_from_start).sec + (trajectory.joint_trajectory.points[j].time_from_start).nanosec * pow(10, -9);
            tf = dur_next - dur_prev;

            auto qi = trajectory.joint_trajectory.points[j].positions;
            auto qf = trajectory.joint_trajectory.points[j + 1].positions;

            auto qi_dot = trajectory.joint_trajectory.points[j].velocities;
            auto qf_dot = trajectory.joint_trajectory.points[j + 1].velocities;

            auto qi_dot_dot = trajectory.joint_trajectory.points[j].accelerations;
            auto qf_dot_dot = trajectory.joint_trajectory.points[j + 1].accelerations;

            if (scale_factor != 1.0)
            {
                tf = tf * scale_factor;
                for (int m = 0; m < int(qi.size()); m++)
                {
                    qi_dot[m] = qi_dot[m] / scale_factor;
                    qi_dot_dot[m] = qi_dot_dot[m] / pow(scale_factor, 2);

                    qf_dot[m] = qf_dot[m] / scale_factor;
                    qf_dot_dot[m] = qf_dot_dot[m] / pow(scale_factor, 2);
                }
            }

            update_coeff(coeff, tf, qi, qi_dot, qi_dot_dot, qf, qf_dot, qf_dot_dot);

            return tf;
        }

        void
        execute_robot(rclcpp::Node::SharedPtr node_, const moveit_msgs::msg::RobotTrajectory &trajectory, const double &scale_factor, const double &rate, const std::string &topic_name)
        {
            // create the rate
            rclcpp::Rate loop_rate(rate);
            // Create a ROS2 publisher to publish the joint commands on topic 'topic name' at rate 'rate'
            auto joint_cmd_pub = node_->create_publisher<sensor_msgs::msg::JointState>(topic_name, 1);

            // Create the joint state msg
            sensor_msgs::msg::JointState joint_cmd;
            joint_cmd.position.resize(trajectory.joint_trajectory.points.at(0).positions.size());
            joint_cmd.velocity.resize(trajectory.joint_trajectory.points.at(0).positions.size());
            joint_cmd.name.resize(trajectory.joint_trajectory.points.at(0).positions.size());
            joint_cmd.header = trajectory.joint_trajectory.header;
            joint_cmd.name = trajectory.joint_trajectory.joint_names;

            double tf = 0.0;
            double t = 0.0;
            Eigen::Matrix<double, 6, Eigen::Dynamic> coeff = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, this->num_joints);
            int num_points_traj = int(trajectory.joint_trajectory.points.size());

            for (int j = 0; j < num_points_traj - 1; j++)
            {
                tf = interpolate_traj(trajectory, scale_factor, coeff, j);

                // Create a timer to publish the joint commands at the specified rate
                const double sleep_time = 1 / rate;
                double t0 = node_->get_clock()->now().seconds();
                while (t <= tf)
                {
                    t = node_->get_clock()->now().seconds() - t0;
                    for (int i = 0; i < this->num_joints; i++)
                    {
                        joint_cmd.position.at(i) = quintic_q(t, coeff, i);
                        joint_cmd.velocity.at(i) = 0.0; // quintic_qdot(t, coeff, i);
                    }

                    joint_cmd_pub->publish(joint_cmd);
                    std::cout << "Publishing " << j << "/" << num_points_traj << " " << t << "/" << tf << std::endl;

                    // sleep for (1/rate) seconds
                    node_->get_clock()->sleep_for(std::chrono::seconds(int(sleep_time)));
                    // loop_rate.sleep();
                }
                t = 0;
            }
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