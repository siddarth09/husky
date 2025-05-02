#include <iostream>
#include <map>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

int main(int argc, char **argv)
{
    using namespace pinocchio;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_solver_node");

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
        node,
        "/joint_trajectory_controller/follow_joint_trajectory"
    );
    
    // Wait until the action server is ready
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting!");
        return 1;
    }

    // Load URDF
    const std::string model_path = "/home/siddarth/husky_ws/src/husky_description/urdf/husky_description.urdf"; // Change your path!
    Model model;
    pinocchio::urdf::buildModel(model_path, model);
    Data data(model);

    std::cout << "Model loaded: " << model.name << " with " << model.nq << " DOFs." << std::endl;

    // Active joints
    std::vector<std::string> active_joint_names = {
        "RF_HAA", "RF_HFE", "RF_KFE",
        "LF_HAA", "LF_HFE", "LF_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE"
    };

    std::vector<JointIndex> active_joint_ids;
    for (const auto& joint_name : active_joint_names)
    {
        active_joint_ids.push_back(model.getJointId(joint_name));
    }

    // Foot frames
    std::map<std::string, FrameIndex> foot_frames = {
        {"rf_foot", model.getFrameId("RF_FOOT")},
        {"lf_foot", model.getFrameId("LF_FOOT")},
        {"rh_foot", model.getFrameId("RH_FOOT")},
        {"lh_foot", model.getFrameId("LH_FOOT")}
    };

    // Desired foot positions
    std::map<std::string, Eigen::Vector3d> target_positions = {
        {"rf_foot", Eigen::Vector3d( 1.2, -1.1, 0.0)},
        {"lf_foot", Eigen::Vector3d( 1.2,  1.1, 0.0)},
        {"rh_foot", Eigen::Vector3d(-1.2, -1.1, 0.0)},
        {"lh_foot", Eigen::Vector3d(-1.2,  1.1, 0.0)}
    };

    // Initial guess
    Eigen::VectorXd q = neutral(model);
    q[model.joints[model.getJointId("RF_HFE")].idx_q()] = -0.5;
    q[model.joints[model.getJointId("RF_KFE")].idx_q()] = 1.0;
    q[model.joints[model.getJointId("LF_HFE")].idx_q()] = -0.5;
    q[model.joints[model.getJointId("LF_KFE")].idx_q()] = 1.0;
    q[model.joints[model.getJointId("RH_HFE")].idx_q()] = -0.5;
    q[model.joints[model.getJointId("RH_KFE")].idx_q()] = 1.0;
    q[model.joints[model.getJointId("LH_HFE")].idx_q()] = -0.5;
    q[model.joints[model.getJointId("LH_KFE")].idx_q()] = 1.0;
    Eigen::VectorXd q_init = q;
    // IK parameters
    const double eps = 1e-2;
    const int IT_MAX = 3000;
    const double DT = 0.001;
    const double damp = 1e-4;

    bool success = true;

    // Solve IK for each foot
    for (const auto& foot : foot_frames)
    {
        const std::string foot_name = foot.first;
        const FrameIndex frame_id = foot.second;
        const Eigen::Vector3d target_pos = target_positions.at(foot_name);

        std::cout << "\n[INFO] Solving IK for foot: " << foot_name << std::endl;

        for (int iter = 0; iter < IT_MAX; ++iter)
        {
            forwardKinematics(model, data, q);
            updateFramePlacements(model, data);

            const SE3 &oMf = data.oMf[frame_id];
            Eigen::Vector3d pos_current = oMf.translation();
            Eigen::Vector3d error = pos_current - target_pos;

            if (error.norm() < eps)
            {
                std::cout << "[INFO] " << foot_name << " converged in " << iter << " iterations. Error norm: " << error.norm() << std::endl;
                break;
            }

            Eigen::MatrixXd J_full(6, model.nv);
            computeFrameJacobian(model, data, q, frame_id, J_full);

            Eigen::MatrixXd J_active(3, active_joint_ids.size());
            for (size_t k = 0; k < active_joint_ids.size(); ++k)
            {
                J_active.col(k) = J_full.topRows(3).col(model.joints[active_joint_ids[k]].idx_v());
            }

            Eigen::MatrixXd JJt = J_active * J_active.transpose();
            JJt.diagonal().array() += damp;
            Eigen::VectorXd v = -J_active.transpose() * JJt.ldlt().solve(error);

            Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model.nv);
            for (size_t k = 0; k < active_joint_ids.size(); ++k)
            {
                v_full(model.joints[active_joint_ids[k]].idx_v()) = v(k);
            }

            q = integrate(model, q, v_full * DT);

            if (iter % 100 == 0)
                std::cout << "Iteration " << iter << " Error norm: " << error.norm() << std::endl;
        }
    }

    std::cout << "\n[INFO] Final Solved Joint Configuration:\n" << q.transpose() << std::endl;

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = node->now();
    traj_msg.joint_names = active_joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point1, point2;

    // Fill start point using q_init
    for (const auto& joint_name : traj_msg.joint_names)
    {
        point1.positions.push_back(q_init[model.joints[model.getJointId(joint_name)].idx_q()]);
        point1.velocities.push_back(0.0);
    }
    point1.time_from_start.sec = 0;

    // Fill final point using solved q
    for (const auto& joint_name : traj_msg.joint_names)
    {
        point2.positions.push_back(q[model.joints[model.getJointId(joint_name)].idx_q()]);
        point2.velocities.push_back(0.0);
    }
    point2.time_from_start.sec = 2;

    traj_msg.points.push_back(point1);
    traj_msg.points.push_back(point2);

    // Prepare Action Goal
    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj_msg;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [](GoalHandleFollowJointTrajectory::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            std::ostringstream oss;
            oss << "[FEEDBACK] Current positions: ";
            for (const auto& pos : feedback->actual.positions)
                oss << pos << " ";
            RCLCPP_INFO(rclcpp::get_logger("ik_solver_node"), "%s", oss.str().c_str());
        };  

    send_goal_options.result_callback = [](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("ik_solver_node"), "Goal succeeded!");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ik_solver_node"), "Goal failed!");
        }
    };

    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        rclcpp::shutdown();
        return 1;
    }

    // Spin the node to process feedback and result
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}
