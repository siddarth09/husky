#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

class IKCommander : public rclcpp::Node {
public:
    explicit IKCommander(const std::string &urdf_path);
    void computeAndPublish();

private:
    bool solve_leg_ik(const pinocchio::Model &model,
                      pinocchio::Data &data,
                      const std::string &foot_frame,
                      const Eigen::Vector3d &target_pos,
                      Eigen::VectorXd &q,
                      int max_iters = 5000,
                      double eps = 1e-4,
                      double alpha = 1e-2);

    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
    std::map<std::string, std::vector<std::string>> leg_joint_map_;
    std::string urdf_path_;
};
