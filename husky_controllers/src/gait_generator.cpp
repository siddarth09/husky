// gait_planner_node.cpp
// ROS 2 Node to generate basic trot gait foot trajectories

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unordered_map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

using std::placeholders::_1;
using namespace std::chrono_literals;

class GaitPlannerNode : public rclcpp::Node {
public:
    GaitPlannerNode() : Node("gait_planner_node") {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&GaitPlannerNode::cmdVelCallback, this, _1));

        foot_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/foot_targets", 10);

        gait_timer_ = this->create_wall_timer(20ms, std::bind(&GaitPlannerNode::updateGait, this));

        base_velocity_.setZero();
        initFootPositions();
        RCLCPP_INFO(this->get_logger(), "🐾 Gait Planner Node Initialized");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr foot_target_pub_;
    rclcpp::TimerBase::SharedPtr gait_timer_;

    Eigen::Vector3d base_velocity_;
    std::unordered_map<std::string, Eigen::Vector3d> foot_positions_;

    const std::vector<std::string> leg_names = {"RF", "LF", "RH", "LH"};

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        base_velocity_ << msg->linear.x, msg->linear.y, msg->angular.z;
    }

    void initFootPositions() {
        foot_positions_["RF"] = Eigen::Vector3d(0.2, -0.1, -0.25);
        foot_positions_["LF"] = Eigen::Vector3d(0.2,  0.1, -0.25);
        foot_positions_["RH"] = Eigen::Vector3d(-0.2, -0.1, -0.25);
        foot_positions_["LH"] = Eigen::Vector3d(-0.2,  0.1, -0.25);
    }

    void updateGait() {
        double time = this->now().seconds();
        double step_height = 0.05;
        double cycle_duration = 0.5;

        geometry_msgs::msg::PoseArray foot_array;
        foot_array.header.stamp = this->now();
        foot_array.header.frame_id = "base_link";

        for (const auto &leg : leg_names) {
            Eigen::Vector3d foot = foot_positions_[leg];

            bool is_swing = ((leg == "LF" || leg == "RH") && fmod(time, cycle_duration*2) < cycle_duration) ||
                            ((leg == "RF" || leg == "LH") && fmod(time, cycle_duration*2) >= cycle_duration);

            if (is_swing) {
                foot.z() = -0.25 + step_height * std::sin(2 * M_PI * fmod(time, cycle_duration) / cycle_duration);
                foot.x() += base_velocity_.x() * 0.01; // forward swing
            } else {
                foot.z() = -0.25;
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = foot.x();
            pose.position.y = foot.y();
            pose.position.z = foot.z();
            pose.orientation.w = 1.0;  // No rotation
            foot_array.poses.push_back(pose);
        }

        foot_target_pub_->publish(foot_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
