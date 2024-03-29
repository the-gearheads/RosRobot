#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "DriveSubsystem.h"


namespace ros {
  class DriveNode : public rclcpp::Node {
    public:
    DriveNode();
    void periodic();

    private:
    void setVelocity(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    frc::DriveSubsystem drive{};

  };
}