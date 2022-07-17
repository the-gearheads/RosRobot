#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "frc/XboxController.h"

namespace ros {
  class ControllerTwistPublisher : public rclcpp::Node {
    public:
    ControllerTwistPublisher();
    void periodic();

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;
    frc::XboxController controller{0};
  };
}