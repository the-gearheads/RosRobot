#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "frc/DriverStation.h"

namespace ros {
  class DsNode : public rclcpp::Node {
    public:
    DsNode();
    void periodic();

    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _modePublisher;
  };
}