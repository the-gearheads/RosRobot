#include "ControllerTwistPublisher.h"
#include "frc/MathUtil.h"

using namespace ros;

ControllerTwistPublisher::ControllerTwistPublisher() : Node("rio_teleop") {
  twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  this->declare_parameter<bool>("enabled", false);
}

// Crude controller implementation, not really meant for production use, need to make something to publish via another node
void ControllerTwistPublisher::periodic() {
  geometry_msgs::msg::Twist twist;
  bool enabled;
  double move,rotate = 0;

  //this->get_parameter<bool>("enabled", enabled);
  //if(enabled) {
  if(true) {
    move = -frc::ApplyDeadband(controller.GetLeftY(), 0.08)*2;
    rotate = -frc::ApplyDeadband(controller.GetRightX(), 0.08)*2.2;
  }

  twist.linear.x = move;
  twist.angular.z = rotate;

  twistPublisher->publish(twist);  
}