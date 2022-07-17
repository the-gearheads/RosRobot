// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// ROS2 Galactic has some GLIBC deprecation errors, let's suppress those (or at least some of them)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "DriveNode.h"
#include "DsNode.h"
#include "ControllerTwistPublisher.h"


#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  private:
    std::shared_ptr<ros::DriveNode> drive_node;
    std::shared_ptr<ros::DsNode> ds_node;
    std::shared_ptr<ros::ControllerTwistPublisher> controller_node;
};
