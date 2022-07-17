// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

void Robot::RobotInit() {
  rclcpp::init(0, NULL);

  std::cout << "Robot::RobotInit()" << std::endl;

  drive_node = std::make_shared<ros::DriveNode>();
  ds_node = std::make_shared<ros::DsNode>();
  controller_node = std::make_shared<ros::ControllerTwistPublisher>();
}

void Robot::RobotPeriodic() {
  rclcpp::spin_some(drive_node);
  rclcpp::spin_some(ds_node);

  drive_node->periodic();
  ds_node->periodic();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // Really only makes sense during teleop, but either way we might not even use these mode settings eventually
  // If we make things even more custom (custom dashboard maybe??)
  controller_node->periodic();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif