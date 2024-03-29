#include "DriveNode.h"
#include <iostream>
using std::placeholders::_1;

namespace ros {
  DriveNode::DriveNode() : Node("drive_node") {
    // Create a subscriber for the drive_node's velocity command.
    RCLCPP_INFO(this->get_logger(), "Subscribing to /cmd_vel");
    velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&DriveNode::setVelocity, this, _1));
    odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    drive.zeroEncoders();
  }

  void DriveNode::setVelocity(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if(msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
      RCLCPP_WARN(this->get_logger(), "Rotating an axis we physically cannot. Refusing to move. Reminder:");
      RCLCPP_WARN(this->get_logger(), "Linear X for F-B movement, Angular Z for rotation");
      return;
    }

    drive.drive(msg->linear.x, msg->angular.z);
  }

  void DriveNode::periodic() {
    drive.periodic();

    // Publish robot position
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "odom";

    frc::Pose2d pose = drive.odometry.GetPose();

    // Convert euler angles to quaternion, then set it
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.Rotation().Radians().value());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    t.transform.translation.x = pose.X().value();
    t.transform.translation.y = pose.Y().value();

    tf_broadcaster->sendTransform(t);

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "world";
    odom.child_frame_id = "odom";

    odom.pose.pose.position.x = pose.X().value();
    odom.pose.pose.position.y = pose.Y().value();

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    frc::ChassisSpeeds speeds = drive.getSpeeds();

    odom.twist.twist.linear.x = speeds.vx.value();
    odom.twist.twist.angular.z = speeds.vy.value();

    odometry_pub->publish(odom);

  }
}