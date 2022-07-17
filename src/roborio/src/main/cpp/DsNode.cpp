#include "DsNode.h"
#include "frc/DriverStation.h"

namespace ros {
    DsNode::DsNode() : Node("ds_node") {
        _modePublisher = this->create_publisher<std_msgs::msg::String>("/ds/mode", 10);
    }
        
    void DsNode::periodic() {
        auto mode = std_msgs::msg::String();

        /* Publish our current mode */
        if(frc::DriverStation::IsTeleopEnabled())
            mode.data = "teleop";

        if(frc::DriverStation::IsAutonomousEnabled())
            mode.data = "auton";

        if(frc::DriverStation::IsTest())
            mode.data = "test";

        if(frc::DriverStation::IsDisabled())
            mode.data = "disabled";

        _modePublisher->publish(mode);

    }
}