# RosRobot

Our attempt at a rewrite of robot code for ROS (galactic, unfortunately will be EOL soon).
ROS runs on the roborio and on other nodes, likely to be a Jetson Nano. The src/roborio folder,
while being inside the workspace, is not integrated into the colcon build system at all andshould be built separately.
It has bundled binaries for Linux athena (RoboRIO) and Linux x86_64, and Windows. For Windows, please ensure that you have correctly
installed all the dependencies from https://docs.ros.org/en/galactic/Installation/Windows-Install-Binary.html if you haven't already.

New packages should be added to the src/ directory. 

ROS for the RoboRIO was built using https://github.com/person4268/RoborioROS2, which is a
fork of Team 4145's cross-compilation code, which can be found at github.com/WorthingtonRobotics/RoborioROS2
