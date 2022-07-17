# RosRobot

Our attempt at a rewrite of robot code for ROS (galactic, unfortunately will be EOL soon).
ROS runs on the roborio and on other nodes, likely to be a Jetson Nano. The src/roborio folder,
while being inside the workspace, is not integrated into the colcon build system at all andshould be built separately.
It has bundled binaries for Linux athena (RoboRIO) and Linux x86_64, Windows suppoprt is planned. 

New packages should be added to the src/ directory. 
