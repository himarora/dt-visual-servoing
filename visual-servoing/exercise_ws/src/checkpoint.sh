#!/bin/bash
echo "Running checkpoints script..."
source /opt/ros/noetic/setup.bash
rostopic pub -1 /agent/line_detector_node/key_pressed std_msgs/String "fire"