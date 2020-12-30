#!/bin/bash
echo "Running visual servoing..."
source /opt/ros/noetic/setup.bash
rostopic pub -1 /agent/line_detector_node/start_vs std_msgs/String "fire"