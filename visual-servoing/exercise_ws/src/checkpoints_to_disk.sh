#!/bin/bash
echo "Saving checkpoints to disk..."
source /opt/ros/noetic/setup.bash
rostopic pub -1 /agent/line_detector_node/checkpoints_to_disk std_msgs/String "fire"