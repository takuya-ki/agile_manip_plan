#!/bin/bash
# Run the robot-model visualization demo inside the running container.
# No backends are required - this demo only brings up
# robot_state_publisher + joint_state_publisher + RViz.
#
# Usage: bash utils/run_display_demo.sh

echo "=== Robot Model Visualization ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_description display.launch.py"
