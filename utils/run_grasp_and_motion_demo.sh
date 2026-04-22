#!/bin/bash
# Run the integrated grasp + motion planning demo inside the running
# container.
#
# Usage: bash utils/run_grasp_and_motion_demo.sh

echo "=== Grasp + Motion Planning Pipeline ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_examples grasp_and_motion_demo.launch.py"
