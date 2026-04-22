#!/bin/bash
# Run the obstacle-aware grasp + motion pipeline inside the running
# container.
#
# Usage: bash utils/run_obstacle_aware_demo.sh

echo "=== Obstacle-Aware Grasp + Motion Pipeline ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_examples obstacle_aware_demo.launch.py"
