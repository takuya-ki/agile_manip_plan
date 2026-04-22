#!/bin/bash
# Run the cuMotion trajectory-planning demo inside the running
# container.
#
# Usage: bash utils/run_cumotion_demo.sh

echo "=== cuMotion Demo ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_examples cumotion_demo.launch.py"
