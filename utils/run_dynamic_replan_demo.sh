#!/bin/bash
# Run the dynamic replanning demo inside the running container.
# A moving box obstacle (driven by the node's internal sinusoidal
# oscillator by default) is replanned around at several Hz; the
# latest trajectory is pushed to RViz every tick so the adapting
# plan is visible as the obstacle sweeps.
#
# Usage: bash utils/run_dynamic_replan_demo.sh

echo "=== Dynamic Replanning Pipeline ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_examples dynamic_replan_demo.launch.py"
