#!/bin/bash
# Run the GraspGen demo inside the running container.
#
# Usage: bash utils/run_graspgen_demo.sh [antipodal|suction]
# The argument selects which grasp-type config YAML to load. Defaults
# to ``antipodal``.

GRASP_TYPE=${1:-antipodal}

echo "=== GraspGen Demo (${GRASP_TYPE}) ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 launch agile_manip_examples graspgen_demo.launch.py \
         config:=/colcon_ws/install/agile_manip_examples/share/agile_manip_examples/config/graspgen_${GRASP_TYPE}.yaml"
