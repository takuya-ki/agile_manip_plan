#!/bin/bash
# Run the benchmark harness inside the running agile_manip_plan
# container. Produces a per-iteration CSV (default in /tmp) and a
# short median/mean/success-rate summary on stdout.
#
# Usage:
#   bash utils/run_benchmark.sh [selection_mode] [iterations]
#
#   selection_mode: highest_confidence (default) | multi_criteria | manual
#   iterations:     default 20
#
# Prerequisite: ``bash utils/start_backends.sh`` has already brought
# the cuMotion MoveGroup action server and the GraspGen service up.
set -euo pipefail

MODE="${1:-highest_confidence}"
ITER="${2:-20}"

echo "=== Benchmark: selection_mode=${MODE}, iterations=${ITER} ==="

docker exec -it agile_manip_plan_container bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /colcon_ws/install/setup.bash && \
     ros2 run agile_manip_examples benchmark_harness --ros-args \
         --params-file /colcon_ws/install/agile_manip_examples/share/agile_manip_examples/config/benchmark.yaml \
         -p selection_mode:=${MODE} \
         -p iterations:=${ITER}"
