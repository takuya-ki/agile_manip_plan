#!/bin/bash
# Bring up the GraspGen service and cuMotion MoveGroup action server
# inside their respective containers. Blocks until both are ready, then
# exits so the caller can run a demo client.
#
# Usage:
#   bash utils/start_backends.sh [--graspgen-config PATH]
#
# The default GraspGen config is the antipodal mesh example shipped
# with graspgen_ros. Pass ``--graspgen-config`` to switch grippers or
# input modalities.
set -euo pipefail

GRASPGEN_CONFIG="/ros2_ws/src/graspgen_tutorials/config/example_mesh_antipodal_generator.yaml"
LAUNCH_TIMEOUT_SEC=180

while [[ $# -gt 0 ]]; do
    case "$1" in
        --graspgen-config)
            GRASPGEN_CONFIG="$2"
            shift 2
            ;;
        *)
            echo "Unknown arg: $1" >&2
            exit 1
            ;;
    esac
done

wait_for() {
    local container="$1"
    local check_cmd="$2"
    local label="$3"
    local start_ts
    start_ts="$(date +%s)"
    while true; do
        if docker exec "${container}" bash -c "${check_cmd}" >/dev/null 2>&1; then
            echo "  [ready] ${label}"
            return 0
        fi
        if (( $(date +%s) - start_ts > LAUNCH_TIMEOUT_SEC )); then
            echo "  [timeout] ${label} did not come up within ${LAUNCH_TIMEOUT_SEC}s" >&2
            return 1
        fi
        sleep 3
    done
}

echo "== Clearing any stale backend processes =="
# Running multiple mesh_graspgen_service instances makes /generate_grasp
# round-robin across them, so RViz alternates between their different
# object meshes and grasp sets. stop_backends handles pkill -9 + retry.
bash "$(dirname "$0")/stop_backends.sh" >/dev/null 2>&1 || true

echo "== Starting GraspGen service =="
# Use the upstream GraspGen example mesh (/share/obj_mesh/example.stl,
# set via ${GRASPGEN_CONFIG}). cuMotion reliably solves IK for this
# geometry; earlier cylinder experiments all failed pose-based IK
# even though joint-space planning worked. Pass --graspgen-config to
# override.
docker exec -d graspgen_container bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 run graspgen_tutorials mesh_graspgen_service \
        --ros-args --params-file ${GRASPGEN_CONFIG} \
        -p no_visualization:=true \
        > /tmp/graspgen_server.log 2>&1"
wait_for graspgen_container \
    "source /opt/ros/jazzy/setup.bash && ros2 service list | grep -q /generate_grasp" \
    "/generate_grasp service"

echo "== Starting cuMotion move_group stack =="
docker exec -d agile_manip_plan_container bash -c "\
    export ISAAC_ROS_WS=/colcon_ws && \
    source /opt/ros/jazzy/setup.bash && \
    source /colcon_ws/install/setup.bash && \
    ros2 launch agile_manip_moveit_config move_group.launch.py launch_rviz:=false \
        > /tmp/move_group.log 2>&1"
wait_for agile_manip_plan_container \
    "grep -q 'ready for planning' /tmp/move_group.log" \
    "cumotion_planner ready"
wait_for agile_manip_plan_container \
    "source /opt/ros/jazzy/setup.bash && ros2 action list | grep -q cumotion/move_group" \
    "cumotion/move_group action"

echo "== Pre-generating grasps (single GraspGen inference reused by all demos) =="
# Call the service once up front. graspgen_ros broadcasts the resulting
# per-grasp frames on /tf_static, which the demo clients pick up
# directly; they will skip a second service call when they see an
# existing ``grasp_0`` frame, so all demos share the same grasp set.
docker exec agile_manip_plan_container bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    ros2 service call /generate_grasp std_srvs/srv/Empty {} \
        > /tmp/graspgen_prewarm.log 2>&1" || true
sleep 2
echo "  [done] pre-generated grasps available on /tf_static"

echo ""
echo "Backends are ready. Inspect logs with:"
echo "  docker exec graspgen_container tail -f /tmp/graspgen_server.log"
echo "  docker exec agile_manip_plan_container tail -f /tmp/move_group.log"
echo ""
echo "Stop them with: bash utils/stop_backends.sh"
