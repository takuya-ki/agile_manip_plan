#!/bin/bash
# Stop the GraspGen service and cuMotion MoveGroup stack started by
# ``utils/start_backends.sh``. Uses ``pkill -9`` and retries so stale
# processes from previous sessions are guaranteed to exit before the
# caller starts a new stack (otherwise several ``mesh_graspgen_service``
# instances can coexist and take turns responding to /generate_grasp
# with different meshes).
set -euo pipefail

# Pattern trick: wrapping one character in ``[X]`` keeps the regex match
# against target processes (whose cmdlines contain the literal character)
# while preventing pkill/pgrep from matching its own ``bash -c`` parent
# (whose cmdline contains the literal ``[X]`` sequence, which the regex
# does not match). Without this, pkill SIGKILLs the shell running it and
# docker exec returns 137, causing the script to abort under set -e.
force_kill() {
    local container="$1"
    local pattern="$2"
    docker exec "${container}" bash -c "pkill -9 -f '${pattern}' 2>/dev/null || true"
}

wait_gone() {
    local container="$1"
    local pattern="$2"
    local count_cmd="pgrep -af '${pattern}' 2>/dev/null | wc -l"
    for _ in $(seq 1 20); do
        local remaining
        remaining=$(docker exec "${container}" bash -c "${count_cmd}" || echo 0)
        if [[ "${remaining}" -eq 0 ]]; then
            return 0
        fi
        docker exec "${container}" bash -c \
            "pkill -9 -f '${pattern}' 2>/dev/null || true"
        sleep 0.5
    done
    echo "  [warn] ${pattern} survived in ${container}" >&2
}

echo "== Stopping cuMotion stack in agile_manip_plan_container =="
for pat in '[c]umotion_planner_node' \
           '[s]tatic_planning_scene' \
           'moveit_ros_move_group/[m]ove_group' \
           'ros2 launch [a]gile_manip_moveit_config' \
           'robot_state_publisher/[r]obot_state_publisher' \
           'joint_state_publisher/[j]oint_state_publisher'; do
    force_kill agile_manip_plan_container "${pat}"
done
wait_gone agile_manip_plan_container '[c]umotion_planner_node'
wait_gone agile_manip_plan_container 'moveit_ros_move_group/[m]ove_group'

echo "== Stopping GraspGen services in graspgen_container =="
for pat in 'graspgen_tutorials/[m]esh_graspgen_service' \
           'graspgen_tutorials/[p]ointcloud_graspgen_service' \
           'graspgen_tutorials/[p]ointcloud_collisionfree_graspgen_service'; do
    force_kill graspgen_container "${pat}"
done
wait_gone graspgen_container 'graspgen_tutorials/[m]esh_graspgen_service'

echo "Done."
