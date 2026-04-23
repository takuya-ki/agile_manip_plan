# agile_manip_plan

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

ROS 2 example programs and Docker environment for agile manipulation planning with an LBR iiwa 14 R820 and Robotiq 2F-140 gripper. This repository integrates [GraspGen](https://github.com/UOsaka-Harada-Laboratory/graspgen_ros) for GPU-accelerated grasp generation and [Isaac ROS cuMotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion) for CUDA-accelerated trajectory planning via MoveIt 2.

## Features

- **GraspGen demo**: real `/generate_grasp` service call with grasp TF collection
- **cuMotion demo**: real `cumotion/move_group` planning request with trajectory replay
- **Integrated pipeline**: GraspGen grasp generation → selected grasp pose → cuMotion trajectory planning
- **Obstacle-aware pipeline**: GraspGen + cuMotion with a static box obstacle injected into the MoveIt planning scene, producing a visibly different detour trajectory compared with the obstacle-free pipeline
- **Dynamic replanning pipeline**: continuously replans around a moving box obstacle at several Hz -- either driven by an external pose topic or an internal oscillator -- and pushes the latest trajectory to RViz every tick

## Dependencies

### Software

- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [Docker 27.4.1](https://docs.docker.com/engine/install/ubuntu/)
- [Docker Compose 2.32.1](https://docs.docker.com/compose/install/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### GPU (tested)

| GPU | Architecture | VRAM | CUDA | NVIDIA Driver |
| --- | --- | --- | --- | --- |
| NVIDIA GeForce RTX 3090 | Ampere | 24 GB | 12.6 | 560+ |
| NVIDIA GeForce RTX 4090 | Ada Lovelace | 24 GB | 12.6 | 560+ |

> **Minimum**: NVIDIA Ampere or later with 8 GB VRAM.

### Hardware

- [KUKA LBR iiwa 14 R820](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa)
- [Robotiq 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)

## Installation

```bash
git clone --recursive https://github.com/takuya-ki/agile_manip_plan.git
cd agile_manip_plan
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --parallel
```

`graspgen_ros` is the only Git submodule (hence `--recursive`). The
Isaac ROS trees (`isaac_ros_cumotion` with nested `curobo`, `isaac_ros_common`,
`isaac_manipulator`) are cloned directly into `/colcon_ws/src/` by
[docker/Dockerfile](docker/Dockerfile) at image-build time; `docker-compose.yaml`
bind-mounts only the host-editable local packages (`agile_manip_description`,
`agile_manip_examples`, `agile_manip_moveit_config`,
`robotiq_2f_gripper_description`, `graspgen_ros`) over the workspace, so the
Isaac sources baked into the image stay visible at runtime without any
entrypoint or symlink bootstrap.

## Usage

Start the container and enter it:

```bash
docker compose up -d
docker exec -it agile_manip_plan_container bash
```

`colcon build` runs automatically on container startup, so the workspace is ready as soon as you enter.

### Backend management (on the host)

Every demo except the plain robot-model visualization talks to a
GraspGen service and the cuMotion `MoveGroup` action server, each
running in its respective container. Two host-side helpers manage
their lifecycle so you don't have to remember the `docker exec`
incantations:

```bash
bash utils/start_backends.sh   # launch GraspGen + cuMotion, pre-warm grasps
bash utils/stop_backends.sh    # clean shutdown (pkill -9 + retry)
```

`start_backends.sh` calls `/generate_grasp` once at the end so the
resulting grasp TF frames and `MarkerArray` are ready on latched
topics; every demo client then reuses the same grasp set instead of
invoking GraspGen again. Iterate on one demo by running
`bash utils/run_<demo>_demo.sh` while the backends are up, and stop
them with `stop_backends.sh` when finished.

Each demo lists both the host wrapper (`bash utils/run_<demo>_demo.sh`,
runs from the repository root) and the equivalent `ros2 launch`
command you would issue from inside the container (`docker exec -it
agile_manip_plan_container bash`).

### 1. Robot model visualization

No backends needed.

<img src="dataset/demo/01_display_demo.png" width="640"></img>

```bash
# host
bash utils/run_display_demo.sh
# container
ros2 launch agile_manip_description display.launch.py
```

### 2. GraspGen demo

Requires the GraspGen backend (`bash utils/start_backends.sh`). Calls
the real `/generate_grasp` service and republishes the grasp TFs as a
`PoseArray`. The `graspgen_ros` source tree is tracked as a git
submodule.

<img src="dataset/demo/02_graspgen_demo.png" width="640"></img>

```bash
# host (antipodal is the default; pass ``suction`` to switch)
bash utils/run_graspgen_demo.sh
bash utils/run_graspgen_demo.sh suction
# container
ros2 launch agile_manip_examples graspgen_demo.launch.py
ros2 launch agile_manip_examples graspgen_demo.launch.py \
    config:=$(ros2 pkg prefix agile_manip_examples)/share/agile_manip_examples/config/graspgen_suction.yaml
```

### 3. cuMotion demo

Requires the cuMotion `MoveGroup` action server from
`bash utils/start_backends.sh`. Sends a real
`moveit_msgs/action/MoveGroup` request and replays the returned
trajectory in RViz.

<img src="dataset/demo/03_cumotion_demo.png" width="640"></img>

```bash
# host
bash utils/run_cumotion_demo.sh
# container
ros2 launch agile_manip_examples cumotion_demo.launch.py
```

### 4. Integrated grasp + motion planning pipeline

Requires both backends. Calls GraspGen for candidate grasp TFs,
selects one pose, and asks cuMotion for a trajectory plan to it.

<img src="dataset/demo/04_grasp_and_motion_demo.png" width="640"></img>

```bash
# host
bash utils/run_grasp_and_motion_demo.sh
# container
ros2 launch agile_manip_examples grasp_and_motion_demo.launch.py
```

### 5. Obstacle-aware pipeline

Requires both backends. Attaches a static box `CollisionObject` to
each cuMotion goal via
`MoveGroup.Goal.planning_options.planning_scene_diff` so cuRobo's
world-collision model includes it, producing a visibly different
detour trajectory compared with the obstacle-free pipeline.

<img src="dataset/demo/05_obstacle_aware_demo.png" width="640"></img>

```bash
# host
bash utils/run_obstacle_aware_demo.sh
# container
ros2 launch agile_manip_examples obstacle_aware_demo.launch.py
```

### 6. Dynamic replanning pipeline

Requires both backends. A moving box obstacle (driven by the node's
internal sinusoidal oscillator by default, or by any external
publisher on `/dynamic_obstacle/pose`) is attached to a fresh
cuMotion goal at `replan_rate_hz` (default 5 Hz). The robot is held
at the home pose; only the planned trajectory -- the blue line
strip -- morphs as the obstacle sweeps, so the replanning cadence is
visible in RViz.

<img src="dataset/demo/06_dynamic_replan_demo.png" width="640"></img>

```bash
# host
bash utils/run_dynamic_replan_demo.sh
# container
ros2 launch agile_manip_examples dynamic_replan_demo.launch.py
```

Override the replan rate or obstacle motion from the command line:

```bash
ros2 launch agile_manip_examples dynamic_replan_demo.launch.py \
    config:=$(ros2 pkg prefix agile_manip_examples)/share/agile_manip_examples/config/dynamic_replan.yaml
```

(or pass `-p replan_rate_hz:=8.0` etc. directly to
`ros2 run agile_manip_examples dynamic_replan_planner`.)

## Benchmark

A headless [`benchmark_harness`](colcon_ws/src/agile_manip_examples/agile_manip_examples/benchmark_harness.py)
node replays the GraspGen + cuMotion plan request `iterations` times
and writes per-iteration metrics (planning time, waypoints, trajectory
length, final residual, error code) to a CSV. Use it to quantify the
"ultra-fast planning" claim and to compare grasp-selection strategies
on a common workload.

### Results

Planning-only timing (cuMotion pose goal from home joint configuration
to a GraspGen pose, 20 iterations, single pre-warmed grasp set of 32
candidates). Hardware: **NVIDIA GeForce RTX 3090 (24 GB, CUDA 12.6)**.

Two strategies are compared for choosing *which* GraspGen candidate to
plan to (both use the same cuMotion planner with identical settings):

- **Confidence-first**: sort by GraspGen confidence, try the highest
  score first -- the default in existing launches.
- **Multi-criteria**: weighted score combining GraspGen confidence
  with a reachability sub-score that favours grasps inside the
  iiwa14's comfortable reach envelope (peaks at 0.5 m from the base,
  smoothly decaying with a cosine window). Weights are ROS
  parameters; both terms are intentionally task-agnostic.

*On confidence*: this is not a value this repository computes.
GraspGen's own neural network outputs a success-probability estimate
in `[0, 1]` for every candidate it generates; those values are
written out alongside the grasp poses to the YAML at
`grasp_result_path` (e.g.
`/colcon_ws/src/graspgen_ros/share/grasp_result/example_mesh_antipodal.yaml`),
and [`load_grasp_scores`](colcon_ws/src/agile_manip_examples/agile_manip_examples/graspgen_utils.py)
just reads them. The score reflects what GraspGen's model learned
about grasp stability on the object mesh.

| Grasp selection strategy        | success | median (ms) | mean (ms) | p95 (ms) | min-max (ms)   | median traj. length (m) | max residual (mm) |
|---------------------------------|---------|-------------|-----------|----------|----------------|-------------------------|-------------------|
| Confidence-first                | 20/20   | **192.1**   | 193.2     | 211.1    | 180.8 – 211.5  | 0.662                   | 0.01              |
| Multi-criteria (conf + reach)   | 20/20   | **160.3**   | 168.2     | 201.4    | 153.4 – 201.6  | 0.585                   | 0.01              |

Both strategies land within 0.01 mm of the requested pose. Total
per-cycle time is dominated by the MoveGroup action round-trip (the
cuMotion solve itself is well below that budget). In this scene
Multi-criteria picks a reach-biased grasp that yields a shorter
trajectory and a faster overall plan on the median, illustrating
that confidence alone does not always pick the cheapest candidate;
tuning `multi_criteria_weight_{confidence,reach}` lets you slide
between them.

### Reproduce

```bash
# Host
docker compose up -d
bash utils/start_backends.sh

# 20 iterations per mode (default config lives in
# colcon_ws/src/agile_manip_examples/config/benchmark.yaml)
bash utils/run_benchmark.sh highest_confidence 20
bash utils/run_benchmark.sh multi_criteria 20

bash utils/stop_backends.sh
```

Each run prints a summary on stdout and writes
`/tmp/benchmark_<timestamp>.csv` inside the container. Override the
output path or any other parameter via the `ros2 run --ros-args -p`
command that the wrapper expands.

### Dynamic replanning rate

The dynamic replanning demo exercises a tighter closed loop: a fresh
cuMotion goal every tick with a moving obstacle attached to the
planning-scene diff. Running the default oscillator for 30 s on the
same RTX 3090 hardware yields:

| Replan rate target | Sustained rate | Successful plans | Per-plan latency |
|--------------------|----------------|------------------|------------------|
| 5 Hz               | **~4.8 Hz**    | 118 / 146 (81%)  | 170 – 190 ms     |

Failed ticks are expected: whenever the obstacle fully covers the
line from home to the selected grasp, cuMotion returns
`NO_IK_SOLUTION` and the demo simply waits one tick for the obstacle
to move out of the way. To reproduce:

```bash
bash utils/start_backends.sh
bash utils/run_dynamic_replan_demo.sh     # watch RViz for ~30 s
bash utils/stop_backends.sh
```

## Directory structure

```text
agile_manip_plan/
├── docker/
│   ├── Dockerfile                          # Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.6
│   └── isaac_ros_common_stub/              # minimal isaac_ros_common replacement
├── docker-compose.yaml
├── colcon_ws/src/
│   ├── agile_manip_description/            # iiwa14 + Robotiq 2F-140 URDF
│   ├── agile_manip_examples/               # demo nodes and launch files
│   ├── agile_manip_moveit_config/          # MoveIt 2 + cuMotion configuration
│   ├── graspgen_ros/                       # GraspGen upstream (git submodule)
│   └── robotiq_2f_gripper_description/     # Robotiq 2F-140 URDF + meshes
└── utils/                                  # host wrappers (start / stop / run demos)
```

## Key concepts

### GraspGen

[GraspGen](https://github.com/UOsaka-Harada-Laboratory/graspgen_ros) is an NVIDIA GPU-accelerated grasp generation tool. It takes object meshes or point clouds as input and generates candidate grasp poses (antipodal or suction). In this repository, `graspgen_ros` is tracked as a submodule and its outputs are used as target end-effector poses for trajectory planning.

### Isaac ROS cuMotion

[Isaac ROS cuMotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion) provides CUDA-accelerated motion planning integrated as a MoveIt 2 planner plugin. It leverages [cuRobo](https://github.com/NVlabs/curobo) for parallel trajectory optimization on the GPU. In this repository it is intentionally kept as an external dependency and cloned into the Docker image during build.

## Authors and Contributors

- [Takuya Kiyokawa](https://takuya-ki.github.io/)
- [Claude Code](https://claude.com/claude-code) (Anthropic)

## License

This software is released under the [BSD-3-Clause License](LICENSE).
