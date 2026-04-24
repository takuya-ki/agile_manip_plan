ARG CUDA_VERSION=12.6.3
ARG UBUNTU_VERSION=24.04
ARG ISAAC_ROS_CUMOTION_REF=v4.3-0
ARG ISAAC_ROS_COMMON_REF=v4.3-0
ARG ISAAC_MANIPULATOR_REF=v4.3-0
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}
ARG ISAAC_ROS_CUMOTION_REF
ARG ISAAC_ROS_COMMON_REF
ARG ISAAC_MANIPULATOR_REF

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ISAAC_ROS_WS=/colcon_ws

# ============================================================
# 1. Basic dependencies
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    gnupg2 \
    libgl1 \
    libegl1 \
    libgl1-mesa-dri \
    libglib2.0-0 \
    locales \
    lsb-release \
    mesa-utils \
    ninja-build \
    python3-pip \
    software-properties-common \
    sudo \
    vim \
    wget \
    x11-xserver-utils \
    byobu \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 2. ROS 2 Jazzy installation
# ============================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 3. Python dependencies
# ============================================================
# numpy pinned <2 because curobo_core extensions are compiled against
# NumPy 1.x; cupy pinned <14 because 14.x requires NumPy >= 2.
RUN pip3 install --break-system-packages --ignore-installed \
    setuptools==75.8.2 \
    "numpy<2" \
    scipy \
    matplotlib \
    open3d \
    trimesh \
    yourdfpy \
    warp-lang \
    hdbscan \
    setuptools-scm \
    transforms3d

# PyTorch + torchvision with CUDA 12.1 wheels (cuMotion targets cu121).
RUN pip3 install --break-system-packages \
    --index-url https://download.pytorch.org/whl/cu121 \
    torch==2.4.0 \
    torchvision==0.19.0

# CuPy built for CUDA 12.x (curobo uses cupy for sphere collision).
RUN pip3 install --break-system-packages "cupy-cuda12x<14"

# ============================================================
# 4. rosdep initialization
# ============================================================
RUN rosdep init || true && rosdep update

# ============================================================
# 5. Isaac ROS sources cloned directly into the workspace
# ============================================================
# These trees live inside the image. docker-compose bind-mounts only the
# local packages (agile_manip_*, robotiq_2f_gripper_description,
# graspgen_ros) over /colcon_ws/src, so these Isaac directories remain
# visible exactly as Docker built them.
RUN mkdir -p /colcon_ws/src \
    && git clone --depth 1 --branch ${ISAAC_ROS_CUMOTION_REF} \
       --recurse-submodules --shallow-submodules \
       https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git \
       /colcon_ws/src/isaac_ros_cumotion \
    && git clone --depth 1 --branch ${ISAAC_ROS_COMMON_REF} \
       https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
       /colcon_ws/src/isaac_ros_common_full \
    && git clone --depth 1 --branch ${ISAAC_MANIPULATOR_REF} \
       https://github.com/NVIDIA-ISAAC-ROS/isaac_manipulator.git \
       /colcon_ws/src/isaac_manipulator

# ``COLCON_IGNORE`` on the packages we don't build. These are not
# time-based optimisations: each one fails to build in this image
# because it depends on something we deliberately do not ship here,
# namely NITROS / GXF, NVIDIA VPI, Isaac Sim, the Panda/UR example
# robots, or upstream nvblox_msgs (ESDF-based obstacle avoidance is
# not used; see the sed patch below). Listing the *kept* subpackages
# per repo is shorter and auto-prunes any new packages upstream adds
# unless we explicitly opt them in.
RUN keep_only() { \
        repo="$1"; shift; \
        for dir in /colcon_ws/src/${repo}/*/; do \
            name="$(basename "${dir}")"; \
            keep=0; \
            for want in "$@"; do \
                if [ "${name}" = "${want}" ]; then keep=1; break; fi; \
            done; \
            if [ ${keep} -eq 0 ] && [ -f "${dir}/package.xml" ]; then \
                touch "${dir}/COLCON_IGNORE"; \
            fi; \
        done; \
    }; \
    keep_only isaac_ros_cumotion \
        curobo_core \
        isaac_ros_cumotion \
        isaac_ros_cumotion_interfaces \
        isaac_ros_cumotion_moveit \
        isaac_ros_cumotion_python_utils \
        isaac_ros_cumotion_robot_description \
    && keep_only isaac_ros_common_full \
        isaac_ros_launch_utils \
        isaac_ros_test \
    && keep_only isaac_manipulator \
        isaac_manipulator_interfaces \
        isaac_manipulator_ros_python_utils

# --------------------------------------------------------------
# Load-bearing upstream patches -- read before modifying either.
# --------------------------------------------------------------
# Both of the blocks below are workarounds for things the upstream
# Isaac ROS stack hard-couples to NVIDIA-only dependencies (nvblox +
# VPI) that we intentionally do not vendor here. If either patch
# silently stops applying -- for example because upstream refactors
# the import line or renames the ament resource -- the Dockerfile
# will still "succeed" but cuMotion will fail to import at runtime.
# When bumping the pinned upstream commit, verify both steps still
# produce the expected output (see in-line comments).

# [PATCH 1] cuMotion's Python planner node hard-imports
# ``nvblox_msgs.srv`` at module load even when the ESDF path is
# disabled (``read_esdf_world`` stays False in this project).
# Lazy-import the symbol and strip the ``nvblox_msgs`` rosdep entry
# from the package manifest so we can run cuMotion without cloning
# the full Isaac ROS nvblox stack.
#
# Regression signal: after a ``docker compose build`` run,
#   grep '^from nvblox_msgs' /colcon_ws/src/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py
# must return nothing. If upstream moves the import to another line
# or spells it differently, the sed above is a no-op and the
# container fails at first cuMotion service call with ImportError.
RUN sed -i \
    's|^from nvblox_msgs.srv import EsdfAndGradients$|try:\n    from nvblox_msgs.srv import EsdfAndGradients\nexcept ImportError:\n    EsdfAndGradients = None|' \
    /colcon_ws/src/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py \
    && sed -i '/<depend>nvblox_msgs<\/depend>/d' \
       /colcon_ws/src/isaac_ros_cumotion/isaac_ros_cumotion/package.xml \
    && ! grep -q '^from nvblox_msgs' \
        /colcon_ws/src/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py

# [PATCH 2] Minimal stand-in for the ``isaac_ros_common`` package. The
# upstream package requires NVIDIA VPI (libnvvpi4) which is not
# vendored here; downstream cuMotion packages only consume the
# ``isaac_ros_common_scripts_path`` / ``isaac_ros_common_cmake_path``
# ament_index resources plus a handful of cmake helpers, so a stub
# that installs those is enough. ``package.xml`` and
# ``CMakeLists.txt`` are committed under
# ``docker/isaac_ros_common_stub/``; ``scripts/`` and ``cmake/`` are
# copied from the upstream clone at build time so we don't have to
# vendor the upstream content in git.
#
# Regression signal: if upstream adds a new ament resource or a new
# cmake macro that a downstream cuMotion package consumes, the build
# here fails with a missing-resource / missing-macro error at
# ``colcon build`` time. When that happens, copy the missing file
# into the stub rather than pulling in the real ``isaac_ros_common``.
COPY docker/isaac_ros_common_stub/ /colcon_ws/src/isaac_ros_common_stub/
RUN mkdir -p /colcon_ws/src/isaac_ros_common_stub/scripts \
             /colcon_ws/src/isaac_ros_common_stub/cmake \
    && cp /colcon_ws/src/isaac_ros_common_full/isaac_ros_common/scripts/isaac_ros_common-version-info.py \
          /colcon_ws/src/isaac_ros_common_full/isaac_ros_common/scripts/isaac_ros_version_embed.py \
          /colcon_ws/src/isaac_ros_common_stub/scripts/ \
    && cp -r /colcon_ws/src/isaac_ros_common_full/isaac_ros_common/cmake/. \
             /colcon_ws/src/isaac_ros_common_stub/cmake/

# ============================================================
# 6. Shell setup
# ============================================================
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /colcon_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc \
    && echo "export ISAAC_ROS_WS=/colcon_ws" >> /root/.bashrc

WORKDIR /colcon_ws
