# First Stage: Build the project with build dependencies
FROM ros:humble-ros-base AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2
COPY . ./src
COPY --from=ghcr.io/rxsio/firo_common:humble /ros2/install /ros2/install

RUN apt-get update \
        && rosdep update \
        && source /opt/ros/$ROS_DISTRO/setup.bash \
        && rosdep install --from-paths src --ignore-src -iy -tbuild -tbuildtool -tbuild_export -tbuildtool_export \
        && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
        && rm -rf /var/lib/apt/lists/*

# Second Stage: Create a clean image with only exec dependencies and build results
FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2
COPY --from=build /ros2/install /ros2/install

RUN apt-get update \
        && apt-get install -y python3-rosdep \
        && rosdep init \
        && rosdep update \
        && source /opt/ros/$ROS_DISTRO/setup.bash \
        && rosdep install --from-paths install/**/share --ignore-src -iy --dependency-types=exec \
        && apt-get purge -y python3-rosdep \
        && SUDO_FORCE_REMOVE=yes apt-get autoremove --purge -y \
        && rm -rf /var/lib/apt/lists/* \
        && echo "source $PWD/install/setup.bash" >> ~/.bashrc

# Set the default entrypoint
CMD ["/bin/bash"]
