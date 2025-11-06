ARG TAG="humble-ros-base-jammy"
FROM ros:$TAG

ENV HOME=/home/ws_rmf/
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install build dependencies
RUN apt-get update && apt-get install -y \
    g++-11 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-jinja2 \
    python3-jsonschema \
    python3-icecream \
    git \
    wget \
    curl \
    cmake \
    build-essential \
    libboost-all-dev \
    libwebsocketpp-dev \
    nlohmann-json3-dev \
    uuid-dev \
    libproj-dev \
    libccd-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages that are available
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake-vendor-package \
    ros-humble-backward-ros \
    || true

RUN mkdir -p /home/ws_rmf/src

WORKDIR /home/ws_rmf/

# Create repos file for RMF dependencies
RUN echo "repositories:" > rmf.repos && \
    echo "  rmf/ament_cmake_catch2:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/ament_cmake_catch2.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_api_msgs:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_api_msgs.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_battery:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_battery.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_building_map_msgs:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_building_map_msgs.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_task:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_task.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_traffic:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_traffic.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  rmf/rmf_utils:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/rmf_utils.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  thirdparty/nlohmann_json_schema_validator_vendor:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/nlohmann_json_schema_validator_vendor.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos && \
    echo "  thirdparty/pybind11_json_vendor:" >> rmf.repos && \
    echo "    type: git" >> rmf.repos && \
    echo "    url: https://github.com/open-rmf/pybind11_json_vendor.git" >> rmf.repos && \
    echo "    version: main" >> rmf.repos

# Import RMF dependencies using vcs
RUN vcs import src < rmf.repos

# Copy rmf-ros2 source code
COPY . src/rmf/rmf_ros2

# Install dependencies - continue even if some fail
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -yr || true

# Build the workspace
RUN /ros_entrypoint.sh \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
    sed -i '$isource "/home/ws_rmf/install/setup.bash"' /ros_entrypoint.sh && \
    rm -rf build devel src

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
