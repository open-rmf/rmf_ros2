ARG TAG="foxy-ros-base-focal"
FROM ros:$TAG

ENV HOME /home/ws_rmf/

RUN apt-get update && apt-get install -y g++-8 \
    ros-foxy-rmw-cyclonedds-cpp
RUN mkdir -p /home/ws_rmf

WORKDIR  /home/ws_rmf/
COPY . src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -yr

RUN /ros_entrypoint.sh \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
    sed -i '$isource "/home/ws_rmf/install/setup.bash"' /ros_entrypoint.sh && \
    rm -rf build devel src

# todo: should have a multistage build

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
