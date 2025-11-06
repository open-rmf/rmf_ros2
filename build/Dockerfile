ARG TAG="latest"
FROM builder-rosdep:$TAG

ARG NETRC

SHELL ["bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update
RUN mkdir -p /opt/rmf/src
WORKDIR /opt/rmf
RUN echo ${NETRC} > /root/.netrc

# copy rmf source repos
COPY /build/rmf-src src

RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    --skip-keys roscpp  \
    --skip-keys actionlib \
    --skip-keys rviz \
    --skip-keys catkin \
    --skip-keys move_base \
    --skip-keys amcl \
    --skip-keys turtlebot3_navigation \
    --skip-keys turtlebot3_bringup \
    --skip-keys move_base_msgs \
    --skip-keys dwa_local_planner \
    --skip-keys map_server \
    --skip-keys ignition \
    -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DNO_DOWNLOAD_MODELS=True

RUN sed -i '$isource "/opt/rmf/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]