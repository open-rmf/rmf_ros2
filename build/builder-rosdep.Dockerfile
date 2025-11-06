ARG ROS_DISTRO="humble"

FROM ros:$ROS_DISTRO

RUN apt update && apt install -y \
  cmake \
  curl \
  python3-pip \
  git \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN ls /etc/apt/sources.list.d/
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
#RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
#    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAMERUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
RUN sudo dpkg -i /tmp/ros2-apt-source.deb


RUN apt update && apt install -y \
  python3-colcon-common-extensions \
  python3-icecream \
  && apt-get upgrade -y \
  && apt-get install -y --fix-missing \
  && rm -rf /var/lib/apt/lists/*

# download cyclonedds and use clang for humble
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
      apt update && apt install -y \
        clang-13 \
        lldb-13 \
        lld-13 \
        ros-humble-rmw-cyclonedds-cpp \
        && rm -rf /var/lib/apt/lists/* \
      && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++-13 100; \
    fi