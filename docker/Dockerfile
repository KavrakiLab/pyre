FROM ros:melodic-ros-base
MAINTAINER Constantinos Chamzas chamzas@rice.edu

# Download Dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get update && \
    apt-get install -y \
      build-essential wget pkg-config python-catkin-tools cmake git zip \
      libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
      ros-${ROS_DISTRO}-moveit
      
# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws/src

#Clone robowflex_resources
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources

#Clone pyre 
RUN git clone --depth 1 https://github.com/KavrakiLab/pyre

#Clone only robowflex_library and robowflex_ompl instead of all of robowflex
WORKDIR /ws
RUN mkdir -p repos
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex ./repos/

RUN ln -s ../repos/robowflex_library src/robowflex_library 
RUN ln -s ../repos/robowflex_ompl src/robowflex_ompl

WORKDIR /ws

## Build everything 
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify
