ARG TAG
FROM ubuntu:${TAG}
ARG TAG # Redeclare to use in this stage

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update \
  && apt upgrade -y \
  && apt install -y cmake curl git python3 liboctomap-dev

# Add ROS2 sources to install ROS infrastructure tools
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && apt install /tmp/ros2-apt-source.deb \
  && apt update -y -qq

# Install and configure ROS infrastructure tools
RUN apt install -y python3-vcstool python3-colcon-common-extensions python3-rosdep \
  && rosdep init \
  && rosdep update

# Install the dependency repositories
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/tesseract
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract \
  cd ${WORKSPACE_DIR} \
  && if [ "$TAG" = "focal" ]; then \
       vcs import src < src/tesseract/dependencies_focal.repos --shallow; \
     else \
       vcs import src < src/tesseract/dependencies.repos --shallow; \
     fi \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract \
  cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log
