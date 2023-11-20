ARG TAG
FROM ubuntu:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update \
  && apt install -y \
    curl \
    lsb-release \
    git \
    build-essential

# Install ROS sources and tooling dependencies
COPY .github/workflows/add_ros_apt_sources.sh /tmp/add_ros_apt_sources.sh
RUN ./tmp/add_ros_apt_sources.sh \
  && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    liboctomap-dev \
  && rosdep init \
  && rosdep update

# Install the dependency repositories
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/tesseract
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract \
  cd ${WORKSPACE_DIR} \
  && vcs import src < src/tesseract/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract \
  cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log
