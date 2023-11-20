ARG TAG
FROM ubuntu:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update \
  && apt upgrade -y \
  && apt install -y cmake curl git python3 python3-distutils python3-pip liboctomap-dev \
  && python3 -m pip install vcstool -q \
  && python3 -m pip install colcon-common-extensions -q \
  && python3 -m pip install rosdep -q \
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
