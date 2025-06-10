ARG UBUNTU_TAG
FROM ubuntu:${UBUNTU_TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# Install
RUN apt update \
  && apt upgrade -y \
  && apt install -y sudo cmake curl git python3 wget gnupg2 g++ gcc software-properties-common locales gdb \
  && locale-gen en_US.UTF-8

# Set environment variables to use UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Add ROS Independent Colcon
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && apt install /tmp/ros2-apt-source.deb \
  && apt update -y -qq

# Add Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add Colcon
RUN curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | bash

# Add Tesseract Robotics PPA
RUN add-apt-repository ppa:levi-armstrong/tesseract-robotics

# Install the dependency repositories
RUN apt update \
  && apt install -y \
  python3-colcon-common-extensions \
  libbenchmark-dev \
  google-perftools \
  libeigen3-dev \
  libboost-all-dev \
  coinor-libipopt-dev \
  liborocos-kdl-dev \
  libconsole-bridge-dev \
  liboctomap-dev \
  libfcl-dev \
  libpcl-dev \
  libompl-dev \
  libyaml-cpp-dev \
  libtinyxml2-dev \
  libbullet-dev \
  libbullet-extras-dev \
  libgraphviz-dev \
  libqwt-qt5-dev \
  libqt5svg5-dev \
  libqglviewer-dev-qt5 \
  libxcb-cursor-dev \
  libgz-rendering7-dev \
  libgz-common5-av-dev \
  libgz-common5-profiler-dev \
  taskflow \
  qt-advanced-docking-system

# Install the Qt Creator and Plugin
RUN wget https://download.qt.io/official_releases/qtcreator/14.0/14.0.2/cpack_experimental/qtcreator-opensource-linux-x86_64-14.0.2.deb \
  && dpkg -i qtcreator-opensource-linux-x86_64-14.0.2.deb \
  && echo 'export PATH=$PATH:/opt/qt-creator/bin' >> ~/.bashrc \
  && echo 'export PATH=$PATH:/opt/qt-creator/bin' >> ~/.profile \
  && echo 'export XDG_DATA_DIRS=$XDG_DATA_DIRS:/opt/qt-creator/share/' >> ~/.profile \
  && wget https://github.com/ros-industrial/ros_qtc_plugin/releases/download/14.2/ROSProjectManager-14.2-Linux-x86_64.deb \
  && dpkg -i ROSProjectManager-14.2-Linux-x86_64.deb

# Define build arguments for USERNAME, UID, and GID
ARG USERNAME
ARG USER_ID
ARG GROUP_ID

# Echo Args
RUN echo "USERNAME: ${USERNAME}" \
  && echo "USER_ID: ${USER_ID}" \
  && echo "GROUP_ID: ${GROUP_ID}" \
  && echo ""

# Create a group and user with the specified username, UID, and GID
RUN groupadd -g ${GROUP_ID} ${USERNAME} \
  && useradd -l -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME} -d /home/${USERNAME} \
  && chown ${USER_ID}:${GROUP_ID} /home/${USERNAME} \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the default user
USER ${USERNAME}
WORKDIR /home/${USERNAME}
