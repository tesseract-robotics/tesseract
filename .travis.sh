#!/bin/bash

# Software License Agreement - BSD License
#
# Inspired by MoveIt! travis https://github.com/ros-planning/moveit_core/blob/09bbc196dd4388ac8d81171620c239673b624cc4/.travis.yml
# Inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis
# Inspired by ROS Industrial https://github.com/ros-industrial/industrial_ci
#
# Author:  Dave Coleman, Isaac I. Y. Saito, Robert Haschke

export CI_SOURCE_PATH=$(pwd) # The repository code in this pull request that we are testing
export HIT_ENDOFSCRIPT=false
export REPOSITORY_NAME=${PWD##*/}
export CATKIN_WS=/root/catkin_ws

# Helper functions
source ${CI_SOURCE_PATH}/.travis_utils.sh

# Run all CI in a Docker container
if ! [ "$IN_DOCKER" ]; then
    logHighlight "---"
    logHighlight "Testing branch '$TRAVIS_BRANCH' of '$REPOSITORY_NAME' on ROS '$ROS_DISTRO'"

    logHighlight "Starting Docker image: $DOCKER_IMAGE"

    # Pull first to allow us to hide console output
    docker pull $DOCKER_IMAGE > /dev/null

    # Start Docker container
    docker run \
        -e TRAVIS \
        -e ROS_REPO \
        -e ROS_DISTRO \
        -e BEFORE_SCRIPT \
        -e CI_SOURCE_PATH \
        -e UPSTREAM_WORKSPACE \
        -e TRAVIS_BRANCH \
        -e TEST \
        -e TEST_BLACKLIST \
        -e CC \
        -e CXX \
        -e CFLAGS \
        -e CXXFLAGS \
        -e IN_DOCKER=true \
        -v $(pwd):/root/$REPOSITORY_NAME \
        -v $HOME/.ccache:/root/.ccache \
        -t \
        --network host \
        $DOCKER_IMAGE \
        /bin/bash -c "cd /root/$REPOSITORY_NAME; source .travis.sh;"
    return_value=$?

    if [ $return_value -eq 0 ]; then
        logHighlight "$DOCKER_IMAGE container finished successfully"
        HIT_ENDOFSCRIPT=true;
        exit 0
    fi
    logError "$DOCKER_IMAGE container finished with errors"
    exit 1 # error
fi

# If we are here, we can assume we are inside a Docker container
logHighlight "Inside Docker container"

# Define CC/CXX defaults and print compiler version info
export CC=${CC:-cc}
export CXX=${CXX:-c++}
$CXX --version

# Update the sources
travis_run apt-get -qq update

# Install Catkin Tools
travis_run apt-get -qq install -y python-catkin-tools

# Install Clang Format
travis_run apt-get -qq install -y clang-format

# Make sure the packages are up-to-date
travis_run apt-get -qq dist-upgrade

# Split for different tests
for t in $TEST; do
    case "$t" in
        "clang-format")
            check_clang_format || exit 1
            exit 0 # This runs as an independent job, do not run regular travis test
        ;;
    esac
done

# Enable ccache
travis_run apt-get -qq install ccache
export PATH=/usr/lib/ccache:$PATH

# Install and run xvfb to allow for X11-based unittests on DISPLAY :99
travis_run apt-get -qq install xvfb mesa-utils
Xvfb -screen 0 640x480x24 :99 &
export DISPLAY=:99.0
travis_run_true glxinfo

# Setup rosdep - note: "rosdep init" is already setup in base ROS Docker image
travis_run rosdep update

# Create workspace
travis_run mkdir -p $CATKIN_WS/src
travis_run cd $CATKIN_WS/src

# Install dependencies necessary to run build using .rosinstall files
if [ ! "$UPSTREAM_WORKSPACE" ]; then
    export UPSTREAM_WORKSPACE="debian";
fi
case "$UPSTREAM_WORKSPACE" in
    debian)
        logInfo "Obtain deb binary for upstream packages."
        ;;
    http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
        travis_run wstool init .
        # Handle multiple rosintall entries.
        (  # parentheses ensure that IFS is automatically reset
            IFS=','  # Multiple URLs can be given separated by comma.
            for rosinstall in $UPSTREAM_WORKSPACE; do
                travis_run wstool merge -k $rosinstall
            done
        )
        ;;
    *) # Otherwise assume UPSTREAM_WORKSPACE is a local file path
        travis_run wstool init .
        if [ -e $CI_SOURCE_PATH/$UPSTREAM_WORKSPACE ]; then
            # install (maybe unreleased version) dependencies from source
            travis_run wstool merge file://$CI_SOURCE_PATH/$UPSTREAM_WORKSPACE
        else
            logError "No rosinstall file found, aborting" && exit 1
        fi
        ;;
esac

# download upstream packages into workspace
if [ -e .rosinstall ]; then
    # ensure that the downstream is not in .rosinstall
    travis_run_true wstool rm $REPOSITORY_NAME
    # perform shallow checkout: only possible with wstool init
    travis_run mv .rosinstall rosinstall
    travis_run cat rosinstall
    travis_run wstool init --shallow . rosinstall
fi

# link in the repo we are testing
travis_run ln -s $CI_SOURCE_PATH .

# Debug: see the files in current folder
travis_run ls -a

# Run before script
if [ "${BEFORE_SCRIPT// }" != "" ]; then
    travis_run sh -c "${BEFORE_SCRIPT}";
fi

# Install source-based package dependencies
travis_run rosdep install -y -q -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO

# Change to base of workspace
travis_run cd $CATKIN_WS

# Initialize Catkin Workspace
travis_run catkin init

# Configure catkin
travis_run catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3"

# Console output fix for: "WARNING: Could not encode unicode characters"
export PYTHONIOENCODING=UTF-8

# For a command that doesn’t produce output for more than 10 minutes, prefix it with travis_run_wait
travis_run_wait 60 catkin build --no-status --summarize || exit 1

travis_run ccache -s

# Source the new built workspace
travis_run source install/setup.bash;

# Choose which packages to run tests on
logHighlight "Test blacklist: $TEST_BLACKLIST"
logHighlight "--------------"
TEST_PKGS=$(catkin_topological_order $CATKIN_WS/src/$REPOSITORY_NAME --only-names | grep -Fvxf <(echo "$TEST_BLACKLIST" | tr ' ;,' '\n') | tr '\n' ' ')

if [ -n "$TEST_PKGS" ]; then
    TEST_PKGS="$TEST_PKGS --no-deps";
    # Fix formatting of list of packages to work correctly with Travis
    IFS=' ' read -r -a TEST_PKGS <<< "$TEST_PKGS"
fi
logHighlight "Test packages: $TEST_PKGS"

# Run catkin package tests
travis_run catkin build ${TEST_PKGS[@]} --no-status --summarize --make-args tests

# Run non-catkin package tests
travis_run catkin build ${TEST_PKGS[@]} --no-status --summarize --catkin-make-args run_tests

# Show failed tests
for file in $(catkin_test_results | grep "\.xml:" | cut -d ":" -f1); do
    travis_run cat $file
done

# Show test results summary and throw error if necessary
travis_run catkin_test_results

logHighlight "Travis script has finished successfully"
HIT_ENDOFSCRIPT=true
exit 0
