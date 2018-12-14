#!/bin/bash
#********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, University of Colorado, Boulder
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Univ of CO, Boulder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/

# Author: Dave Coleman <dave@dav.ee>, Robert Haschke
# Desc: Utility functions used to make CI work better in Travis

#######################################

RED='\e[31m'
GREEN='\e[32m'
YELLOW='\e[93m'
NC='\e[0m' # No Color

function logError {
    echo -e "${RED}$1${NC}"
}

function logWarn {
    echo -e "${YELLOW}$1${NC}"
}

function logInfo {
    echo -e "${NC}$1${NC}"
}

function logHighlight {
    echo -e "${GREEN}$1${NC}"
}

export TRAVIS_FOLD_COUNTER=0

#######################################
# Start a Travis fold with timer
#
# Arguments:
#   travis_fold_name: name of line
#   command: action to run
#######################################
function travis_time_start {
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    local COMMAND=${@:2} # all arguments except the first

    # Start fold
    echo -e "\e[0Ktravis_fold:start:$TRAVIS_FOLD_NAME"
    # Output command being executed
    echo -e "\e[0Ktravis_time:start:$TRAVIS_TIME_ID\e[34m$COMMAND\e[0m"
}

#######################################
# Wraps up the timer section on Travis CI (that's started mostly by travis_time_start function).
#
# Arguments:
#   travis_fold_name: name of line
#######################################
function travis_time_end {
    if [ -z $TRAVIS_START_TIME ]; then
        echo '[travis_time_end] var TRAVIS_START_TIME is not set. You need to call `travis_time_start` in advance.';
        return;
    fi
    local TRAVIS_END_TIME=$(date +%s%N)
    local TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))

    # Output Time
    echo -e "travis_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\e[0K"
    # End fold
    echo -e -n "travis_fold:end:$TRAVIS_FOLD_NAME\e[0m"

    unset TRAVIS_START_TIME
    unset TRAVIS_TIME_ID
    unset TRAVIS_FOLD_NAME
}

#######################################
# Display command in Travis console and fold output in dropdown section
#
# Arguments:
#   command: action to run
#######################################
function travis_run_impl() {
  local command=$@

  let "TRAVIS_FOLD_COUNTER += 1"
  travis_time_start ros_industrial_ci.$TRAVIS_FOLD_COUNTER $command
  # actually run command
  $command
  result=$?
  travis_time_end
  return $result
}

#######################################
# Run a command and do folding and timing for it
#   Return the exit status of the command
function travis_run() {
  travis_run_impl $@ || exit $?
}

#######################################
# Same as travis_run but return 0 exit status, thus ignoring any error
function travis_run_true() {
  travis_run_impl $@ || return 0
}

#######################################
# Same as travis_run, but issue some output regularly to indicate that the process is still alive
# from: https://github.com/travis-ci/travis-build/blob/d63c9e95d6a2dc51ef44d2a1d96d4d15f8640f22/lib/travis/build/script/templates/header.sh
function travis_run_wait() {
  local timeout=$1 # in minutes

  if [[ $timeout =~ ^[0-9]+$ ]]; then
    # looks like an integer, so we assume it's a timeout
    shift
  else
    # default value
    timeout=20
  fi

  local cmd=$@
  let "TRAVIS_FOLD_COUNTER += 1"
  travis_time_start ros_industrial_ci.$TRAVIS_FOLD_COUNTER $cmd

  # Disable bash's job control messages
  set +m
  # Run actual command in background
  $cmd &
  local cmd_pid=$!

  travis_jigger $cmd_pid $timeout $cmd &
  local jigger_pid=$!
  local result

  {
    wait $cmd_pid 2>/dev/null
    result=$?
    # if process finished before jigger, stop the jigger too
    ps -p$jigger_pid 2>&1>/dev/null && kill $jigger_pid
  }

  echo
  travis_time_end

  return $result
}

#######################################
function travis_jigger() {
  local cmd_pid=$1
  shift
  local timeout=$1
  shift
  local count=0

  echo -n "Waiting for process to finish "
  while [ $count -lt $timeout ]; do
    count=$(($count + 1))
    echo -ne "."
    sleep 60 # wait 60s
  done

  echo -e "\n\033[31;1mTimeout (${timeout} minutes) reached. Terminating \"$@\"\033[0m\n"
  kill -9 $cmd_pid
}

#######################################
function check_clang_format() {
  # Change to source directory.
  travis_run cd $CI_SOURCE_PATH

  # This directory can have its own .clang-format config file but if not, MoveIt's will be provided
  if [ ! -f .clang-format ]; then
      logError "The repository does not contain .clang-format file"
      exit 1
  fi

  CLANG_PKGS=$((echo "$TEST_BLACKLIST" | tr ' ;,' '\n') | tr '\n' ' ')
  if [ -n "$CLANG_PKGS" ]; then
      # Fix formatting of list of packages to work correctly with Travis
      IFS=' ' read -r -a CLANG_PKGS <<< "$CLANG_PKGS"
  fi

  PRUNE=""
  CNT=0
  for i in "${CLANG_PKGS[@]}"
  do
      if [[ $CNT -eq 0 ]]; then
          PRUNE="-name $i"
      else
          PRUNE="$PRUNE -o -name $i"
      fi
      let CNT+=1
  done

  # Run clang-format
  logHighlight "Running clang-format"
  if [[ $CNT -eq 0 ]]; then
      find . -type f -regex '.*\.\(cpp\|hpp\|cc\|cxx\|h\|hxx\)' -exec clang-format -style=file -i {} \;
  else
      find . -type d \( $PRUNE \) -prune -o -type f -regex '.*\.\(cpp\|hpp\|cc\|cxx\|h\|hxx\)' -exec clang-format -style=file -i {} \;
  fi

  logHighlight "Showing changes in code style:"
  travis_run git --no-pager diff

  # Make sure no changes have occured in repo
  if ! git diff-index --quiet HEAD --; then
      # changes
      logError "clang-format test failed: changes required to comply to formatting rules. See diff above.";
      exit 1 # error
  fi

  logHighlight "Passed clang-format test"
}
