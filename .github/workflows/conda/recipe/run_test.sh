#!/bin/sh

set -e

colcon test --event-handlers console_direct+ --return-code-on-test-failure