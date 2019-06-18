#!/bin/bash
# Copyright 2019 Robert Bosch GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## Helper script for ROS tracing
##
## Call this by providing these arguments:
##  1. a session name [optional; 'ros2' will be used]
##  2. a wait time before killing and stopping (in seconds)
##  3. a roslaunch/rosrun command
## ex: ./trace.sh 3 roslaunch tracecompass_ros_testcases pub_sub.launch

source ./${BASH_SOURCE%/*}/../../../../install/local_setup.bash

## Parameters

# if no parameters were given, exit with error 
if [ -z "$1" ] ; then
    echo "Error: no parameters were given!"
    exit 1
elif [ "$1" == "-h" ] ; then
    echo -e "ROS tracing helper script.\n" \
    "Provide 3 arguments:\n" \
        "1. the lttng session name [optional; 'ros2' will be used]\n" \
        "2. the wait time before killing and stopping (in seconds)\n" \
        "3. the ros2 run command\n" \
    "Example: ./trace.sh ros-trace 3 ros2 run tracetools tracetools_status"
    exit 0
fi

# session name
session_name="$1"
case "$session_name" in
    ''|*[!0-9]*) # not a number: good
    shift
    ;;
    *) # number: so use a default session name
    session_name="ros2"
    ;;
esac

# wait time (seconds) before killing and stopping
sleep_time="$1"
case "$sleep_time" in
    ''|*[!0-9]*) # not a number: error!
    echo "Error: not a valid sleep time!"
    exit 1
    ;;
    *) # number: good
    shift
    ;;
esac

# command from remaining arguments
if [ -z "$1" ] ; then
    echo "Error: no command was given!"
    exit 1
fi
launch_cmd="$@"
launch_cmd+=" &"

## Trace

# create lttng session (and set output to traces/ directory)
lttng create $session_name --output=./${BASH_SOURCE%/*}/../traces/$session_name/

# enable events
./${BASH_SOURCE%/*}/setup-lttng.sh

# start
lttng start

# preload UST library
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liblttng-ust-cyg-profile.so

# launch
eval "$launch_cmd"

# wait a bit and kill
echo "waiting $sleep_time..."
sleep $sleep_time
echo "killing"
kill $!

# wait again for everything to shutdown
echo "waiting for shutdown..."
sleep 2
echo "stopping"

# stop & destroy
lttng stop
lttng destroy
