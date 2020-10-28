#!/usr/bin/env bash

# The first line must stay there! It says this script will be run
# in bash.

# ----------------------------------------------------------------------- #
# Setup the error handler.                                                #
# ----------------------------------------------------------------------- #
# exit when any command fails                                             #
set -e                                                                    #                      
# keep track of the last executed command                                 #
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG #
# echo an error message before exiting                                    #
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT   #
# ----------------------------------------------------------------------- #

CALLER_PATH=$(pwd)
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

echo -e "\e[1;32m
##################################################
# Launch ar demo (first make sure the device is  #
# launched and vins is built).                   #
##################################################\e[0m"

# Check the camera topic
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! $(rostopic info /mynteye/right/camera_info) ]
then
    echo -e "\e[1;31m# MUST launch camera first.\e[0m"
    exit 1
fi

source "$SCRIPT_PATH/../../devel/setup.bash"
roslaunch ar_demo mynteye_d.launch