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


# Must run as root, which is required by camera sdk.
if [ "$EUID" -ne 0 ]
then
    echo -e "\e[1;31m# [ERROR] MUST run as root.\e[0m"
    exit 1
fi

echo -e "\e[1;32m
##################################################
# Launch the camera, please make sure the sdk is #
# installed and vins is built.                   #
##################################################\e[0m"

# Input sdk path.
if [ ! $1 ]
then
    echo -e "\e[1;31m
# Tell me where to find 'MYNT-EYE-D-SDK':\e[0m"
    read SDK_PATH
else
    SDK_PATH=$1
fi

# Try to find sdk.
if [ -f $SDK_PATH/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.bash ]
then
    SDK_FULL_PATH=$SDK_PATH/MYNT-EYE-D-SDK/
elif [ -f $SDK_PATH/wrappers/ros/devel/setup.bash ]
then
    SDK_FULL_PATH=$SDK_PATH/
else
    echo -e "\e[1;31m
# [ERROR] Cannot find camera sdk at $SDK_PATH/MYNT-EYE-D-SDK/ or $SDK_PATH/.\e[0m"
    exit 1
fi

echo -e "\e[1;32m
# Launching camera...\e[0m"

source $SDK_FULL_PATH/wrappers/ros/devel/setup.bash
roslaunch mynteye_wrapper_d vins_fusion.launch