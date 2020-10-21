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

# SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
CALLER_PATH=$(pwd)

echo -e "\e[1;32m
############################################################
# This script will walk you through setting up the camera. #
# Note that some of the operations requiring plugging in   #
# or out the camera, which will be notified and will wait  #
# until complete.                                          #
# -------------------------------------------------------- #
# FAQ:                                                     #
#   1. Device CANNOT be found after 'make init'. Make sure #
#      '--privileged' is set when container runs.          #
#   2. Cannot connect to display. Call 'xhost +' on host   #
#      to enable the link from container.                  #
############################################################

# STEP 1: Download the code.\e[1;31m
# Temp files will be placed at current folder, press any
# key to continue...\e[0m"

read -n 1 # wait for keypress.

if [ ! -d MYNT-EYE-D-SDK ]
then
  # Have some special changes in this repo and branch.
  git clone https://github.com/shihengW/MYNT-EYE-D-SDK.git --branch dev
else
  echo "# Found sdk, no need to clone again."
fi

cd MYNT-EYE-D-SDK
git checkout dev # MUST use the dev branch.

echo -e "\e[1;32m
# STEP 2: Initialize the build process.\e[1;31m
# YOU MUST plugin the device before continue, press any
# buttom when you done...\e[0m"

read -n 1 # wait for keypress.
apt update && echo y | make init

echo -e "\e[1;32m
# STEP 3: Build from source.\e[1;31m
# MUST plugout and plugin again before continue, press
# any buttom when you done...\e[0m"

read -n 1 # wait for keypress.
source /opt/ros/kinetic/setup.bash && echo y | make all
make install # Required by fusion.

echo -e "\e[1;32m
# COMPLETE!\e[0m"
