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
# Launch the camera, please make sure the sdk is #
# installed and vins is built.                   #
##################################################\e[0m"

if [ ! $1 ]
then
    echo -e "\e[1;32m# Tell me where to find 'MYNT-EYE-D-SDK':\e[0m"
    read SDK_PATH
fi

echo -e "\e[1;32m
# Launching camera...\e[0m"
source "$SDK_PATH/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.bash"
roslaunch mynteye_wrapper_d vins_fusion.launch stream_mode:=1