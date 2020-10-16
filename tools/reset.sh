echo -e "\e[1;31m# Reseting vins-estimator...\e[0m"

source /opt/ros/kinetic/setup.bash
rostopic pub --once /reset std_msgs/Bool true