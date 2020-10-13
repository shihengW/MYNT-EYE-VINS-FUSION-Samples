FROM ros:kinetic-perception

# Install dependencies.
RUN apt-get update && apt-get install -y \
    cmake                                \
    gdb                                  \
    libatlas-base-dev                    \
    libgoogle-glog-dev                   \
    libsuitesparse-dev                   \
    python-catkin-tools                  \
    ros-${ROS_DISTRO}-cv-bridge          \
    ros-${ROS_DISTRO}-image-transport    \
    ros-${ROS_DISTRO}-message-filters    \
    ros-${ROS_DISTRO}-rviz               \
    ros-${ROS_DISTRO}-tf &&              \
    rm -rf /var/lib/apt/lists/*

# Install eigen (to be sure of the version).
RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && git checkout tags/3.3.8 &&             \
    mkdir build && cd build && cmake .. &&             \
    make install && cd ../../ && rm -rf eigen

# Install ceres.
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && cmake . && make -j3 && make install &&       \
    cd ../ && rm -rf ceres-solver
