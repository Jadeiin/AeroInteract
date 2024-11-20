#---
# name: sam_ros
# group: robots
# depends: [ros:noetic-desktop, nanoowl, nanosam, realsense]
# test: test.sh
#---
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG ROS_PACKAGE=pcl_conversions
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

WORKDIR /workspace

ARG SAMROS_REPO="https://git.qunn.eu/poesty/sam_ros" \
    SAMROS_BRANCH=wall_detection

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     libpcl-dev && \
#     rm -rf /var/lib/apt/lists/* && \
#     apt-get clean

RUN cd ros_catkin_ws && \
    rosinstall_generator ${ROS_PACKAGE} --rosdistro ${ROS_DISTRO} --deps --tar --exclude RPP > ${ROS_DISTRO}-${ROS_PACKAGE}.rosinstall && \
    vcs import --retry 100 --input ${ROS_DISTRO}-${ROS_PACKAGE}.rosinstall ./src && \
    apt-get update && \
    rosdep install -y \
	 --from-paths ./src \
	 --ignore-packages-from-source \
	 --rosdistro ${ROS_DISTRO} \
	 --skip-keys "python3-pykdl libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN git clone --branch=${SAMROS_BRANCH} ${SAMROS_REPO} /opt/sam_ros && \
    mkdir -p /workspace/catkin_ws/src && \
    ln -s /opt/sam_ros /workspace/catkin_ws/src && \
    cd /workspace/catkin_ws && \
    catkin_make