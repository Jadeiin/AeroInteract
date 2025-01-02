#---
# name: sam_ros:base
# group: robots
# depends: [ros:noetic-desktop, nanoowl, nanosam]
# test: test.sh
#---
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PACKAGE_PATH=/opt/ros/${ROS_DISTRO}/share
ENV ROS_PYTHON_VERSION=3

WORKDIR /workspace

ARG SAMROS_REPO="https://git.qunn.eu/poesty/sam_ros" \
    SAMROS_BRANCH=develop

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     libpcl-dev && \
#     rm -rf /var/lib/apt/lists/* && \
#     apt-get clean

RUN cd ros_catkin_ws && \
    rosinstall_generator pcl_conversions --rosdistro ${ROS_DISTRO} --deps --tar --exclude RPP > ${ROS_DISTRO}-sam_ros.rosinstall && \
    vcs import --retry 100 --input ${ROS_DISTRO}-sam_ros.rosinstall ./src && \
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
    /bin/bash -c 'cd /workspace/catkin_ws; source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

WORKDIR /workspace/catkin_ws