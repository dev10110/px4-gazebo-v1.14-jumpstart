# FROM ros:humble
FROM px4io/px4-dev-ros2-foxy

SHELL ["/usr/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt-get update && apt-get install -y --no-install-recommends vim cmake build-essential wget

RUN apt-get install -y --no-install-recommends python3-pip 
RUN python3 -m pip install empy pyros-genmsg setuptools kconfiglib


## install gazebo 
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-ign


## install gazebo ros dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-camera-info-manager ros-${ROS_DISTRO}-gazebo-ros-pkgs


## install image transport
RUN apt-get install -y \ 
  ros-${ROS_DISTRO}-image-transport-plugins \ 
  ros-${ROS_DISTRO}-compressed-image-transport

## upgrade the RTPS 
RUN rm -rf /usr/local/include/fastrtps /usr/local/share/fastrtps /usr/local/lib/libfastrtps* \
	&& git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.11.1 /tmp/FastRTPS-2.11.1 \
	&& cd /tmp/FastRTPS-2.11.1 \
	&& mkdir build && cd build \
	&& cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
	&& cmake --build . --target install -- -j $(nproc) \
	&& rm -rf /tmp/*


WORKDIR /root

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
RUN echo 'source /root/colcon_ws/install/local_setup.bash' >> /root/.bashrc


