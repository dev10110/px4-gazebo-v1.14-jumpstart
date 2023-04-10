#FROM ros:foxy
FROM px4io/px4-dev-ros2-foxy

SHELL ["/usr/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt-get update && apt-get install -y --no-install-recommends vim tmux cmake build-essential ssh

RUN apt-get install -y --no-install-recommends python3-pip 
RUN python3 -m pip install empy pyros-genmsg setuptools kconfiglib

WORKDIR /root

RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc
RUN echo 'source /root/colcon_ws/install/local_setup.bash' >> /root/.bashrc
