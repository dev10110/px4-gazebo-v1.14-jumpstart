FROM ros:foxy

SHELL ["/usr/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends vim tmux cmake build-essential

RUN apt-get install -y --no-install-recommends python3-pip 
RUN python3 -m pip install empy pyros-genmsg setuptools

WORKDIR /root

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build

RUN cmake ..  
RUN make -j
RUN make install
RUN ldconfig /usr/local/lib/

WORKDIR /root

RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc
