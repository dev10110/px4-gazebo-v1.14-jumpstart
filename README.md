## Setup

Clone this repo:
```
git clone ...
cd px4-gazebo-v1.14-jumpstart
git submodule update --init --recursive
```

Clone PX4 into this directory:
```
git clone ...
```

Now we need to make sure that px4 build will recognize the tags:
```
git submodule update --init --recursive
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream --tags
```

Start the docker container:
```
docker compose build
docker compose up &
```

## Inside the docker:

Build the ros code:
```
cd /root/colcon_ws
colcon build
```


## Run
(Build and) run the px4 code:
```
PX4_MICRODDS_NS="drone1" make px4_sitl gazebo-classic
```

To start the microXRCE agent:
```
source /root/colcon_ws/install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

To start controlling the quad
```
source /root/colcon_ws/install/setup.bash
rviz2
```
and then add the Panels > Add a New Panel > dasc_robot_gui/Teleop
to be able to control the quadrotor. 
