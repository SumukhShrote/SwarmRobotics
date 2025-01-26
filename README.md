# Swarm Robotics

## Requirements
- PX4
- ROS2
- Micro XRCE-DDS Agent & Client
- QGroundControl

## Installation and Setup

1. Go to [official PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html) and install the setup according to the steps given. Here we use ROS2 Humble.

2. Go to [QGC Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) and install QGroundControl according to the steps as well.

3. Open the terminal, and go to the folder ~/PX4-Autopilot/Tools/simulation/gz
```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
```
Make the simulation-gazebo file an executable by running
```bash
chmod +x simulation-gazebo
```
This file will be used to launch the world file. 

## Executing
1. Open the terminal and run QGC as an executable.
```bash
./QGroundControl.AppImage
```

2. Open another terminal and run the simulation-gazebo file.
```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
python3 simulation-gazebo
```
If you wish to run it with a custom world file
```bash
python3 simulation-gazebo --world <world_name>
```

3. In another terminal, open the ~/PX4-Autopilot directory and spawn your drone using the command
```bash
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```
For more drones, you can open new terminals use the command,
```bash
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
```
Change the pose according to your requirements and as you spawn new drones, increase the value of arguement i (like -i 3 or 4 or so on).

4. In a new terminal, run the micro XRCE-DDS agent
```bash
MicroXRCEAgent udp4 -p 8888
```

