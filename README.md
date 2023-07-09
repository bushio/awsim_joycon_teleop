# awsim_joycon_teleop
Controle node AWSIM  by NintenDo Switch Joycon

# Environment
This tool uses AWSIM and Autoware.
Please refer to the following to set up the environment. 
[aichallenge2023-sim](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim)


# Clone repository
Please clone this repository and put in "docker/aichallenge_ws/src".

```
$git clone https://github.com/AutomotiveAIChallenge/aichallenge2023-sim.git
$cd aichallenge2023-sim/docker/aichallenge/aichallenge_ws/src/
$git clone git@github.com:bushio/awsim_joycon_teleop.git
```

# Download AWSIM
- Download "AWSIM_AIChallenge_Ubuntu_v*.*.zip" from [here](https://drive.google.com/drive/folders/1zONmvBjqMzveemkZmNdd4icbpwnDYvTq?usp=sharing).
- Unzip files and put in "aichallenge2023-sim/docker/aichallenge/".
- Copy "pointcloud_map.pcd" and "lanelet2_map.osm" to "aichallenge2023-sim/docker/aichallenge/mapfile/" 

## Reference
- Please refere to [SetUP](https://automotiveaichallenge.github.io/aichallenge2023-sim/setup/index.html#awsimubuntu) about how to setup AWSIM. 


# Demo 
To use this tool, you need to start three shells.

## First shell (RUN AWSIM)
1. Run rocker for awsim.
```
$cd aichallenge2023-sim/docker/
$rocker --nvidia --x11 --user --net host --privileged --volume aichallenge:/aichallenge --volume /dev/input/:/dev/input/ --volume /var/run/dbus/:/var/run/dbus/ -- aichallenge-train
```
2. Run AWSIM
```
$cd /aichallenge
$source aichallenge_ws/install/setup.bash 
$./AWSIM/AWSIM.x86_64
```

## Second shell (RUN e2e_simulator)
1. Run rocker.
- Run the container in the same way as the first shell.

2. RUN e2e_simulator

```
$cd /aichallenge/
$bash build.sh 
$source install/setup.bash
$ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=golfcart sensor_model:=awsim_sensor_kit map_path:=/aichallenge/mapfile
```
3. Initialize pose
Push "2D Pose Estimate" button and initialize pose of the vehicle.


## Third shell (RUN awsim_joycon_teleop)
1. Run rocker.
- Run the container in the same way as the first shell.

2. intall pygame
```
$ pip install pygame
```

3. Connect joycon
From "Setting/Bluetooth" on Ubuntu, find "Joy-con(R)" and connect with it.

4. Run awsim_joycon_teleop
```
$cd /aichallenge/
$bash build.sh
$source install/setup.bash
$ros2 run awsim_joycon_teleop awsim_joycon_teleop
```

