# IEEE RAS Summer Project: Payload Dropping Drone

Works with these versions: 

- ArduPilot Copter ver 4.3.7

- Dronekit ver 2.9.2

- pymavlink ver 2.4.39

- mavros ver 1.16.0

- Gazebo ver 11.13.0-1~focal

- ROSN oetic

System Requirements: Ubuntu 20.04, 8 GB RAM

## Installation
[ArduPilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

Note: Change ```git checkout Copter-4.0.4``` to ```git checkout Copter-4.3.7```

[Gazebo](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)

[ROS Noetic](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)

[Simulations](https://github.com/Intelligent-Quads/iq_sim/tree/master)

Note:

```runway.world``` must be replaced in ```~/catkin_ws/src/iq_sim/worlds```

```aruco_visual_marker_0```, ```aruco_visual_marker_0_pad``` and ```drone_with_camera``` must be added/replaced in ```~/catkin_ws/src/iq_sim/launch```

## To Launch
Terminal 1: ```roslaunch iq_sim runway.launch``` - ensure ROS-Noetic is installed beforehand

Terminal 2: ```cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console``` - ensure ArduPilot and ArduPilot Gazebo plugins are installed

Terminal 3: ```python3.8 main.py``` - make sure this is running in the folder where drone scripts are stored

Terminal 4: ```python tryCam.py``` - this is to get camera view, integration with the script will be done soon. 
```python3.8 main.py```

## Warnings
- Never use DroneKit SITL
