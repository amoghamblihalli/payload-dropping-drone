# IEEE RAS Summper Project: Payload Dropping Drone

Works with the latest versions of ArduPilot, DroneKit, PyMAVLink, MAVROS, ROS Noetic, Gazebo11 (August 2023).
Successfully runs on Python 3.8

System Requirements: Ubuntu 20.04, 8 GB RAM

## Installation
[ArduPilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

Note: Change ```git checkout Copter-4.0.4``` to ```git checkout Copter-4.3.7```

[Gazebo](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)
[ROS Noetic](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)


## To Launch
In one terminal,

```cd ~/ardupilot/Arducopter sim_vehicle.py -v copter```

In another terminal wherever DroneKit (main.py, etc.) are saved,

```python3.8 main.py```

## Warnings
- Never use DroneKit SITL
