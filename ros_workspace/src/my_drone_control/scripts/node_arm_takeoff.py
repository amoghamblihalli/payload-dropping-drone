#!/usr/bin/env python3

import rospy
import dronekit as dk
import time

def arm_and_takeoff(TargetAlt):
    while not vehicle.is_armable:
        rospy.loginfo("Waiting for vehicle to initialise...")
        time.sleep(1)
    
    rospy.loginfo("Arming motors...")
    vehicle.mode = dk.VehicleMode("GUIDED")  # Always set to GUIDED while arming
    vehicle.armed = True

    while not vehicle.armed:
        rospy.loginfo("Waiting for arming...")
        time.sleep(1)

    rospy.loginfo("Taking off!")
    vehicle.simple_takeoff(TargetAlt)

    while True:
        rospy.loginfo("Altitude: %s", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAlt * 0.95:
            rospy.loginfo("Reached target altitude")
            break
        time.sleep(1)

    rospy.loginfo("Current Velocity: %s m/s", vehicle.velocity)
    rospy.loginfo("Current Altitude: %s m", vehicle.location.global_relative_frame.alt)
    rospy.loginfo("Current Heading: %s deg", vehicle.heading)
    rospy.loginfo("Current Position- Lat:%s Lon:%s",
                  vehicle.location.global_relative_frame.lat,
                  vehicle.location.global_relative_frame.lon)

if __name__ == '__main__':
    rospy.init_node('arm_takeoff_node')
    rospy.loginfo("Starting the D1-Phantom...")
    connection_string = 'tcp:127.0.0.1:5763'  # Change to your connection string if needed
    vehicle = dk.connect(connection_string, wait_ready=True)

    # Input some variables here
    alt = float(input("Enter target altitude: "))
    vel = float(input("Etner target velocity: "))
    arm_and_takeoff(alt)