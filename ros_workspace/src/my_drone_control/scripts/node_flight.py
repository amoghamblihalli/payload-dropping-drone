#!/usr/bin/env python3

import rospy
import dronekit as dk
import time
import math as m

def get_distance_metres(location1, location2): # distance between 2 points using Haversine formula
    dlat = m.radians(location2.lat - location1.lat)
    dlon = m.radians(location2.lon - location1.lon)
    a = m.sin(dlat / 2) * m.sin(dlat / 2) + m.cos(m.radians(location1.lat)) * m.cos(m.radians(location2.lat)) * m.sin(dlon / 2) * m.sin(dlon / 2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1 - a))
    distance = 6371000 * c  # Approximate radius of the Earth in meters
    return distance

def flight_to_destination():
    # Implement flight control logic here
    pass

if __name__ == '__main__':
    rospy.init_node('flight_node')
    piclocation = dk.LocationGlobalRelative(-35.362, 149.165234, 20)  # Modify the target location if needed
    rospy.loginfo("On my way to %s", piclocation)
    connection_string = 'tcp:127.0.0.1:5763'  # Change to your connection string if needed
    vehicle = dk.connect(connection_string, wait_ready=True)

    # Wait for the vehicle to be armable and armed
    while not vehicle.is_armable:
        rospy.loginfo("Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = dk.VehicleMode("GUIDED")  # Always set to GUIDED while arming
    vehicle.armed = True

    while not vehicle.armed:
        rospy.loginfo("Waiting for arming...")
        time.sleep(1)

    # Command the vehicle to takeoff to the target altitude
    vehicle.simple_takeoff(piclocation.alt)

    # Wait until vehicle reaches the target location
    while vehicle.mode.name == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, piclocation)
        rospy.loginfo("Distance to target: %s", remaining_distance)

        if remaining_distance <= 1:  # Specify the desired threshold distance
            rospy.loginfo("Reached target location!")
            rospy.loginfo("Arrived at: %s", vehicle.location.global_relative_frame)
            break

        time.sleep(1)