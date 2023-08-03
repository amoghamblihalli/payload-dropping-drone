import time
import dronekit as dk
import math as m
import pymavlink as pml
import camera as c

# Distance between 2 points using Haversine formula
def get_distance_metres(location1, location2): 
    dlat = m.radians(location2.lat - location1.lat)
    dlon = m.radians(location2.lon - location1.lon)
    a = m.sin(dlat / 2) * m.sin(dlat / 2) + m.cos(m.radians(location1.lat)) * m.cos(m.radians(location2.lat)) * m.sin(dlon / 2) * m.sin(dlon / 2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1 - a))
    distance = 6371000 * c  # Approximate radius of the Earth in meters
    return distance

def GPS_flight(vehicle, location):
    print("On my way to", location)
    vehicle.simple_goto(location)
    # Wait until vehicle reaches the target location
    while vehicle.mode.name == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, location)
        print("Distance to target: ", remaining_distance)
            
        if remaining_distance <= 1:  # Specify the desired threshold distance
            print("Reached target location!")
            print(vehicle.location.global_relative_frame)
            break
        time.sleep(1)

def Relative_flight(vehicle, x_distance_metres, y_distance_metres,TargetAlt):

    R = 6371000

    location1 = vehicle.location.global_relative_frame

    # Convert distances from meters to radians
    x_distance_radians = x_distance_metres / R
    y_distance_radians = y_distance_metres / R

    # Convert latitude and longitude to radians
    lat1_radians = m.radians(location1.lat)
    lon1_radians = m.radians(location1.lon)

    # Calculate the new latitude and longitude
    lat2_radians = lat1_radians + y_distance_radians
    lon2_radians = lon1_radians + x_distance_radians / m.cos(lat1_radians)

    # Convert new latitude and longitude from radians to degrees
    lat2_degrees = m.degrees(lat2_radians)
    lon2_degrees = m.degrees(lon2_radians)

    # Create a new LocationGlobalRelative object for location2
    location = dk.LocationGlobalRelative(lat2_degrees, lon2_degrees, TargetAlt)

    GPS_flight(vehicle, location)

def search_algo(vehicle, TargetAlt):
    
    Relative_flight(vehicle, 30,30, TargetAlt)
    
    i = 30

    while i > 0:

        Relative_flight(vehicle, -60,0, TargetAlt)
        Relative_flight(vehicle, 0,-1,TargetAlt)
        Relative_flight(vehicle,60,0, TargetAlt)
        Relative_flight(vehicle, 0,-1,TargetAlt)    
        
        i-=1

        ''' 
        if c.detect == "DETECTED":
            break
        '''