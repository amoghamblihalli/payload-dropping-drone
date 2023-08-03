# Importing libraries
import dronekit as dk
import time
import argparse

# Import Functions
import takeoff_land as tl
import flight as f
import camera as c

# Starting SITL
print("Starting D1-Thunderbolt...")
connection_string = 'udp:127.0.0.1:14551'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = dk.connect(connection_string, wait_ready=True)

# Menu Functions
TargetAlt = float(input("Enter target altitude: "))
TargetVel = float(input("Enter target velocity: "))
flight_mode = int(input("Press 1 for GPS Navigation, press 2 for Relative Navigation"))

if flight_mode == 1:
    lat = float(input("Enter target latitude (degrees): "))
    lon = float(input("Enter target longitude (degrees): "))
    location = dk.LocationGlobalRelative(lat,lon,TargetAlt)

elif flight_mode == 2:
    x_distance_metres = float(input("Enter x-axis distance (meters): "))
    y_distance_metres = float(input("Enter y-axis distance (meters): "))

# Functioning Functions

tl.arm_and_takeoff(vehicle,TargetAlt,TargetVel)

if flight_mode == 1:
    f.GPS_flight(vehicle,location)
elif flight_mode == 2:
    f.Relative_flight(vehicle,x_distance_metres,y_distance_metres,TargetAlt)

tl.return_to_launch(vehicle)
tl.shutdown(vehicle)


