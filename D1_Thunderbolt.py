# import libraries here
import dronekit_sitl
import dronekit as dk
import time
import cv2 as cv
import math as m
import detect_marker as dm

# starting sim...
print("Starting the D1-Phantom...")
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string() # or 'tcp:127.0.0.1:11345'

# connecting to drone...
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = dk.connect(connection_string, wait_ready=True)

# input some variables here
alt = float(input("Enter target altitude: "))
vel = float(input("Enter target velocity: "))

# commencing pre-check ops
def pre_check():
    print("\nGet all vehicle attribute values:")
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print("   Major version number: %s" % vehicle.version.major)
    print("   Minor version number: %s" % vehicle.version.minor)
    print("   Patch version number: %s" % vehicle.version.patch)
    print("   Release type: %s" % vehicle.version.release_type())
    print("   Release version: %s" % vehicle.version.release_version())
    print("   Stable release?: %s" % vehicle.version.is_stable())
    print(" Autopilot capabilities")
    print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
    print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
    print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
    print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
    print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
    print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
    print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
    print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
    print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
    print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
    print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" Battery: %s" % vehicle.battery)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print(" Heading: %s" % vehicle.heading)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    print(" Airspeed: %s" % vehicle.airspeed)    # settable
    print(" Mode: %s" % vehicle.mode.name)    # settable
    print(" Armed: %s" % vehicle.armed)    # settable

# arming and taking-off
def arm_takeoff(TargetAlt):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors...")
    vehicle.mode = dk.VehicleMode("GUIDED")  # always set to GUIDED while arming
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(TargetAlt)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAlt * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    print("Current Velocity: %s m/s" % vehicle.velocity)
    print("Current Altitude: %s m" % vehicle.location.global_relative_frame.alt)
    print("Current Heading: %s deg" % vehicle.heading)
    print("Current Position- Lat:%s Lon:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,))
    print("Setting Airspeed to: %s" % vel)

# commencing camera ops - take da pic

def cam_ops_pic():
    for cam_port in range(0,4):  # loop to auto-wsitc
        cam = cv.VideoCapture(cam_port)
        result, image = cam.read()

        if result:  # No error, camera is successfully opened
            cv.imshow("Drone Cam", image)  # Showing result
            cv.imwrite("DroneCam.jpeg", image)  # Saving image
            time.sleep(2)
            cv.destroyAllWindows()
            cam.release()  # Release the camera resources
            break
        else:
            print("Failed to open camera port {cam_port}")

# commencing image processing ops - find da target
def img_proc(real_world_width, real_world_height):
    # Convert the centroid coordinates to real-world position coordinates
    current_location = vehicle.location.global_relative_frame
    dm.detect()
    centroid_x = dm.detect(centroid_x)
    centroid_y = dm.detect(centroid_y)
    image_path = "DroneCam.jpg"
    image = cv.imread(image_path)
    image_height, image_width, _ = image.shape
    image_scale_x = image_width / real_world_width
    image_scale_y = image_height / real_world_height
    real_world_x = centroid_x / image_scale_x
    real_world_y = centroid_y / image_scale_y

    # Convert the real-world coordinates to GPS coordinates
    dlat = m.radians(real_world_y / 6371000)
    dlon = m.radians(real_world_x / (6371000 * m.cos(m.radians(current_location.lat))))
    new_latitude = current_location.lat + m.degrees(dlat)
    new_longitude = current_location.lon + m.degrees(dlon)

    # Create a new LocationGlobalRelative object with the converted coordinates
    new_location = dk.LocationGlobalRelative(new_latitude, new_longitude)
    return new_location

piclocation = dk.LocationGlobalRelative(-35.362, 149.165234, 20)

def get_distance_metres(location1, location2): # distance between 2 points using Haversine formula
    dlat = m.radians(location2.lat - location1.lat)
    dlon = m.radians(location2.lon - location1.lon)
    a = m.sin(dlat / 2) * m.sin(dlat / 2) + m.cos(m.radians(location1.lat)) * m.cos(m.radians(location2.lat)) * m.sin(dlon / 2) * m.sin(dlon / 2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1 - a))
    distance = 6371000 * c  # Approximate radius of the Earth in meters
    return distance

# commencing navigation ops - go to da target
def flight():
    print("On my way to", piclocation)
    vehicle.simple_goto(piclocation)
    # Wait until vehicle reaches the target location
    while vehicle.mode.name == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, piclocation)
        print("Distance to target: ", remaining_distance)
            
        if remaining_distance <= 1:  # Specify the desired threshold distance
            print("Reached target location!")
            print(vehicle.location.global_relative_frame)
            break
        time.sleep(1)

# commencing drop - drop da load
def dropper():
    print("Dropping payload at %s", vehicle.location.global_relative_frame)
    time.sleep(5)

# commencing landing ops - land da drone
def return_n_land():
    print("Returning to Launch")
    vehicle.mode = dk.VehicleMode("RTL")
    time.sleep(30)
    print("Arrived at: ",vehicle.location.global_relative_frame)

# commencing shutdown ops - shutdown da drone
def shutdown():
    print("Close vehicle object")
    vehicle.close()
    if sitl is not None:
        sitl.stop()
    print("Complete")

# execute order 66 - order of ops
pre_check()
arm_takeoff(alt)
cam_ops_pic()
flight()
dropper()
return_n_land()
shutdown()
