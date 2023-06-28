# import statements
import dronekit_sitl
from dronekit import *
import time
import cv2 as cv

# starting sim
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# connecting to vehicle
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# pre-check attributes
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

# arm & take off sequence 
def arm_n_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print("Waiting for initialization..")
        time.sleep(1)
    print("Starting to arm motors..")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming..")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    
    while True:
        print("Current Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)

# camera - take a pic
def take_pic():
    cam_port = 0
    cam = cv.VideoCapture(cam_port)
    result, image = cam.read()

    if result: # no error then works :)
        cv.imshow("Drone Cam", image) # showing result 
        cv.imwrite("DroneCam.jpeg", image) # saving image
        cv.waitKey(0)
        cv.destroyWindow("Drone Cam")
    else:
        print("No image detected. Please try again")

# process image
def process_pic():
    global piclocation
    print("Processing image")
    # insert location code here?

    piclocation = LocationGlobalRelative(-34.364114, 149.166022,20)

# navigation - waypoint?
def goto_dropzone():
    global piclocation
    print("Set default/target airspeed to 3")
    vehicle.airspeed = 3
    print("On my way to", piclocation)
    vehicle.simple_goto(piclocation)


# drop da bomb
def dropper():
    print("Dropping payload")

# return to launch & land
def return_n_land():
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

# close & shut down sim

def shutdown():
    print("Close vehicle object")
    vehicle.close()
    if sitl is not None:
        sitl.stop()
    print("Complete")

# execute order 66 - main
pre_check()
arm_n_takeoff(20)
take_pic()
process_pic()
goto_dropzone()
dropper()
return_n_land()
shutdown()
