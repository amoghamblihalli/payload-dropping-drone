import time
import dronekit as dk

def arm_and_takeoff(vehicle, TargetAlt, TargetVel):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = dk.VehicleMode("GUIDED") # always needs to be set GUIDED while arming
    time.sleep(3)
    print("Mode: %s" % vehicle.mode.name)
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
    print("Setting Airspeed to: %s" % TargetVel)

def land_and_disarm(vehicle):

    print("Ensuring mode is to GUIDED")
    vehicle.mode = dk.VehicleMode("GUIDED") # always needs to be set GUIDED while arming
    time.sleep(3)
    print("Mode: %s" % vehicle.mode.name)
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Landing! Steer Clear!")

    i = vehicle.location.global_relative_frame.alt

    while i > 0:
        vehicle.simple_takeoff(i-1)
        print("Current Altitude is %s" % vehicle.location.global_relative_frame.alt)        
    
    pass

def return_to_launch(vehicle):
    print("Returning to Launch")
    vehicle.mode = dk.VehicleMode("RTL")
    time.sleep(30)
    print("Arrived at: ",vehicle.location.global_relative_frame)

def shutdown(vehicle):
    print("Close vehicle object")
    vehicle.close()
    print("Complete")

