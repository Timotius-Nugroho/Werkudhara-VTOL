from __future__ import division
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from time import sleep

#vehicle = connect('udp:127.0.0.1:14551',wait_ready=True)
vehicle = connect('/dev/ttyACM0',baud=115200,wait_ready=True)

def send_body_velocity(Cx, Cy, velocity_z) :
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        Cx, Cy, velocity_z,
        0, 0, 0,
        0, 0) 
    vehicle.send_mavlink(msg)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    print("Arming motors")

    vehicle.armed = True
    while not(vehicle.armed):
        print(" Waiting for arming...")
        sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
            print("Reached target altitude")
            break
        sleep(1)

arm_and_takeoff(1)
print("Take off Selesai")
vehicle_speed = 1 #kecepatan resultan drone (m/s)
vehicle.mode = VehicleMode("GUIDED")

while True :
    x = float(input('jarak x(m) ='))
    y = float(input('jarak y(m) ='))
    d = (x**2 + y**2)**0.5

    dt = 0.2
    t = d/vehicle_speed
    
    vx = (x*vehicle_speed)/d
    vy = (y*vehicle_speed)/d
    if (vehicle.location.global_relative_frame.alt<=1.5) :
        vz = -0.1
    else :
        vz = 0
    print('waktu tempuh=',round(t, 2),' s')
    for i in range(0, int(t/dt)) :
        send_body_velocity(vx, vy, vz)
        sleep(dt)
    send_body_velocity(0, 0, 0)
    

    
