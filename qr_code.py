from __future__ import division
import cv2
from pyzbar import pyzbar
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

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
    print("Arming motors")
    vehicle.armed = True

    while not(vehicle.armed):
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
            print("Reached target altitude")
            break
        time.sleep(1)



def read_barcodes(frame):
    global barcodes
    barcodes = pyzbar.decode(frame)
    if len(barcodes)>0 :

        if len(barcodes)>1 :
            area = [br.rect.width*br.rect.height for br in barcodes]
            idx_max = area.index(max(area))
        else :
            idx_max = 0

        x, y , w, h = barcodes[idx_max].rect
        barcode_text = barcodes[idx_max].data.decode('utf-8')
        print(barcode_text)
        cv2.rectangle(frame, (x, y),(x+w, y+h), (0, 255, 0), 2)
        x = int(x + 0.5*w)
        y = int(y + 0.5*h)
        Cz=(y-240)/950 #atas bawah
        Cy=-(x-320)/950 #kiri kanan
        print(Cy)
        cv2.putText(frame, "O", (x,y), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        send_body_velocity(0,Cy,Cz)

    return frame

def qrscan():
    
    camera = cv2.VideoCapture(0)
    frame_width = int(camera.get(3))
    frame_height = int(camera.get(4))
    out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
    ret, frame = camera.read()
    while ret:
        ret, frame = camera.read()
        a=time.time()
        frame = read_barcodes(frame)
        if len(barcodes)>0 :
            print("t= ",time.time()-a)
        out.write(frame)
        cv2.imshow('Barcode reader', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    camera.release()
    out.release()
    cv2.destroyAllWindows()

arm_and_takeoff(1.5)
print("Take off Selesai")
vehicle.mode = VehicleMode("GUIDED")
qrscan()
input("Tekan Enter Untuk Land")
vehicle.mode=VehicleMode("LAND")
time.sleep(4)
