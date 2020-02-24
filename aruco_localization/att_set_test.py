from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import sys, termios, tty, os, time
import numpy as np
import dronekit_helper

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

#Create a message listener for all messages.
def listener(self, name, message):
    print 'message: %s' % message


#setup dronekit vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=False)
print("Dronekit vehicle connection successful")
print("Arming motors")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
vehicle.armed = True
print("First arming process")
while not vehicle.armed: time.sleep(1)
vehicle.on_message('*')


pitch_angle = 0
roll_angle = 0
throttle = 0

try:
    while True:
        char = getch()
        if char == "w":
            print("Pitch forward")
            pitch_angle -= 5
        elif char == "s":
            print("Pitch backward")
            pitch_angle += 5
        elif char == "a":
            print("Roll left")
            roll_angle-=5
        elif char == "d":
            print("Roll right")
            roll_angle+= 5
        elif char == "t":
            print("Throttle up")
            throttle+=.1
        elif char == "g":
            print("Throttle down")
            throttle-=.1
        elif char == "q":
            print("Quit")
            raise ValueError
	
	listener()	
	
        dronekit_helper.send_attitude_cmd(roll_angle, pitch_angle, throttle, vehicle)
        time.sleep(.1)
finally:
    print("Disarming motors")
    while vehicle.armed:
        vehicle.armed = False
        time.sleep(0.5)
        print("Armed: %s" % vehicle.armed)
    exit(0)
