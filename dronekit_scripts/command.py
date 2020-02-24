from __future__ import print_function
import sys, termios, tty, os, time
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

DEBUG = 0

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if(str(ch) == "\x03"):
        print("Disarming motors")
        throttle = 980
        print(throttle)
        if (not DEBUG):
            vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
            while(vehicle.armed == True):
                vehicle.armed = False
                time.sleep(0.5)
                print("Armed: %s" % vehicle.armed)
        exit(0)

    return ch


throttle = 980
roll = 1500
pitch = 1500
yaw = 1500
delay = 0.1

if(not DEBUG):
    print('Connecting...')
    vehicle = connect('/dev/ttyUSB0', wait_ready=False)
print("Ready")

try:
    while True:
        char = getch()
        if (char == "z"):
            print("Arming motors")
            if (not DEBUG):
                vehicle.mode = VehicleMode("STABILIZE")
                for i in range(0, 2, 1):
                    vehicle.armed = True
                    time.sleep(0.5)
                print("Armed: %s" % vehicle.armed)

        if (char == "x"):
            print("Disarming motors")
            throttle = 980
            print(throttle)
            if (not DEBUG):
                vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
                while (vehicle.armed == True):
                    vehicle.armed = False
                    time.sleep(0.5)
                    print("Armed: %s" % vehicle.armed)

        elif (char == "w"):
            throttle = throttle + 2
            print(throttle)
            if (not DEBUG):
                vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
            time.sleep(delay)

        elif (char == "s"):
            throttle = throttle - 2
            print(throttle)
            if (not DEBUG):
                vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
            time.sleep(delay)

        elif (char == "p"):
            #max = 1250
            max = int(input(">>> "))
            try:
                if(vehicle.armed):
                    for i in range(1100, max, 2):
                        for j in range(1, 3, 1):
                            print(i)
                            if (not DEBUG):
                                vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': i, '4': 1500}
                            time.sleep(0.1)
                    for i in range(max, 1100, -10):
                        for j in range(1, 3, 1):
                            print(i)
                            if (not DEBUG):
                                vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': i, '4': 1500}
                            time.sleep(0.1)
                    time.sleep(0.5)
                    if (not DEBUG):
                        print("Disarming motors")
                        throttle = 980
                        print(throttle)
                        if (not DEBUG):
                            for j in range(1, 5, 1):
                                vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
                                time.sleep(0.2)
                            while (vehicle.armed == True):
                                vehicle.armed = False
                                time.sleep(0.5)
                                print("Armed: %s" % vehicle.armed)
                else:
                    print("Motors not armed yet")
            except NameError:
                pass

except KeyboardInterrupt:
    print("Crash")
    print("Disarming motors")
    throttle = 980
    print(throttle)
    if (not DEBUG):
        for j in range(1, 5, 1):
            vehicle.channels.overrides = {'1': roll, '2': pitch, '3': throttle, '4': roll}
            time.sleep(0.2)
        while (vehicle.armed == True):
            vehicle.armed = False
            time.sleep(0.5)
            print("Armed: %s" % vehicle.armed)
    exit(0)
