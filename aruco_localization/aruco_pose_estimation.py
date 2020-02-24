from __future__ import print_function
import sys, termios, tty, os, time
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from simple_pid import PID
import dronekit_helper


"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""

# Flags to change based on platform
CAMERA = 3  # 0 for webcam, 1 for picam, 2 for realsense infrared, 3 for realsense color
VISUALIZE = True
PRINT_FREQUENCY = 1
last_time = -999
frame_counter = 0

# --- Define Tag
id_to_find = 72
marker_size = 3.937  # -edit inches [cm] output will also be in this unit
x_pos, y_pos, z_pos = 0,0,0


# ------------------------------------------------------------------------------
# ------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
# ------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# --- Get the camera calibration path
calib_path = "./"
if CAMERA == 0:
    camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')
elif CAMERA == 1:
    camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')
else:
    camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_rs.txt', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_rs.txt', delimiter=',')

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# --- Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# --- Capture the videocamera (this may also be a video or a picture)
if CAMERA == 0:
    cap = cv2.VideoCapture(0)
    # -- Set the camera size as the one it was calibrated with
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
elif CAMERA == 1:
    import picamera

    cap = picamera.PiCamera()
    cap.resolution = (1280, 720)
    cap.framerate = 24
    time.sleep(2)

    # image = np.empty((240, 320, 3), dtype=np.uint8)
    # cap.capture(image, 'bgr')
elif CAMERA == 2:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.infrared, 1280, 720, rs.format.any, 0)
    # Start streaming
    pipeline.start(config)
elif CAMERA == 3:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    # turn off frame buffering
    for sensor in pipeline.get_active_profile().get_device().query_sensors():
        print(sensor.get_option(rs.option.frames_queue_size))
        print("Set frame queue size to 0")
        sensor.set_option(rs.option.frames_queue_size, 0)
        print(sensor.get_option(rs.option.frames_queue_size))


# -- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

print("Camera initialized")

pitch_pid = PID(1, 0.01, 0.005, setpoint=0)
roll_pid = PID(1, 0.01, 0.005, setpoint=0)

#setup dronekit vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=False)
print("Arming motors")
# Copter should arm in GUIDED_NOGPS mode
vehicle.mode = VehicleMode("ALT_HOLD")
print(vehicle.mode)
vehicle.armed = True
vehicle.mode = VehicleMode("GUIDED_NOGPS")
time.sleep(1)
print(vehicle.mode)

while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)


try:
    while True:
        
        # -- Read the camera frame
        if CAMERA == 0:
            ret, frame = cap.read()
        elif CAMERA == 1:
            frame = np.empty((720, 1280, 3), dtype=np.uint8)
            cap.capture(frame, 'bgr')
        elif CAMERA ==2:
            frames = pipeline.wait_for_frames()
            ir_frame = frames.get_infrared_frame()
            frame = np.asanyarray(ir_frame.get_data())
        elif CAMERA == 3:
            try:
                frames = pipeline.wait_for_frames(500)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
            except:
                print("Frame didn't come in .5 seconds")
                continue

        # -- Convert in gray scale
        if CAMERA != 2:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red
        else:
            gray = frame

        # -- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(gray, dictionary=aruco_dict, parameters=parameters,
                                                     cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        marker_found = False
        xpos=0
        ypos=0
        zpos=0
        if not (ids is None) and ids[0] == id_to_find:
            marker_found = True
            # -- ret = [rvec, tvec, ?]
            # -- array of rotation and position of each marker in camera frame
            # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # -- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # -- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

            # -- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
            math.degrees(roll_marker), math.degrees(pitch_marker),
            math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.1f  y=%4.1f  z=%4.1f" % (pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            x_pos, y_pos, z_pos = pos_camera[0], pos_camera[1], pos_camera[2]

            # -- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
            math.degrees(roll_camera), math.degrees(pitch_camera),
            math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            x_pos, y_pos, z_pos = pos_camera[0], pos_camera[1], pos_camera[2]

            # Calculate angles using PID control and send to vehicle
            # x_pid, y_pid, z_pid = pid_control(x_pos, y_pos, z_pos)
            pitch_angle = pitch_pid(y_pos)
            roll_angle = roll_pid(-x_pos)

            # roll_angle = -x_pid
            # pitch_angle = y_pid
            dronekit_helper.set_attitude(vehicle, roll_angle=roll_angle,pitch_angle=pitch_angle, thrust=0.51)



        #Code below runs even if frame not found

        # --- Display the frame
        if VISUALIZE:
            # print("Frame shown")
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', frame)
            cv2.waitKey(1)

        #update timing data
        cur_time = time.time()
        frame_counter += 1
        if cur_time >= last_time + 1 / PRINT_FREQUENCY:
            # Print position and attitude into output log
            if marker_found:
                print(str_position)
                print(str_attitude)
                # print("PID outputs (x,y,z): ", x_pid, y_pid, z_pid)
                print("Angles (roll,pitch): ", roll_angle, pitch_angle)

            # print framerate
            time_elapsed = cur_time - last_time
            framerate = frame_counter / time_elapsed
            print("Framerate: " + str(round(framerate, 2)) + "\n")

            frame_counter = 0
            last_time = cur_time



        
except:
    print(sys.exc_info())
    raise 
finally:
    print("Closing camera interfaces")
    if CAMERA == 2 or CAMERA == 3:
        pipeline.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()