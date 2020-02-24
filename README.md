# ENI-Robotic-Inspection
Do run the calibration code, do :
python camera_calibration.py

To get the parameters of the camera, where the camera parameters are:

#Reprojection Error - (ret)
#Camera Matrix - (mtx)
#Distortion Coefficients - (dist)
#Rotation Vector - (rvecs)
#Translation Vector - (tvecs)
Do:
print(ret), print(mtx), print(dist), print(rvecs), print(tvecs)

To see the results of the calibration, uncomment lines 82 to 103 and run the python script again using:
python camera_calibration.py

To see the helper functions that interface with aruco_pose_estimation.py, got to:
aruco_localization/dronekit_helper.py.
The algorithm can be found in dronekit_helper.pid_control, implemented based on:
http://robotsforroboticists.com/pid-control/

#Folders
Aruco_localization: contains landing scripts and advanced dronekit testing functionality
Aruco_pose_estimation.py: main script that processes camera frames and then sends commands to Dronekit to land the quadcopter
Att_set_test.py: keyboard interface for sending desired attitude commands to dronekit
*.txt files: camera calibration coefficients

Dronekit_scripts: contains scripts for testing basic dronekit functionalities

Camera_cal: camera calibration code
Cameracalib.py: modified version of script at https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/cameracalib.py that performs live calibration of the camera

Camera_cal2: legacy camera calibration code

Root folder
Camera_calibration.py, argmin.py, utils.py: all legacy camera calibration scripts
