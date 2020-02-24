import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

from importlib import reload
import utils; reload(utils)
from utils import *

calibration_dir = "camera_cal2"
test_imgs_dir   = "test_images"

cal_imgs_paths = glob.glob(calibration_dir + "/*.jpg") #linking path to calibration images

# Get a given chessboard image
cal_img_path = cal_imgs_paths[11]
cal_img = load_image(cal_img_path)
plt.imshow(cal_img)
plt.show()

cx = 6
cy = 4

def findChessboardCorners(img, nx, ny):
    """
    Finds the chessboard corners of the supplied image (must be grayscale)
    nx and ny parameters respectively indicate the number of inner corners in the x and y directions
    """
    return cv2.findChessboardCorners(img, (nx, ny), None)

def showChessboardCorners(img, nx, ny, ret, corners):
    """
    Draws the chessboard corners of a given image
    nx and ny parameters respectively indicate the number of inner corners in the x and y directions
    ret and corners should represent the results from cv2.findChessboardCorners()
    """
    c_img = cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
    plt.axis('off')
    plt.imshow(img)
    plt.show()

ret, corners = findChessboardCorners(to_grayscale(cal_img), cx, cy)
showChessboardCorners(cal_img, cx, cy, ret, corners)

def findImgObjPoints(imgs_paths, nx, ny):
    """
    Returns the objects and image points computed for a set of chessboard pictures taken from the same camera
    nx and ny parameters respectively indicate the number of inner corners in the x and y directions
    """
    objpts = []
    imgpts = []
    
    # Pre-compute what our object points in the real world should be (the z dimension is 0 as we assume a flat surface)
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    
    for img_path in imgs_paths:
        img = load_image(img_path)
        gray = to_grayscale(img)
        ret, corners = findChessboardCorners(gray, nx, ny)
        
        if ret:
            # Found the corners of an image
            imgpts.append(corners)
            # Add the same object point since they don't change in the real world
            objpts.append(objp)
    
    return objpts, imgpts

opts, ipts = findImgObjPoints(cal_imgs_paths, cx, cy)

def undistort_image(img, objpts, imgpts):
    """
    Returns an undistorted image
    The desired object and image points must also be supplied to this function
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpts, imgpts, to_grayscale(img).shape[::-1], None, None)
    print(tvecs)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

cal_img_example = load_image(cal_imgs_paths[0])
cal_img_undist = undistort_image(cal_img_example, opts, ipts)
# fig, ax = plt.subplots(1, 2, figsize=(10,7))
# ax[0].imshow(cal_img_example)
# ax[0].axis("off")
# ax[0].set_title("Distorted Image")

#ax[1].imshow(cal_img_undist)
#ax[1].axis("off")
#ax[1].set_title("Undistorted Image")

#plt.show()




#Camera Parameters
#Reprojection Error - (ret)
#Camera Matrix - (mtx)
#Distortion Coefficients - (dist)
#Rotation Vector - (rvecs)
#Translation Vector - (tvecs)


#ret=  2.279248181486007 
#mtx = [[3.49367451e+03 0.00000000e+00 1.15118331e+03]
#      [0.00000000e+00 3.46030756e+03 5.62693325e+02]
#      [0.00000000e+00 0.00000000e+00 1.00000000e+00]]

#dist = [[-0.48553696  0.16663135  0.02262255  0.01942334  0.21125403]]

#rvecs = [array([[-0.09077477],
#       [ 0.86856792],
#       [-0.27121281]]), array([[ 0.09181248],
#       [-0.43919929],
#       [ 0.69794075]]), array([[-0.16729474],
#       [-0.10686317],
#       [-0.0103707 ]]), array([[-0.19768712],
#       [ 0.05736848],
#       [-0.10678754]]), array([[-0.19855341],
#       [ 0.0179893 ],
#       [-0.05251507]]), array([[-0.18092865],
#       [ 0.0666005 ],
#       [-0.05089166]])]

# tvecs = [array([[ 0.45834474],
#       [ 0.61618785],
#      [27.21221272]]), array([[-5.09633558],
#       [-3.41589302],
#       [24.31060378]]), array([[ 4.55328145],
#       [ 5.13475466],
#       [24.40293696]]), array([[ 0.36543751],
#       [-2.61966863],
#       [25.55562892]]), array([[ 4.70895042],
#       [-2.26872976],
#       [25.6844452 ]]), array([[-6.90937965],
#       [-2.53749242],
#       [25.94762083]])]
