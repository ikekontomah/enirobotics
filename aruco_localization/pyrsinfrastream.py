import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1280, 720, rs.format.any, 0)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame()

        # Convert images to numpy arrays
        ir_image = np.asanyarray(ir_frame.get_data())

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', ir_image)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
