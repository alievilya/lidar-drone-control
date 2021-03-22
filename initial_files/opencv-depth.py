import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API



print("Environment Ready")
# Setup:

# Skip 5 first frames to give the Auto-Exposure time to adjust
while True:
    pipe = rs.pipeline()
    cfg = rs.config()
    # cfg.enable_device_from_file("../object_detection.bag")
    profile = pipe.start(cfg)

    # Store next frameset for later processing:
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    # Cleanup:
    pipe.stop()
    print("Frames Captured")

    color = np.asanyarray(color_frame.get_data())
    plt.rcParams["axes.grid"] = False
    plt.rcParams['figure.figsize'] = [12, 6]

    colorizer = rs.colorizer()

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

    # Show the two frames together:
    images = np.hstack((color, colorized_depth))
    plt.imshow(images)


    depth = np.asanyarray(aligned_depth_frame.get_data())


    # Get data scale from the device and convert to meters
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    depth = depth * depth_scale
    dist,_,_,_ = cv2.mean(depth)
    plt.show()

    print(dist)
