# import the necessary packages
import numpy as np
import argparse
import imutils
import glob
import cv2
import ffmpeg
# construct the argument parser and parse the arguments


# load the image image, convert it to grayscale, and detect edges
template = cv2.imread('tmp4.png', 1)
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 50, 200)
(tH, tW) = template.shape[:2]
cv2.imshow("Template", template)


# link ="rtsp://192.168.1.22:3340/298CGBKR0A0A48"
link ="rtsp://admin:admin@192.168.1.18:554/1/h264major"
# link ="save_data/out_1.mp4"
target_fps = 10
probe = ffmpeg.probe(link)
video_stream = next((stream for stream in probe['streams'] if stream['codec_type'] == 'video'), None)
width = int(video_stream['width'])
height = int(video_stream['height'])
fps = video_stream['r_frame_rate'].split('/')
fps = float(fps[0]) / float(fps[1])
period = int(fps / target_fps)
cl_channels = 3
packet_size = width * height * cl_channels
process = ffmpeg.input(link).output('-', format='rawvideo', pix_fmt='rgb24').run_async(pipe_stdout=True)
cnt = 0
to_visualize = False

while process.poll() is None:
    img_rgb = None
    packet = process.stdout.read(packet_size)


    img_np = np.frombuffer(packet, np.uint8).reshape([height, width, cl_channels])
    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        # cv2.imshow('s', img_rgb)
        # cv2.waitKey(1)



# loop over the images to find the template in
# for imagePath in glob.glob(args["images"] + "/*.jpg"):
    # load the image, convert it to grayscale, and initialize the
    # bookkeeping variable to keep track of the matched region

    # image = cv2.imread(imagePath)
    if img_rgb is None:
        continue
    else:
        image = img_rgb
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = None
        # loop over the scales of the image
        for scale in np.linspace(0.2, 1.0, 3)[::-1]:
            # resize the image according to the scale, and keep track
            # of the ratio of the resizing
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
            # if the resized image is smaller than the template, then break
            # from the loop
            if resized.shape[0] < tH or resized.shape[1] < tW:
                break

            # detect edges in the resized, grayscale image and apply template
            # matching to find the template in the image
            edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
            # check to see if the iteration should be visualized
            if to_visualize:
                # draw a bounding box around the detected region
                clone = np.dstack([edged, edged, edged])
                cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
                              (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                # cv2.imshow("Visualize", clone)
                # cv2.waitKey(1)
            # if we have found a new maximum correlation value, then update
            # the bookkeeping variable
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)

        # unpack the bookkeeping variable and compute the (x, y) coordinates
        # of the bounding box based on the resized ratio
        (_, maxLoc, r) = found
        (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
        (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
        # draw a bounding box around the detected result and display the image
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
        cv2.namedWindow( "Image", cv2.WINDOW_FREERATIO )
        cv2.imshow("Image", image)
        cv2.waitKey(1)