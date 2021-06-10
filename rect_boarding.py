import os

import cv2
import imutils
import numpy as np
from HandleCommands import HandleCommand, IntelCamera, select_object, get_contours, initialization, AlignDrone

font = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture("rtsp://admin:admin@192.168.1.52:554/1/h264major")

def find_centroid(bbox):
    return int((bbox[0] + bbox[2]) / 2), int((bbox[1] + bbox[3]) / 2)


def histogram_equalize(image):
    img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    # equalize the histogram of the Y channel
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    # convert the YUV image back to RGB format
    image_equalized = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return image_equalized


def image_filtering(image):
    image = imutils.resize(image, width=640)
    img_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    sat_max = int(np.max(img_HSV[:, :, 2]))
    sat_low = int(sat_max*0.6)
    low_hsv = (0, 0, sat_low)
    high_hsv = (255, 50, sat_max)
    img_threshold = cv2.inRange(img_HSV, low_hsv, high_hsv)
    img_gray = cv2.GaussianBlur(img_threshold, (7, 7), 0)
    img_gray = cv2.dilate(img_gray, None, iterations=5)
    # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 51, 2)
    return img_gray


def return_bboxes(contours, img=None):
    bboxes = []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        if area < 100:
            continue
        if len(approx) != 4:
            continue
        elif len(approx) == 4:
            # cv2.drawContours(img, [approx], 0, (255, 0, 0), 5)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            if img is not None:
                cv2.putText(img, "Rectangle", (x, y), font, 1, (0))
            bboxes.append((area, approx))
    if len(bboxes) == 0:
        print('no contours detected')
        return None
    else:
        return bboxes

if __name__ == "__main__":
    to_draw_biggest = True
    ret = True
    images = os.listdir('data')
    aligned_status = 0
    CommandsHandler = HandleCommand()
    AligningDrone = AlignDrone()

    for img_name in images:
    # while True:
        # ret, img = cap.read()
        # if not ret:
        #     break

        img = cv2.imread("data/{}".format("test13.jpg"), cv2.IMREAD_COLOR)
        img = imutils.resize(img, width=640)
        img_eq = histogram_equalize(img)
        res_img = np.hstack((img, img_eq))
        # img = img_eq
        img_threshold = image_filtering(img)

        # res_img2 = np.hstack((img_eq, img_threshold))
        cv2.imshow("res", res_img)
        cv2.imshow("img_threshold", img_threshold)
        cv2.waitKey(0)

        contours, _ = cv2.findContours(img_threshold, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        bboxes = return_bboxes(contours, img=img)
        if not bboxes:
            continue
        biggest_contour = max(bboxes, key=lambda x: x[0])[1]
        cv2.drawContours(img, [biggest_contour], 0, (255, 0, 0), 5)
        # max_bbox = [biggest_contour[n][0] for n in range(len(biggest_contour))]
        (x, y, w, h) = cv2.boundingRect(biggest_contour)
        bbox_max = [x, y, x+w, y+h]

        img_centroid = (int(img.shape[1] / 2), int(img.shape[0] / 2))  # [x, y]
        bbox_centroid = find_centroid(bbox_max)
        x_delta, y_delta = [img_centroid[n] - bbox_centroid[n] for n in range(2)]
        #  if x_delta > 0 than need to move right, y_delta > 0 then need to move back

        #  aligned statuses meanings:
        #  0 - not aligned yet,
        #  1 - first command was sent (roll 0.1)
        #  2 - successfully performed aligning
        #  3 - aligning is not necessary
        if aligned_status == 0:
            dist_to_drone = np.append(dist_to_drone, dist)
            centers_of_drone = np.append(centers_of_drone, drone_center[0])
            if len(dist_to_drone) < 10:
                print('init')
                time.sleep(1)
            elif len(dist_to_drone) == 10:
                mean_dist = np.median(dist_to_drone)
                print('init dist: ', mean_dist)
                mean_center_of_drone = np.median(centers_of_drone)
                AligningDrone.set_init_coords(mean_dist, mean_center_of_drone)
                aligned_status = AligningDrone.initial_move()
                dist_to_drone = np.array([])
                centers_of_drone = np.array([])
            continue
        elif aligned_status == 1:
            dist_to_drone = np.append(dist_to_drone, dist)
            centers_of_drone = np.append(centers_of_drone, drone_center[0])
            if len(dist_to_drone) < 10:
                print('last')
                time.sleep(1)
            elif len(dist_to_drone) == 10:
                mean_dist = np.median(dist_to_drone)
                print('last dist: ', mean_dist)
                mean_center_of_drone = np.median(centers_of_drone)
                AligningDrone.set_last_coords(mean_dist, mean_center_of_drone)
                aligned_status = AligningDrone.handle_aligning()
                dist_to_drone = np.array([])
                centers_of_drone = np.array([])
            continue
        elif aligned_status == 2:
            print('drone is aligned')
            aligned_status = 3

        condition2d = CommandsHandler.update2d(subtraction=subtraction)
        # comm = CommandsHandler.sendcommand2d()

        commands_dist = np.append(commands_dist, dist)
        if len(commands_dist) == 10:
            mean_dist = np.median(commands_dist)
            commands_dist = np.array([])
        if condition2d:
            condition3d = CommandsHandler.update3d(distance=mean_dist)
            print(condition3d)

        if x_delta > 10:
            print('move right')
        elif x_delta < -10:
            print('move left')
        else:
            print('x: ok')
        if y_delta > 10:
            print('step back')
        elif y_delta < -10:
            print('step forward')
        else:
            print('y: ok')

        cv2.rectangle(img, [bbox_max[0], bbox_max[1]], [bbox_max[2], bbox_max[3]], (0, 0, 255))
        cv2.line(img, [img_centroid[0], img_centroid[1]-15], [img_centroid[0], img_centroid[1]+15], color=(0, 255, 0))
        cv2.line(img, [img_centroid[0]-15, img_centroid[1]], [img_centroid[0]+15, img_centroid[1]],
                 color=(0, 255, 0))
        cv2.circle(img, (bbox_centroid[0], bbox_centroid[1]), img.shape[0]//100, (0, 0, 255), -1)

        cv2.namedWindow("shapes", cv2.WINDOW_NORMAL)
        cv2.namedWindow("res", cv2.WINDOW_NORMAL)
        cv2.namedWindow("img_threshold", cv2.WINDOW_NORMAL)

        cv2.imshow("shapes", img)

        cv2.resizeWindow("shapes", 1280, 720)
        cv2.resizeWindow("res", 1280, 720)
        cv2.resizeWindow("img_threshold", 1280, 720)
        cv2.waitKey(0)
    cv2.destroyAllWindows()