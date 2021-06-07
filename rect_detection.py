import cv2
import numpy as np
font = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture("rtsp://admin:admin@192.168.1.52:554/1/h264major")

def find_centroid(bbox):
    return int((bbox[0] + bbox[2]) / 2), int((bbox[1] + bbox[3]) / 2)



if __name__ == "__main__":
    to_draw_biggest = True
    ret = True
    while True:
        # ret, img = cap.read()
        # if not ret:
        #     break
        img = cv2.imread("data/test.jpg", cv2.IMREAD_COLOR)

        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # low_hsv = (21, 114, 175)
        # high_hsv = (32, 140, 200)

        # low_hsv = (15, 114, 175)
        low_hsv = (5, 0, 235)
        # high_hsv = (32, 140, 250)
        high_hsv = (255, 50, 255)
        img_threshold = cv2.inRange(img_HSV, low_hsv, high_hsv)

        img_gray = cv2.GaussianBlur(img_threshold, (17, 17), 0)
        img_gray = cv2.dilate(img_gray, None, iterations=4)
        # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # _, img_gray = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(img_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        bboxes = []
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if area < 200:
                continue

            if len(approx) != 4:
                continue
            elif len(approx) == 4:
                # cv2.drawContours(img, [approx], 0, (255, 0, 0), 5)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                cv2.putText(img, "Rectangle", (x, y), font, 1, (0))
                bboxes.append((area, approx))

        biggest_contour = max(bboxes, key=lambda x: x[0])[1]
        # biggest_contour = bboxes[6][1]
        cv2.drawContours(img, [biggest_contour], 0, (255, 0, 0), 5)
        # max_bbox = [biggest_contour[n][0] for n in range(len(biggest_contour))]
        (x, y, w, h) = cv2.boundingRect(biggest_contour)
        bbox_max = [x, y, x+w, y+h]

        img_centroid = (int(img.shape[1] / 2), int(img.shape[0] / 2))  # [x, y]
        bbox_centroid = find_centroid(bbox_max)
        x_delta, y_delta = [img_centroid[n] - bbox_centroid[n] for n in range(2)]
        #  if x_delta > 0 than need to move right, y_delta > 0 then need to move back
        if x_delta > 10:
            print('move right')
        elif x_delta < -10:
            print('move left')
        else:
            print('x is ok')
        if y_delta > 10:
            print('step back')
        elif y_delta < -10:
            print('step forward')
        else:
            print('y is ok')

        cv2.rectangle(img, [bbox_max[0], bbox_max[1]], [bbox_max[2], bbox_max[3]], (0, 0, 255))
        cv2.line(img, [img_centroid[0], img_centroid[1]-15], [img_centroid[0], img_centroid[1]+15], color=(0, 255, 0))
        cv2.line(img, [img_centroid[0]-15, img_centroid[1]], [img_centroid[0]+15, img_centroid[1]],
                 color=(0, 255, 0))
        cv2.circle(img, (bbox_centroid[0], bbox_centroid[1]), img.shape[0]//100, (0, 0, 255), -1)

        cv2.namedWindow("shapes", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("img_gray", cv2.WINDOW_NORMAL)
        cv2.namedWindow("img_threshold", cv2.WINDOW_NORMAL)

        cv2.imshow("shapes", img)
        # cv2.imshow("img_gray", img_gray)
        # img_threshold = cv2.cvtColor(img_threshold, cv2.COLOR_HSV2BGR)
        cv2.imshow("img_threshold", img_threshold)

        cv2.resizeWindow("shapes", 1280, 720)
        cv2.resizeWindow("Threshold", 1280, 720)
        cv2.resizeWindow("img_threshold", 1280, 720)
        cv2.waitKey(1)
    cv2.destroyAllWindows()