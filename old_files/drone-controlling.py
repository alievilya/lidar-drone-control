import pyrealsense2 as rs
import numpy as np
import cv2
import os
import subprocess

print(os.listdir())
# SendCommand.java


# os.chdir("C:/Users/User/IdeaProjects/ugcs-java-sdk/ucs-client/src/main/java/com/ugcs/ucs/client/samples")
# subprocess.call(
#         'java -cp .;* SendCommand -c direct_vehicle_control -a pitch=0.5 "EMU-101"',
#         shell=True)
# print(os.listdir())
# os.chdir(wd)

rect_endpoint_tmp = []
rect_bbox = []
drawing = False


def select_object(img):
    """
    Interactive select rectangle ROIs and store list of bboxes.

    Parameters
    ----------
    img :
           image 3-dim.

    Returns
    -------
    bbox_list_rois : list of list of int
           List of bboxes of rectangle rois.
    """

    # mouse callback function
    bbox_list_rois = []

    def draw_rect_roi(event, x, y, flags, param):

        # grab references to the global variables
        global rect_bbox, rect_endpoint_tmp, drawing

        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that drawing is being
        # performed. set rect_endpoint_tmp empty list.
        if event == cv2.EVENT_LBUTTONDOWN:
            rect_endpoint_tmp = []
            rect_bbox = [(x, y)]
            drawing = True

        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # drawing operation is finished
            rect_bbox.append((x, y))
            drawing = False

            # draw a rectangle around the region of interest
            p_1, p_2 = rect_bbox
            cv2.rectangle(img, p_1, p_2, color=(0, 255, 0), thickness=1)
            cv2.imshow('image', img)

            # for bbox find upper left and bottom right points
            p_1x, p_1y = p_1
            p_2x, p_2y = p_2

            lx = min(p_1x, p_2x)
            ty = min(p_1y, p_2y)
            rx = max(p_1x, p_2x)
            by = max(p_1y, p_2y)

            # add bbox to list if both points are different
            if (lx, ty) != (rx, by):
                bbox = [lx, ty, rx, by]
                bbox_list_rois.append(bbox)

        # if mouse is drawing set tmp rectangle endpoint to (x,y)
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            rect_endpoint_tmp = [(x, y)]

    # clone image img and setup the mouse callback function
    img_copy = img.copy()
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('image', 1200, 600)
    cv2.setMouseCallback('image', draw_rect_roi)

    # keep looping until the 'c' key is pressed
    while True:
        # display the image and wait for a keypress
        if not drawing:
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('image', 1200, 600)
            cv2.imshow('image', img)
        elif drawing and rect_endpoint_tmp:
            rect_cpy = img.copy()
            start_point = rect_bbox[0]
            end_point_tmp = rect_endpoint_tmp[0]
            cv2.rectangle(rect_cpy, start_point, end_point_tmp, (0, 255, 0), 1)
            cv2.imshow('image', rect_cpy)

        key = cv2.waitKey(1) & 0xFF
        # if the 'c' key is pressed, break from the loop
        if key == ord('c'):
            break
    # close all open windows
    cv2.destroyAllWindows()

    return bbox_list_rois


if __name__ == "__main__":
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)
    BLUE = (255, 0, 0)

    init_frame = True
    cv2.namedWindow('image')
    # cv2.setMouseCallback('image', click_event)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.infrared, 1024, 768, rs.format.y8, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    pipeline.start(config)

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height
    commands_dist = np.array([])
    mean_dist = 0
    command2 = ''
    # rect_boarding = None
    rect_boarding = [[444, 316, 543, 375]]
    color0 = color1 = color2 = (0, 120, 120)
    while 1:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        ir = frames.get_infrared_frame()
        ir_img = np.array(ir.get_data())
        color_img = np.array(color.get_data())
        depth_img = np.array(depth.get_data())

        # reading the image

        mean_img = np.mean(ir_img)
        # print(mean_img)
        thresh = cv2.threshold(ir_img, mean_img * 1.5, 255, cv2.THRESH_BINARY)[1]
        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # отображаем контуры поверх изображения
        color_img = cv2.resize(color_img, (1024, 768))
        if init_frame:
            if not rect_boarding:
                rect_boarding = select_object(color_img)
                print(rect_boarding)
            area_center = (
            (rect_boarding[0][0] + rect_boarding[0][2]) / 2, (rect_boarding[0][1] + rect_boarding[0][3]) / 2)
            init_frame = False
        else:
            cv2.rectangle(color_img, (rect_boarding[0][0], rect_boarding[0][1]),
                          (rect_boarding[0][2], rect_boarding[0][3]), (255, 0, 0), 3)
        cv2.drawContours(color_img, contours, -1, (255, 0, 0), 3, cv2.LINE_AA, hierarchy, 1)
        cv2.fillPoly(color_img, contours, color=(255, 0, 255))
        cv2.drawContours(ir_img, contours, -1, (255, 0, 0), 3, cv2.LINE_AA, hierarchy, 1)
        area = 0
        (x, y, w, h) = 0, 0, 0, 0
        for contour in contours:
            cur_cont_area = cv2.contourArea(contour)
            if cur_cont_area > area:
                (x, y, w, h) = cv2.boundingRect(contour)
            area = cur_cont_area
        cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        drone_center = (int((x + w / 2)), int((y + h / 2)))
        subtraction = (area_center[0] - drone_center[0], area_center[1] - drone_center[1])
        depth = depth_img[y:y + h, x:x + w].astype(float)
        command0 = ''
        command1 = ''

        dist = 0
        if (x, y, w, h) != (0, 0, 0, 0):

            if subtraction[0] > 10:
                command0 = 'RIGHT'
                color0 = BLUE
            elif subtraction[0] < -10:
                command0 = 'LEFT'
                color0 = BLUE
            else:
                command0 = 'sides OK'
                color0 = GREEN

            if subtraction[1] > 10:
                command1 = 'DOWN'
                color1 = BLUE
            elif subtraction[1] < -10:
                command1 = 'UP'
                color1 = BLUE
            else:
                command1 = 'height OK'
                color1 = GREEN
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth = depth * depth_scale
            dist = np.max(depth)
            commands_dist = np.append(commands_dist, dist)
            if len(commands_dist) == 5:
                mean_dist = np.median(commands_dist)
                commands_dist = np.array([])
            # command3 = dist_handler(mean_dist)
            if command0 == 'sides OK' and command1 == 'height OK':

                if mean_dist > 0 and 2.1 <= mean_dist <= 2.3:
                    command2 = 'dist OK'
                    color2 = GREEN
                elif mean_dist > 0 and mean_dist < 2.2:
                    command2 = 'further'
                    color2 = BLUE
                elif mean_dist > 0 and mean_dist > 2.2:
                    command2 = 'closer'
                    color2 = BLUE
                elif mean_dist == 0:
                    command2 = 'err'
            print(dist)
        cv2.putText(color_img, command0, (700, 600), 0, 1, color0, 4)
        cv2.putText(color_img, command1, (700, 650), 0, 1, color1, 4)
        cv2.putText(color_img, command2, (700, 700), 0, 1, color2, 4)
        cv2.putText(color_img, str(round(dist, 3)), (700, 750), 0, 1, (0, 255, 255), 4)
        cv2.imshow('image', ir_img)
        cv2.imshow('thresh', thresh)
        cv2.imshow('rgb', color_img)
        cv2.imshow('depth', depth_img)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
