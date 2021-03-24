import time

import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os
from collections import OrderedDict
from itertools import groupby
import socket


def all_equal(iterable):
    g = groupby(iterable)
    return next(g, True) and not next(g, False)


class HandleCommand:
    def __init__(self):
        self.host = "localhost"
        self.port = 8080
        self.commands_dict = OrderedDict()
        self.states_dict = OrderedDict()
        self.color_dict = OrderedDict()
        self.initialized_direction = None
        self.movement_speed = 0.01

    def set_color(self, condition):
        color = [0, 0, 0]
        color[condition] = 255
        return color

    def set_state_ok(self, index):
        self.states_dict[index] = 1
        self.commands_dict[index] = "OK"
        self.color_dict[index] = self.set_color(1)

    def set_state_less(self, index):
        self.states_dict[index] = -1
        self.commands_dict[index] = "less"
        self.color_dict[index] = self.set_color(-1)

    def set_state_more(self, index):
        self.states_dict[index] = 0
        self.commands_dict[index] = "more"
        self.color_dict[index] = self.set_color(0)

    #  right - more
    # left - less
    # up - more
    # down - less
    #  index 0 - left-right
    # index 1 - up-down
    # index 2 - far-close
    # import socket
    #
    def send_command(self, com_str):
        time.sleep(5)
        print(com_str)
        # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # sock.connect((self.host, self.port))
        # sock.sendall((com_str + '\n').encode('utf-8'))
        # data = sock.recv(1024)
        # print(data)
        # if data == b'profit\r\n':
        #     print('ok')
        # else:
        #     print('lol')
        # sock.close()

    def control2d(self, vector_subtraction):
        # sides control
        # i = 0 -  left right control
        # i = 1 - up and down
        command_str = None
        if vector_subtraction[0] < -30:
            self.set_state_more(index=0)
            command_str = "roll,{}".format(self.movement_speed)
            self.send_command(com_str=command_str)
        elif vector_subtraction[0] > 30:
            self.set_state_less(index=0)
            command_str = "roll,-{}".format(self.movement_speed)
            self.send_command(com_str=command_str)
        else:
            self.set_state_ok(index=0)
        if vector_subtraction[1] < -30:
            self.set_state_more(index=1)
            command_str = "throttle,{}".format(self.movement_speed)
            self.send_command(com_str=command_str)
        elif vector_subtraction[1] > 30:
            self.set_state_less(index=1)
            command_str = "throttle,-{}".format(self.movement_speed)
            self.send_command(com_str=command_str)
        else:
            self.set_state_ok(index=1)

        # for i in range(2):
        #     if vector_subtraction[i] > 10:
        #         self.set_state_less(index=i)
        #     elif vector_subtraction[i] < -10:
        #         self.set_state_more(index=i)
        #     else:
        #         self.set_state_ok(index=i)

    def is_2d_states_ok(self):
        states = [el for key, el in self.states_dict.items() if el == 1]
        if len(states) != 2:
            return False
        else:
            return all_equal(states)

    def update2d(self, subtraction):
        # if self.initialized_direction is None:
        #     self.send_command("pitch,1.0")
        self.control2d(subtraction)
        return self.is_2d_states_ok()

    def control3d(self, distance):
        if distance > 0 and 2.1 <= distance <= 2.3:
            self.set_state_ok(index=2)
            print('go landing')
        elif 0 < distance < 2.1:
            self.set_state_more(index=2)
            command_str3d = "pitch,-{}".format(self.movement_speed)
            self.send_command(com_str=command_str3d)
        elif distance > 0 and distance > 2.3:
            self.set_state_less(index=2)
            command_str3d = "pitch,{}".format(self.movement_speed)
            self.send_command(com_str=command_str3d)

        # elif distance == 0:
        #     command2 = 'err'

    def is_3d_states_ok(self):
        states = [el for key, el in self.states_dict.items() if el == 1]
        if len(states) != 3:
            return False
        else:
            return all_equal(states)

    def update3d(self, distance):
        self.control3d(distance)
        return self.is_3d_states_ok()

    def get_color(self, index):
        return self.color_dict.get(index, (0, 0, 255))

    def get_command(self, index):
        return self.commands_dict.get(index, "Nan")


class AlignDrone(HandleCommand):
    def __init__(self):
        super().__init__()
        self.roll_val = 0.1
        self.R = 0.5  # distance of the roll operation (m)
        self.height = 1.05  # height from drone (m)
        self.thresh_pixels = 15  # threshold
        self.tresh_meters = 0  # pogreshnost (m)

    def set_init_coords(self, distance, center_value):
        self.init_distance = distance
        self.init_center_value = center_value

    def set_last_coords(self, distance, center_value):
        self.last_distance = distance
        self.last_center_value = center_value

    def find_angle(self):
        self.init_distance_x = (self.init_distance ** 2 - self.height ** 2) ** 0.5
        self.last_distance_x = (self.last_distance ** 2 - self.height ** 2) ** 0.5
        print('init x:', self.init_distance_x, '\nlast x:', self.last_distance_x )
        arg = (self.init_distance_x ** 2 + self.R ** 2 - self.last_distance_x ** 2) / \
              (2 * self.init_distance_x * self.R)
        angle = math.degrees(math.acos(arg))
        print('angle gamma: ', angle)
        return angle

    def initial_move(self):
        self.command_str = "roll,{}".format(self.roll_val)
        self.send_command(com_str=self.command_str)
        return 1

    def move_back(self):
        self.command_str = "roll,-{}".format(self.roll_val)
        self.send_command(com_str=self.command_str)

    def handle_aligning(self):
        self.vector_x = self.last_center_value - self.init_center_value
        gamma = self.find_angle()
        self.vector_dist = self.last_distance_x - self.init_distance_x
        move_angle = 0

        self.command_yaw = ""
        if self.vector_dist < self.tresh_meters:
            if self.vector_x < -1*self.thresh_pixels:  # mb set move_angle 90, if in threshold value
                move_angle = 90 - gamma
                print('moved left {}'.format(self.vector_x))
            elif self.vector_x > self.thresh_pixels:
                move_angle = 90 + gamma
                print('moved right {}'.format(self.vector_x))
            else:
                move_angle = 90
                print('thresholded {}'.format(self.vector_x))
            move_arg = 0.011 * move_angle
            self.command_yaw = "{} yaw,(+){}".format(move_angle, round(move_arg, 3))

        if self.vector_dist > self.tresh_meters:
            if self.vector_x > self.thresh_pixels:
                move_angle = 270 - gamma
                print('moved right {}'.format(self.vector_x))
            elif self.vector_x < -1*self.thresh_pixels:
                move_angle = gamma - 90
                print('moved left {}'.format(self.vector_x))
            else:
                move_angle = 90
                print('thresholded -{}'.format(self.vector_x))
            move_arg = 0.011 * move_angle
            self.command_yaw = "{} yaw,(-){}".format(move_angle, round(move_arg, 3))
        self.move_back()
        self.send_command(com_str=self.command_yaw)
        return 2


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


def get_contours(infrared_img, color_image):
    mean_img = np.mean(infrared_img)
    max_img = np.max(infrared_img)
    thresh = cv2.threshold(infrared_img, mean_img * 1.8, 255, cv2.THRESH_BINARY)[1]
    contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(color_image, contours, -1, (255, 0, 0), 3, cv2.LINE_AA, hierarchy, 1)
    cv2.fillPoly(color_image, contours, color=(255, 0, 255))
    # cv2.drawContours(infrared_img, contours, -1, (255, 0, 0), 3, cv2.LINE_AA, hierarchy, 1)
    area = 0
    (x, y, w, h) = 0, 0, 0, 0
    top_contour = (0, 0, 0, 0)
    for contour in contours:
        cur_cont_area = cv2.contourArea(contour)
        if cur_cont_area > area:
            (x, y, w, h) = cv2.boundingRect(contour)
            crop_ir_img = infrared_img[y:y + h, x:x + w]
            if max_img in crop_ir_img:
                top_contour = (x, y, w, h)

        area = cur_cont_area
    if top_contour:
        return top_contour
    else:
        return (0, 0, 0, 0)  # (x, y, w, h)


def initialization(color_img, rect_boarding):
    if not rect_boarding:
        rect_boarding = select_object(color_img)
        print(rect_boarding)
    area_center = ((rect_boarding[0][0] + rect_boarding[0][2]) / 2, (rect_boarding[0][1] + rect_boarding[0][3]) / 2)
    init_frame = False
    return area_center, rect_boarding


class IntelCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.infrared, 1024, 768, rs.format.y8, 30)
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        self.pipeline.start(self.config)

        self.profile = self.pipeline.get_active_profile()
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height

    def get_size(self):
        return self.w, self.h

    def get_pipeline(self):
        return self.pipeline

    def get_depth_scale(self):
        return self.depth_scale


if __name__ == "__main__":
    CommandsHandler = HandleCommand()

    recall = CommandsHandler.update2d([3, 5, 2.8])
    if recall:
        res = CommandsHandler.update3d(2.7)
    print(res)
