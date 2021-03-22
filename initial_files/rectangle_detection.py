import cv2

rect_endpoint_tmp = []
rect_bbox = []
drawing = False


def check_square(rect):
    arr_x = []
    arr_y = []
    for n in rect:
        for el in n:
            # for c in el:
            arr_x.append(el[0])
            arr_y.append(el[1])
    w1 = abs(arr_x[0] - arr_x[2])
    h1 = abs(arr_y[0] - arr_y[2])
    if w1 * h1 < 100:
        return False
    else:
        return True


def draw_rect(img):
    width = img.shape[1]
    height = img.shape[0]
    resize = False
    if resize:
        y1 = 0
        y2 = height
        x1 = 0
        x2 = width
        img = cv2.resize(img, (width, height))  # resize image
        roi = img[y1:y2, x1:x2]
        # region of interest i.e where the rectangles will be
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert roi into gray
    blur = cv2.GaussianBlur(gray, (5, 5), 1)  # apply blur to roi
    canny = cv2.Canny(blur, 10, 50)  # apply canny to roi
    # Find my contours
    contours = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    # Loop through my contours to find rectangles and put them in a list, so i can view them individually later.
    cntrRect = []
    for i in contours:
        epsilon = 0.15 * cv2.arcLength(i, True)
        approx = cv2.approxPolyDP(i, epsilon, True)
        if len(approx) == 4:
            big_cont = check_square(approx)
            if big_cont:
                cv2.drawContours(img, cntrRect, -1, (255, 23, 155), 2)
                cv2.imshow('Roi Rect ONLY', img)
                cv2.waitKey(0)
                cntrRect.append(approx)
            else:
                continue

    return img, cntrRect


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
    img = cv2.imread('../data/test.jpg')  # read image
    draw_rect(img)
