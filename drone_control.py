import numpy as np
import cv2
from HandleCommands import HandleCommand, IntelCamera, select_object, get_contours, initialization, AlignDrone


# os.chdir("C:/Users/User/IdeaProjects/ugcs-java-sdk/ucs-client/src/main/java/com/ugcs/ucs/client/samples")
# subprocess.call(
#         'java -cp .;* SendCommand -c direct_vehicle_control -a pitch=0.5 "EMU-101"',
#         shell=True)
# print(os.listdir())
# os.chdir(wd)

def get_distance_lidar(depth, depth_scale):
    depth = depth * depth_scale
    dist = np.median(depth)
    return dist


if __name__ == "__main__":
    init_frame = True
    CommandsHandler = HandleCommand()
    AligningDrone = AlignDrone()
    cv2.namedWindow('image')
    IC = IntelCamera()
    w, h = IC.get_size()
    pipeline = IC.get_pipeline()
    depth_scale = IC.get_depth_scale()

    commands_dist = np.array([])
    dist_to_drone = np.array([])
    centers_of_drone = np.array([])
    mean_dist = 0
    # rect_boarding = None
    rect_boarding = [[444, 316, 543, 375]]
    area_center = None
    aligned_status = 0

    while True:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        ir = frames.get_infrared_frame()
        ir_img = np.array(ir.get_data())
        color_img = np.array(color.get_data())
        depth_img = np.array(depth.get_data())

        color_img = cv2.resize(color_img, (1024, 768))
        if area_center is None:
            area_center, rect_boarding = initialization(color_img, rect_boarding)
        else:
            cv2.rectangle(color_img, (rect_boarding[0][0], rect_boarding[0][1]),
                          (rect_boarding[0][2], rect_boarding[0][3]), (255, 0, 0), 3)

        (x, y, w, h) = get_contours(ir_img, color_img)
        cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        drone_center = (int((x + w / 2)), int((y + h / 2)))
        subtraction = (area_center[0] - drone_center[0], area_center[1] - drone_center[1])
        depth = depth_img[y:y + h, x:x + w].astype(float)
        dist = 0
        if (x, y, w, h) != (0, 0, 0, 0):
            dist = get_distance_lidar(depth=depth, depth_scale=depth_scale)
            if dist == 0:
                continue
            if aligned_status == 0:

                dist_to_drone = np.append(dist_to_drone, dist)

                centers_of_drone = np.append(centers_of_drone, drone_center[0])
                if len(dist_to_drone) < 10:
                    continue
                elif len(dist_to_drone) == 10:
                    mean_dist = np.median(dist_to_drone)
                    print(mean_dist)
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
                    continue
                elif len(dist_to_drone) == 10:
                    mean_dist = np.median(dist_to_drone)
                    print(mean_dist)
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
        axes = ['X (L-R): ', 'Y (U-D): ', 'Z (F-B): ']
        for index, ax in zip(range(3), axes):
            cv2.putText(color_img, ax + CommandsHandler.get_command(index=index), (750, 630 + index * 40), 0, 1,
                        CommandsHandler.get_color(index=index), 4)
        cv2.putText(color_img, str(round(dist, 3)) + ' m', (750, 750), 0, 1, (0, 255, 255), 4)
        cv2.imshow('rgb', color_img)
        # cv2.imshow('image', ir_img)
        # cv2.imshow('thresh', thresh)
        # cv2.imshow('depth', depth_img)
        if cv2.waitKey(20) & 0xFF == 27:
            break

    cv2.destroyAllWindows()
