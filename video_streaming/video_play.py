
# videofile = cv2.VideoCapture("rtsp://192.168.1.22:3340/298CGBKR0A0A48",cv2.CAP_FFMPEG)
# # videofile = cv2.VideoCapture(0)
# rr, first_frame = videofile.read()
#
# while rr:
#     ret, frame = videofile.read()
#     cv2.imshow('ff', frame)
#     cv2.waitKey(0)

# ffplay -fflags nobuffer -rtsp_transport udp rtsp://192.168.1.22:3340/298CGBKR0A0A48



# ffplay -rtsp_flags listen rtsp://@25.21.155.5:3340
# ffplay -rtsp_transport tcp rtsp://@25.21.155.5:3340
# ffmpeg -i rtsp://@25.21.155.5:3340 -r 15 C:/Users/aliev/Desktop/03.mp4
# ffmpeg -i rtsp://@192.168.241.1:62156 -acodec copy -vcodec copy c:/abc.mp4

# import vlc
# from os import system
# import time
# player= vlc.MediaPlayer("rtsp://192.168.1.22:3340/298CGBKR0A0A48")
# player.play()
# while 1:
#     xx = player.video_take_snapshot(0, '.snapshot.tmp.png', 0, 0)


# system("ffplay -fflags nobuffer -rtsp_transport udp rtsp://192.168.1.22:3340/298CGBKR0A0A48")


# stream = ffmpeg.input('save_data/out_1.mp4')
# stream = ffmpeg.hflip(stream)
# stream = ffmpeg.output(stream, 'save_data/out_156.mp4')
# ffmpeg.run(stream)


# out, _ = (
#     ffmpeg
#         .input('save_data/out_1.mp4')
#         .output('pipe:', format='rawvideo', pix_fmt='rgb24')
#         .run(capture_stdout=True)
# )
# video = np.frombuffer(out, np.uint8).reshape([-1, 480, 640, 3])
#

import cv2

import ffmpeg
import numpy as np

# link ="rtsp://192.168.1.22:3340/298CGBKR0A0A48"
link ="rtsp://admin:admin@192.168.1.18:554/1/h264major"
# link ="save_data/out_1.mp4"
target_fps = 20
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

while process.poll() is None:
    packet = process.stdout.read(packet_size)

    if cnt % period == 0 and cnt > 1:
        img_np = np.frombuffer(packet, np.uint8).reshape([height, width, cl_channels])
        img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        cv2.imshow('s', img_rgb)
        cv2.waitKey(1)

    cnt += 1
