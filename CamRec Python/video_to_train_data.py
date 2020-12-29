import cv2 as cv
import numpy as np
from FisheyeCorrection.undistort import undistort, get_undistort_map
import time

map1, map2 = get_undistort_map(0.8, 1.0, (640, 480))

num_images_per_class = 350
image_size = 100

def get_undist_roi(img, map1, map2):
    img = undistort(img, map1, map2)
    img = img[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]
    return img

video = cv.VideoCapture('train/U.mp4')
if not video.isOpened():
    raise IOError("Cannot open video")

i = 0
while i < num_images_per_class:
    ret, frame = video.read()

    if ret == False:
        break
    
    frame = undistort(cv.resize(frame, (640, 480)), map1, map2)
    frame = cv.resize(frame, (320, 240))
    frame = frame[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]
    frame = cv.resize(frame, (image_size, image_size))
    frame = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    
    cv.imwrite("train/U%d.jpg" % i, frame)

    i += 1
    