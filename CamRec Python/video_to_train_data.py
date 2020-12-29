import cv2 as cv
import numpy as np
from FisheyeCorrection.undistort import undistort, get_undistort_map
import time

map1, map2 = get_undistort_map(0.8, 1.0, (640, 480))

num_images_per_class = 10
image_size = 100
src = 'right.avi'
dst = 'test'

video = cv.VideoCapture(src)
if not video.isOpened():
    raise IOError("Cannot open video")

i = 0
while i < num_images_per_class:
    ret, frame = video.read()

    if ret == False:
        break
    
    frame = cv.resize(frame, (image_size, image_size))
    frame = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    
    cv.imwrite(dst + "%d.jpg" % i, frame)

    i += 1
    