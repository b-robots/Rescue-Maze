import cv2 as cv
import numpy as np
import time

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
    
    frame = cv.resize(frame.copy, (image_size, image_size))
    frame = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    
    cv.imwrite(dst + "%d.jpg" % i, frame)

    i += 1
    