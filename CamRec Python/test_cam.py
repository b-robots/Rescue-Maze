import cv2 as cv
import numpy as np
from FisheyeCorrection.undistort import undistort, get_undistort_map
import time

map1, map2 = get_undistort_map(1.0, 1.01)

cam = cv.VideoCapture(0) # left
if not cam.isOpened():
    raise IOError("Cannot open webcam")
cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

cam2 = cv.VideoCapture(2) # left
if not cam2.isOpened():
    raise IOError("Cannot open webcam")
cam2.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam2.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam2.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

def detect_color(img):
    hsv = cv.cvtColor(cv.blur(img, (5, 5)), cv.COLOR_BGR2HSV)

    min_s = 100
    min_v = 50
    min_color = 20

    red_mask = cv.bitwise_or(cv.inRange(hsv, (0, min_s, min_v), (15, 255, 255)), cv.inRange(hsv, (180 - 15, min_s, min_v), (180, 255, 255)))
    green_mask = cv.inRange(hsv, (35, min_s, min_v), (60 + 15, 255, 255))
    yellow_mask = cv.inRange(hsv, (30 - 15, min_s, min_v), (35, 255, 255))

    redness = red_mask.mean()
    greeness = green_mask.mean()
    yellowness = yellow_mask.mean()

    if redness > greeness and redness > yellowness:
        if redness < min_color:
            return b'N'
        else:
            return b'R'
    elif greeness > redness and greeness > yellowness:
        if greeness < min_color:
            return b'N'
        else:
            return b'G'
    else:
        if yellowness < min_color:
            return b'N'
        else:
            return b'Y'

while True:
    start_time = time.time()
    _, img = cam.read()
    cv.imshow('original', img)
    color = detect_color(img)
    cv.waitKey(1)
    print(color)
    print("fps: " + str(round(1.0 / (time.time() - start_time), 2)))