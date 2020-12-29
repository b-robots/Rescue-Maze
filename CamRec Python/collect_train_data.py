import cv2 as cv
import numpy as np
from FisheyeCorrection.undistort import undistort, get_undistort_map

cam_l = cv.VideoCapture(0) # left
if not cam_l.isOpened():
    raise IOError("Cannot open webcam")
cam_l.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam_l.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam_l.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

cam_r = cv.VideoCapture(2) # right
if not cam_r.isOpened():
    raise IOError("Cannot open webcam")
cam_r.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam_r.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam_r.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

out_l = cv.VideoWriter('left.avi', cv.VideoWriter_fourcc('M','J','P','G'), 10, (320, 240))
out_r = cv.VideoWriter('right.avi', cv.VideoWriter_fourcc('M','J','P','G'), 10, (320, 240))

map1, map2 = get_undistort_map(0.8, 1.0)

def get_undist_roi(img, map1, map2):
    img = undistort(img, map1, map2)
    img = img[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]
    return img

while True:
    _, left_img = cam_l.read()
    _, right_img = cam_r.read()

    left_img = cv.cvtColor(left_img, cv.COLOR_RGB2GRAY)
    right_img = cv.cvtColor(right_img, cv.COLOR_RGB2GRAY)

    left_img = get_undist_roi(left_img, map1, map2)
    right_img = get_undist_roi(right_img, map1, map2)

    out_l.write(left_img)
    out_r.wrote(right_img)

    if cv2.waitKey(1) & 0xFF == ord('e'):
      break

out_l.release()
out_r.release()
cam_l.release()
cam_r.release()