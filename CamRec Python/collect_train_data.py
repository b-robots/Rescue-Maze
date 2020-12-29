import cv2 as cv
import numpy as np
from FisheyeCorrection.undistort import undistort, get_undistort_map

cam_l = cv.VideoCapture(0) # left
if not cam_l.isOpened():
    raise IOError("Cannot open webcam")
cam_l.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam_l.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam_l.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

cam_r = cv.VideoCapture(1) # right
if not cam_r.isOpened():
    raise IOError("Cannot open webcam")
cam_r.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
cam_r.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam_r.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

out_l = cv.VideoWriter('left.avi', cv.VideoWriter_fourcc('M','J','P','G'), 10, (160, 96), False)
out_r = cv.VideoWriter('right.avi', cv.VideoWriter_fourcc('M','J','P','G'), 10, (160, 96), False)

map1, map2 = get_undistort_map(0.8, 1.0)

def get_undist_roi(img, map1, map2):
    img = undistort(img, map1, map2)
    img = img[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]
    return img

last_left = np.zeros((96, 160), dtype=np.uint8)
last_right = np.zeros((96, 160), dtype=np.uint8)

threshold = 0.02

while True:
    _, left_img = cam_l.read()
    _, right_img = cam_r.read()

    left_img = cv.cvtColor(left_img, cv.COLOR_RGB2GRAY)
    right_img = cv.cvtColor(right_img, cv.COLOR_RGB2GRAY)

    left_img = get_undist_roi(left_img, map1, map2)
    right_img = get_undist_roi(right_img, map1, map2)

    left_err = np.mean(np.absolute(left_img / 255.0 - last_left / 255.0))
    right_err = np.mean(np.absolute(right_img / 255.0 - last_right / 255.0))

    if left_err > threshold:
        out_l.write(left_img)
        last_left = left_img

    if right_err > threshold:
        out_r.write(right_img)
        last_right = right_img

    if cv.waitKey(1) & 0xFF == ord('e'):
      break

out_l.release()
out_r.release()
cam_l.release()
cam_r.release()
cv.destroyAllWindows()