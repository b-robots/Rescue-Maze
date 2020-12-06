import cv2
import numpy as np

#Fisheye correction: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

# You should replace these 3 lines with the output in calibration step
DIM=(640, 480)
K=np.array([[781.3524863867165, 0.0, 794.7118000552183], [0.0, 779.5071163774452, 561.3314451453386], [0.0, 0.0, 1.0]])
D=np.array([[-0.042595202508066574], [0.031307765215775184], [-0.04104704724832258], [0.015343014605793324]])

def get_undistort_map(balance=0.0):
    scaled_K = K  # The values of K is to scale with image dimension. If calibration images have different resolution than new images
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, DIM, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)
    return map1, map2

def undistort(img, map1, map2):
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img
