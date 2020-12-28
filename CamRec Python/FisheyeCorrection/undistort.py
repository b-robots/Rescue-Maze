import cv2
import numpy as np

#Fisheye correction: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

DIM=(640, 480)
K=np.array([[356.8072727507862, 0.0, 322.33462259356236], [0.0, 357.06467757792154, 225.33318566698233], [0.0, 0.0, 1.0]])
D=np.array([[-0.015950423247026845], [0.004276614047737227], [-0.029983524993147954], [0.019054965886334024]])

def get_undistort_map(balance=0.0, scale_factor=1.0):
    scaled_K = K  # The values of K is to scale with image dimension. If calibration images have different resolution than new images
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, (round(DIM[0] * scale_factor), round(DIM[1] * scale_factor)), np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)
    return map1, map2

def undistort(img, map1, map2):
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img
