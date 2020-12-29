import cv2
import numpy as np

#Fisheye correction: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

DIM=(320, 240)
K=np.array([[135.1327507971643, 0.0, 161.6007937651761], [0.0, 179.95275429557344, 111.86878581055734], [0.0, 0.0, 1.0]])
D=np.array([[-0.016416623271148], [-0.02855488457571125], [0.034247182505227264], [-0.015755762345239916]])

#DIM=(640, 480)
#K=np.array([[356.8072727507862, 0.0, 322.33462259356236], [0.0, 357.06467757792154, 225.33318566698233], [0.0, 0.0, 1.0]])
#D=np.array([[-0.015950423247026845], [0.004276614047737227], [-0.029983524993147954], [0.019054965886334024]])

def get_undistort_map(balance=0.0, scale_factor=1.0, new_dim=(320, 240)):
    assert new_dim[0]/new_dim[1] == new_dim[0]/new_dim[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    
    dim2 = (round(new_dim[0] * scale_factor), round(new_dim[1] * scale_factor))
    dim3 = new_dim
    
    scaled_K = K * new_dim[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    return map1, map2

def undistort(img, map1, map2):
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img
