from ast import Pass
from matplotlib.pyplot import gray
from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import re
import subprocess 
import numpy as np
#import serial
import math 
from sklearn import svm

def get_trainImages(path, letter,):
    img = cv.imread(path + letter + ".jpg")
    img = cv.resize(img, (320,240))
    img = 255 - img
    #get roi
    map1, map2 = get_undistort_map(0.8, 1.0)
    img = undistort(img, map1, map2)
    gray_img = 255 - cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #cv.imshow("test1", gray_img)
    #[y:y+h, x:x+w] the ":" means from to 
    #img = img[100:300, 50:190]
    
    return gray_img

def augment():
    pass

def get_features(img):
    img = 255 - img
    blur = cv.GaussianBlur(img,(5,5),0)
    #ret3,oth_image = cv.threshold(blur,0,255,cv.THRESH_BINARY + cv.THRESH_OTSU)
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 21, 4)
    contours, hierarchy = cv.findContours(thresh_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][2] < 0]

    cv.imshow("thresh", thresh_img)

    brightest_cnt = None
    brightest = 0
    for contour in contours:
        mask = np.zeros_like(img)
        cv.fillPoly(mask, [contour], 255)
        b = np.sum(np.where(mask > 127,img,0)) / (np.sum(mask) + 1)
        if b > brightest and cv.contourArea(contour) > 100:
            brightest = b
            brightest_cnt = contour
    cnt = np.squeeze(np.asarray(brightest_cnt))
    left_top = np.amin(cnt, 0)
    right_bottom = np.amax(cnt, 0)
    scale = 100.0 / (right_bottom - left_top)
    cnt = (cnt * scale - left_top * scale).astype(np.int32)
    
    test = np.zeros((100, 100), np.uint8)
    cv.fillPoly(test, [cnt], 255)
    cv.imshow("test", test)
    
    cv.waitKey(0)

    

    # # Calculate Moments
    # moments = cv.moments(brightest_cnt)
	# # Calculate Hu Moments
    # huMoments = cv.HuMoments(moments)
    # for i in range(0,7):
    #     huMoments[i] = -1* math.copysign(1.0, huMoments[i]) * math.log10(abs(huMoments[i]) + 1e-100)
    
    # huMoments = np.squeeze(np.asarray(huMoments))
    # huMoments[-1] = np.abs(huMoments[-1])

    return 


def detect_letter(features):
    pass

def get_undist_roi(img, map1, map2, is_right=False):
    img = undistort(img, map1, map2)
    if is_right:
        pass
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    else:
        pass
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

    #[y:y+h, x:x+w] the ":" means from to 
    img = img[100:300, 50:190]

    return img

def get_cam_serial():
    # Prepare the external command to extract serial number. 
    p = subprocess.Popen('v4l2-ctl --list-devices',stdout=subprocess.PIPE, shell=True)
    # Run the command
    (output, err) = p.communicate()
    # Wait for it to finish
    p.status = p.wait()
    # Decode the output
    response = output.decode('utf-8')

    # The regex part 1\.1 needs to be configured for the final usb splitter and the splitter needs to be labeled
    lcamId = re.findall("\.usb-1\.4.*\n.*/dev/video[0-9]", response)[0][-1]
    rcamId = re.findall("\.usb-1\.3.*\n.*/dev/video[0-9]", response)[0][-1]
    # LeftId is left rightId is right 
    return lcamId, rcamId

def detect_Color(img):
    """
    Returrns the detected color in the format: r = 1, y = 2, g = 3, None = 0
    """
    pixels = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    sat = pixels[:, :, 1]
    mask = np.where(np.logical_and(sat > 100, pixels[:, :, 2] > 30), 255, 0)
    if(np.count_nonzero(mask > 80) / 200.0 / 140.0 > 0.06):
        hues = np.float32(pixels[np.where(mask > 127)][:, 0])
        hues *= 2.0
        hue_mean = np.arctan2(np.mean(np.sin(hues / 180 * np.pi)), np.mean(np.cos(hues / 180 * np.pi)))
        r_sim = abs(np.arctan2(np.sin(hue_mean - np.pi * 2.0), np.cos(hue_mean - np.pi * 2.0)))
        y_sim = abs(np.arctan2(np.sin(hue_mean - 0.873), np.cos(hue_mean - 0.873)))
        g_sim = abs(np.arctan2(np.sin(hue_mean - 1.3), np.cos(hue_mean - 1.3))) 
        return np.argmin([r_sim, y_sim, g_sim]) + 1
    
    return 0

def read_all_serial(port):
    if port.in_waiting <= 0:
        return [None]
    return port.read(port.in_waiting)

def main():
    U_train = []
    for i in range(0,7):
        img = get_trainImages("train\\","U"+ str(i))
        U_train.append(get_features(img))
    
    U_test = []
    for i in range(3):
        i = i + 7
        img = get_trainImages("train\\","U"+ str(i))
        U_test.append(get_features(img))

    S_train = []
    for i in range(0,1):
        img = get_trainImages("train\\","S"+ str(i))
        U_test.append(get_features(img))

    # H_train = []
    # for i in range(0,10):
    #     img = get_trainImages("train\\","S"+ str(i))
    #     H_train.append(get_features(img))
    
    svm_U = svm.OneClassSVM(kernel='rbf', gamma='auto', nu=0.7)
    svm_U.fit(U_train)
    print(U_train)
    print(svm_U.predict(U_test))
    print(svm_U.decision_function(U_test))
    print(".......")
    print(svm_U.predict(U_train))
    print(svm_U.decision_function(U_train))

    # svm_S = svm.OneClassSVM(kernel='rbf', gamma='auto', nu=1)
    # svm_S.fit(S_train)
    #print(svm_S.predict(U_train))

    # svm_H = svm.OneClassSVM(kernel='rbf', gamma='auto', nu=1)
    # svm_H.fit(S_train)
    # print(svm_S.predict(H_train))
#     port = serial.Serial('/dev/serial0', baudrate=9600, timeout=0)
#     letterLookup = [b'N',  b'H', b'S', b'U']
#     colorLookup = [b'N',  b'R', b'Y', b'G']

#     map1, map2 = get_undistort_map(0.8, 1.0)
    
#     lcamId, rcamId=get_cam_serial()
#     caml = cv.VideoCapture(int(lcamId)) # left
#     if not caml.isOpened():   
#         raise IOError("Cannot open webcam")
#     caml.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
#     caml.set(cv.CAP_PROP_FRAME_WIDTH, 320)
#     caml.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

#     camr = cv.VideoCapture(int(rcamId)) # right
#     if not camr.isOpened():   
#         raise IOError("Cannot open webcam")
#     camr.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
#     camr.set(cv.CAP_PROP_FRAME_WIDTH, 320)
#     camr.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

#     while True:
#         try:
#             if b'B' in read_all_serial(port):
#                 port.write(b'OK\n')
#             #get picture
#             _, imgl = caml.read()
#             _, imgr = camr.read()
#             #undistort and crop

#             imgr = get_undist_roi(imgr, map1, map2, True)
#             imgl = get_undist_roi(imgl, map1, map2, False)

#             bin_imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
#             bin_imgl = cv.adaptiveThreshold(bin_imgl, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 21, 11)

#             bin_imgr = cv.cvtColor(imgr, cv.COLOR_BGR2GRAY)
#             bin_imgr = cv.adaptiveThreshold(bin_imgr, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 21, 11)

#             get_HU(bin_imgr)

#             #cv.imwrite("testL.jpg", bin_imgl)
#             #cv.imwrite("testR.jpg", bin_imgr)

#             rightOut = colorLookup[detect_Color(imgr)]
#             leftOut = colorLookup[detect_Color(imgl)]
        
#         #if rightOut == colorLookup[0]:
#         #    rightOut = letterLookup[detect_letter(imgr)]

#         #if leftOut == colorLookup[0]:
#         #    leftOut = letterLookup[detect_letter(imgl)]
#             port.write(b'l' + leftOut + b'r' + rightOut + b'\n')
#             print("l: " + str(leftOut), "r: " + str(rightOut))
#         except Exception as e:
#             print(e)
#             continue

        
#     #cv.imwrite("testL.jpg", imgl)
#     #cv.imwrite("testR.jpg", imgr)

if __name__ == "__main__":
    #main()
    img = cv.imread("train\\U0.jpg")
    img2= cv.imread("train\\testU0.jpg")
    img2 = cv.rotate(img2, cv.ROTATE_90_COUNTERCLOCKWISE)

    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    cv.imshow("1", img)
    cv.imshow("2", img2)

    print(get_features(img))
    print(get_features(img2))