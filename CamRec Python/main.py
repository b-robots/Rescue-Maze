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
    
def get_patch(img):
    #adaptiv thresholding 
    img = 255 - img
    blur = cv.blur(img,(7,7))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 71, 10)
    #dilataing small dots/dashes away
    kernel = np.ones((5,5),np.uint8)
    thresh_img = cv.dilate(thresh_img,kernel,iterations = 1)
    #get only contours which have no holes
    contours, hierarchy = cv.findContours(thresh_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][2] < 0]

    cv.imshow("thresh", thresh_img)
    #find the brightest contour which is bigger than 100px
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
    #transform the image into a 100x100px image
    left_top = np.amin(cnt, 0)
    right_bottom = np.amax(cnt, 0)
    scale = 100.0 / (right_bottom - left_top)
    cnt = (cnt * scale - left_top * scale).astype(np.int32)
    
    #put contour on 100x100px img
    patch = np.zeros((100, 100), np.uint8)
    cv.fillPoly(patch, [cnt], 255)

    return patch  

def detect_letter(patch):
    """
    Detects letters returns numbers from 0 to 3 
    where 0 = None, 1 = H, 2 = S, 3 = U
    """
    comp_H = cv.imread("HComp.jpg", cv.IMREAD_GRAYSCALE)
    comp_S = cv.imread("SComp.jpg", cv.IMREAD_GRAYSCALE)
    comp_U = cv.imread("UComp.jpg", cv.IMREAD_GRAYSCALE)

    comp_H = cv.blur(comp_H,(51,51))
    comp_S = cv.blur(comp_S,(51,51))
    comp_U = cv.blur(comp_U,(51,51))

    patch = cv.blur(patch,(51,51))

    cv.imshow("234", patch)
    cv.waitKey(0)

    prediction_H =  np.mean(((comp_H - patch)/255.0)**2)
    prediction_S =  np.mean(((comp_S - patch)/255.0)**2)
    prediction_U =  np.mean(((comp_U - patch)/255.0)**2)

    predictions = [prediction_H, prediction_S, prediction_U]
    print(predictions)
    if(prediction_H > 0.5 or prediction_S > 0.5 or prediction_U > 0.5):
        return np.argmax(predictions) + 1
    
    return 0

def get_undist_roi(img, map1, map2, is_right=False):
    img = undistort(img, map1, map2)
    if is_right:
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    else:
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
    inv_img =  cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #adaptiv thresholding 
    inv_img = 255 - inv_img
    blur = cv.blur(inv_img,(7,7))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 71, 10)
    #dilataing small dots/dashes away
    kernel = np.ones((5,5),np.uint8)
    thresh_img = cv.dilate(thresh_img,kernel,iterations = 1)
    #get only contours which have no holes
    contours, hierarchy = cv.findContours(thresh_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][2] < 0]

    cv.imshow("thresh", thresh_img)
    #find the brightest contour which is bigger than 100px
    brightest_cnt = None
    brightest = 0
    for contour in contours:
        mask = np.zeros_like(img)
        cv.fillPoly(mask, [contour], 255)
        b = np.sum(np.where(mask > 127,img,0)) / (np.sum(mask) + 1)
        if b > brightest and cv.contourArea(contour) > 1000:
            brightest = b
            brightest_cnt = contour
    
    cnt = np.squeeze(np.asarray(brightest_cnt))

    mask = np.zeros_like(img)
    cv.fillPoly(mask, [cnt], (255, 255, 255))
    bgr_avg = np.sum(np.where(mask > 127, img, 0), (0, 1)) / (np.sum(mask[..., 0]))
    print(bgr_avg)
    hsv_avg = np.squeeze(cv.cvtColor(np.asarray([[bgr_avg]], np.float32), cv.COLOR_BGR2HSV))
    print(hsv_avg)
    if(hsv_avg[1] > 0.5 and hsv_avg[2] > 0.5):
        hsv_avg[0] = hsv_avg[0] * np.pi / 180
        r_sim = abs(np.arctan2(np.sin(hsv_avg[0] - np.pi * 2.0), np.cos(hsv_avg[0] - np.pi * 2.0)))
        y_sim = abs(np.arctan2(np.sin(hsv_avg[0] - 0.873), np.cos(hsv_avg[0] - 0.873)))
        g_sim = abs(np.arctan2(np.sin(hsv_avg[0] - 1.3), np.cos(hsv_avg[0] - 1.3)))
        print(r_sim * 180/np.pi)
        print(y_sim * 180/np.pi)
        print(g_sim * 180/np.pi)
        return np.argmin([r_sim, y_sim, g_sim]) + 1
    
    return 0

def read_all_serial(port):
    if port.in_waiting <= 0:
        return [None]
    return port.read(port.in_waiting)

def main():
    
    port = serial.Serial('/dev/serial0', baudrate=9600, timeout=0)
    letterLookup = [b'N',  b'H', b'S', b'U']
    colorLookup = [b'N',  b'R', b'Y', b'G']

    map1, map2 = get_undistort_map(0.8, 1.0)
    
    lcamId, rcamId=get_cam_serial()
    caml = cv.VideoCapture(int(lcamId)) # left
    if not caml.isOpened():   
        raise IOError("Cannot open webcam")
    caml.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    caml.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    caml.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    camr = cv.VideoCapture(int(rcamId)) # right
    if not camr.isOpened():   
        raise IOError("Cannot open webcam")
    camr.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    camr.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    camr.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        try:
            if b'B' in read_all_serial(port):
                port.write(b'OK\n')
            #get picture
            _, imgl = caml.read()
            _, imgr = camr.read()
            #undistort and crop

            imgr = get_undist_roi(imgr, map1, map2, True)
            imgl = get_undist_roi(imgl, map1, map2, False)

            gray_imgl = 255 - cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            patch_imgl = get_patch(gray_imgl)

            gray_imgr = 255 - cv.cvtColor(imgr, cv.COLOR_BGR2GRAY)
            patch_imgr = get_patch(gray_imgr)

            #cv.imwrite("testL.jpg", bin_imgl)
            #cv.imwrite("testR.jpg", bin_imgr)

            rightOut = colorLookup[detect_Color(imgr)]
            leftOut = colorLookup[detect_Color(imgl)]
        
            if rightOut == colorLookup[0]:
                rightOut = letterLookup[detect_letter(imgr)]

            if leftOut == colorLookup[0]:
                leftOut = letterLookup[detect_letter(imgl)]

            port.write(b'l' + leftOut + b'r' + rightOut + b'\n')
            print("l: " + str(leftOut), "r: " + str(rightOut))
        except Exception as e:
            print(e)
            continue

if __name__ == "__main__":
    main()
    
    
    