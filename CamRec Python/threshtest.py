from ast import Pass
from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import pytesseract
import re
import subprocess 
import numpy as np
import serial

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
     

def main():

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
            #get picture
            _, imgl = caml.read()
            _, imgr = camr.read()
            #undistort and crop

            imgr = get_undist_roi(imgr, map1, map2, True)
            imgl = get_undist_roi(imgl, map1, map2, False)
            
            imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            imgl = cv.adaptiveThreshold(imgl, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 21, 11)
            #cv.imshow("",imgl)
            cv.waitKey(1)
        except Exception as e:
            print(e)
            continue

        
        cv.imwrite("testL.jpg", imgl)
        cv.imwrite("testR.jpg", imgr)

if __name__ == "__main__":
    main()

