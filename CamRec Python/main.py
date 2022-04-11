from ast import Pass
from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import pytesseract
import re
import subprocess 
import numpy as np
import serial

def detect_letter(img):
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
    img = img[20:300, 45:195]

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
    avgSat = np.mean(sat)
    if(avgSat > 100):
        hues = pixels[sat > 100][:, 0]
        hues *= 2
        rMask = np.logical_or(hues >= 210, hues < 30)
        yMask = np.logical_and(hues >= 30, hues < 70)
        gMask = np.logical_and(hues >= 70, hues < 140)

        print(np.count_nonzero(rMask))
        print(np.count_nonzero(yMask))
        print(np.count_nonzero(gMask))
        print("----------")

        return np.argmax([np.count_nonzero(rMask), np.count_nonzero(yMask), np.count_nonzero(gMask)]) + 1
    
    return 0

def read_all_serial(port):
    if port.in_waiting <= 0:
        return [None]
    return port.read(port.in_waiting)
        

def main():
    port = serial.Serial('/dev/serial0', baudrate=9600, timeout=0)
    letterLookup = [b'N',  b'H', b'S', b'U']
    colorLookup = [b'N',  b'H', b'S', b'U']

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

    # Wait for 'B' to begin
    while b'B' not in read_all_serial(port):
        pass

    port.write(b'OK\n')

    stop = False
    while not stop:
        #get picture
        _, imgl = caml.read()
        _, imgr = camr.read()
        #undistort and crop
        imgr = get_undist_roi(imgr, map1, map2, True)
        imgl = get_undist_roi(imgl, map1, map2, True)

        rightOut = colorLookup[detect_Color(imgr)]
        leftOut = colorLookup[detect_Color(imgl)]

        if rightOut == colorLookup[0]:
            rightOut = letterLookup[detect_letter(imgr)]

        if leftOut == colorLookup[0]:
            leftOut = letterLookup[detect_letter(imgl)]

        port.write(b'l' + leftOut + b'r' + rightOut + b'\n')

        if b'E' in read_all_serial(port):
            stop = True

        
    #cv.imwrite("testL.jpg", imgl)
    cv.imwrite("testR.jpg", imgr)

if __name__ == "__main__":
    main()
