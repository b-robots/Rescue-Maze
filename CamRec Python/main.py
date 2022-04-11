from ast import Pass
from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import pytesseract
import re
import subprocess 

def get_undist_roi(img, map1, map2, is_right=False):
    img = undistort(img, map1, map2)
    if is_right:
        pass
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    else:
        pass
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
    #img = img[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]

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
    lcamId = re.findall("\.usb-1\.1.*\n.*/dev/video[0-9]", response)[0][-1]
    rcamId = re.findall("\.usb-1\.3.*\n.*/dev/video[0-9]", response)[0][-1]
    # Left is left right is right 
    return (lcamId,rcamId)

def main():
    get_cam_serial()
    pytesseract.pytesseract.tesseract_cmd = r"/usr/bin/tesseract"
    cam = cv.VideoCapture(0) # left
    if not cam.isOpened():
        raise IOError("Cannot open webcam")
    cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    cam.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    _, img = cam.read()
    map1, map2 = get_undistort_map(0.8, 1.0)
    img = get_undist_roi(img, map1, map2, True)
    cv.imwrite("test.jpg", img)
    data = pytesseract.image_to_string(img, lang='eng',config='--psm 10 -c tessedit_char_whitelist=HSUsu')
    print(data)

if __name__ == "__main__":
    main()
