from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import re
import subprocess 
import numpy as np
import serial
import math 
import scipy.signal
import time
import random
import os
import scipy.stats as stats
import tflite_runtime.interpreter as tflite

def makePatch(img):
    x,y,w,h = cv.boundingRect(img)
    img = img[y:y+h, x:x+w]
    return cv.resize(img, (50, 50), interpolation=cv.INTER_NEAREST)

def get_patch2(img):
     #adaptiv thresholding 
    img = 255 - img
    blur = cv.blur(img,(13,13))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 21, 2)
    kernel = np.ones((5,5),np.uint8)
    thresh_img[0, :] = 0
    thresh_img[-1, :] = 0
    thresh_img[:, 0] = 0
    thresh_img[:, -1] = 0
    thresh_img = cv.erode(thresh_img,kernel,iterations=1)
    thresh_img[0, :] = 0
    thresh_img[-1, :] = 0
    thresh_img[:, 0] = 0
    thresh_img[:, -1] = 0

    #cv.imshow("thresh", thresh_img)
    #cv.waitKey(1)

    num, labels, stats, centroids = cv.connectedComponentsWithStats(thresh_img)

    best_mask = None
    best_mask_val = 0
    for i in range(1, num):
        area = stats[i, cv.CC_STAT_AREA]
        if area > 300:
            mask = (labels == i).astype(np.uint8) * 255
            invert = 255 - mask
            inv_num , inv_labels = cv.connectedComponents(invert)
            if inv_num == 2:
                b = np.sum(np.where(mask > 127, img, 0)) / (np.sum(mask) + 1)
                if b > best_mask_val:
                    best_mask_val = b
                    best_mask = mask

    if best_mask is None:
        return None

    return makePatch(best_mask)

def get_undist_roi(img, map1, map2, is_right=False):
    img = cv.resize(img, (160, 120))
    img = undistort(img, map1, map2)
    if is_right:
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    else:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

    img = cv.resize(img, (320, 240))

    #[y:y+h, x:x+w] the ":" means from to 
    img = img[40:, 40:200]

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
    lcamId = re.findall("\.usb-1\.3.*\n.*/dev/video[0-9]", response)[0][-1]
    rcamId = re.findall("\.usb-1\.4.*\n.*/dev/video[0-9]", response)[0][-1]
    # LeftId is left rightId is right 
    return lcamId, rcamId

def detect_Color_Mask(img):
    img = cv.resize(img, None, fx=0.2, fy=0.2)
    img = cv.cvtColor(np.float32(img / 255.0), cv.COLOR_BGR2HSV ) 
    r_mask = np.float32(np.where(((img[:,:,0] > 340) | (img[:,:,0] < 15)) & (img[:,:,1] > 0.5) & (img[:,:,2] > 0.2), 1, 0))
    y_mask = np.float32(np.where((img[:,:,0] > 40) & (img[:,:,0] < 80) & (img[:,:,1] > 0.4) & (img[:,:,2] > 0.4), 1, 0))
    g_mask = np.float32(np.where((img[:,:,0] > 90) & (img[:,:,0] < 150) & (img[:,:,1] > 0.5) & (img[:,:,2] > 0.15), 1, 0)) 

    area = np.shape(img)[0] * np.shape(img)[1]
    r_val = np.sum(r_mask) / area
    y_val = np.sum(y_mask) / area
    g_val = np.sum(g_mask) / area

    # print(r_val)
    #print(y_val)
    #print(g_val)

    vals = [r_val, y_val, g_val]
    res = np.argmax(vals)
    if vals[res] > 0.04:
        return res + 1
    
    return 0

def detect_Color(img):
    """
    Returrns the detected color in the format: r = 1, y = 2, g = 3, None = 0
    """
    inv_img =  cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #adaptiv thresholding 
    inv_img = 255 - inv_img
    blur = cv.blur(inv_img,(7,7))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 31, 2)
    #dilataing small dots/dashes away
    #kernel = np.ones((5,5),np.uint8)
    #thresh_img = cv.dilate(thresh_img,kernel,iterations = 1)
    #get only contours which have no holes
    contours, hierarchy = cv.findContours(thresh_img, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_L1)
    contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][2] < 0]

    #cv.imshow("thresh", thresh_img)
    #find the brightest contour which is bigger than 100px
    brightest_cnt = None
    brightest = 0
    for contour in contours:
        mask = np.zeros_like(img)
        cv.fillPoly(mask, [contour], 255)
        b = np.sum(np.where(mask > 127,img,0)) / (np.sum(mask) + 1)
        if b > brightest and cv.contourArea(contour) > 500:
            brightest = b
            brightest_cnt = contour
    
    if brightest_cnt is None:
        return None
    cnt = np.squeeze(np.asarray(brightest_cnt))

    mask = np.zeros_like(img)
    cv.fillPoly(mask, [cnt], (255, 255, 255))
    cv.imshow("He", mask)
    cv.waitKey(1)
    bgr_avg = np.sum(np.where(mask > 127, img, 0), (0, 1)) / (np.sum(mask[..., 0]))
    print(bgr_avg)
    hsv_avg = np.squeeze(cv.cvtColor(np.asarray([[bgr_avg]], np.float32), cv.COLOR_BGR2HSV))
    print(hsv_avg)
    if(hsv_avg[1] > 0.4 ):
        print("Test")
        hsv_avg[0] = hsv_avg[0] * np.pi / 180
        r_sim = abs(np.arctan2(np.sin(hsv_avg[0] - np.pi * 2.0), np.cos(hsv_avg[0] - np.pi * 2.0)))
        y_sim = abs(np.arctan2(np.sin(hsv_avg[0] - 0.873), np.cos(hsv_avg[0] - 0.873)))
        g_sim = abs(np.arctan2(np.sin(hsv_avg[0] - 1.3), np.cos(hsv_avg[0] - 1.3)))
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

    model = tflite.Interpreter(model_path='CamRec Python/converted_model.tflite')
    input_index = model.get_input_details()[0]["index"]
    output_index = model.get_output_details()[0]["index"]
    model.resize_tensor_input(input_index, [2, 50, 50, 1])
    model.allocate_tensors()

    while True:
        t = time.time()
        try:
            if b'B' in read_all_serial(port):
                port.write(b'OK\n')
                print("Arduino")
            #get picture
            _, imgl = caml.read()
            #cv.imwrite("testcol.jpg", imgl)
            _, imgr = camr.read()
#            print(f"fps: {1 / (time.time() - t)}")

            l_valid = True
            r_valid = True

            #print(f"fps: {1 / (time.time() - t)}")

            #gray_imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            
            #patch_imgl = get_patch2(gray_imgl)
            #patch_imgl = cv.imread("trainNew/H101.jpg", cv.IMREAD_GRAYSCALE)
            #patch_imgl = normalize_linethickness(patch_imgl)
            #features_imgl = get_features(patch_imgl)
            #cv.imshow("",imgl)
            #cv.waitKey(0)

            leftOut = colorLookup[detect_Color_Mask(imgl)] 
#print(f"fps: {1 / (time.time() - t)}")
#            continue
            if leftOut == b'N':
                rightOut = colorLookup[detect_Color_Mask(imgr)]
            else:
                rightOut = b'N'
            
            if leftOut != b'N':
                print('l: ' + str(leftOut))
                port.write(b'l' + leftOut + b'r' + b'N' + b'\n')
                print(f"fps: {1 / (time.time() - t)}")
                continue

            if rightOut != b'N':
                print('r: ' + str(rightOut))
                port.write(b'l' + b'N' + b'r' + rightOut + b'\n')
                print(f"fps: {1 / (time.time() - t)}")
                continue

            port.write(b'l' + b'N' + b'r' + b'N' + b'\n')
            continue

            imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            imgr = cv.cvtColor(imgr, cv.COLOR_BGR2GRAY)

            #undistort and crop
            imgr = get_undist_roi(imgr, map1, map2, True)
            imgl = get_undist_roi(imgl, map1, map2, False)
            cv.imshow("", imgl)
            cv.waitKey(1)
            imgl = get_patch2(imgl)
            
            imgr = get_patch2(imgr)
            
            if imgl is None:
                l_valid = False
            
            if imgr is None:
                r_valid = False
            
            both_images = np.zeros((2, 50, 50, 1), np.float32)
            
            if l_valid:
                #cv.imshow("l", imgl)
                #cv.waitKey(1)
                imgl = imgl / max(np.amax(imgl), 1.0)
                imgl = np.reshape(imgl, (50, 50, 1))
                both_images[0] = imgl
            
            if r_valid:
                #cv.imshow("r", imgr)
                #cv.waitKey(1)
                imgr = imgr / max(np.amax(imgr), 1.0)
                imgr = np.reshape(imgr, (50, 50, 1))
                both_images[1] = imgr
            
            if l_valid or r_valid:
                model.set_tensor(input_index, both_images.astype(dtype='float32'))
                model.invoke()
                output = model.tensor(output_index)
                
                leftOut = letterLookup[np.argmax(output()[0])]
                rightOut = letterLookup[np.argmax(output()[1])]
                
                print('l: ' + str(leftOut))
                print('r: ' + str(rightOut))
                port.write(b'l' + leftOut + b'r' + rightOut + b'\n')
                print(f"fps: {1 / (time.time() - t)}")
                continue
               
            port.write(b'l' + b'N' + b'r' + b'N' + b'\n')

                
                #print(letterLookup[round(np.argmax(output()[0]))])
                #print(letterLookup[round(np.argmax(output()[1]))])
            
            print(f"fps: {1 / (time.time() - t)}")

            #gray_imgr = cv.cvtColor(imgr, cv.COLOR_BGR2GRAY)
            #patch_imgr = get_patch(gray_imgr)
            #if patch_imgr is None:
               #r_valid = False

            #if l_valid:
                #print(np.squeeze(svm.predict(np.asarray([features_imgl]))[1]))
                #print(features_imgl)
                #cv.imshow("patch", patch_imgl)
                #cv.waitKey(1)
                
                #print(f"Res: {letterLookup[detect_letter_HUregions(patch_imgl, comps)]}", "fps: " + str(1 / (t - time.time())))
                
            
            #cv.imwrite("testL.jpg", bin_imgl)
            #cv.imwrite("testR.jpg", bin_imgr)

            # rightOut = colorLookup[detect_Color(imgr)]
            # leftOut = colorLookup[detect_Color(imgl)]
        
            #  if rightOut == colorLookup[0] and r_valid:
            #      rightOut = letterLookup[detect_letter(patch_imgr)]

            #  if leftOut == colorLookup[0] and l_valid:
            #      leftOut = letterLookup[detect_letter(patch_imgl)]
            #print("l: " + str(leftOut), "r: " + str(rightOut))
        except Exception as e:
            print(e)
            continue

def make_trainImages(map1, map2):
    from pynput import keyboard
    global is_y
    global is_n
    is_y = False
    is_n = False

    def on_press(key):
        global is_y
        global is_n
        try:
            print(key.char[0])
            if key.char[0] == 'y':
                is_y = True
            elif key.char[0] == 'n':
                is_n = True
        except AttributeError:
            print('special key {0} pressed'.format(key))

    lcamId, rcamId=get_cam_serial()
    caml = cv.VideoCapture(int(lcamId)) # left
    if not caml.isOpened():   
        raise IOError("Cannot open webcam")
    caml.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    caml.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    caml.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print("to continoue press y to go to the next press n")
    orders = ""
    ltr_cnt = 1
    letterLookup = ['N' ,'H', 'S', 'U']
    cnt = 200
    for l in letterLookup:
        print("we are at: " + l)
        is_n = False
        while not is_n:
            _, img = caml.read()
            img = get_undist_roi(img, map1, map2, False)
            valid = True
            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            patch = get_patch2(gray_img)
            if patch is None:
               valid = False

            if valid:
                cv.imshow("lul",patch)
                cv.waitKey(1)

                if is_y:
                    cv.imwrite("trainNew/" + l + str(cnt) + ".jpg", patch)
                    print(l + str(cnt))
                    cnt += 1
                    is_y = False
            if is_n:
                break
        else:
            cnt = 0
            continue

if __name__ == "__main__":
    main()
    #print(os.listdir('trainNew/'))
    #main()
    #map1, map2 = get_undistort_map(0.8, 1.0)
    #get_compImages(map1, map2)
   
