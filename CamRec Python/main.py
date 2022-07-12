from FisheyeCorrection.undistort import undistort, get_undistort_map
import cv2 as cv
import re
import subprocess 
import numpy as np
#import serial
import math 
from pynput import keyboard
import scipy.signal
import time
import random
import os
import scipy.stats as stats
import tflite_runtime.interpreter as tflite

def initialize_SVM():
    svm = cv.ml.SVM_create()
    svm.setKernel(cv.ml.SVM_RBF)
    svm.setType(cv.ml.SVM_C_SVC)
    svm.setC(2.67)
    svm.setGamma(5.383)
    return svm

def hog(img , bin_n = 8):
    img = np.asarray(img)
    gx = cv.Sobel(img, cv.CV_32F, 1, 0)
    gy = cv.Sobel(img, cv.CV_32F, 0, 1)
    mag, ang = cv.cartToPolar(gx, gy)
    bins = np.int32(bin_n*ang/(2*np.pi))    # quantizing binvalues in (0...16)
    M = img.shape[0] // 3
    N = img.shape[1] // 3    
    bin_cells = []
    for x in range(3):
        for y in range(3):
           bin_cells.append(bins[x*M:x*M+M,y*N:y*N+N]) 
    
    mag_cells = []
    for x in range(3):
        for y in range(3):
           mag_cells.append(mag[x*M:x*M+M,y*N:y*N+N]) 
    hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
    hist = np.hstack(hists)     # hist is a 64 bit vector

    return hist

def normalize_linethickness(img, target=15):
    try:
        img[0, :] = 0
        img[99, :] = 0
        img[:, 0] = 0
        img[:, 99] = 0
        dist = cv.distanceTransform(img, cv.DIST_L2, 3)
        maxima = scipy.signal.argrelextrema(dist, np.greater, order=2)
        med_thick = np.median(dist[maxima]) * 2.0
        if  med_thick is None:
            return None
        kernel_size = round(med_thick - target) // 2 * 2 + 1
        kernel_size = max(1, kernel_size)
        kernel = np.ones((kernel_size, kernel_size),np.uint8)
        img = cv.erode(img, kernel, iterations = 1)
        return img
    
    except Exception as e:
         print(e)
         return None

def get_trainImage(path, letter, num):
    img = cv.imread(path + letter + str(num) + ".jpg", cv.IMREAD_GRAYSCALE)
    return img        

def get_features(img):
    if img is None:
        return None
    img = cv.blur(img,(30,30))
    hogdata = np.float32(np.squeeze(hog(img)))
    
    return hogdata

def makePatch(img):
    x,y,w,h = cv.boundingRect(img)
    img = img[y:y+h, x:x+w]
    return cv.resize(img, (50, 50), interpolation=cv.INTER_NEAREST)

def augment(img):
    if random.random() < 0.5:
        size = random.randint(3, 9)
        img = cv.erode(img, np.ones((size,size)))
    else:
        size = random.randint(3, 9)
        bigger = np.zeros((120, 120), np.uint8)
        bigger[10:110, 10:110] = img
        img = cv.dilate(bigger, np.ones((size,size)))
        img = makePatch(img)

    if random.random() < 0.0:
        # perspective
        originPts = np.float32([[0, 0], [0, 100], [100, 0], [100, 100]])
        destPts = (np.random.normal(0.0, 10.0, (4,2)) + originPts).astype(np.float32)
        destPts = destPts - np.amin(destPts, 0)
        destPts = destPts / np.amax(destPts, 0) * 100
        m = cv.getPerspectiveTransform(originPts, destPts)
        img = cv.warpPerspective(img, m, (100, 100), flags=cv.INTER_NEAREST)
    else:
        # affine
        sx = random.random() * 0.4 - 0.2
        sy = random.random() * 0.4 - 0.2
        mu = 0
        std = 5
        a = -10
        b = 10
        angle = stats.truncnorm.rvs((a - mu) / std, (b - mu) / std, loc=mu, scale=std)
        rot = cv.getRotationMatrix2D((50, 50), angle, 1)
        shear = np.float32([
            [1, sx, 0],
            [sy, 1, 0],
            [0, 0, 1]])
        m = np.identity(3, np.float32)
        m[:2, :] = rot
        m = np.matmul(m, shear)
        bigger = np.zeros((200, 200), np.uint8)
        bigger[50:150, 50:150] = img
        img = cv.warpAffine(bigger, m[:2, :], (200, 200), flags=cv.INTER_NEAREST)
        img = makePatch(img)

    return img

def get_patch(img):
    #adaptiv thresholding 
    img = 255 - img
    blur = cv.blur(img,(11,11))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 31, 2)
    #dilataing small dots/dashes away
    kernel = np.ones((5,5),np.uint8)
    thresh_img = cv.dilate(thresh_img,kernel,iterations = 1)
    #get only contours which have no holes
    #cv.imshow("thresh", thresh_img)
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
        if b > brightest and cv.contourArea(contour) > 100:
            brightest = b
            brightest_cnt = contour
    
    if brightest_cnt is None:
        return None

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

def get_patch2(img):
     #adaptiv thresholding 
    img = 255 - img
    blur = cv.blur(img,(11,11))
    thresh_img = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 31, 2)
    #dilataing small dots/dashes away
    kernel = np.ones((5,5),np.uint8)
    thresh_img = cv.erode(thresh_img,kernel,iterations = 1)

    num, labels, stats, centroids = cv.connectedComponentsWithStats(thresh_img)

    best_mask = None
    best_mask_val = 0
    for i in range(1, num):
        area = stats[i, cv.CC_STAT_AREA]
        if area > 1000:
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

def train_SVM():
    trainingData = []
    labels = []

    names = os.listdir("trainNew/")

    Ns = [name for name in names if name[0] == "N" ]
    Hs = [name for name in names if name[0] == "H" ]
    Ss = [name for name in names if name[0] == "S" ]
    Us = [name for name in names if name[0] == "U" ]

    print(len(Ns))
    print(len(Hs))
    print(len(Ss))
    print(len(Us))

    #get None data
    for n in Ns:
        img = []
        img.append(cv.imread("trainNew/"+ n, cv.IMREAD_GRAYSCALE))
        print(n)
        labels.append(0)
        for _ in range(20):
            try:
                img.append(augment(img[0]))
                labels.append(0)
            except:
                print("poppy picture")
        features = [get_features(im) for im in img]
        trainingData.extend(features)
 
    #get H data
    for h in Hs:
        img = []
        img.append(cv.imread("trainNew/"+ h, cv.IMREAD_GRAYSCALE))
        labels.append(1)
        for _ in range(20):
            try:
                img.append(augment(img[0]))
                labels.append(1)
            except:
                print("poppy picture")
        features = [get_features(im) for im in img]
        trainingData.extend(features)

    #get S data
    for s in Ss:
        img = []
        img.append(cv.imread("trainNew/"+ s, cv.IMREAD_GRAYSCALE))
        labels.append(2)
        for _ in range(20):
            try:
                img.append(augment(img[2]))
                labels.append(2)
            except:
                print("poppy picture")
        features = [get_features(im) for im in img]
        trainingData.extend(features)

    #get U data
    for u in Us:
        img = []
        img.append(cv.imread("trainNew/"+ u, cv.IMREAD_GRAYSCALE))
        labels.append(3)
        for _ in range(20):
            try:
                img.append(augment(img[0]))
                labels.append(3)
            except:
                print("poppy picture")
        features = [get_features(im) for im in img]
        trainingData.extend(features)

    print(len(trainingData))
    print(len(labels))

    trainingData = np.asarray(trainingData, dtype=np.float32)
    labels = np.asarray(labels)

    svm = initialize_SVM()
    svm.train(trainingData, cv.ml.ROW_SAMPLE, labels)
    return svm

def detect_letter_overlay(patch):
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

    # cv.imshow("234", patch)
    # cv.waitKey(0)

    prediction_H =  np.mean(((comp_H - patch)/255.0)**2)
    prediction_S =  np.mean(((comp_S - patch)/255.0)**2)
    prediction_U =  np.mean(((comp_U - patch)/255.0)**2)

    predictions = [prediction_H, prediction_S, prediction_U]
    print(predictions, end='')
    if(prediction_H > 0.5 or prediction_S > 0.5 or prediction_U > 0.5):
        return np.argmax(predictions) + 1
    
    return 0

def get_HUdist(patch, comp):

    patch1 = patch[:, :50]
    patch2 = patch[:, 50:]
    patch3 = patch
    comp1 = comp[:, :50]
    comp2 = comp[:, 50:]
    comp3 = comp

    res1 = abs(cv.matchShapes(patch1,comp1,cv.CONTOURS_MATCH_I1,0))
    res2 = abs(cv.matchShapes(patch2,comp2,cv.CONTOURS_MATCH_I1,0))
    res3 = abs(cv.matchShapes(patch3,comp3,cv.CONTOURS_MATCH_I1,0))

    a1 = 1 / (40*res1+1)
    a2 = 1 / (40*res2+1)
    a3 = 1 / (40*res3+1)

    a = (a1 * a2 * a3)**(1.0/3) 
    try:
        b = 1 / a
    except:
        return 1
    return (1 - a) / (40*a)

def detect_letter_HUregions(patch, comps):
    res = []
    for comp in comps:
        res.append(get_HUdist(patch, comp))

    res = np.asarray(res)
    lowest = np.argmin(res)
    if res[lowest] < 0.002:
        print(res, end='')
        return lowest + 1

    return 0

def get_undist_roi(img, map1, map2, is_right=False):
    img = undistort(img, map1, map2)
    if is_right:
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    else:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

    #[y:y+h, x:x+w] the ":" means from to 
    img = img[:, 50:190]

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

def detect_Color_pixel(img):
    img = img[90:230, 120:121]
    img = cv.resize(img, (100,100))
    print("ye")
    cv.imshow("hgf",img)
    cv.waitKey(1) 

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

def get_compImages(map1, map2):
    global is_y
    global is_s
    is_y = False
    is_s = False
    def on_press(key):
        global is_y
        global is_s
        try:
            print(key.char[0])
            if key.char[0] == 'y':
                is_y = True
            elif key.char[0] == 's':
                print("weeeeeee")
                is_s = True
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
    
    print("to keep picture press y to skip s")
    orders = ""
    ltr_cnt = 1
    letterLookup = ['H', 'S', 'U']
    for l in letterLookup:
        print("is this an " + l)
        is_s = False
        while not is_s:
            _, img = caml.read()
            img = get_undist_roi(img, map1, map2, False)
            valid = True
            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            patch = get_patch(gray_img)
            patch = normalize_linethickness(patch)
            if patch is None:
               valid = False
            
            if valid:
                cv.imshow("lul",patch)
                cv.waitKey(1)

                if is_y:
                    cv.imwrite(l + "Comp.jpg", patch)
                    is_y = False
                    break  
        else:
            continue           

def make_trainImages(map1, map2):
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
    cnt = 0
    for l in letterLookup:
        print("we are at: " + l)
        is_n = False
        while not is_n:
            _, img = caml.read()
            img = get_undist_roi(img, map1, map2, False)
            valid = True
            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            patch = get_patch2(gray_img)
            patch = normalize_linethickness(patch)
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
    
def main():
    
    #port = serial.Serial('/dev/serial0', baudrate=9600, timeout=0)
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
        #try:
            # if b'B' in read_all_serial(port):
            #     port.write(b'OK\n')
            #get picture
            _, imgl = caml.read()
            _, imgr = camr.read()

            l_valid = True
            r_valid = True

            #undistort and crop
            imgr = get_undist_roi(imgr, map1, map2, True)
            imgl = get_undist_roi(imgl, map1, map2, False)

            #gray_imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            
            #patch_imgl = get_patch2(gray_imgl)
            #patch_imgl = cv.imread("trainNew/H101.jpg", cv.IMREAD_GRAYSCALE)
            #patch_imgl = normalize_linethickness(patch_imgl)
            #features_imgl = get_features(patch_imgl)
            
            # features_imgl = detect_Color_pixel(imgl)

            imgl = cv.cvtColor(imgl, cv.COLOR_BGR2GRAY)
            imgl = get_patch2(imgl)

            imgr = cv.cvtColor(imgr, cv.COLOR_BGR2GRAY)
            imgr = get_patch2(imgr)

            if imgl is None:
                l_valid = False
            
            if imgr is None:
                r_valid = False
            
            both_images = np.zeros((2, 50, 50, 1), np.float32)
            
            if l_valid:
                cv.imshow("l", imgl)
                cv.waitKey(1)
                imgl = imgl / max(np.amax(imgl), 1.0)
                imgl = np.reshape(imgl, (50, 50, 1))
                both_images[0] = imgl
            
            if r_valid:
                cv.imshow("r", imgr)
                cv.waitKey(1)
                imgr = imgr / max(np.amax(imgr), 1.0)
                imgr = np.reshape(imgr, (50, 50, 1))
                both_images[1] = imgr
            
            if l_valid or r_valid:
                model.set_tensor(input_index, both_images.astype(dtype='float32'))
                model.invoke()
                output = model.tensor(output_index)
                
                print(letterLookup[round(np.argmax(output()[0]))])
                print(letterLookup[round(np.argmax(output()[1]))])
            
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

            #port.write(b'l' + leftOut + b'r' + rightOut + b'\n')
            #print("l: " + str(leftOut), "r: " + str(rightOut))
        #except Exception as e:
            #print(e)
            #continue

if __name__ == "__main__":
    main()
    #print(os.listdir('trainNew/'))
    #main()
    #map1, map2 = get_undistort_map(0.8, 1.0)
    #get_compImages(map1, map2)
    # img = get_trainImage("S", "Comp")
    # img = get_features(img)
    # cv.imshow("tets", img)
    # cv.waitKey(0)
    #make_trainImages(map1, map2)