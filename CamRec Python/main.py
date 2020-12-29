import tflite_runtime.interpreter as tflite
import cv2 as cv
import numpy as np
import time
import threading
from FisheyeCorrection.undistort import undistort, get_undistort_map
import serial

image_size = 100
min_s = 100
min_v = 50
min_color = 100

final_images = [np.zeros((1, image_size, image_size, 1))] * 2 # 2 images with image_size x image_size in grayscale
detected_colors = [None] * 2

threads = []
image_final_lock = threading.Lock()

cameras = []

map1, map2 = get_undistort_map(0.8, 1.0)

stop_threads = False

def start_threads():
    stop_threads = False
    
    threads.append(threading.Thread(target=process_img, args=(0,)))
    threads.append(threading.Thread(target=process_img, args=(1,)))
    
    for thread in threads:
        thread.start()

def stop_threads_block():
    global stop_threads
    stop_threads = True

    for thread in threads:
        thread.join()

    threads.clear()
    stop_threads = False

def get_undist_roi(img, map1, map2):
    img = undistort(img, map1, map2)
    img = img[round(120 * (1.0 - 2.0 / 5.0)) : round(120 * (1.0 + 2.0 / 5.0)), 160 : 320]
    return img

def process_img(indx):
    while not stop_threads:
        _, img = cameras[indx].read()

        if img is None:
            continue
        
        img = get_undist_roi(img, map1, map2)

        if not detect_color(img, indx):
            preprocess_img(img, indx)

def detect_color(img, indx):
    hsv = cv.cvtColor(cv.blur(img, (3, 3)), cv.COLOR_BGR2HSV)

    red_mask = cv.bitwise_or(cv.inRange(hsv, (0, min_s, min_v), (15, 255, 255)), cv.inRange(hsv, (180 - 15, min_s, min_v), (180, 255, 255)))
    green_mask = cv.inRange(hsv, (35, min_s, min_v), (60 + 15, 255, 255))
    yellow_mask = cv.inRange(hsv, (30 - 15, min_s, min_v), (35, 255, 255))

    redness = red_mask.mean()
    greeness = green_mask.mean()
    yellowness = yellow_mask.mean()

    color = None

    if redness > greeness and redness > yellowness:
        if redness > min_color:
            color = b'R'
    elif greeness > redness and greeness > yellowness:
        if greeness > min_color:
            color = b'G'
    else:
        if yellowness > min_color:
            color = b'Y'

    with image_final_lock:
        detected_colors[indx] = color
            
    if color == None:
        return False
    else:
        return True

def preprocess_img(img, indx):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img = cv.resize(img, (image_size, image_size))
    img = img / 255.0
    img = (img - img.mean()) / img.std()
    
    with image_final_lock:
        final_images[indx] = img = np.reshape(img, (1, image_size, image_size, 1))

def read_all_serial(port):
    if port.in_waiting <= 0:
        return [None]
    return port.read(port.in_waiting)

interpreter = tflite.Interpreter(model_path='converted_model.tflite')

input_index = interpreter.get_input_details()[0]["index"]
output_index = interpreter.get_output_details()[0]["index"]

interpreter.allocate_tensors()

letter_lookup = [b'H', b'S', b'U', b'N']

# user must be in group 'dialout'
port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=0)

while True:

    cam_l = cv.VideoCapture(0) # left
    if not cam_l.isOpened():
        raise IOError("Cannot open webcam")
    cam_l.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    cam_l.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cam_l.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    cam_r = cv.VideoCapture(2) # right
    if not cam_r.isOpened():
        raise IOError("Cannot open webcam")
    cam_r.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    cam_r.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cam_r.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

    cameras.clear()

    cameras.append(cam_l)
    cameras.append(cam_r)

    # Wait for 'B' to begin
    while b'B' not in read_all_serial(port):
        pass

    start_threads()
    stop = False
    
    while not stop:
        start_time = time.time()

        left_no_color = False
        right_no_color = False

        left_out = b'N'
        right_out = b'N'

        with image_final_lock:
            if detected_colors[0] == None:
                interpreter.set_tensor(input_index, final_images[0].astype(dtype='float32'))
                left_no_color = True
            else:
                left_out = detected_colors[0]

        if left_no_color:
            interpreter.invoke()
            output = interpreter.tensor(output_index)
            left_out = letter_lookup[np.argmax(output()[0])]

        with image_final_lock:
            if detected_colors[1] == None:
                interpreter.set_tensor(input_index, final_images[1].astype(dtype='float32'))
                right_no_color = True
            else:
                right_out = detected_colors[1]

        if right_no_color:
            interpreter.invoke()
            output = interpreter.tensor(output_index)
            right_out = letter_lookup[np.argmax(output()[0])]

        print("left: " + str(left_out) + ", right: " + str(right_out) + ", fps: " + str(round(1.0 / (time.time() - start_time), 2)))
        port.write(b'l' + left_out + b'r' + right_out + b'\n')

        if b'E' in read_all_serial(port):
            stop = True
        
    stop_threads_block()

    cam_l.release()
    cam_r.release()
