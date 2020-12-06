import tflite_runtime.interpreter as tflite
import cv2 as cv
import numpy as np
import time
import threading
from FisheyeCorrection.undistort import undistort, get_undistort_map
import serial

image_size = 100
raw_images = [None] * 2
final_images = np.zeros((2, image_size, image_size, 1)) # 2 images with image_size x image_size in grayscale

threads = []
image_raw_locks = [threading.Lock(), threading.Lock()]
image_final_lock = threading.Lock()

cameras = []

map1, map2 = get_undistort_map()

stop_threads = False

def start_threads():
    threads.append(threading.Thread(target=read_cam, args=(0,)))
    threads.append(threading.Thread(target=preprocess_img, args=(0,)))
    #threads.append(threading.Thread(target=read_cam, args=(1,)))
    threads.append(threading.Thread(target=preprocess_img, args=(1,)))
    
    for thread in threads:
        thread.start()

def stop_threads_block():
    global stop_threads
    stop_threads  = True
    for thread in threads:
        thread.join()
    threads.clear()

def read_cam(indx):
    while not stop_threads:
        _, img = cameras[indx].read()
        with image_raw_locks[indx]:
            raw_images[indx] = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            raw_images[indx+1] = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

def preprocess_img(indx):
    while not stop_threads: 
        with image_raw_locks[indx]:
            img = raw_images[indx]
        
        if img is None:
            continue
        
        img = undistort(img, map1, map2)
        height, width = img.shape
        new_width = height * 3.0 / 4.0
        img = img[0:height, int(width // 2 - new_width // 2):int(width // 2 + new_width // 2)]
        img = cv.resize(img, (image_size, image_size))
        img = img / 255.0
        img = np.reshape(img, (image_size, image_size, 1))
        img = (img - img.mean()) / img.std()
        
        with image_final_lock:
            final_images[indx] = img

def read_all_serial(port):
    if port.in_waiting <= 0:
        return [None]
    return port.read(port.in_waiting)

cam1 = cv.VideoCapture(0)
if not cam1.isOpened():
    raise IOError("Cannot open webcam")

#cam2 = cv.VideoCapture(1)
#if not cam2.isOpened():
#    raise IOError("Cannot open webcam")

cameras.append(cam1)
#cameras.append(cam2)

interpreter = tflite.Interpreter(model_path='converted_model.tflite')

input_index = interpreter.get_input_details()[0]["index"]
output_index = interpreter.get_output_details()[0]["index"]

interpreter.resize_tensor_input(input_index, [2, image_size, image_size, 1])
interpreter.resize_tensor_input(output_index, [2, 4])

interpreter.allocate_tensors()

letter_lookup = [b'H', b'S', b'U', b'N']

# user must be in group 'dialout'
port = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=0) # /dev/ttyS4 on NanoPi

while True:

    # Wait for 'B' to begin
    while b'B' not in read_all_serial(port):
        pass

    start_threads()
    stop = False
    
    while not stop:
        start_time = time.time()

        with image_final_lock:
            interpreter.set_tensor(input_index, final_images.astype(dtype='float32'))

        interpreter.invoke()
        output = interpreter.tensor(output_index)

        print("left: " + letter_lookup[np.argmax(output()[0])] + ", right: " + letter_lookup[np.argmax(output()[1])] + ", fps: " + str(round(1.0 / (time.time() - start_time), 2)))
        
        port.write(b'l' + letter_lookup[np.argmax(output()[0])] + b'r' + letter_lookup[np.argmax(output()[1])])

        if b'E' in read_all_serial(port):
            stop = True
        
    stop_threads_block()

    cam1.release()
    #cam2.release()