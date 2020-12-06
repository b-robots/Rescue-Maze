import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import numpy as np
import tensorflow as tf
from tensorflow import keras
from keras import models
from custom_layers import MobileNetV1Block, Conv2DBlock
import cv2 as cv

model = models.load_model("model.h5", custom_objects={'Conv2DBlock' : Conv2DBlock})

image_size = 100

for file in os.listdir(os.fsencode("C:/Users/patzi/Pictures/Letters_Brobots/test")):
    filename = os.fsdecode(file)
    if filename.endswith(".jpg"):
        img = cv.imread("C:/Users/patzi/Pictures/Letters_Brobots/test/" + filename, cv.IMREAD_GRAYSCALE)
        img = cv.resize(img, (image_size, image_size))
        img = img / 255.0
        img = (img - img.mean()) / img.std()
        img = np.reshape(img, (1, image_size, image_size, 1))
        print(filename + " - " + str(np.round(model.predict(img)[0], decimals=3)))