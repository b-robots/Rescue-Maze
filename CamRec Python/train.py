import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import numpy as np
import tensorflow as tf
import tensorflow_addons as tfa
from tensorflow import keras
from keras import layers, models, losses, regularizers, preprocessing
import pathlib
import time
import cv2 as cv
from custom_layers import MobileNetV1Block, Conv2DBlock
from keras.utils import vis_utils
import math
import random

file_path = 'train/'
batch_size = 64
num_samples = 1114
num_train = 500
num_validation = num_samples - num_train
epochs = 150
steps_per_epoch = num_train // batch_size
image_size = 100
checkpoint_path = "cp_weights.ckpt"
dropout_rate = 0.5
start_lr = 1e-3
lr_decay = 0.9
lr_decay_step = 50

def evaluate_lite_model(interpreter):
    input_index = interpreter.get_input_details()[0]["index"]
    output_index = interpreter.get_output_details()[0]["index"]

    for file in os.listdir(os.fsencode("C:/Users/patzi/Pictures/Letters_Brobots/test")):
        filename = os.fsdecode(file)
        if filename.endswith(".jpg"):
            img = cv.imread("C:/Users/patzi/Pictures/Letters_Brobots/test/" + filename, cv.IMREAD_GRAYSCALE)
            img = cv.resize(img, (image_size, image_size))
            img = img / 255.0
            img = (img - img.mean()) / img.std()
            img = np.reshape(img, (1, image_size, image_size, 1))

            interpreter.set_tensor(input_index, tf.convert_to_tensor(img, dtype="float32"))
            interpreter.invoke()
            output = interpreter.tensor(output_index)

            print(filename + " - " + str(np.round(output()[0], decimals=3)))

class PrintLRCallback(keras.callbacks.Callback):
    def __init__(self):
        super().__init__()

    def on_epoch_begin(self, epoch, logs=None):
        current_decayed_lr = self.model.optimizer._decayed_lr(tf.float32).numpy()
        print("lr: {:0.7f}".format(current_decayed_lr))

class AccuracyStoppingCallback(keras.callbacks.Callback):
    def __init__(self, value=0.9):
        super().__init__()
        self.value = value

    def on_epoch_end(self, epoch, logs={}):
        current = logs.get('val_accuracy')
        if current is None:
            warnings.warn("Early stopping requires %s available!" % self.monitor, RuntimeWarning)

        if current > self.value:
            self.model.stop_training = True

def process_path(file_path):
    image = tf.io.read_file(file_path)
    image = tf.image.decode_jpeg(image, channels=1)
    image = tf.image.resize(image, (image_size, image_size))
    image = tf.cast(image, dtype='float32') / 255.0
    label = tf.strings.split(file_path, "\\")[-1]
    label = tf.strings.substr(label, 0, 1)
    if label == 'H':
        label = tf.constant([1, 0, 0, 0], dtype='float32')
    elif label == 'S':
        label = tf.constant([0, 1, 0, 0], dtype='float32')
    elif label == 'U':
        label = tf.constant([0, 0, 1, 0], dtype='float32')
    else:
        label = tf.constant([0, 0, 0, 1], dtype='float32')
    return image, label

def augment(image, label):
    # do nothing
    rand_num = tf.random.uniform([], 0, 1.0)
    if  rand_num < 0.2: # 20%
        return image, label

    # gaussian noise
    rand_num = tf.random.uniform([], 0, 1.0)
    if  rand_num < 0.6: # 60%
        if tf.math.argmax(label) == 4:
            image = image + tf.random.normal(shape=(image_size, image_size, 1), mean=0.0, stddev=0.1)
        else:
            image = image + tf.random.normal(shape=(image_size, image_size, 1), mean=0.0, stddev=0.02)
        image = tf.clip_by_value(image, 0.0, 1.0)

    # random zoom (crop + padding)
    rand_num = tf.random.uniform([], 0, 1.0)
    if  rand_num > 0.6: # 40%
        image = tf.image.resize(image, size=(image_size + 15, image_size + 15))
        image = tf.image.random_crop(image, size=(image_size, image_size, 1))
    elif rand_num < 0.4: # 40%
        scaling = tf.random.uniform([], 0.8, 1.0)
        new_size = tf.math.round(image_size * scaling)
        image = tf.image.resize(image, size=(new_size, new_size))

        if tf.random.uniform([], 0, 1.0) > 0.5:
            background = tf.random.uniform(shape=(image_size, image_size, 1)) # noise padding
        else:
            background = tf.reshape(tf.repeat(tf.random.uniform([], 0, 1.0), image_size**2), (image_size, image_size, 1)) # constant padding

        max_delta = image_size - new_size
        x_offset = tf.cast(tf.math.round(tf.random.uniform([], 0, max_delta)), dtype='int32')
        y_offset = tf.cast(tf.math.round(tf.random.uniform([], 0, max_delta)), dtype='int32')
        image = image + 0.1
        image = tf.image.pad_to_bounding_box(image, y_offset, x_offset, image_size, image_size)
        image = image - 0.1
        image = tf.where(tf.math.less(image, 0), background, image)

    # randomly change aspect ratio
    rand_num = tf.random.uniform([], 0, 1.0)
    if rand_num > 0.8: # 20%
        image = tf.image.resize(image, size=(image_size, tf.random.uniform([], image_size, image_size + 20)))
        image = tf.image.random_crop(image, size=(image_size, image_size, 1))
    elif rand_num < 0.2: # 20%
        image = tf.image.resize(image, size=(tf.random.uniform([], image_size, image_size + 20), image_size))
        image = tf.image.random_crop(image, size=(image_size, image_size, 1))

    # random cutout
    rand_num = tf.random.uniform([], 0, 1.0)
    if rand_num < 0.5: # 50%
        mask_size = tf.random.uniform([2], minval=2, maxval=30)
        mask_size = tf.cast(tf.math.round(mask_size / 2.0) * 2, dtype='int32')

        if tf.random.uniform([], 0, 1.0) > 0.5:
            background = tf.random.uniform(shape=(image_size, image_size, 1)) # noise padding
        else:
            background = tf.reshape(tf.repeat(tf.random.uniform([], 0, 1.0), image_size**2), (image_size, image_size, 1)) # constant padding

        image = image + 0.1
        image = tfa.image.random_cutout(tf.expand_dims(image, 0), mask_size)
        image = tf.squeeze(image, 0)
        image = image - 0.1
        image = tf.where(tf.math.less(image, 0), background, image)

    # rotate
    rand_num = tf.random.uniform([], 0, 1.0)
    if rand_num < 0.3: # 30%
        if tf.random.uniform([], 0, 1.0) > 0.5:
            background = tf.random.uniform(shape=(image_size, image_size, 1)) # noise padding
        else:
            background = tf.reshape(tf.repeat(tf.random.uniform([], 0, 1.0), image_size**2), (image_size, image_size, 1)) # constant padding

        image = image + 0.1
        image = tfa.image.rotate(tf.expand_dims(image, 0), tf.clip_by_value(tf.random.normal([], mean=0.0, stddev=math.radians(3)), -math.radians(5), math.radians(5)))
        image = tf.squeeze(image, 0)
        image = image - 0.1
        image = tf.where(tf.math.less(image, 0), background, image)

    # shear
    rand_num = tf.random.uniform([], 0, 1.0)
    if rand_num < 0.3: # 30%
        if tf.random.uniform([], 0, 1.0) > 0.5:
            background = tf.random.uniform(shape=(image_size, image_size, 1)) # noise padding
        else:
            background = tf.reshape(tf.repeat(tf.random.uniform([], 0, 1.0), image_size**2), (image_size, image_size, 1)) # constant padding

        shear_x = tf.clip_by_value(tf.random.normal([], mean=0.0, stddev=0.05), -0.1, 0.1)
        shear_y = tf.clip_by_value(tf.random.normal([], mean=0.0, stddev=0.05), -0.1, 0.1)
        image = tfa.image.shear_x(tf.cast(tf.image.grayscale_to_rgb(image * 254 + 1), dtype='uint8'), shear_x, 0)
        image = tfa.image.shear_y(image, shear_y, 0)
        image = (tf.cast(image, dtype='float32') - 1.0) / 254.0
        image = tf.image.rgb_to_grayscale(image)
        image = tf.where(tf.math.less(image, 0), background, image)

    # blur
    rand_num = tf.random.uniform([], 0, 1.0)
    if rand_num < 0.25: # 25%
        filter_size = random.randint(1, 4)
        image = tfa.image.mean_filter2d(image, filter_shape=filter_size)
    elif rand_num > 0.75: # 25%
        # motion blur
        kernel_size = 3
        kernel = np.zeros([kernel_size, kernel_size])
        kernel[:, kernel_size // 2] = 1.0 / kernel_size
        kernel_motion_blur = tf.convert_to_tensor(kernel, tf.float32)
        kernel_motion_blur = tf.reshape(kernel_motion_blur, (kernel_size, kernel_size, 1, 1))
        image = tf.nn.conv2d(tf.expand_dims(image, 0), kernel_motion_blur, strides=(1,1,1,1), padding='SAME')
        image = tf.squeeze(image, 0)

    image = tf.clip_by_value(image, 0.0, 1.0)
    return image, label

def preprocess(image, label):
    image = tf.image.per_image_standardization(image)
    return image, label

def print_dataset(dataset):
    for i in range(20):
        labels = tf.data.experimental.get_single_element(dataset.take(1))[1]
        images = tf.data.experimental.get_single_element(dataset.take(1))[0]
        image = images[i] * 0.3 + 0.5
        max_val = np.max(image)
        min_val = np.min(image)
        image = (image - min_val) / (max_val - min_val)
        image = image * 255.0
        image = np.clip(image, 0, 255).astype(dtype='uint8')
        cv.imshow('img', image)
        cv.waitKey()
    cv.destroyAllWindows()

def create_model():
    inputs = keras.Input(shape=(image_size, image_size, 1))
    x = Conv2DBlock(60, 5, strides=2)(inputs)
    x = Conv2DBlock(45, 3)(x)
    x = layers.Flatten()(x)
    x = layers.Dropout(dropout_rate)(x)
    x = layers.Dense(70)(x)
    x = layers.ReLU(6.0)(x)
    outputs = layers.Dense(4, activation="softmax")(x)

    model = keras.Model(inputs=inputs, outputs=outputs)

    print(model.summary())

    lr_schedule = keras.optimizers.schedules.ExponentialDecay(
        start_lr,
        decay_steps=lr_decay_step,
        decay_rate=lr_decay)

    model.compile(
        loss=losses.CategoricalCrossentropy(),
        optimizer=keras.optimizers.Adam(learning_rate=lr_schedule),
        metrics=['accuracy'])

    return model

def create_datasets(folder):
    dataset = tf.data.Dataset.list_files(str(pathlib.Path(folder + "*.jpg")), shuffle=True, seed=1234)
    dataset = dataset.map(process_path)
    ds_val = dataset.take(num_validation).map(preprocess).batch(batch_size)
    ds_train = dataset.skip(num_validation).repeat(batch_size * steps_per_epoch * epochs // num_train + 1).map(augment).map(preprocess).batch(batch_size)
    return ds_train.prefetch(tf.data.experimental.AUTOTUNE), ds_val.prefetch(tf.data.experimental.AUTOTUNE)

ds_train, ds_val = create_datasets(file_path)

#print_dataset(ds_train)

model = create_model()

#model.load_weights(checkpoint_path)

# Create a callback that saves the model's weights every 5 epochs
cp_callback = keras.callbacks.ModelCheckpoint(
    filepath=checkpoint_path, 
    verbose=1, 
    save_weights_only=True,
    save_freq=steps_per_epoch * 5)

#model.fit(ds_train, validation_data=ds_val, epochs=epochs, steps_per_epoch=steps_per_epoch, callbacks=[cp_callback, PrintLRCallback()], verbose=2)

#model.save("model.h5")

# Convert to int8 TfLite Model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

def representative_dataset_gen():
    for i in range(100):
        yield [tf.random.normal(shape=(1, image_size, image_size, 1))]

converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.representative_dataset = representative_dataset_gen
tflite_quant_model = converter.convert()

del ds_train
del ds_val

with open("converted_model.tflite", "wb") as f:
    f.write(tflite_quant_model)

print("TfLite-Model:")
interpreter = tf.lite.Interpreter(model_content=tflite_quant_model)
interpreter.allocate_tensors()
evaluate_lite_model(interpreter)

print("Original-Model:")
for file in os.listdir(os.fsencode("C:/Users/patzi/Pictures/Letters_Brobots/test")):
    filename = os.fsdecode(file)
    if filename.endswith(".jpg"):
        img = cv.imread("C:/Users/patzi/Pictures/Letters_Brobots/test/" + filename, cv.IMREAD_GRAYSCALE)
        img = cv.resize(img, (image_size, image_size))
        img = img / 255.0
        img = (img - img.mean()) / img.std()
        img = np.reshape(img, (1, image_size, image_size, 1))
        print(filename + " - " + str(np.round(model.predict(img)[0], decimals=3)))