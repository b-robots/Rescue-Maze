import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, losses
import pathlib
import cv2 as cv
import random
import scipy.stats as stats

file_path = 'trainNew/'
batch_size = 128
epochs = 100
image_size = 50
checkpoint_path = "cp_weights.ckpt"
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
            img = np.reshape(img, (image_size, image_size, 1))

            interpreter.set_tensor(input_index, tf.convert_to_tensor([img, img], dtype="float32"))
            interpreter.invoke()
            output = interpreter.tensor(output_index)

            print(filename + " - " + str(np.round(output()[0], decimals=3)))

class PrintLRCallback(keras.callbacks.Callback):
    def __init__(self):
        super().__init__()

    def on_epoch_begin(self, epoch, logs=None):
        current_decayed_lr = self.model.optimizer._decayed_lr(tf.float32).numpy()
        print("lr: {:0.7f}".format(current_decayed_lr))

def makePatch(img):
    x,y,w,h = cv.boundingRect(img)
    if w > 1 and h > 1:
        img = img[y:y+h, x:x+w]
    return cv.resize(img, (image_size, image_size), interpolation=cv.INTER_NEAREST)

def process_path(file_path):
    image = tf.io.read_file(file_path)
    image = tf.image.decode_jpeg(image, channels=1)
    image = tf.image.resize(image, (image_size, image_size))
    image = tf.cast(image, dtype='float32')
    label = tf.strings.split(file_path, "\\")[-1]
    label = tf.strings.substr(label, 0, 1)
    if label == 'H':
        label = tf.constant([0, 1, 0, 0], dtype='float32')
    elif label == 'S':
        label = tf.constant([0, 0, 1, 0], dtype='float32')
    elif label == 'U':
        label = tf.constant([0, 0, 0, 1], dtype='float32')
    else:
        label = tf.constant([1, 0, 0, 0], dtype='float32')
    return image, label

def augment(img, label):
    img = np.squeeze(img.numpy())
    strength = 1.0
    if tf.argmax(label) == 0:
        strength = 2.0

    if random.random() < 0.5:
        size = random.randint(3, 5)
        img = cv.erode(img, np.ones((size,size)))
    else:
        size = random.randint(3, 7)
        bigger = np.zeros((60, 60), np.uint8)
        bigger[5:55, 5:55] = img
        img = cv.dilate(bigger, np.ones((size,size)))
        img = makePatch(img)

    if random.random() < 0.0:
        # perspective
        originPts = np.float32([[0, 0], [0, image_size], [image_size, 0], [image_size, image_size]])
        destPts = (np.random.normal(0.0, 10.0 * strength, (4,2)) + originPts).astype(np.float32)
        destPts = destPts - np.amin(destPts, 0)
        destPts = destPts / np.amax(destPts, 0) * image_size
        m = cv.getPerspectiveTransform(originPts, destPts)
        img = cv.warpPerspective(img, m, (image_size, image_size), flags=cv.INTER_NEAREST)
        img = makePatch(img)
    else:
        # affine
        sx = (random.random() * 0.5 - 0.25) * strength
        sy = (random.random() * 0.5 - 0.25) * strength
        mu = 0
        std = 5 * strength
        a = -10
        b = 10
        angle = stats.truncnorm.rvs((a - mu) / std, (b - mu) / std, loc=mu, scale=std)
        rot = cv.getRotationMatrix2D((image_size // 2, image_size // 2), angle, 1)
        shear = np.float32([
            [1, sx, 0],
            [sy, 1, 0],
            [0, 0, 1]])
        m = np.identity(3, np.float32)
        m[:2, :] = rot
        m = np.matmul(m, shear)
        bigger = np.zeros((100, 100), np.uint8)
        bigger[25:75, 25:75] = img
        img = cv.warpAffine(bigger, m[:2, :], (100, 100), flags=cv.INTER_NEAREST)
        img = makePatch(img)

    return tf.constant(np.reshape(img, (image_size, image_size, 1)), tf.float32), label

def augment_tf(img, label):
    img, label = tf.py_function(augment, [img, label], [tf.float32, tf.float32])
    img.set_shape([image_size, image_size, 1])
    label.set_shape([4])
    return img, label

def preprocess(image, label):
    image = image / tf.maximum(tf.reduce_max(image), 1)
    return image, label

def print_dataset(dataset):
    images, labels = tf.data.experimental.get_single_element(dataset.take(1))
    for i in range(20):
        image = image * 255.0
        image = np.clip(image, 0, 255).astype(dtype='uint8')
        print(labels[i])
        cv.imshow('img', image)
        cv.waitKey()
    cv.destroyAllWindows()

def create_model():
    inputs = keras.Input(shape=(image_size, image_size, 1), dtype='float32')
    x = layers.Conv2D(32, 3, padding='same')(inputs)
    x = layers.BatchNormalization()(x)
    x = layers.ELU()(x)
    x = layers.MaxPool2D()(x)
    x = layers.Dropout(0.7)(x)
    x = layers.Conv2D(32, 3, padding='same')(x)
    x = layers.BatchNormalization()(x)
    x = layers.ELU()(x)
    x = layers.MaxPool2D()(x)
    x = layers.SpatialDropout2D(0.7)(x)
    x = layers.Conv2D(16, 3, padding='same')(x)
    x = layers.BatchNormalization()(x)
    x = layers.ELU()(x)
    x = layers.MaxPool2D()(x)
    x = layers.SpatialDropout2D(0.7)(x)
    x = layers.Conv2D(16, 3, padding='same')(x)
    x = layers.BatchNormalization()(x)
    x = layers.ELU()(x)
    x = layers.MaxPool2D()(x)
    x = layers.Flatten()(x)
    x = layers.Dropout(0.7)(x)
    x = layers.Dense(64)(x)
    x = layers.LeakyReLU()(x)
    outputs = layers.Dense(4, activation='softmax')(x)

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
    ds_train = dataset.repeat(200).map(augment_tf).repeat(10).map(preprocess)
    rep_ds = ds_train.take(20).batch(2)
    ds_train = ds_train.batch(batch_size)
    return ds_train.prefetch(tf.data.experimental.AUTOTUNE), rep_ds.prefetch(tf.data.experimental.AUTOTUNE)

ds_train, rep_ds = create_datasets(file_path)

# print_dataset(ds_train)

model = create_model()

# model.load_weights(checkpoint_path)

# Create a callback that saves the model's weights every 5 epochs
cp_callback = keras.callbacks.ModelCheckpoint(
    filepath=checkpoint_path, 
    verbose=1, 
    save_weights_only=True,
    save_freq=10 * 5)

model.fit(ds_train, epochs=epochs, steps_per_epoch=10, callbacks=[cp_callback, PrintLRCallback()], verbose=2)

model.save("model.h5")

# Convert to int8 TfLite Model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
def rep_ds_gen():
    for example in rep_ds.take(100):
        yield [example[0]]

converter.representative_dataset = rep_ds_gen
tflite_quant_model = converter.convert()

with open("converted_model.tflite", "wb") as f:
    f.write(tflite_quant_model)

print("TfLite-Model:")
interpreter = tf.lite.Interpreter(model_content=tflite_quant_model)
interpreter.resize_tensor_input(interpreter.get_input_details()[0]["index"], [2, 50, 50, 1])
interpreter.allocate_tensors()
evaluate_lite_model(interpreter)

print("Original-Model:")
for file in os.listdir(os.fsencode("C:/Users/patzi/Pictures/Letters_Brobots/test")):
    filename = os.fsdecode(file)
    if filename.endswith(".jpg"):
        img = cv.imread("C:/Users/patzi/Pictures/Letters_Brobots/test/" + filename, cv.IMREAD_GRAYSCALE)
        img = cv.resize(img, (image_size, image_size))
        img = img / 255.0
        img = np.reshape(img, (1, image_size, image_size, 1))
        print(filename + " - " + str(np.round(model.predict(img)[0], decimals=3)))