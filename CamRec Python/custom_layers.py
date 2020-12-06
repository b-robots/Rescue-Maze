import numpy as np
import tensorflow as tf
from tensorflow import keras
from keras import layers

class Conv2DBlock(layers.Layer):
    def __init__(self, filters=32, kernel_size=3, **kwargs):
        super().__init__(**kwargs)
        self.filters = filters
        self.kernel_size = kernel_size
        self.kernel_size = kernel_size
        self.conv = layers.Conv2D(filters, kernel_size)
        self.batch_norm = layers.BatchNormalization()
        self.activation = layers.ReLU(6.0)
        self.pooling = layers.MaxPool2D()
 
    def call(self, input_tensor):
        x = self.conv(input_tensor)
        x = self.activation(x)
        x = self.batch_norm(x)
        x = self.pooling(x)
        return x

    def get_config(self):
        return {'filters' : self.filters,
                'kernel_size' : self.kernel_size}

class MobileNetV1Block(layers.Layer):
    def __init__(self, strides=1, filters=32, kernel_size=3, **kwargs):
        super().__init__(**kwargs)
        self.strides = strides
        self.filters = filters
        self.kernel_size = kernel_size
        self.depthwise_conv = layers.DepthwiseConv2D(kernel_size=kernel_size, strides=strides, padding='same')
        self.batch_norm_1 = layers.BatchNormalization()
        self.pointwise_conv = layers.Conv2D(filters=filters, kernel_size=1, padding='same')
        self.batch_norm_2 = layers.BatchNormalization()

    def call(self, input_tensor, training=False):
        x = self.depthwise_conv(input_tensor)
        x = self.batch_norm_1(x, training=training)
        x = layers.ReLU(6.0)(x)
        x = self.pointwise_conv(x)
        x = self.batch_norm_2(x, training=training)
        x = layers.ReLU(6.0)(x)
        return x
    
    def get_config(self):
        return {'strides' : self.strides,
                'filters' : self.filters,
                'kernel_size' : self.kernel_size}