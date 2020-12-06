import cv2 as cv
import numpy as np
import os
import shutil
import re

folder_path = "C:/Users/patzi/Pictures/Letters_Brobots/"

def modify_letter_img(letter):
    i = 0
    for file in os.listdir(os.fsencode(folder_path + letter + "_unmodified")):
        filename = os.fsdecode(file)
        if filename.endswith(".jpg"):
            img = cv.imread(folder_path + letter + "_unmodified/" + filename, cv.IMREAD_GRAYSCALE)
            img = cv.resize(img, (100, 100))
            for a in range(1):
                for b in range(1):
                    for c in range(1):
                        for d in range(1):
                            # Gauss-Rauschen
                            img_mod = img.astype('f')
                            if d == 1:
                                img_mod = np.random.normal(0.0, 0.05, (100, 100)) * 255.0 + img_mod
                            # Helligkeitsvariationen
                            if c == 1:
                                img_mod = img_mod + np.random.normal(0.0, 0.1) * 255.0
                            # Zoom
                            if b == 1:
                                scale = np.clip(np.random.normal(1.0, 0.3), 0.5, 2)
                                img_mod = cv.resize(img_mod, None, fx=scale, fy=scale)
                                size, _ = img_mod.shape
                                if size >= 100:
                                    img_mod = img_mod[int(size / 2 - 50) : int(size / 2 + 50), int(size / 2 - 50) : int(size / 2 + 50)]
                                else:
                                    img_tmp = np.random.normal(0.5, 0.05, (100, 100)) * 255.0
                                    img_tmp[int(50 - size / 2) : int(50 + size / 2), int(50 - size / 2) : int(50 + size / 2)] = img_mod
                                    img_mod = img_tmp
                            if a == 1:
                                tx = min(max(np.random.normal(0.0, 10.0), -15.0), 15.0)
                                ty = min(max(np.random.normal(0.0, 10.0), -15.0), 15.0) * 2336 / 4160
                                M = np.float32([[1, 0, tx], [0, 1, ty]])
                                img_mod = np.clip(img_mod, 0.0, 255.0) + 0.1
                                img_mod = cv.warpAffine(img_mod, M, (100, 100))
                                with np.nditer(img_mod, op_flags=['readwrite']) as it:
                                    for x in it:
                                        if x <= 0.09:
                                            x[...] = np.random.uniform(0.5, 0.05) * 255 + 0.1
                                img_mod = img_mod - 0.1
                            cv.imwrite(folder_path + "train/" + letter + str(i) + ".jpg", img_mod)
                            i+=1

def generate_none_img():
    i = 0
    for file in os.listdir(os.fsencode(folder_path + "None_unmodified")):
        filename = os.fsdecode(file)
        if filename.endswith(".jpg"):
            img = cv.imread(folder_path + "None_unmodified/" + filename, cv.IMREAD_GRAYSCALE)
            img = cv.resize(img, (100, 100))
            for a in range(3): # 3 Translationsvarianten
                for b in range(3): # 3 Zoomvarianten
                    for c in range(3): # 3 Helligkeitsvarianten
                        for d in range(3): # 3 Noise-Varianten
                            # Gauss-Rauschen
                            img_mod = img.astype('f')
                            if d != 0:
                                img_mod = np.random.normal(0.0, 0.1, (100, 100)) * 255.0 + img_mod
                            # Helligkeitsvariationen
                            if c != 0:
                                img_mod = img_mod + np.random.normal(0.0, 0.2) * 255.0
                            # Zoom
                            if b != 0:
                                scale = np.clip(np.random.normal(1.0, 0.5), 0.25, 4)
                                img_mod = cv.resize(img_mod, None, fx=scale, fy=scale)
                                size, _ = img_mod.shape
                                if size >= 100:
                                    img_mod = img_mod[int(size / 2 - 50) : int(size / 2 + 50), int(size / 2 - 50) : int(size / 2 + 50)]
                                else:
                                    img_tmp = np.random.normal(0.5, 0.1, (100, 100)) * 255.0
                                    img_tmp[int(50 - size / 2) : int(50 + size / 2), int(50 - size / 2) : int(50 + size / 2)] = img_mod
                                    img_mod = img_tmp
                            if a != 0:
                                tx = min(max(np.random.normal(0.0, 10.0), -15.0), 15.0)
                                ty = min(max(np.random.normal(0.0, 10.0), -15.0), 15.0) * 2336 / 4160
                                M = np.float32([[1, 0, tx], [0, 1, ty]])
                                img_mod = np.clip(img_mod, 0.0, 255.0) + 0.1
                                img_mod = cv.warpAffine(img_mod, M, (100, 100))
                                with np.nditer(img_mod, op_flags=['readwrite']) as it:
                                    for x in it:
                                        if x <= 0.09:
                                            x[...] = np.random.uniform(0.5, 0.05) * 255 + 0.1
                                img_mod = img_mod - 0.1
                            cv.imwrite(folder_path + "train/None" + str(i) + ".jpg", img_mod)
                            i+=1

modify_letter_img("H")
modify_letter_img("S")
modify_letter_img("U")
#generate_none_img()