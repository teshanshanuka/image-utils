#!/usr/bin/python3

# Author: Teshan Liyanage <teshanuka@gmail.com>

"""
From consecutive frames of a video, select significantly different images and put them into
a seperate folder
"""

import numpy as np
import cv2
import sys
import os
import shutil

def img_diff(img1_, img2_):
    img1, img2 = cv2.imread(img1_), cv2.imread(img2_)
    assert img1 is not None and img2 is not None, "Images are not there probably. {} {}".format(img1_, img2_)
    assert img1.shape == img2.shape, "Images must be of same shape to be compared"

    d = cv2.subtract(img1, img2)
    return np.sum(d**2) / img1.size

def get_files(folder, ext):
    return [os.path.join(folder,f) for f in os.listdir(folder) if f.endswith(ext)]

def copy_img_with_marking(img, op_folder):
    shutil.copy(img, op_folder)
    txt = os.path.splitext(img)[0] + ".txt"
    if not os.path.isfile(txt):
        with open(txt, 'w') as _:
            pass
    shutil.copy(txt, op_folder)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: {} <image folder> <output folder> <diff limit(float)>".format(sys.argv[0]))
        sys.exit()

    folder = sys.argv[1]
    op_folder = sys.argv[2]
    diff_limit = float(sys.argv[3])  # 0.35

    assert os.path.isdir(folder), "No folder named " + folder
    if not os.path.isdir(op_folder):
        os.mkdir(op_folder)

    imgs = get_files(folder, ext=".png")
    imgs.sort()

    print("Copying from {}".format(folder))

    count = 0
    saved = 0
    done = False
    while True:
        diff = 0.
        first = imgs[count]
        if count == 0:
            copy_img_with_marking(first, op_folder)
            saved += 1
        while diff < diff_limit:
            count += 1
            if count == len(imgs):
                done = True
                break
            diff = img_diff(first, imgs[count])
        if done:
            break
        copy_img_with_marking(imgs[count], op_folder)
        saved += 1

    print("Saved {} images from {}".format(saved, len(imgs)))
    # close all windows
    cv2.destroyAllWindows()
