#!/usr/bin/python3

"""
Select some random images
"""

import os
import sys
import random
import shutil

def copy_img_with_marking(img, op_folder):
    shutil.copy(img, op_folder)
    txt = os.path.splitext(img)[0] + ".txt"
    if not os.path.isfile(txt):
        with open(txt, 'w') as _:
            pass
    shutil.copy(txt, op_folder)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: {} <folder> <output folder> <#images to pick>".format(sys.argv[0]))
        sys.exit(1)
    folder = sys.argv[1]
    out_folder = sys.argv[2]
    #portion = .05
    no_imgs = int(sys.argv[3])
    ext = ".png"

    imgs = []
    for subf in os.walk(folder, topdown=True):
        if subf[0].endswith("image_raw"):
            for f in subf[2]:
                if f.endswith(ext):
                    txt = os.path.splitext(f)[0] + ".txt"
                    with open(os.path.join(subf[0], txt), 'r') as fd:
                        t = fd.read().strip()
                    if t:
                        imgs.append(os.path.join(subf[0], f))

    #selected = random.sample(imgs, int(round(len(imgs)*portion)))
    selected = random.sample(imgs, no_imgs)

    print("Selected {} random images out of {}".format(len(selected), len(imgs)))

    if not os.path.isdir(out_folder):
        os.mkdir(out_folder)

    for rimg in selected:
        copy_img_with_marking(rimg, out_folder)
