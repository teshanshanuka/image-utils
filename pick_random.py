#!/usr/bin/python3

# Author: Teshan Liyanage <teshanuka@gmail.com>

"""
Select some random images
"""

import os
import sys
import random
import shutil


idx = 0

def copy_img_with_marking(img, op_folder):
    global idx
    bs, ext = os.path.splitext(img)
    img_out = f"{os.path.basename(bs)}_{idx}{ext}"
    txt = f"{bs}.txt"
    txt_out = f"{os.path.basename(bs)}_{idx}.txt"

    shutil.copy(img, os.path.join(op_folder, img_out))
    if not os.path.isfile(txt):
        with open(txt, 'w') as _:
            pass
    shutil.copy(txt, os.path.join(op_folder, txt_out))

    idx += 1

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: {} <folder> <output folder> <#images to pick> [<folder for the rest>]".format(sys.argv[0]))
        sys.exit(1)
    folder = sys.argv[1]
    out_folder = sys.argv[2]
    #portion = .05
    no_imgs = int(sys.argv[3])
    ext = ".png"

    rest_op_folder = sys.argv[4] if len(sys.argv) == 5 else None
    

    imgs = []
    img_bases = []
    for subf in os.walk(folder, topdown=True):
        for f in subf[2]:
            if f.endswith(ext):
                txt = os.path.splitext(f)[0] + ".txt"
                img_bases.append(f)

                with open(os.path.join(subf[0], txt), 'r') as fd:
                    t = fd.read().strip()
                if t:
                    imgs.append(os.path.join(subf[0], f))

    #selected = random.sample(imgs, int(round(len(imgs)*portion)))
    print(f"Found {len(imgs)} images")
    selected = random.sample(imgs, no_imgs)

    print("Selected {} random images out of {}".format(len(selected), len(imgs)))

    if not os.path.isdir(out_folder):
        os.mkdir(out_folder)

    for rimg in selected:
        copy_img_with_marking(rimg, out_folder)

    if rest_op_folder is not None:
        if not os.path.isdir(rest_op_folder):
            os.mkdir(rest_op_folder)

        rest = [img for img in imgs if img not in selected]
        for rimg in rest:
            copy_img_with_marking(rimg, rest_op_folder)
