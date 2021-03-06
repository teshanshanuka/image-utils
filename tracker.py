#!/usr/bin/python3

from imutils.video import FPS
import argparse
import imutils
import time
import cv2
import os
import random
import sys
import re

INSTRUCTIONS = """
***Image annotation by object tracking for YOLO training***
'e'         - start tracking
'n'         - next image
'b'         - previous image
'u'         - toggle marked box (if not tracking)
'l'         - delete annotation of marked box
'q'         - quit
'12346789'  - change marked box size
'wasd'      - move marked box
'5'         - toggle expand/shrink box
'+-'        - increase/decrease box size change step
't'         - toggle class
'c'         - clear marked box
'r'         - restart
"""

def get_tracker(tracker):
    (major, minor) = cv2.__version__.split(".")[:2]

    if int(major) == 3 and int(minor) < 3:
        return cv2.Tracker_create(tracker.upper())
    else:
        OPENCV_OBJECT_TRACKERS = {
        	"csrt": cv2.TrackerCSRT_create,
        	"kcf": cv2.TrackerKCF_create,
        	"boosting": cv2.TrackerBoosting_create,
        	"mil": cv2.TrackerMIL_create,
        	"tld": cv2.TrackerTLD_create,
        	"medianflow": cv2.TrackerMedianFlow_create,
        	"mosse": cv2.TrackerMOSSE_create
        }
        return OPENCV_OBJECT_TRACKERS[tracker]()

def drawBBox(frame, box, color=(0, 255, 0), thickness=2, text=""):
    assert len(box) == 4, "box should be a tuple of length 4"
    H, W = frame.shape[:2]
    x, y, w, h = [int(v) for v in box]
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
    if text:
        cv2.putText(frame, text, (x + 5, min(H, y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)

def update_box(box, update, step):
    (x, y, w, h) = [int(v) for v in box]
    if "T" in update:
        y -= step
        h += step
    if "B" in update:
        h += step
    if "L" in update:
        x -= step
        w += step
    if "R" in update:
        w += step
    if "u" in update:
        y -= abs(step)
    if "d" in update:
        y += abs(step)
    if "l" in update:
        x -= abs(step)
    if "r" in update:
        x += abs(step)
    return (x, y, w, h)

def save_data(class_, box, data_file, img_height, img_width, tol=5):
    for i in range(1):
        if box is not None:
            x, y, w, h = box
            if x >= img_width or y >= img_height:
                box = None
                break
            if x + w > img_width:
                w = img_width - x
            if y + h > img_height:
                h = img_height - y
            if x < 0:
                w += x
                x = 0
            if y < 0:
                h += y
                y = 0
            if w < tol or h < tol:
                box = None
                break
            x, y, w, h = x/img_width, y/img_height, w/img_width, h/img_height
            x += w/2
            y += h/2
        if box is None or w == 0. or h == 0.:
            box = None
            break
    with open(data_file, 'a') as fd:
        if box is not None:
            w_str = "{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(class_, x, y, w, h)
            fd.write(w_str)
    if box is not None:
        return w_str

def del_line(file, line_no):
    assert file.endswith(".txt"), "File is not a .txt"
    with open(file, 'r') as fd:
        t = fd.read().strip().split('\n')
    # print("{}\ndeleting {}".format(file,t))
    if not t: return

    del t[line_no]
    with open(file, 'w') as fd:
        if t:
            fd.write('\n'.join(t) + '\n')
    # print("new str: {}".format('\n'.join(t[:-1]) + '\n'))

def edit_class(file, line_no, new_class):
    assert file.endswith(".txt"), "File is not a .txt"
    with open(file, 'r') as fd:
        t = fd.read().strip().split('\n')
    t[line_no] = re.sub('^\d', str(new_class), t[line_no])
    with open(file, 'w') as fd:
        fd.write('\n'.join(t) + '\n')


def parse_data(data_file, img_height, img_width):
    data = []
    if not os.path.isfile(data_file):
        return data
    with open(data_file, 'r') as fd:
        for l in fd:
            assert len(l.split()) == 5, "Wrong format data in file {}\nData: {}".format(data_file, l)
            c, *box = l.split()
            d = {}
            d['str'] = l
            d['class'] = int(c)
            box = list(map(float, box))
            box[0] = int(round(box[0]*img_width - box[2]*img_width/2))
            box[1] = int(round(box[1]*img_height - box[3]*img_height/2))
            box[2] = int(round(box[2]*img_width))
            box[3] = int(round(box[3]*img_height))
            d['box'] = tuple(box)
            data.append(d)
    return data

def random_color():
    return (random.randint(0,255), random.randint(0,255), random.randint(0,255))

def track(folder=".", class_file="obj.names", tracker_="kcf", img_format=".png"):
    tracker = None

    imgs = [f for f in filter(lambda x: x.endswith(img_format), os.listdir(folder))]
    assert len(imgs) > 0, "No {} images found in {}".format(img_format, os.path.join(os.getcwd(), folder))

    with open(class_file, 'r') as fd:
        classes = fd.read().strip().split('\n')

    cv2.namedWindow('img')

    imgs.sort()
    frames = [cv2.imread(os.path.join(folder,f)) for f in imgs]
    quit = False

    select_class = 0

    print("OpenCV version:", cv2.__version__)
    print(INSTRUCTIONS)

    print("Classes:", classes)
    step = 10

    while not quit:
        img_data = [f for f in os.listdir(folder) if f.endswith(".txt")]

        if len(img_data) == 0: print("No previous annotation data found")

        restart = False
        img_idx = 0
        box = None
        warn = ""
        marked_box = prev_marked_box = 10

        while img_idx < len(imgs):
            frame, img = frames[img_idx], imgs[img_idx]
            txt_file = os.path.join(folder, img.split('.')[0] + ".txt")

            (H, W) = frame.shape[:2]
            text = "Frame {}/{}".format(img_idx+1, len(frames))
            update = ""
            rand_color = random_color()
            orig_frame = frame.copy()

            cv2.putText(frame, text, (10, H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, img, (10, H-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if warn:
                cv2.putText(frame, warn, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            base_frame = frame.copy()

            img_data = []

            img_data = parse_data(txt_file, H, W)

            invalid = []
            if img_data:
                for i, data in enumerate(img_data):
                    if data['box'] == (0.0, 0.0, 0.0, 0.0):
                        print("found 0000 in " + txt_file)
                        invalid.append(i)

            for i in sorted(invalid, reverse=True):
                del img_data[i]
                del_line(txt_file, i)

            while marked_box >= len(img_data):
                marked_box -= 1

            if box is not None:
                success, box = tracker.update(orig_frame)
                if not success:
                    print("Lost tracking")
                    box = None

            while True:
                frame = base_frame.copy()
                if box is not None:
                    drawBBox(frame, box, text=classes[select_class])

                if img_data:
                    for i, data in enumerate(img_data):
                        e_box = data['box']
                        e_class = classes[data['class']]
                        drawBBox(frame, e_box, color=rand_color, text=e_class, thickness=1)
                        if i == marked_box:
                            cv2.circle(frame, e_box[:2], 5, rand_color, -1)

                cv2.imshow('img', frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("e"):
                    box = cv2.selectROI("img", frame, fromCenter=False, showCrosshair=True)
                    tracker = get_tracker(tracker_)
                    tracker.init(orig_frame, box)

                    frame = base_frame.copy()
                    drawBBox(frame, box)

                elif key == ord("n"):
                    box_str = save_data(select_class, box, txt_file, H, W)
                    img_idx += 1
                    marked_box = prev_marked_box
                    break

                elif key == ord("b"):
                    img_idx = (img_idx - 1) % len(imgs)
                    box = None
                    marked_box = prev_marked_box
                    break

                elif key == ord("r"):
                    restart = True
                    break

                elif key == ord("q"):
                    quit = True
                    break

                elif key == ord("t"):
                    if box is not None:
                        select_class = (select_class + 1) % len(classes)
                    elif img_data:
                        img_data[marked_box]['class'] = (img_data[marked_box]['class'] + 1) % len(classes)
                        edit_class(txt_file, marked_box, img_data[marked_box]['class'])

                elif key == ord("c"):
                    box = None

                elif key == ord("l"):
                    if img_data:
                        prev_marked_box = marked_box
                        del img_data[marked_box]
                        del_line(txt_file, marked_box)
                        marked_box = (marked_box - 1) % len(img_data) if img_data else 0

                elif key == ord("u"):
                    if img_data:
                        marked_box = prev_marked_box = (marked_box - 1) % len(img_data)

                elif key == ord("1"):
                    update = "BL"
                elif key == ord("2"):
                    update = "B"
                elif key == ord("3"):
                    update = "BR"
                elif key == ord("6"):
                    update = "R"
                elif key == ord("9"):
                    update = "TR"
                elif key == ord("8"):
                    update = "T"
                elif key == ord("7"):
                    update = "TL"
                elif key == ord("4"):
                    update = "L"
                elif key == ord("w"):
                    update = "u"
                elif key == ord("s"):
                    update = "d"
                elif key == ord("a"):
                    update = "l"
                elif key == ord("d"):
                    update = "r"

                elif key == ord("+"):
                    step += 1
                elif key == ord("-"):
                    step -= 1
                elif key == ord("5"):
                    step = -step

                if update and box is not None:
                    box = update_box(box, update, step)
                    frame = base_frame.copy()
                    drawBBox(frame, box)
                    tracker.clear()

                    tracker = get_tracker(tracker_)
                    tracker.init(orig_frame, box)
                update = ""

                # if key != 255: print(key)

            if quit or restart:
                break

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: ./tracker <image folder> [<class name file> default:obj.names]")
        sys.exit()
    classes = "obj.names"

    folder = sys.argv[1]
    if len(sys.argv) > 2:
        classes = sys.argv[2]

    track(folder, classes, img_format='.png')
    # close all windows
    cv2.destroyAllWindows()
