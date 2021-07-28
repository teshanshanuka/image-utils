#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Teshan Liyanage <teshanuka@gmail.com>

# Quick delete a class -> for F in */*.txt; do sed -i '/^0/d' $F; done

"""
Save images from a rosbag
"""

import os
import sys
import argparse
import cv2
import os
import numpy as np

import rosbag
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


def extract(bag, topics, op_dir):
    print("Extracting images from {} on topics {} into {}".format(bag, list(topics.keys()), op_dir))

    if not os.path.isdir(op_dir):
        os.mkdir(op_dir)
    for d in topics.values():
        _d = os.path.join(op_dir, d)
        if not os.path.isdir(_d):
            os.mkdir(_d)

    bag = rosbag.Bag(bag, "r")

    info = bag.get_type_and_topic_info()
    # if not all(topic in info.topics.keys() for topic in topics):
    #     print("Topics not available in bag. Available topics: {}".format(info.topics.keys()))
    #     return

    bridge = CvBridge()
    count = {topic: 0 for topic in topics}
    for topic, msg, t in bag.read_messages(topics=topics.keys()):
        if not isinstance(msg, Image):  # compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        out_file = os.path.join(op_dir, topics[topic], "{:06}_{}.png".format(count[topic], str(msg.header.stamp)[3:12]))
        cv2.imwrite(out_file, cv_img)
        # print "Wrote image %i to path %s" % (count, os.path.join(op_dir,
        #                                                         "{:06}{}.png".format(count, msg.header.stamp)))
        count[topic] += 1

    bag.close()
    print("\nDONE! frame counts: {}\n".format(count))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: {} <bag> <output folder>".format(sys.argv[0]))
        sys.exit()

    bag_, op_dir_ = sys.argv[1:]

    topics_ = {
        "/small_hauler_1/camera/left/image_raw/compressed": "left_h1",
        "/small_hauler_1/camera/right/image_raw/compressed": "right_h1",
        "/small_hauler_2/camera/left/image_raw/compressed": "left_h2",
        "/small_hauler_2/camera/right/image_raw/compressed": "right_h2",
        "/small_hauler_3/camera/left/image_raw/compressed": "left_h3",
        "/small_hauler_3/camera/right/image_raw/compressed": "right_h3",
        "/small_scout_1/camera/left/image_raw/compressed": "left_s1",
        "/small_scout_1/camera/right/image_raw/compressed": "right_s1",
        "/small_scout_2/camera/left/image_raw/compressed": "left_s2",
        "/small_scout_2/camera/right/image_raw/compressed": "right_s2",
        "/small_excavator_1/camera/left/image_raw/compressed": "left_e1",
        "/small_excavator_1/camera/right/image_raw/compressed": "right_e1"
    }

    extract(bag_, topics_, op_dir_)
