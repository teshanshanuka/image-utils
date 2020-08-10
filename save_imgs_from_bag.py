#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""
Save images from a rosbag

File structure:

folder1 --bag1
        --bag2
        .
        .
folder2 --bag1
        --bag2
        .
        .
.
.
"""

import os
import sys
import argparse

#import cv2
import os

#import rosbag
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge

def main(bag, topic, op_dir, op_subdirs=None):
    print "Extracting images from {} on topics {} into {}".format(bag, topics, op_dir)

    if not os.path.isdir(op_dir):
        os.mkdir(op_dir)

    if op_subdirs:
        for subdir in op_subdirs.values():
            if not os.path.isdir(os.path.join(op_dir, subdir)):
                os.mkdir(os.path.join(op_dir, subdir))

    bag = rosbag.Bag(bag, "r")

    info = bag.get_type_and_topic_info()
    if not all(topic in info.topics.keys() for topic in topics):
        print "Topics not available in bag. Available topics: {}".format(info.topics.keys())
        return

    bridge = CvBridge()
    count = {topic:0 for topic in topics}
    for topic, msg, t in bag.read_messages(topics=topics):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if op_subdirs:
            cv2.imwrite(os.path.join(op_dir, op_subdirs[topic],
                                    "{:06}_{}.png".format(count[topic], str(msg.header.stamp)[3:12])),
                        cv_img)
        else:
            cv2.imwrite(os.path.join(op_dir,
                                    "{}_{:06}_{}.png".format(topic.split('/')[-1], count[topic], str(msg.header.stamp)[3:12])),
                        cv_img)
        # print "Wrote image %i to path %s" % (count, os.path.join(op_dir,
        #                                                         "{:06}{}.png".format(count, msg.header.stamp)))
        count[topic] += 1

    bag.close()
    print "\nDONE! frame counts: {}\n".format(count)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: {} <image folder> <output folder>".format(sys.argv[0]))
        sys.exit()

    # bag = 'catkin_ws_05/2019-07-24-16-28-20.bag'
    topics = ['/camera/color/image_raw','/camera/depth/image_rect_raw']
    # op_dir = bag.split('/')[-1].split('.')[0]
    op_subdirs = {topic: topic.split('/')[-1] for topic in topics}
    
    for folder in filter(os.path.isdir, map(lambda x: os.path.join(sys.argv[1], x), os.listdir(sys.argv[1]))):
        print(folder)
        for bagfile in filter(lambda x: x.endswith('.bag'), os.listdir(folder)):
            bag = os.path.join(folder,bagfile)
            op_dir = os.path.join(sys.argv[2], bag.split('/')[-1].split('.')[0])
            print(bag, op_dir)
            main(bag, topics, op_dir, op_subdirs)

    # main(bag, topics, op_dir, op_subdirs)
