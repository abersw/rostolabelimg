#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Extract images from a rosbag.
"""

import os, sys
import argparse
import array

import cv2

import rospy
import rosbag
from rospkg import RosPack, ResourceNotFound
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

runSaveImages = 0
runDNN = 1
runXML = 1

model = cv2.dnn.readNetFromTensorflow('/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/frozen_inference_graph.pb',
                                        '/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')


classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}



def id_class_name(class_id, classes):
  for key, value in classes.items():
    if class_id == key:
      return value

def doesPkgExist(pkgName):
    print("check if package exists")
    rospack = RosPack()
    getPkgPath = ""
    try:
        getPkgPath = rospack.get_path(pkgName)
    except ResourceNotFound as err:
        print("ERROR: Couldn't find ", err)
        print("Shutting down program")
        sys.exit(1)
    return getPkgPath

def saveImage(outputDir, count, cv_img):
    cv2.imwrite(os.path.join(outputDir, "frame%06i.png" % count), cv_img)

def currentFrame():
    print("current frame is ")

def main():
    """Extract a folder of images from a rosbag.
    """
    rospy.init_node('rostolabelimg_node', anonymous=True)

    topicName = rospy.get_param("/wheelchair_robot/param/rostolabelimg/image_topic")
    bagLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/bag_location")
    imgLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/img_location")
    xmlLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/xml_location")
    confidenceThreshold = rospy.get_param("/wheelchair_robot/param/rostolabelimg/confidence_threshold")

    pkgLocation = doesPkgExist("wheelchair_dump")
    print("Package Location is ", pkgLocation)

    bag = rosbag.Bag(bagLocation, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[topicName]):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if runSaveImages == 1:
            cv2.imwrite(os.path.join(imgLocation, "frame%06i.png" % count), cv_image)
            print( "Wrote image %i" % count )

        if runDNN == 1:
            image = cv2.resize(cv_image, (0,0), fx=1.0, fy=1.0)
            image_height, image_width, _ = image.shape
            model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
            output = model.forward()
            objectNoInFrame = 0
            print("start frame")

            for detection in output[0, 0, :, :]:
                confidence = detection[2]
                if confidence > confidenceThreshold:
                    class_id = detection[1]
                    class_name=id_class_name(class_id,classNames) #add +1 to class_id for fast-resnet
                    print(str(str(class_id) + " " + str(detection[2])  + " " + class_name))
                    box_x = detection[3] * image_width
                    box_y = detection[4] * image_height
                    box_width = detection[5] * image_width
                    box_height = detection[6] * image_height

                    """
                    label = "{}: {:.2f}".format(class_name, confidence)
                    cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), 2)
                    cv2.putText(image,label ,(int(box_x), int(box_y+.02*image_height)),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 250), 2)
                    """
                    objectNoInFrame += 1
        print("total objects in frame are " , objectNoInFrame)

        #sleep(0.5)


        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()