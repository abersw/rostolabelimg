#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Extract images from a rosbag.
"""

import os, sys

import cv2

import rospy
import rosbag
from rospkg import RosPack, ResourceNotFound
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

runSaveImages = 1
runDNN = 1
runXML = 1

DEBUG_XML = 0

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

def startFrameXML(foldername, filename, path, database, image, depth, segmented):
    frameInfo = "<annotation>\n"
    frameInfo += "<folder>" + foldername + "</folder>\n"
    frameInfo += "<filename>" + filename + "</filename>\n"
    frameInfo += "<path>" + path + filename + "</path>\n"
    frameInfo += "<source>\n"
    frameInfo += "<database>" + database + "</database>\n"
    frameInfo += "</source>\n"
    frameInfo += "<size>\n"
    image_height, image_width, _ = image.shape
    frameInfo += "<width>" + str(image_width) + "</width>\n"
    frameInfo += "<height>" + str(image_height) + "</height>\n"
    frameInfo += "<depth>" + str(depth) + "</depth>\n"
    frameInfo += "</size>\n"
    frameInfo += "<segmented>" + str(segmented) + "</segmented>\n"
    if DEBUG_XML:
        print(frameInfo)
    return frameInfo

def objectDetectedXML(name, xmin, ymin, xmax, ymax):
    objectInfo = "<object>\n"
    objectInfo += "<name>" + name + "</name>\n"
    objectInfo += "<pose>Unspecified</pose>\n"
    objectInfo += "<truncated>0</truncated>\n"
    objectInfo += "<difficult>0</difficult>\n"
    objectInfo += "<bndbox>\n"
    objectInfo += "<xmin>" + str(xmin) + "</xmin>\n"
    objectInfo += "<ymin>" + str(ymin) + "</ymin>\n"
    objectInfo += "<xmax>" + str(xmax) + "</xmax>\n"
    objectInfo += "<ymax>" + str(ymax) + "</ymax>\n"
    objectInfo += "</bndbox>\n"
    objectInfo += "</object>\n"
    if DEBUG_XML:
        print(objectInfo)
    return objectInfo

def endFrameXML():
    frameInfo = "</annotation>"
    if DEBUG_XML:
        print(frameInfo)
    return frameInfo

def main():
    """Extract a folder of images from a rosbag.
    """
    rospy.init_node('rostolabelimg_node', anonymous=True)

    topicName = rospy.get_param("/wheelchair_robot/param/rostolabelimg/image_topic")
    bagLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/bag_location")
    imgLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/img_location")
    xmlLocation = rospy.get_param("/wheelchair_robot/param/rostolabelimg/xml_location")
    confidenceThreshold = rospy.get_param("/wheelchair_robot/param/rostolabelimg/confidence_threshold")

    #pkgLocation = doesPkgExist("wheelchair_dump")
    #print("Package Location is ", pkgLocation)

    bag = rosbag.Bag(bagLocation, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[topicName]):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        frameName = "frame%06i" % count
        frameNameImg = frameName + ".png"
        print("frame name is ", frameName)

        if runSaveImages == 1:
            cv2.imwrite(os.path.join(imgLocation, frameNameImg), cv_image)
            print( "Wrote image %i" % count )

        writeToXML = startFrameXML("img", frameNameImg, imgLocation, "Unknown", cv_image, 3, 0)

        if runDNN == 1:
            image = cv2.resize(cv_image, (0,0), fx=1.0, fy=1.0)
            image_height, image_width, _ = image.shape
            model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
            output = model.forward()
            objectNoInFrame = 0

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

                    writeToXML += objectDetectedXML(class_name, int(box_width), int(box_height), int(box_x), int(box_y))

                    """
                    label = "{}: {:.2f}".format(class_name, confidence)
                    cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), 2)
                    cv2.putText(image,label ,(int(box_x), int(box_y+.02*image_height)),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 250), 2)
                    """
                    objectNoInFrame += 1
            print("total objects in frame are " , objectNoInFrame)
        writeToXML += endFrameXML()
        XMLfile = open(xmlLocation + frameName + ".xml", "w")
        n = XMLfile.write(writeToXML)
        XMLfile.close()
        #sleep(0.5) #slow down each frame for reading


        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()