/*
 * label_objects.cpp
 * rostolabelimg
 * version: 0.0.1 Majestic Maidenhair
 * Status: pre-alpha
*/

#include <ros/ros.h> //main ROS library
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "sensor_msgs/Image.h"
#include "rostolabelimg/annotatedObjects.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

static const int DEBUG_main = 0;
static const int DEBUG_frameCallback_rawData = 0;

void frameCallback(const rostolabelimg::annotatedObjects::ConstPtr &data) {
    cout << "start frame" << endl;
    int totalObjectsInFrame = data->totalObjectsInFrame;
    if (DEBUG_frameCallback_rawData) {
        for (int isObject = 0; isObject < totalObjectsInFrame; isObject++) {
            cout <<
            "name " << data->name[isObject] << ", " <<
            "truncated " << data->truncated[isObject] << ", " <<
            "difficult " << data->difficult[isObject] << ", " <<
            "xmin " << data->xmin[isObject] << ", " <<
            "ymin " << data->ymin[isObject] << ", " <<
            "xmax " << data->xmax[isObject] << ", " <<
            "ymax " << data->ymax[isObject] << ", " <<
            "totalObjectsInFrame " << data->totalObjectsInFrame << endl;
        }
    }
    std::string pathLocation = "/home/tomos/ros/wheelchair/catkin_ws/"; //location of dir to save to
    for (int isObject = 0; isObject < totalObjectsInFrame; isObject++) {
        std::string requestObjectName = data->name[isObject];
        std::string annotationLocation = pathLocation + requestObjectName + ".jpg"; //append file name to path location
        std::cout << annotationLocation << std::endl; //print out path location
    }

    cout << "end frame" << endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rostolabelimg_node");
    ros::NodeHandle n;

    ros::Rate rate(10.0);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);

    ros::Subscriber sub_mobilenet_object = n.subscribe("/wheelchair_robot/mobilenet/labelimg", 1000, frameCallback);
    //object_depth_pub = n.advertise<wheelchair_msgs::foundObjects>("wheelchair_robot/dacop/depth_sensing/detected_objects", 1000); //publish topic for object locations

    if (ros::isShuttingDown()) {
        //do something
    }
    if (DEBUG_main) {
        cout << "spin \n";
    }
    ros::spin();
    rate.sleep();

    return 0;
}