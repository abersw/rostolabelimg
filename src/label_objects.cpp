/*
 * label_objects.cpp
 * rostolabelimg
 * version: 0.0.1 Majestic Maidenhair
 * Status: pre-alpha
*/

#include <ros/ros.h> //main ROS library
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "sensor_msgs/Image.h"
#include <std_srvs/Empty.h>
#include "rostolabelimg/annotatedObjects.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

static const int DEBUG_main = 0;
static const int DEBUG_getFrameName = 0;
static const int DEBUG_frameCallback_rawData = 0;

ros::NodeHandle *ptr_n;

int frameID = 0;

std::string image_topic_name;
std::string bagfile_name;

/*std::string getFrameName() {
    std::string currentFrameName = "frame-" + std::to_string(frameID);
    if (DEBUG_getFrameName) {
        cout << "current frame name is " << currentFrameName << endl;
    }
    frameID++;
    return currentFrameName;
}

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
    std::string currentFrameName = getFrameName();
    std::string pathLocation = "/home/tomos/ros/wheelchair/catkin_ws/src/wheelchair_dump/dump/rostolabelimg/img/"; //location of dir to save to
    for (int isObject = 0; isObject < totalObjectsInFrame; isObject++) {
        //std::string requestObjectName = data->name[isObject];
        
    }

    std::string annotationLocation = pathLocation + currentFrameName + ".jpg"; //append file name to path location
        std::cout << annotationLocation << std::endl; //print out path location
        ptr_n->setParam("/wheelchair_robot/image_saver_object_annotation/filename_format", annotationLocation); //set path location in parameter server
        std::string s;
        if (ptr_n->getParam("/wheelchair_robot/image_saver_object_annotation/filename_format", s)) { //get parameter to confirm
            ROS_INFO("Got param: %s", s.c_str()); //print out parameter
        }
        else {
            ROS_ERROR("Failed to get param 'my_param'"); //couldn't retrieve parameter
        }

    ros::ServiceClient client = ptr_n->serviceClient<std_srvs::Empty>("/wheelchair_robot/image_saver_object_annotation/save"); //call service with empty type
    std_srvs::Empty srv;
    if (client.call(srv))
    {
    ROS_INFO("successfully called service"); //service successfully called
    }
    else {
    ROS_WARN("oops"); //service failed to call
    }

    cout << "end frame" << endl;
}*/

void readBag() {
    cv::Mat cvimage;
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/zed/zed_node/left/image_rect_color"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view) {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
            //std::cout << s->image << std::endl;
            cvimage = cv_bridge::toCvCopy(s)->image;
            cout << "found image" << endl;
            cv::imshow("image", cvimage);
            cv::waitKey(1);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rostolabelimg_node");
    ros::NodeHandle n;
    ptr_n = &n;

    ros::Rate rate(10.0);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);
    image_topic_name = "//zed/zed_node/left/image_rect_color";
    bagfile_name = "/home/tomos/ros/wheelchair/catkin_ws/src/wheelchair_dump/dump/bags/env1-run1_2021-08-05-14-32-31.bag";
    readBag();

    //ros::Subscriber sub_mobilenet_object = n.subscribe("/wheelchair_robot/mobilenet/labelimg", 1000, frameCallback);
    //object_depth_pub = n.advertise<wheelchair_msgs::foundObjects>("wheelchair_robot/dacop/depth_sensing/detected_objects", 1000); //publish topic for object locations

    /*if (ros::isShuttingDown()) {
        //do something
    }
    if (DEBUG_main) {
        cout << "spin \n";
    }*/
    //ros::spin();
    //rate.sleep();

    return 0;
}