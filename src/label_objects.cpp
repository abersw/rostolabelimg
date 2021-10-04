/*
 * label_objects.cpp
 * rostolabelimg
 * version: 0.0.1 Majestic Maidenhair
 * Status: pre-alpha
*/

#include "tof_tool/tof_tool_box.h"

#include "sensor_msgs/Image.h"


using namespace std;

TofToolBox *tofToolBox;

static const int DEBUG_main = 0;


int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "rostolabelimg_node");
    ros::NodeHandle n;

    ros::Rate rate(10.0);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);
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