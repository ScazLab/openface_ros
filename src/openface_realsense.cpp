#include "openface_ros/openface_ros.h"
#include <ros/ros.h>

#define fx 615.6707153320312
#define fy 615.962158203125
#define cx 328.0010681152344
#define cy 241.31031799316406
#define threshold 0.30

int main(int argc, char** argv)
{
    ros::init(argc, argv, "openface_realsense", ros::init_options::AnonymousName);
    // OpenFaceRos openFaceRos("openface_realsense", fx, fy, cx, cy, threshold, false);
    
    ros::NodeHandle nh("~");
    std::string name;
    nh.getParam("camera_device_name", name);

    OpenFaceRos openFaceRos(name, fx, fy, cx, cy, threshold, false);

    ros::spin();
    return 0;

}