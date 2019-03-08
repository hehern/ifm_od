#ifndef _CAMERA_POINTS_H_
#define _CAMERA_POINTS_H_
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include "UDPGet.h"
namespace Camera
{
class Camera_points
{
public:
    Camera_points(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~Camera_points(){};
    void pcl_pixel_publish();
    void od_result_publish();
    void calibration_result_publish();
    void logic_publish();
    void marker_publish();
private:
    ros::Publisher pcl_pub;
    ros::Publisher od_result_pub;
    ros::Publisher calibration_pub;
    ros::Publisher logic_pub;
    ros::Publisher marker_pub;
    std::string frame_id;
    double x_offset;
    int port_number;
    std::string devip_str;
    UDPGet udpget;
}; 
}//namespace Camera
#endif