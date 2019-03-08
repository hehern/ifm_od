#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>

#include "socketcan.h"

namespace Can_bridge
{
class Can_bridge
{
public:
  Can_bridge(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~Can_bridge(){};

private:
  ros::Subscriber yaw_rate_sub, speed_sub;
  //struct can_frame yaw_rate_frame, speed_frame, direction_frame;
  Socketcan socketcan;
  void yawrate_sent_callback(const sensor_msgs::Imu::ConstPtr &rec);
  void speed_sent_callback(const socket_can::SpeedMilSteer::ConstPtr &rec);
};
}//namespace Can_bridge
#endif