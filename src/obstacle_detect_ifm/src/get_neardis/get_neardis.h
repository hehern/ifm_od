#ifndef OBSTACLE_DETECT_IFM_GET_NEARDIS_H
#define OBSTACLE_DETECT_IFM_GET_NEARDIS_H

#include <ros/ros.h>
#include "../types.h"

namespace get_neardis
{
class Get_neardis
{
public:
  Get_neardis(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~Get_neardis(){};
  float GetNearestDistance(VPointCloud::Ptr pcl_in);
  int CoutNumOfNearestNeighborPoints(const int num_couter, VPointCloud::Ptr &pcl_in);

private:
  int neardis_num_threshold_;
  double neardis_dis_threshold_;
  // Base dimension of vehicle
  double wheelbase_;
  double front_overhang_;
};
}//namespace get_neardis
#endif