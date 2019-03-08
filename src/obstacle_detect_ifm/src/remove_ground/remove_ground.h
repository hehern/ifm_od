#ifndef OBSTACLE_DETECTION_IFM_REMOVE_GROUND_H
#define OBSTACLE_DETECTION_IFM_REMOVE_GROUND_H

#include <cmath>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include "../types.h"

namespace remove_ground
{
class Remove_ground
{
public:
  Remove_ground(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~Remove_ground(){};
  void GetFilteredPclWithWindow(VPointCloud::Ptr &pcl_in_);
  void RemoveIsolatedPoints(VPointCloud::Ptr &raw_pcl_);
  VPointCloud::Ptr GetFilteredPclWithGrid(VPointCloud::Ptr raw_pcl_);
  VPointCloud::Ptr GetFilteredWithNearestNeighborVariance(VPointCloud::Ptr raw_pcl_);

private:
  int KWindowRows;
  int KWindowCols;
  double height_of_hole_;
  double covariance_threshold_;
  double grid_length_;  //grid size in x
  double grid_width_;   //grid size in y
  double max_ground_height_threshold_;
  double min_dis_neighbour_points_threshold_;
  int min_num_neighbour_points_threshold_;
};
}//namespace remove_ground
#endif