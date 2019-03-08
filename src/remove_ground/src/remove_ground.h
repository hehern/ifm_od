#ifndef REMOVE_GROUND_H
#define REMOVE_GROUND_H

#include <cmath>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
struct PointXYZIR
{
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))
typedef PointXYZIR VPoint; 
typedef pcl::PointCloud<VPoint> VPointCloud;
namespace remove_ground
{
class Remove_ground
{
public:
  Remove_ground(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~Remove_ground(){};

private:
  ros::Subscriber subscriber;
  ros::Publisher publisher;
  std::string frame_id;
  int KWindowRows;
  int KWindowCols;
  double height_of_hole_;
  double covariance_threshold_;
  double grid_length_;  //grid size in x
  double grid_width_;   //grid size in y
  double max_ground_height_threshold_;
  double min_dis_neighbour_points_threshold_;
  int min_num_neighbour_points_threshold_;
  void pcl_callback(const VPointCloud::ConstPtr &rec);
  void GetFilteredPclWithWindow(VPointCloud::Ptr &pcl_in_);
  void RemoveIsolatedPoints(VPointCloud::Ptr &raw_pcl_);
  VPointCloud::Ptr GetFilteredPclWithGrid(VPointCloud::Ptr raw_pcl_);
  VPointCloud::Ptr GetFilteredWithNearestNeighborVariance(VPointCloud::Ptr raw_pcl_);
  template <typename T>
  T X2(const T &x) {return x * x;}
};
}//namespace remove_ground
#endif