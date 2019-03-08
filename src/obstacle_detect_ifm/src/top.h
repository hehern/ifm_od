#ifndef OBSTACLE_DETECTION_IFM_TOP_H_
#define OBSTACLE_DETECTION_IFM_TOP_H_

#include "types.h"
#include "pcl_clip/pcl_clip.h"
#include "remove_ground/remove_ground.h"
#include "area/front_area_ros.h"
#include "get_neardis/get_neardis.h"

#include <chrono>
#include <vector>

#include <can_msgs/Frame.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace top {
class Top {
 public:
  Top(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Top() {};
  using SyncPcl_t =
    message_filters::sync_policies::ApproximateTime<VPointCloud, VPointCloud>;

 private:
  void PclCallback(const VPointCloud::ConstPtr &rec1, const VPointCloud::ConstPtr &rec2);
  void PclCallback(const VPointCloud::ConstPtr &rec1);
  void SpeedCallback(const std_msgs::Float32::ConstPtr &rec);
  void SteerCallback(const std_msgs::Float32::ConstPtr &rec);
  
  pcl_clip::Pcl_clip clip_;
  remove_ground::Remove_ground rg_;
  area::FrontAreaRos area_;
  get_neardis::Get_neardis dis_;

  std::string vehicle_frame_id_;
  std::string map_frame_id_;

  ros::Subscriber speed_subscriber_;
  ros::Subscriber steer_subscriber_;
  ros::Subscriber pcl_subscriber_;

  ros::Publisher pcl_obstacle_publisher_;             //滤出地面之后的点云
  ros::Publisher area_contour_publisher_;             //动态监控区域绘制
  ros::Publisher obstacle_in_which_area_publisher_;   //障碍物所在区域标志位
  ros::Publisher nearest_dis_publisher_;              //最近障碍物点距离发布

  bool is_displaying_pcl_;
  double speed_;
  double steer_;
};
}
#endif
