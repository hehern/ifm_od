#include "top.h"

namespace top {
Top::Top(ros::NodeHandle nh, ros::NodeHandle pnh) : rg_(nh, pnh), clip_(nh, pnh), dis_(nh, pnh)
{
  std::string topic;
  int queue_size;

  pnh.param("ros_default_queue_size", queue_size, 1);
  pnh.param("is_displaying_pcl", is_displaying_pcl_, true);
  pnh.param("map_frame_id", map_frame_id_, std::string("map"));
  pnh.param("vehicle_frame_id", vehicle_frame_id_, std::string("vehicle"));

  area_.GetParameters(nh, pnh, "front_area");

  pnh.param("obstacle_pcl_topic", topic, std::string("obstacle_pcl_ifm"));
  pcl_obstacle_publisher_ = nh.advertise<VPointCloud>(topic, queue_size);

  pnh.param("area_contour_topic", topic, std::string("area_contour_ifm"));
  area_contour_publisher_ = nh.advertise<visualization_msgs::Marker>(topic, queue_size);

  pnh.param("obstacle_in_which_area_topic", topic, std::string("obstacle_in_which_area_ifm"));
  obstacle_in_which_area_publisher_ = nh.advertise<std_msgs::UInt32>(topic, queue_size);

  pnh.param("nearest_dis_topic", topic, std::string("nearest_dis"));
  nearest_dis_publisher_ = nh.advertise<std_msgs::Float32>(topic, queue_size);


  pnh.param("pcl_topic_right", topic, std::string("/IFM/ifm_points_right"));
  pcl_subscriber_ = nh.subscribe(topic, queue_size, &Top::PclCallback, this);

  pnh.param("speed_topic", topic, std::string("speed"));
  speed_subscriber_ = nh.subscribe(topic, queue_size, &Top::SpeedCallback, this);

  pnh.param("steer_topic", topic, std::string("steer"));
  steer_subscriber_ = nh.subscribe(topic, queue_size, &Top::SteerCallback, this);
}

void Top::PclCallback(const VPointCloud::ConstPtr &rec1)
{
  if (rec1->points.size() > 0)
  {
    //clip the pcl
    VPointCloud::Ptr in = clip_.GetPcl(rec1);
    //remove of ground
    VPointCloud::Ptr obstacle_out = rg_.GetFilteredWithNearestNeighborVariance(in);
    //clear the danger level of areas
    area_.ClearDangerLevel();
    for (const auto& p : obstacle_out->points)
    {
      area_.UpdateCountInArea(p.x, p.y, p.z);
    }
    area_.Draw(area_contour_publisher_);

    ROS_INFO("danger level: %x", area_.GetDangerLevel());
    std_msgs::UInt32::Ptr result_out(new std_msgs::UInt32);
    result_out->data = area_.GetDangerLevel();
    obstacle_in_which_area_publisher_.publish(result_out);

    //Only calculate the distance if there are obstacles in the monitoring area
    //the scope of pcl could be optimized
    if(result_out->data > 0)
    {
      std_msgs::Float32::Ptr nearest_dis(new std_msgs::Float32);
      nearest_dis->data = dis_.GetNearestDistance(obstacle_out);
      nearest_dis_publisher_.publish(nearest_dis);
    }

    if (is_displaying_pcl_)
    {
      obstacle_out->header.stamp = pcl_conversions::toPCL(ros::Time::now());
      obstacle_out->header.frame_id = vehicle_frame_id_;
      if (obstacle_out->width > 0)//if there is no point, ros will collapse
      { 
        pcl_obstacle_publisher_.publish(obstacle_out);   //发布障碍物点云信息
      }
    }
  }
  else // no points in this frame, send stop
  {
    area_.SetAllDangerLevelTrue();
    area_.Draw(area_contour_publisher_);
    std_msgs::UInt32::Ptr result_out(new std_msgs::UInt32);
    result_out->data = 0xff;
    obstacle_in_which_area_publisher_.publish(result_out);

    //publish empty pcl
    VPointCloud::Ptr out(new VPointCloud);
    out->width = 0;
    out->height = 0;
    if (is_displaying_pcl_) {
      out->header.stamp = pcl_conversions::toPCL(ros::Time::now());
      out->header.frame_id = vehicle_frame_id_;
      pcl_obstacle_publisher_.publish(out);
    }
  }
}
void Top::SpeedCallback(const std_msgs::Float32::ConstPtr &rec) {
  speed_ = rec->data;
}
void Top::SteerCallback(const std_msgs::Float32::ConstPtr &rec) {
  steer_ = rec->data;
  area_.Input(speed_, steer_);
}
}
