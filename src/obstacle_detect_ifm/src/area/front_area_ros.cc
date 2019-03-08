/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#include "area/front_area_ros.h"

namespace area {
void FrontAreaRos::GetParameters(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &pnh,
                                 const std::string &name) {
  name_ = name;
  pnh.param("vehicle_frame_id", vehicle_frame_id_, std::string("vehicle"));
  double wheelbase;
  double front_overhang;
  pnh.param("wheelbase", wheelbase, 2.84);
  pnh.param("front_overhang", front_overhang, 1.1);
  // Read all area name
  if (pnh.hasParam("area_list")) {
    pnh.getParam("area_list", area_list_);
    if (area_list_.empty()) {
      ROS_INFO("No any area is configured");
    } else {
      for (auto& area_name : area_list_) {
        data_.insert(std::pair<std::string, FrontArea>(area_name, FrontArea()));
      }
      double base_length;
      double base_width;
      int shifts = 0;
      for (auto& area_name : area_list_) {
        FrontArea &area = data_.at(area_name);
        area.wheelbase_ = wheelbase;
        area.front_overhang_ = front_overhang;
        pnh.param(area_name + "_" + "base_length", base_length, 3.0);
        area.base_min_x_ = wheelbase + front_overhang;
        area.base_max_x_ = wheelbase + front_overhang + base_length;
        pnh.param(area_name + "_" + "base_width", base_width, 1.5);
        area.base_min_y_ = -base_width / 2;
        area.base_max_y_ = base_width / 2;
        pnh.param(area_name + "_" + "min_z", area.base_min_z_, -0.1);
        pnh.param(area_name + "_" + "max_z", area.base_max_z_, 2.5);
        pnh.param(area_name + "_" + "length_k_speed", area.length_k_speed_, 1.0);
        pnh.param(area_name + "_" + "length_k_steer", area.length_k_steer_, 1.0);
        int num;
        pnh.param(area_name + "_" + "num_threshold", num, 10);
        area.num_threshold_ = num;
        area.danger_level_ = 1 << shifts;
        ++shifts;
        pnh.param(area_name + "_" + "color_r", area.r_, 0.0);
        pnh.param(area_name + "_" + "color_g", area.g_, 1.0);
        pnh.param(area_name + "_" + "color_b", area.b_, 0.0);
        pnh.param(area_name + "_" + "color_a", area.a_, 1.0);
        area.Update(0, 0);
      }
      is_initialized_ = true;
    }
  } else {
    ROS_INFO("No parameter named: area_list");
  }
}
void FrontAreaRos::Input(const double &speed, const double &steer) {
  if (is_initialized_) {
    for (auto& area_name : area_list_) {
      FrontArea &area = data_.at(area_name);
      area.Update(speed, steer);
    }
  } else {
    ROS_INFO("%s should be initialized before using.", name_.c_str());
  }
}
void FrontAreaRos::Draw(const ros::Publisher &area_draw_publisher)const {
  visualization_msgs::Marker::Ptr line_list(new visualization_msgs::Marker);
  line_list->header.frame_id = vehicle_frame_id_;
  line_list->header.stamp = ros::Time::now();
  line_list->ns = vehicle_frame_id_;
  line_list->action = visualization_msgs::Marker::ADD;
  line_list->pose.orientation.w = 1.0;
  line_list->id = 0;
  line_list->type = visualization_msgs::Marker::LINE_LIST;
  line_list->scale.x = 0.05;
  line_list->scale.y = 0.05;
  line_list->scale.z = 0.05;
  // Configs contour color here
  if (total_danger_level_ == 0) {
    line_list->color.r = 0;
    line_list->color.g = 1.0;
    line_list->color.b = 0;
    line_list->color.a = 0.5;
  } else {
    for (const auto& area_name : area_list_) {
      const FrontArea &area = data_.at(area_name);
      if (total_danger_level_ & area.danger_level_) {
        line_list->color.r = area.r_;
        line_list->color.g = area.g_;
        line_list->color.b = area.b_;
        line_list->color.a = area.a_;
        break;
      }
    }
  }

  for (const auto& area_name : area_list_) {
    const FrontArea &area = data_.at(area_name);
    if (area.IsStraight()) {
      base::DrawCuboid(line_list,
                       area.base_min_x_, area.base_min_y_, area.base_min_z_,
                       area.base_min_x_ + area.area_length_, area.base_max_y_,
                       area.base_max_z_);
    } else {
      double center_x = area.GetCenterX();
      double center_y = area.GetCenterY();
      double outer_arc_theta = area.GetOuterArcAngle();
      double outer_arc_start_x = area.GetOuterArcStartX();
      double outer_arc_start_y = area.GetOuterArcStartY();
      double outer_arc_end_x = area.GetOuterArcEndX();
      double outer_arc_end_y = area.GetOuterArcEndY();
      double mid_arc_theta = area.GetMidArcAngle();
      double mid_arc_start_x = area.GetMidArcStartX();
      double mid_arc_start_y = area.GetMidArcStartY();
      double mid_arc_end_x = area.GetMidArcEndX();
      double mid_arc_end_y = area.GetMidArcEndY();
      base::DrawSquare(line_list,
                       outer_arc_start_x, outer_arc_start_y, area.base_min_z_,
                       mid_arc_start_x, mid_arc_start_y, area.base_max_z_);
      base::DrawSquare(line_list,
                       outer_arc_end_x, outer_arc_end_y, area.base_min_z_,
                       mid_arc_end_x, mid_arc_end_y, area.base_max_z_);
      base::DrawArc(line_list, center_x, center_y,
                    outer_arc_start_x, outer_arc_start_y, area.base_min_z_,
                    outer_arc_theta, kNumSegmentsToDrawArc);
      base::DrawArc(line_list, center_x, center_y,
                    outer_arc_start_x, outer_arc_start_y, area.base_max_z_,
                    outer_arc_theta, kNumSegmentsToDrawArc);
      base::DrawArc(line_list, center_x, center_y,
                    mid_arc_start_x, mid_arc_start_y, area.base_min_z_,
                    mid_arc_theta, kNumSegmentsToDrawArc);
      base::DrawArc(line_list, center_x, center_y,
                    mid_arc_start_x, mid_arc_start_y, area.base_max_z_,
                    mid_arc_theta, kNumSegmentsToDrawArc);
    }
  }
  area_draw_publisher.publish(line_list);
}
}
