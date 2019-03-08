/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#ifndef OBSTACLE_DETECTION_AREA_FRONT_AREA_ROS_H_
#define OBSTACLE_DETECTION_AREA_FRONT_AREA_ROS_H_

#include "area/front_area.h"
#include "base/draw.h"

#include <vector>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace area {
class FrontAreaRos {
 public:
  FrontAreaRos(): is_initialized_(false), name_("FrontAreaRos") {}
  virtual ~FrontAreaRos() {}
  void GetParameters(const ros::NodeHandle &nh,
                     const ros::NodeHandle &pnh,
                     const std::string &name);
  void Input(const double &speed, const double &steer);
  void Draw(const ros::Publisher &area_draw_publisher)const ;
  void UpdateCountInArea(const double &x, const double &y, const double &z) {
    for (auto& id : area_list_) {
      FrontArea& area = data_.at(id);
      if (!area.IsDanger()) {
        if (area.IsIn(x, y, z)) {
          ++area.num_in_;
        }
      } else {
        total_danger_level_ |= area.danger_level_;
      }
    }
  }
  void ClearDangerLevel() {
    total_danger_level_ = 0;
    for (auto& id : area_list_) {
      FrontArea& area = data_.at(id);
      area.num_in_ = 0;
    }
  }
  unsigned int GetDangerLevel()const {return total_danger_level_;}
  void SetAllDangerLevelTrue() {total_danger_level_ = 0xffffffff;}
 private:
  bool is_initialized_;
  std::string name_;
  std::string vehicle_frame_id_;
  std::unordered_map<std::string, FrontArea> data_;
  std::vector<std::string> area_list_;
  unsigned int total_danger_level_;
  static constexpr int kNumSegmentsToDrawArc = 8;
};
}
#endif
