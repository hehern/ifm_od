/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#ifndef OBSTACLE_DETECTION_BASE_DRAW_H_
#define OBSTACLE_DETECTION_BASE_DRAW_H_

#include <visualization_msgs/Marker.h>

namespace base {
void DrawCuboid(visualization_msgs::Marker::Ptr line_list,
                double p1_x, double p1_y, double p1_z,
                double p7_x, double p7_y, double p7_z);
void DrawPlane(visualization_msgs::Marker::Ptr plane,
               double a, double b, double c, double d,
               double x_min, double x_max,
               double y_min, double y_max);
void DrawSquare(visualization_msgs::Marker::Ptr line_list,
                 double p1_x, double p1_y, double p1_z,
                 double p3_x, double p3_y, double p3_z);
void DrawArc(visualization_msgs::Marker::Ptr line_list,
             double center_x, double center_y,
             double p_x, double p_y,
             double height, double theta, int segment);
void DrawArrow(visualization_msgs::Marker::Ptr arrows,
               double a, double b, double c,
               double start_x, double start_y, double start_z,
               double norm);
void DrawGrid(visualization_msgs::Marker::Ptr line_list,
              double x_min, double x_max,
              double y_min, double y_max,
              size_t row, size_t column);
void DrawConeSurface(visualization_msgs::Marker::Ptr trangle_list,
                     double pitch,
                     double length,
                     double theta_min,
                     double theta_max,
                     size_t segment = 8);
}
#endif
