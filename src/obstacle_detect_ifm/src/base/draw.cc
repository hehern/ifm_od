/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#include "base/draw.h"

namespace base {
void DrawCuboid(visualization_msgs::Marker::Ptr line_list,
                double p1_x, double p1_y, double p1_z,
                double p7_x, double p7_y, double p7_z) {
  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

  p1.x = p1_x;
  p1.y = p1_y;
  p1.z = p1_z;
  p7.x = p7_x;
  p7.y = p7_y;
  p7.z = p7_z;
  p2.x = p7.x;
  p2.y = p1.y;
  p2.z = p1.z;
  p3.x = p7.x;
  p3.y = p7.y;
  p3.z = p1.z;
  p4.x = p1.x;
  p4.y = p7.y;
  p4.z = p1.z;
  p5.x = p1.x;
  p5.y = p1.y;
  p5.z = p7.z;
  p6.x = p7.x;
  p6.y = p1.y;
  p6.z = p7.z;
  p8.x = p1.x;
  p8.y = p7.y;
  p8.z = p7.z;

  line_list->points.push_back(p1);
  line_list->points.push_back(p2);
  line_list->points.push_back(p1);
  line_list->points.push_back(p4);
  line_list->points.push_back(p1);
  line_list->points.push_back(p5);
  line_list->points.push_back(p7);
  line_list->points.push_back(p3);
  line_list->points.push_back(p7);
  line_list->points.push_back(p6);
  line_list->points.push_back(p7);
  line_list->points.push_back(p8);
  line_list->points.push_back(p2);
  line_list->points.push_back(p3);
  line_list->points.push_back(p2);
  line_list->points.push_back(p6);
  line_list->points.push_back(p5);
  line_list->points.push_back(p6);
  line_list->points.push_back(p5);
  line_list->points.push_back(p8);
  line_list->points.push_back(p4);
  line_list->points.push_back(p3);
  line_list->points.push_back(p4);
  line_list->points.push_back(p8);
} //void DrawCuboid(visualization_msgs::Marker::Ptr line_list,...

void DrawSquare(visualization_msgs::Marker::Ptr line_list,
                 double p1_x, double p1_y, double p1_z,
                 double p3_x, double p3_y, double p3_z) {
  geometry_msgs::Point p1, p2, p3, p4;

  p1.x = p1_x;
  p1.y = p1_y;
  p1.z = p1_z;
  p3.x = p3_x;
  p3.y = p3_y;
  p3.z = p3_z;
  p2.x = p3_x;
  p2.y = p3_y;
  p2.z = p1_z;
  p4.x = p1_x;
  p4.y = p1_y;
  p4.z = p3_z;
  line_list->points.push_back(p1);
  line_list->points.push_back(p2);
  line_list->points.push_back(p1);
  line_list->points.push_back(p4);
  line_list->points.push_back(p3);
  line_list->points.push_back(p2);
  line_list->points.push_back(p3);
  line_list->points.push_back(p4);
} //void DrawSquare(visualization_msgs::Marker::Ptr line_list,...

void DrawArc(visualization_msgs::Marker::Ptr line_list,
             double center_x, double center_y,
             double p_x, double p_y,
             double height, double theta, int segment) {
  geometry_msgs::Point p1, p2;

  p1.x = p_x;
  p1.y = p_y;
  p1.z = height;
  p2.z = height;
  double delta = theta / (double)segment;
  for (int i = 0; i < segment; i++) {
    p2.x = center_x + (p1.x - center_x) * cos(delta) - (p1.y - center_y) * sin(delta);
    p2.y = center_y + (p1.y - center_y) * cos(delta) + (p1.x - center_x) * sin(delta);
    line_list->points.push_back(p1);
    line_list->points.push_back(p2);
    p1 = p2;
  }
} //void DrawArc(visualization_msgs::Marker::Ptr line_list,...

void DrawPlane(visualization_msgs::Marker::Ptr plane,
               double a, double b, double c, double d,
               double x_min, double x_max,
               double y_min, double y_max) {
  geometry_msgs::Point p1, p2, p3, p4;

  if (c == 0) {
    return;
  }
  double A = -a / c;
  double B = -b / c;
  double C = -d / c;
  p1.x = x_min;
  p1.y = y_min;
  p1.z = A * p1.x + B * p1.y + C;
  p2.x = x_max;
  p2.y = y_min;
  p2.z = A * p2.x + B * p2.y + C;
  p3.x = x_max;
  p3.y = y_max;
  p3.z = A * p3.x + B * p3.y + C;
  p4.x = x_min;
  p4.y = y_max;
  p4.z = A * p4.x + B * p4.y + C;
  plane->points.push_back(p1);
  plane->points.push_back(p2);
  plane->points.push_back(p3);
  plane->points.push_back(p1);
  plane->points.push_back(p3);
  plane->points.push_back(p4);
} //void DrawPlane(visualization_msgs::Marker::Ptr plane,...

void DrawArrow(visualization_msgs::Marker::Ptr arrows,
               double a, double b, double c,
               double start_x, double start_y, double start_z,
               double norm) {
  geometry_msgs::Point start, end;
  start.x = start_x;
  start.y = start_y;
  start.z = start_z;
  end.x = start_x + a * norm;
  end.y = start_y + b * norm;
  end.z = start_z + c * norm;
  arrows->points.push_back(start);
  arrows->points.push_back(end);
} //void DrawArrow(visualization_msgs::Marker::Ptr arrows,

void DrawGrid(visualization_msgs::Marker::Ptr line_list,
              double x_min, double x_max,
              double y_min, double y_max,
              size_t row, size_t column) {
  geometry_msgs::Point p1, p2;
  if (row > 0) {
    double delta_x = (x_max - x_min) / row;
    p1.x = x_min;
    p1.y = y_min;
    p1.z = 0;
    p2.x = p1.x;
    p2.y = y_max;
    p2.z = 0;
    for (size_t i = 0; i <= row; i++) {
      line_list->points.push_back(p1);
      line_list->points.push_back(p2);
      p1.x += delta_x;
      p2.x = p1.x;
    }
  }
  if (column > 0) {
    double delta_y = (y_max - y_min) / column;
    p1.x = x_min;
    p1.y = y_min;
    p1.z = 0;
    p2.x = x_max;
    p2.y = p1.y;
    p2.z = 0;
    for (size_t i = 0; i <= row; i++) {
      line_list->points.push_back(p1);
      line_list->points.push_back(p2);
      p1.y += delta_y;
      p2.y = p1.y;
    }
  }
} //void DrawGrid(visualization_msgs::Marker::Ptr line_list,

void DrawConeSurface(visualization_msgs::Marker::Ptr trangle_list,  //draw cone with many trangle
                     double pitch,
                     double length,
                     double theta_min,
                     double theta_max,
                     size_t segment) {
  geometry_msgs::Point p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  if (theta_min > theta_max) {
    std::swap(theta_min, theta_max);
  }
  if (segment < 1) {
    segment = 1;
  }
  double theta_step = (theta_max - theta_min) / segment;
  for (size_t i = 0; i < segment; ++i) {
    double theta_start = theta_min + i * theta_step;
    double theta_end = theta_start + theta_step;
    geometry_msgs::Point p2, p3;
    p2.z = length * sin(pitch);
    p3.z = length * sin(pitch);
    double length_xy = length * cos(pitch);
    p2.x = length_xy * cos(theta_start);
    p2.y = length_xy * sin(theta_start);
    p3.x = length_xy * cos(theta_end);
    p3.y = length_xy * sin(theta_end);
    trangle_list->points.push_back(p1);
    trangle_list->points.push_back(p2);
    trangle_list->points.push_back(p3);
  }
} //void DrawConeSurface(visualization_msgs::Marker::Ptr trangle_list,  //draw cone with many trangle

}
