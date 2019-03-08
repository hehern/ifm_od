/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#include "area/front_area.h"

namespace area {
// Computes radius and update area length
// Assumption: the turning center is on the rear axis
// R_inner=R_v*cos(steer)-width/2
// R_mid=sqrt(R_in^2+min_x^2)
// R_outer=sqrt((R_v*cos(steer)+width/2)^2+min_x^2)
// Maybe it is better to let R_mid=R_outer-width_v
void FrontArea::Update(const double speed, const double steer) {
  curvature_vehicle_ = sin(steer) / wheelbase_;
  if (IsTurningLeft()) {
    double cos_steer = cos(steer);
    curvature_inner_ = curvature_vehicle_ /
                       (cos_steer - curvature_vehicle_ * base_max_y_);
    // Another method: R_mid=R_outer-width
    // Current method: the start of inner arc is fixed
    curvature_mid_ = curvature_inner_ /
                     sqrt(1 + X2(curvature_inner_ * base_min_x_));
    if (curvature_inner_ * base_min_x_ > 1) {
      curvature_inner_ = 1 / base_min_x_;
    }
    double temp = X2(cos_steer + curvature_vehicle_ * base_max_y_);
    temp += X2(curvature_vehicle_ * base_min_x_);
    curvature_outer_ = curvature_vehicle_ / sqrt(temp);
  } else if (IsTurningRight()) {
    double cos_steer = cos(steer);
    curvature_inner_ = curvature_vehicle_ /
                       (cos_steer - curvature_vehicle_ * base_min_y_);
    curvature_mid_ = curvature_inner_ /
                     sqrt(1 + X2(curvature_inner_ * base_min_x_));
    if (-curvature_inner_ * base_min_x_ > 1) {
      curvature_inner_ = -1 / base_min_x_;
    }
    double temp = X2(cos_steer - curvature_vehicle_ * base_max_y_);
    temp += X2(curvature_vehicle_ * base_min_x_);
    curvature_outer_ = curvature_vehicle_ / sqrt(temp);
  } else {
    curvature_inner_ = 0;
    curvature_outer_ = 0;
  }
  area_length_ = base_max_x_ - base_min_x_ +
                 length_k_speed_ * speed * speed +
                 length_k_steer_ * std::abs(steer) * 180 / M_PI;
  if (area_length_ < 0.7 * (base_max_x_ - base_min_x_)) {
    area_length_ = 0.7 * (base_max_x_ - base_min_x_);
  }
  if (area_length_ > 1.5 * (base_max_x_ - base_min_x_)) {
    area_length_ = 1.5 * (base_max_x_ - base_min_x_);
  }
}
bool FrontArea::IsIn(const double x, const double y, const double z) const {
  if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
    if ((z < base_max_z_) && (z > base_min_z_) && (x > base_min_x_)) {
      if (IsStraight()) {
        if (x > base_min_x_ + area_length_) {
          return false;
        } else if ((y > base_max_y_) || (y < base_min_y_)) {
          return false;
        } else {
          return true;
        }
      } else {
        if (IsInOuterCircle(x, y) && (!IsInMidCircle(x, y))) {
          double center_x = GetCenterX();
          double center_y = GetCenterY();
          double end_x = GetOuterArcEndX();
          double end_y = GetOuterArcEndY();
          double D = center_y * end_x - center_x * end_y;
          double A = (end_y - center_y) / D;
          double B = (center_x - end_x) / D;
          if (A * x + B * y + 1 < 0) {
            return false;
          } else {
            return true;
          }
        } else {
          return false;
        }
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}
}
