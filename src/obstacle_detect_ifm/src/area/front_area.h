/**
  *author: Jianlin Zhang
  *email: zhang_jianlin@outlook.com
  *date: 20181111
  **/

#ifndef AREA_FRONT_AREA_H_
#define AREA_FRONT_AREA_H_

#include <cmath>

namespace area {
// Predicts the area vehicle will pass.
class FrontArea {
 public:
  FrontArea() :
    wheelbase_(2.0), front_overhang_(0.5),
    base_min_x_(3), base_min_y_(-1.5), base_min_z_(0),
    base_max_x_(10.0), base_max_y_(1.5), base_max_z_(2.5),
    length_k_speed_(1.0), length_k_steer_(1.0),
    area_length_(0),
    curvature_vehicle_(0), curvature_inner_(0),
    curvature_outer_(0), curvature_mid_(0),
    num_in_(0), num_threshold_(0), danger_level_(0),
    r_(1), g_(0), b_(0), a_(1) {}
  virtual ~FrontArea() {}
  void Update(const double speed, const double steer);
  bool IsIn(const double x, const double y, const double z) const;
  bool IsDanger() const {return num_in_ > num_threshold_;}
  bool IsTurningLeft() const {return (curvature_vehicle_ > kLittleCurvature);}
  bool IsTurningRight() const {return (curvature_vehicle_ < -kLittleCurvature);}
  bool IsStraight() const {return (!IsTurningLeft()) & (!IsTurningRight());}
  bool IsInInnerCircle(const double x, const double y) const {
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    return X2(curvature_inner_) * (X2(x - center_x) + X2(y - center_y)) < 1;
  }
  bool IsInOuterCircle(const double x, const double y) const {
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    return X2(curvature_outer_) * (X2(x - center_x) + X2(y - center_y)) < 1;
  }
  bool IsInMidCircle(const double x, const double y) const {
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    return X2(curvature_mid_) * (X2(x - center_x) + X2(y - center_y)) < 1;
  }
  double GetCenterX() const {return 0;}
  double GetCenterY() const {
    return sqrt(1 - X2(curvature_vehicle_ * wheelbase_)) /
           curvature_vehicle_;
  }
  double GetOuterArcAngle() const {return area_length_ * curvature_outer_;}
  double GetOuterArcStartX() const {return base_min_x_;}
  double GetOuterArcStartY() const {
    return curvature_vehicle_ > 0 ? base_min_y_ : base_max_y_;
  }
  double GetMidArcStartX() const {return base_min_x_;}
  double GetMidArcStartY() const {
    return curvature_vehicle_ > 0 ? base_max_y_ : base_min_y_;
  }
  double GetOuterArcEndX() const {
    const double theta = GetOuterArcAngle();
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    const double start_x = GetOuterArcStartX();
    const double start_y = GetOuterArcStartY();
    return center_x + (start_x - center_x) * cos(theta) -
           (start_y - center_y) * sin(theta);
  }
  double GetOuterArcEndY() const {
    const double theta = GetOuterArcAngle();
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    const double start_x = GetOuterArcStartX();
    const double start_y = GetOuterArcStartY();
    return center_y + (start_y - center_y) * cos(theta) +
           (start_x - center_x) * sin(theta);;
  }
  double GetMidArcEndX() const {
    if (!IsStraight()) {
      const double outer_end_x = GetOuterArcEndX();
      const double center_x = GetCenterX();
      const double k = curvature_outer_ / curvature_mid_;
      return center_x + k * (outer_end_x - center_x);
    } else {
      return base_min_x_ + area_length_;
    }
  }
  double GetMidArcEndY() const {
    if (!IsStraight()) {
      const double outer_end_y = GetOuterArcEndY();
      const double center_y = GetCenterY();
      const double k = curvature_outer_ / curvature_mid_;
      return center_y + k * (outer_end_y - center_y);
    } else {
      return GetMidArcStartY();
    }
  }
  double GetMidArcAngle() const {
    const double start_x = GetMidArcStartX();
    const double start_y = GetMidArcStartY();
    const double end_x = GetMidArcEndX();
    const double end_y = GetMidArcEndY();
    const double center_x = GetCenterX();
    const double center_y = GetCenterY();
    const double cross = (start_x - center_x) * (end_y - center_y) -
                         (start_y - center_y) * (end_x - center_x);
    const double dot = (start_y - center_y) * (end_y - center_y) +
                       (start_x - center_x) * (end_x - center_x);
    return atan2(cross, dot);
  }
  void SetVehicleDimision(const double wheelbase, const double front_overhang) {
    wheelbase_ = wheelbase;
    front_overhang_ = front_overhang;
  }
  void SetBaseArea(const double x_min, const double x_max,
                   const double y_min, const double y_max,
                   const double z_min, const double z_max) {
    base_min_x_ = x_min;
    base_max_x_ = x_max;
    base_min_y_ = y_min;
    base_max_y_ = y_max;
    base_min_z_ = z_min;
    base_max_z_ = z_max;
  }
  void GetBaseArea(double *x_min, double *x_max,
                   double *y_min, double *y_max,
                   double *z_min, double *z_max) const {
    *x_min = base_min_x_;
    *x_max = base_max_x_;
    *y_min = base_min_y_;
    *y_max = base_max_y_;
    *z_min = base_min_z_;
    *z_max = base_max_z_;
  }
  void SetK(const double length_k_speed, const double length_k_steer) {
    length_k_speed_ = length_k_speed;
    length_k_steer_ = length_k_steer;
  }
  void SetColor(const double r, const double g, const double b, const double a) {
    r_ = r;
    g_ = g;
    b_ = b;
    a_ = a;
  }
  template<typename ColorT>
  void GetColor(const ColorT &p_color) const {
    p_color->r = r_;
    p_color->g = g_;
    p_color->b = b_;
    p_color->a = a_;
  }

 private:
  template<typename T>
  T X2(const T &in) const {return in * in;}
  // Base dimension of vehicle
  double wheelbase_;
  double front_overhang_;
  // Base area
  double base_min_x_;
  double base_min_y_;
  double base_min_z_;
  double base_max_x_;
  double base_max_y_;
  double base_max_z_;
  // Morphing parameters
  double length_k_speed_;
  double length_k_steer_;
  // Contour when running
  double area_length_;
  double curvature_vehicle_;
  double curvature_inner_;
  double curvature_outer_;
  double curvature_mid_;
  static constexpr double kLittleCurvature = 1e-6;
  // Number of points in area
  std::size_t num_in_;
  std::size_t num_threshold_;
  unsigned int danger_level_;
  // Color
  double r_;
  double g_;
  double b_;
  double a_;

  friend class FrontAreaRos;
};
}
#endif