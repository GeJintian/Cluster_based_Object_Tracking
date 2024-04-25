#ifndef DETECTION_FRENET_H
#define DETECTION_FRENET_H

#include <memory>
#include <vector>
// clang-format off
#include <stdexcept>
#include <boost/math/interpolators/cardinal_quintic_b_spline.hpp>
// clang-format on
#include <iostream>

namespace detection_helper{

float wrapValue(float value, float range);
float wrapAngle(float angle);

struct CartesianPoint {
  float x{0};
  float y{0};
};

struct FrenetPoint {
  float s{0};
  float d{0};
};

struct Reference {
  explicit Reference(const std::vector<CartesianPoint>& points);

  float x(float s) const;

  float y(float s) const;

  float tangent(float s) const;

  float normal(float s) const;

  float curvature(float s) const;


  CartesianPoint xy(const FrenetPoint& fp) const;

  FrenetPoint sd(const CartesianPoint& cp, float s_guess) const;

  const float& getLength() const;

 private:
  float length;
  float spacing;

  const std::vector<CartesianPoint> pts;

  using cqbs = boost::math::interpolators::cardinal_quintic_b_spline<float>;

  std::unique_ptr<cqbs> x_spline;
  std::unique_ptr<cqbs> y_spline;
  std::unique_ptr<cqbs> t_spline;
  std::unique_ptr<cqbs> n_spline;
  std::unique_ptr<cqbs> k_spline;
};
}

#endif