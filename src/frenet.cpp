#include "detection/frenet.hpp"
#include <cassert>
#include <iostream>
#include <utility>


namespace detection_helper{
float wrapValue(float value, float range) {
  return value - range * std::floor(value / range);
}

float wrapAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::pair<float, float> calcJointDerivatives(const std::vector<float>& y,
                                             float spacing) {
  std::pair<float, float> ders;

  const auto last_idx = y.size() - 1;

  float tmp = -y[last_idx - 3] / 60 + 3 * y[last_idx - 2] / 20 -
              3 * y[last_idx - 1] / 4 + 3 * y[1] / 4 - 3 * y[2] / 20 +
              y[3] / 60;

  ders.first = tmp / spacing;

  tmp = y[last_idx - 3] / 90 - 3 * y[last_idx - 2] / 20 +
        3 * y[last_idx - 1] / 2 - 49 * y[0] / 18 + 3 * y[1] / 2 -
        3 * y[2] / 20 + y[3] / 90;

  ders.second = tmp / (spacing * spacing);

  return ders;
}

Reference::Reference(const std::vector<CartesianPoint>& points)
    : pts{points} {
  const auto n_pts = pts.size();

  std::vector<float> abscissa(n_pts);
  std::vector<float> x_vec(n_pts);
  std::vector<float> y_vec(n_pts);

  length = 0.0f;

  abscissa[0] = length;
  x_vec[0] = pts[0].x;
  y_vec[0] = pts[0].y;

  for (std::size_t i = 1; i < n_pts; ++i) {
    const float x = pts[i].x;
    const float y = pts[i].y;
    const float Dx = x - pts[i - 1].x;
    const float Dy = y - pts[i - 1].y;
    length += std::sqrt(Dx * Dx + Dy * Dy);
    abscissa[i] = length;
    x_vec[i] = x;
    y_vec[i] = y;
  }

  spacing = length / (n_pts - 1);

  const auto spacing_corr = spacing * (1.0f + 1e-4f / n_pts);

  const auto x_ders = calcJointDerivatives(x_vec, spacing);
  x_spline = std::make_unique<cqbs>(x_vec, 0.0f, spacing_corr, x_ders, x_ders);

  const auto y_ders = calcJointDerivatives(y_vec, spacing);
  y_spline = std::make_unique<cqbs>(y_vec, 0.0f, spacing_corr, y_ders, y_ders);

  std::vector<float> t(n_pts);
  std::vector<float> n(n_pts);
  std::vector<float> k(n_pts);

  float s, dx, dy, ddx, ddy;
  for (std::size_t i = 0; i < n_pts; ++i) {
    s = abscissa[i];
    dx = x_spline->prime(s);
    dy = y_spline->prime(s);
    ddx = x_spline->double_prime(s);
    ddy = y_spline->double_prime(s);

    const float num = dx * ddy - ddx * dy;
    const float den = std::sqrt(std::pow(dx * dx + dy * dy, 3));

    t[i] = std::atan2(dy, dx);
    n[i] = std::atan2(dx, -dy);
    k[i] = num / den;
  }

  auto t_unrolled = t[0];
  auto n_unrolled = n[0];
  for (std::size_t i = 1; i < n_pts; ++i) {
    t_unrolled += wrapAngle(t[i] - t_unrolled);
    t[i] = t_unrolled;
    n_unrolled += wrapAngle(n[i] - n_unrolled);
    n[i] = n_unrolled;
  }

  t_spline = std::make_unique<cqbs>(t, 0.0f, spacing_corr);

  n_spline = std::make_unique<cqbs>(n, 0.0f, spacing_corr);

  k.back() = k.front();
  const auto k_ders = calcJointDerivatives(k, spacing);
  k_spline = std::make_unique<cqbs>(k, 0.0f, spacing_corr, k_ders, k_ders);
}

float Reference::x(float s) const { return (*x_spline)(s); }

float Reference::y(float s) const { return (*y_spline)(s); }

float Reference::tangent(float s) const {
  const auto s_safe = wrapValue(s, length);
  return wrapAngle((*t_spline)(s_safe));
}

float Reference::normal(float s) const {
  const auto s_safe = wrapValue(s, length);
  return wrapAngle((*n_spline)(s_safe));
}

float Reference::curvature(float s) const { return (*k_spline)(s); }


CartesianPoint Reference::xy(const FrenetPoint& fp) const {
  const auto s_safe = wrapValue(fp.s, length);

  const float n = normal(s_safe);

  return {
      .x = x(s_safe) + fp.d * std::cos(n),
      .y = y(s_safe) + fp.d * std::sin(n),
  };
}

FrenetPoint Reference::sd(const CartesianPoint& cp, float s_guess) const {
  static constexpr uint8_t MAX_NEWTON_ITERS = 20;
  const float NEWTON_TOLERANCE = spacing * 0.01f;
  if(s_guess<0) s_guess += length;

  float s_ = wrapValue(s_guess, length);

  CartesianPoint cp_guess = {
      .x = x(s_),
      .y = y(s_),
  };

  auto x_dist = cp.x - x(s_);
  auto y_dist = cp.y - y(s_);
  auto min_dist2 = x_dist * x_dist + y_dist * y_dist;
  if (min_dist2 > 20 * 20) {
    const auto start_idx = static_cast<std::size_t>(s_ / spacing);
    for (std::size_t i = start_idx; i < start_idx + pts.size(); ++i) {
      const auto idx = i % (pts.size() - 1);
      x_dist = cp.x - pts[idx].x;
      y_dist = cp.y - pts[idx].y;
      const auto dist2 = x_dist * x_dist + y_dist * y_dist;
      if (dist2 < min_dist2) {
        min_dist2 = dist2;
        s_ = spacing * static_cast<float>(idx);
      }
    }
  }

  float x, y, dx, dy, ddx, ddy, Dx, Dy;
  for (uint8_t i = 0; i < MAX_NEWTON_ITERS; ++i) {
    x = (*x_spline)(s_);
    y = (*y_spline)(s_);
    dx = x_spline->prime(s_);
    dy = y_spline->prime(s_);
    ddx = x_spline->double_prime(s_);
    ddy = y_spline->double_prime(s_);

    Dx = x - cp.x;
    Dy = y - cp.y;

    const float jac = Dx * dx + Dy * dy;
    const float hes = Dx * ddx + dx * dx + Dy * ddy + dy * dy;

    const float Ds = jac / hes;

    if (std::isnan(Ds) || std::abs(Ds) < NEWTON_TOLERANCE / 2) break;

    s_ = wrapValue(s_ - Ds, length);
  }

  FrenetPoint fp;
  fp.s = s_;
  fp.d = (Dx * dy - Dy * dx) / std::sqrt(dx * dx + dy * dy);

  return fp;
}

const float& Reference::getLength() const { return length; }
}