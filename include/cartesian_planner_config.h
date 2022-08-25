#pragma once
#include <tuple>
#include <cmath>

namespace cartesian_planner {

class VehicleParam {
public:
  /**
   * L_F, front hang length of the ego vehicle (m)
   */
  double front_hang_length = 0.96;

  /**
   * L_W, wheelbase of the ego vehicle (m)
   */
  double wheel_base = 2.80;

  /**
   * L_R, rear hang length of the ego vehicle (m)
   */
  double rear_hang_length = 0.929;

  /**
   * L_B, width of the ego vehicle (m)
   */
  double width = 1.942;

  /**
   * Upper bound of v(t) (m/s)
   */
  double max_velocity = 8.0;

  /**
   * Lower and upper bounds of a(t) (m/s^2)
   */
  double min_acceleration = -1.0, max_acceleration = 1.0;

  /**
   * Upper bound of |jerk(t)| (m/s^3)
   */
  double jerk_max = 2.0;

  /**
   * Upper bound of |\phi(t)| (rad)
   */
  double phi_max = 0.85;

  /**
   * Upper bound of |\omega(t)| (rad/s)
   */
  double omega_max = 1.5;

  double radius;
  double f2x, r2x;

  VehicleParam() {
    double length = (wheel_base + rear_hang_length + front_hang_length);
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - rear_hang_length;
    f2x = 0.75 * length - rear_hang_length;
  }

  template<class T>
  std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }
};

struct CartesianPlannerConfig {
  /**
   * Number of finite elements used to discretize an OCP
   */
  int nfe = 320;

  /**
   * Time horizon length (s)
   */
  //double tf = 16;
  double tf = 28;

  /**
   * Weighting parameter in Eq.(2)
   */
  double opti_w_u = 5000;

  /**
   * weighting parameter in Eq.(3)
   */
  double opti_w_r_theta = 20.0;

  /**
   * weighting parameter in Eq.(4)
   */
  double opti_w_rw = 5000.0;

  /**
   * Maximum iteration number in Alg.1
   */
  int opti_iter_max = 10;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e5;

  /**
   * Multiplier to enlarge w_penalty during the iterations
   */
  double opti_alpha = 10;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-6;
  //double opti_varepsilon_tol = 1e-8;

  VehicleParam vehicle;
};

}
