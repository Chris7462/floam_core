// local header
#include "floam_core/lidar_optimization.hpp"


namespace floam_core
{

void getTransformFromSe3(
  const Eigen::Matrix<double, 6, 1> & se3,
  Eigen::Quaterniond & q,
  Eigen::Vector3d & t)
{
  Eigen::Vector3d omega(se3.data());
  Eigen::Vector3d upsilon(se3.data() + 3);

  double theta = omega.norm();
  Eigen::Matrix3d Omega = skew(omega);

  Eigen::Matrix3d R;
  Eigen::Matrix3d V;

  if (theta < 1e-10) {
    R = Eigen::Matrix3d::Identity() + Omega;
    V = R;
  } else {
    Eigen::Matrix3d Omega2 = Omega * Omega;
    R = Eigen::Matrix3d::Identity() +
      std::sin(theta) / theta * Omega +
      (1.0 - std::cos(theta)) / (theta * theta) * Omega2;
    V = Eigen::Matrix3d::Identity() +
      (1.0 - std::cos(theta)) / (theta * theta) * Omega +
      (theta - std::sin(theta)) / (theta * theta * theta) * Omega2;
  }

  q = Eigen::Quaterniond(R);
  t = V * upsilon;
}

Eigen::Matrix3d skew(const Eigen::Vector3d & mat_in)
{
  Eigen::Matrix3d skew_mat;
  skew_mat.setZero();
  skew_mat(0, 1) = -mat_in(2);
  skew_mat(0, 2) = mat_in(1);
  skew_mat(1, 0) = mat_in(2);
  skew_mat(1, 2) = -mat_in(0);
  skew_mat(2, 0) = -mat_in(1);
  skew_mat(2, 1) = mat_in(0);
  return skew_mat;
}


// EdgeAnalyticCostFunction

EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(
  Eigen::Vector3d curr_point,
  Eigen::Vector3d last_point_a,
  Eigen::Vector3d last_point_b)
: curr_point_(curr_point),
  last_point_a_(last_point_a),
  last_point_b_(last_point_b)
{
}

bool EdgeAnalyticCostFunction::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
  Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);

  Eigen::Vector3d lp = q_last_curr * curr_point_ + t_last_curr;
  Eigen::Vector3d nu = (lp - last_point_a_).cross(lp - last_point_b_);
  Eigen::Vector3d de = last_point_a_ - last_point_b_;
  double de_norm = de.norm();

  residuals[0] = nu.norm() / de_norm;

  if (jacobians != nullptr && jacobians[0] != nullptr) {
    Eigen::Matrix3d skew_lp = skew(lp);
    Eigen::Matrix<double, 3, 6> dp_by_se3;
    dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
    dp_by_se3.block<3, 3>(0, 3).setIdentity();

    Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
    J_se3.setZero();

    Eigen::Matrix3d skew_de = skew(de);
    J_se3.block<1, 6>(0, 0) =
      -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
  }

  return true;
}


// SurfNormAnalyticCostFunction

SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(
  Eigen::Vector3d curr_point,
  Eigen::Vector3d plane_unit_norm,
  double negative_OA_dot_norm)
: curr_point_(curr_point),
  plane_unit_norm_(plane_unit_norm),
  negative_OA_dot_norm_(negative_OA_dot_norm)
{
}

bool SurfNormAnalyticCostFunction::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
  Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);

  Eigen::Vector3d lp = q_last_curr * curr_point_ + t_last_curr;

  residuals[0] = plane_unit_norm_.dot(lp) + negative_OA_dot_norm_;

  if (jacobians != nullptr && jacobians[0] != nullptr) {
    Eigen::Matrix3d skew_lp = skew(lp);
    Eigen::Matrix<double, 3, 6> dp_by_se3;
    dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
    dp_by_se3.block<3, 3>(0, 3).setIdentity();

    Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
    J_se3.setZero();

    J_se3.block<1, 6>(0, 0) = plane_unit_norm_.transpose() * dp_by_se3;
  }

  return true;
}


// PoseSE3Manifold

bool PoseSE3Manifold::Plus(
  const double * x,
  const double * delta,
  double * x_plus_delta) const
{
  Eigen::Map<const Eigen::Vector3d> trans(x + 4);

  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_t;
  getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), delta_q, delta_t);

  Eigen::Map<const Eigen::Quaterniond> quater(x);
  Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
  Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

  quater_plus = (delta_q * quater).normalized();
  trans_plus = delta_q * trans + delta_t;

  return true;
}

bool PoseSE3Manifold::PlusJacobian(
  const double * /*x*/,
  double * jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows(6).setIdentity();
  j.bottomRows(1).setZero();
  return true;
}

bool PoseSE3Manifold::Minus(
  const double * y,
  const double * x,
  double * y_minus_x) const
{
  Eigen::Map<const Eigen::Vector3d> trans(x + 4);

  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_t;
  getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(y_minus_x), delta_q, delta_t);

  Eigen::Map<const Eigen::Quaterniond> quater(x);
  Eigen::Map<const Eigen::Quaterniond> quater_y(y);
  Eigen::Map<Eigen::Quaterniond> quater_minus(y_minus_x);
  Eigen::Map<Eigen::Vector3d> trans_minus(y_minus_x + 4);

  quater_minus = delta_q.inverse() * quater_y;
  trans_minus = delta_q.inverse() * trans - delta_t;

  return true;
}

bool PoseSE3Manifold::MinusJacobian(
  const double * /*x*/,
  double * jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows(6).setIdentity();
  j.bottomRows(1).setZero();
  return true;
}

}  // namespace floam_core
