#pragma once

// ceres header
#include <ceres/ceres.h>

// eigen header
#include <Eigen/Dense>


namespace floam_core
{

void getTransformFromSe3(
  const Eigen::Matrix<double, 6, 1> & se3,
  Eigen::Quaterniond & q,
  Eigen::Vector3d & t);

Eigen::Matrix3d skew(const Eigen::Vector3d & mat_in);


class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
  EdgeAnalyticCostFunction(
    Eigen::Vector3d curr_point,
    Eigen::Vector3d last_point_a,
    Eigen::Vector3d last_point_b);

  virtual ~EdgeAnalyticCostFunction() {}

  virtual bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override;

private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d last_point_a_;
  Eigen::Vector3d last_point_b_;
};


class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
  SurfNormAnalyticCostFunction(
    Eigen::Vector3d curr_point,
    Eigen::Vector3d plane_unit_norm,
    double negative_OA_dot_norm);

  virtual ~SurfNormAnalyticCostFunction() {}

  virtual bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override;

private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d plane_unit_norm_;
  double negative_OA_dot_norm_;
};


class PoseSE3Manifold : public ceres::Manifold
{
public:
  PoseSE3Manifold() {}
  virtual ~PoseSE3Manifold() {}

  virtual bool Plus(
    const double * x,
    const double * delta,
    double * x_plus_delta) const override;

  virtual bool PlusJacobian(
    const double * x,
    double * jacobian) const override;

  virtual bool Minus(
    const double * y,
    const double * x,
    double * y_minus_x) const override;

  virtual bool MinusJacobian(
    const double * x,
    double * jacobian) const override;

  virtual int AmbientSize() const override { return 7; }
  virtual int TangentSize() const override { return 6; }
};

} // namespace floam_core
