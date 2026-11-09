#include "fusion_quality.h"

#include <Eigen/Eigenvalues>

void refreshFusionWeights(FusionQualityState &state)
{
  if (!state.adaptive_weight_enabled)
  {
    state.w_lidar = 1.0;
    state.w_visual = 1.0;
    return;
  }

  const double q_lidar = clampScalar(state.q_lidar, 0.0, 1.0);
  const double q_visual = clampScalar(state.q_visual, 0.0, 1.0);
  const double quality_sum = q_lidar + q_visual + 1e-6;
  const double relative_lidar = 2.0 * q_lidar / quality_sum;
  const double relative_visual = 2.0 * q_visual / quality_sum;
  const double absolute_lidar = clampScalar(q_lidar, 0.15, 1.0);
  const double absolute_visual = clampScalar(q_visual, 0.15, 1.0);
  state.w_lidar = clampScalar(absolute_lidar * relative_lidar, 0.10, 1.80);
  state.w_visual = clampScalar(absolute_visual * relative_visual, 0.10, 1.80);
}

double computeLidarQuality(double effective_ratio, double mean_residual,
                           const Eigen::Matrix3d &translation_information)
{
  // An eigenvalue ratio also detects weak directions oblique to the world axes.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(translation_information, Eigen::EigenvaluesOnly);
  double balance = 0.0;
  if (solver.info() == Eigen::Success)
  {
    const Eigen::Vector3d &eigenvalues = solver.eigenvalues();
    balance = clampScalar(eigenvalues.x() / (eigenvalues.z() + 1e-6), 0.0, 1.0);
  }

  const double feature_quality = clampScalar(effective_ratio / 0.35, 0.0, 1.0);
  const double residual_quality = std::exp(-mean_residual / 0.08);
  return clampScalar(0.55 * feature_quality + 0.30 * residual_quality + 0.15 * balance, 0.05, 1.0);
}

double computeVisualQuality(double coverage, double inlier_ratio, double normalized_error,
                            bool use_tracking_error)
{
  const double coverage_quality = clampScalar(coverage / 0.12, 0.0, 1.0);
  const double error_quality = std::exp(-4.0 * normalized_error);
  if (use_tracking_error)
  {
    const double inlier_quality = clampScalar(inlier_ratio / 0.60, 0.0, 1.0);
    return clampScalar(0.50 * inlier_quality + 0.30 * coverage_quality + 0.20 * error_quality, 0.05, 1.0);
  }
  return clampScalar(0.70 * coverage_quality + 0.30 * error_quality, 0.05, 1.0);
}
