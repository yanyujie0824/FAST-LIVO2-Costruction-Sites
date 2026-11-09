#ifndef FUSION_QUALITY_H_
#define FUSION_QUALITY_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

struct FusionQualityState
{
  bool adaptive_weight_enabled = true;
  double q_lidar = 0.85;
  double q_visual = 0.85;
  double w_lidar = 1.0;
  double w_visual = 1.0;

  double lidar_effective_ratio = 0.0;
  double lidar_mean_residual = 0.0;
  double visual_inlier_ratio = 0.0;
  double visual_coverage = 0.0;
  double visual_mean_error = 0.0;
  double lidar_map_sampling_ratio = 1.0;
  double visual_map_score_threshold = 0.0;
  int lidar_points_inserted_last = 0;
  int visual_points_added_last = 0;
};

inline double clampScalar(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

inline double smoothScalar(double previous, double current, double momentum)
{
  return momentum * previous + (1.0 - momentum) * current;
}

void refreshFusionWeights(FusionQualityState &state);
double computeLidarQuality(double effective_ratio, double mean_residual,
                           const Eigen::Matrix3d &translation_information);
double computeVisualQuality(double coverage, double inlier_ratio, double normalized_error,
                            bool use_tracking_error);

#endif // FUSION_QUALITY_H_
