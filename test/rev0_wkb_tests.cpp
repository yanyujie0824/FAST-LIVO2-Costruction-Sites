#include "dynamic_object_filter.h"
#include "fusion_quality.h"

#include <gtest/gtest.h>

namespace
{
PointCloudXYZI::Ptr makeCloud(std::initializer_list<float> distances)
{
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  cloud->header.frame_id = "lidar";
  for (float distance : distances)
  {
    PointType point{};
    point.x = distance;
    point.y = 0.25f;
    point.z = 0.25f;
    cloud->push_back(point);
  }
  return cloud;
}

class DynamicFilterTest : public testing::Test
{
protected:
  void runFilter(DynamicObjectFilter &filter, const PointCloudXYZI::Ptr &cloud)
  {
    filter.filter(StatesGroup(), M3D::Identity(), V3D::Zero(), cloud,
                  static_cloud, dynamic_cloud, candidate_cloud);
  }

  PointCloudXYZI::Ptr static_cloud;
  PointCloudXYZI::Ptr dynamic_cloud;
  PointCloudXYZI::Ptr candidate_cloud;
};

TEST_F(DynamicFilterTest, DisabledFilterPassesInputThrough)
{
  DynamicObjectFilter filter;
  runFilter(filter, makeCloud({2.25f, 6.25f}));
  EXPECT_EQ(static_cloud->size(), 2u);
  EXPECT_EQ(static_cloud->header.frame_id, "lidar");
  EXPECT_TRUE(dynamic_cloud->empty());
  EXPECT_TRUE(candidate_cloud->empty());

  runFilter(filter, makeCloud({}));
  EXPECT_TRUE(static_cloud->empty());
}

TEST_F(DynamicFilterTest, TracksNewClusterAndKeepsBackground)
{
  DynamicObjectFilterConfig config;
  config.enabled = true;
  config.voxel_size = 1.0;
  config.burn_in_frames = 2;
  config.min_cluster_size = 2;
  config.min_track_duration = 1;
  config.max_tracking_distance = 1.25;
  DynamicObjectFilter filter(config);

  runFilter(filter, makeCloud({6.25f}));
  runFilter(filter, makeCloud({6.25f}));
  runFilter(filter, makeCloud({2.25f, 2.35f, 6.25f}));
  EXPECT_EQ(candidate_cloud->size(), 2u);
  EXPECT_TRUE(dynamic_cloud->empty());
  EXPECT_EQ(static_cloud->size(), 3u);

  runFilter(filter, makeCloud({3.25f, 3.35f, 6.25f}));
  ASSERT_EQ(static_cloud->size(), 1u);
  EXPECT_FLOAT_EQ(static_cloud->front().x, 6.25f);
  ASSERT_EQ(dynamic_cloud->size(), 2u);
  EXPECT_FLOAT_EQ(dynamic_cloud->front().x, 3.25f);
  EXPECT_EQ(dynamic_cloud->header.frame_id, "lidar");

  filter.reset();
  runFilter(filter, makeCloud({3.25f, 3.35f, 6.25f}));
  EXPECT_TRUE(candidate_cloud->empty());
  EXPECT_TRUE(dynamic_cloud->empty());
  EXPECT_EQ(static_cloud->size(), 3u);
}

TEST_F(DynamicFilterTest, ReclassifiesPersistentlyOccupiedVoxel)
{
  DynamicObjectFilterConfig config;
  config.enabled = true;
  config.voxel_size = 1.0;
  config.burn_in_frames = 1;
  config.min_cluster_size = 1;
  config.occupancy_reset_frames = 2;
  DynamicObjectFilter filter(config);

  runFilter(filter, makeCloud({6.25f}));
  runFilter(filter, makeCloud({2.25f, 6.25f}));
  EXPECT_EQ(dynamic_cloud->size(), 1u);
  runFilter(filter, makeCloud({2.25f, 6.25f}));
  EXPECT_TRUE(dynamic_cloud->empty());
  EXPECT_EQ(static_cloud->size(), 2u);
}

TEST(FusionQualityTest, LowersWeightOfDegradedModality)
{
  FusionQualityState state;
  state.q_lidar = 0.05;
  state.q_visual = 1.0;
  refreshFusionWeights(state);
  EXPECT_DOUBLE_EQ(state.w_lidar, 0.10);
  EXPECT_DOUBLE_EQ(state.w_visual, 1.80);

  std::swap(state.q_lidar, state.q_visual);
  refreshFusionWeights(state);
  EXPECT_DOUBLE_EQ(state.w_lidar, 1.80);
  EXPECT_DOUBLE_EQ(state.w_visual, 0.10);

  state.q_lidar = state.q_visual = 0.0;
  refreshFusionWeights(state);
  EXPECT_DOUBLE_EQ(state.w_lidar, 0.10);
  EXPECT_DOUBLE_EQ(state.w_visual, 0.10);

  state.adaptive_weight_enabled = false;
  refreshFusionWeights(state);
  EXPECT_DOUBLE_EQ(state.w_lidar, 1.0);
  EXPECT_DOUBLE_EQ(state.w_visual, 1.0);
}

TEST(FusionQualityTest, DetectsObliqueLidarDegeneracy)
{
  const Eigen::Vector3d normal = Eigen::Vector3d::Ones().normalized();
  const Eigen::Matrix3d oblique_plane = normal * normal.transpose();
  const Eigen::Matrix3d axis_aligned_plane = Eigen::Vector3d::UnitZ() * Eigen::Vector3d::UnitZ().transpose();
  const double oblique_score = computeLidarQuality(0.5, 0.0, oblique_plane);
  EXPECT_NEAR(oblique_score, computeLidarQuality(0.5, 0.0, axis_aligned_plane), 1e-12);
  EXPECT_LT(oblique_score, computeLidarQuality(0.5, 0.0, Eigen::Matrix3d::Identity()));
  EXPECT_LT(computeLidarQuality(0.03, 0.15, oblique_plane), oblique_score);
}

TEST(FusionQualityTest, PenalizesPoorVisualCoverageAndTracking)
{
  const double good = computeVisualQuality(0.15, 0.8, 0.0, true);
  EXPECT_DOUBLE_EQ(good, 1.0);
  EXPECT_LT(computeVisualQuality(0.02, 0.1, 0.5, true), good);
  EXPECT_LT(computeVisualQuality(0.02, 1.0, 0.5, false), good);
}
} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
