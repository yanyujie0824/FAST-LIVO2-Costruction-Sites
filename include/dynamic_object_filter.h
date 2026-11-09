#ifndef DYNAMIC_OBJECT_FILTER_H_
#define DYNAMIC_OBJECT_FILTER_H_

#include "common_lib.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct DynamicObjectFilterConfig
{
  bool enabled = false;
  double voxel_size = 0.2;
  double min_range = 0.5;
  double max_range = 60.0;
  double sensor_z_min = -1.0;
  double sensor_z_max = 1.0;
  int burn_in_frames = 5;
  int temporal_buffer_frames = 2;
  int occupancy_reset_frames = 150;
  int min_cluster_size = 20;
  int min_track_duration = 0;
  double max_tracking_distance = 1.0;
  int cleanup_interval_frames = 30;
  int cleanup_stale_frames = 300;
  int max_raycast_steps = 512;
};

class DynamicObjectFilter
{
public:
  struct VoxelKey
  {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey &other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey &key) const
    {
      std::size_t seed = 0;
      seed ^= std::hash<int>()(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  explicit DynamicObjectFilter(
      const DynamicObjectFilterConfig &config = DynamicObjectFilterConfig());

  void reset();
  bool enabled() const { return config_.enabled; }

  void filter(const StatesGroup &state,
              const M3D &extR,
              const V3D &extT,
              const PointCloudXYZI::Ptr &input_cloud,
              PointCloudXYZI::Ptr &static_cloud,
              PointCloudXYZI::Ptr &dynamic_cloud,
              PointCloudXYZI::Ptr &dynamic_debug_cloud);

private:
  struct VoxelState
  {
    int last_occupied_frame = std::numeric_limits<int>::lowest() / 4;
    int last_free_frame = std::numeric_limits<int>::lowest() / 4;
    int last_update_frame = std::numeric_limits<int>::lowest() / 4;
    int occupied_streak = 0;
    int free_streak = 0;
    bool ever_free = false;
  };

  struct Cluster
  {
    int track_length = 0;
    V3D centroid = V3D::Zero();
    std::vector<int> point_indices;
  };

  struct Track
  {
    V3D centroid;
    int length;
  };

  using VoxelPointMap = std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash>;
  using VoxelStateMap = std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash>;
  using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

  VoxelKey pointToKey(const V3D &point) const;
  V3D transformPointToWorld(const StatesGroup &state,
                            const M3D &extR,
                            const V3D &extT,
                            const PointType &point) const;
  void raycastFreeVoxels(const V3D &origin,
                         const V3D &end,
                         const VoxelKey &end_key,
                         VoxelSet &free_voxels) const;
  std::vector<Cluster> buildClusters(const std::vector<VoxelKey> &dynamic_voxels,
                                     const VoxelSet &dynamic_lookup,
                                     const VoxelPointMap &occupied_points,
                                     const std::vector<V3D> &world_points) const;
  void trackClusters(std::vector<Cluster> &clusters);
  void cleanupStaleVoxels();

  const DynamicObjectFilterConfig config_;
  VoxelStateMap voxel_states_;
  std::vector<Track> previous_tracks_;
  int frame_counter_ = 0;
};

typedef std::shared_ptr<DynamicObjectFilter> DynamicObjectFilterPtr;

#endif // DYNAMIC_OBJECT_FILTER_H_
