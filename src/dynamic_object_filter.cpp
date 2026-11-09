#include "dynamic_object_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

DynamicObjectFilter::DynamicObjectFilter(const DynamicObjectFilterConfig &config)
    : config_(config)
{
}

void DynamicObjectFilter::reset()
{
  voxel_states_.clear();
  previous_tracks_.clear();
  frame_counter_ = 0;
}

DynamicObjectFilter::VoxelKey DynamicObjectFilter::pointToKey(const V3D &point) const
{
  return {
      static_cast<int>(std::floor(point.x() / config_.voxel_size)),
      static_cast<int>(std::floor(point.y() / config_.voxel_size)),
      static_cast<int>(std::floor(point.z() / config_.voxel_size))};
}

V3D DynamicObjectFilter::transformPointToWorld(const StatesGroup &state,
                                               const M3D &extR,
                                               const V3D &extT,
                                               const PointType &point) const
{
  const V3D point_body(point.x, point.y, point.z);
  return state.rot_end * (extR * point_body + extT) + state.pos_end;
}

void DynamicObjectFilter::raycastFreeVoxels(const V3D &origin,
                                            const V3D &end,
                                            const VoxelKey &end_key,
                                            VoxelSet &free_voxels) const
{
  const V3D direction = end - origin;
  if (direction.squaredNorm() < 1e-12)
  {
    return;
  }

  VoxelKey current = pointToKey(origin);
  if (current == end_key)
  {
    return;
  }

  const auto inf = std::numeric_limits<double>::infinity();
  const int step_x = (direction.x() > 0) - (direction.x() < 0);
  const int step_y = (direction.y() > 0) - (direction.y() < 0);
  const int step_z = (direction.z() > 0) - (direction.z() < 0);

  const double next_boundary_x = step_x > 0 ? (current.x + 1) * config_.voxel_size
                                            : current.x * config_.voxel_size;
  const double next_boundary_y = step_y > 0 ? (current.y + 1) * config_.voxel_size
                                            : current.y * config_.voxel_size;
  const double next_boundary_z = step_z > 0 ? (current.z + 1) * config_.voxel_size
                                            : current.z * config_.voxel_size;

  double t_max_x = step_x != 0 ? (next_boundary_x - origin.x()) / direction.x() : inf;
  double t_max_y = step_y != 0 ? (next_boundary_y - origin.y()) / direction.y() : inf;
  double t_max_z = step_z != 0 ? (next_boundary_z - origin.z()) / direction.z() : inf;

  const double t_delta_x = step_x != 0 ? config_.voxel_size / std::abs(direction.x()) : inf;
  const double t_delta_y = step_y != 0 ? config_.voxel_size / std::abs(direction.y()) : inf;
  const double t_delta_z = step_z != 0 ? config_.voxel_size / std::abs(direction.z()) : inf;

  int step_count = 0;
  while (step_count < config_.max_raycast_steps)
  {
    if (t_max_x <= t_max_y && t_max_x <= t_max_z)
    {
      current.x += step_x;
      t_max_x += t_delta_x;
    }
    else if (t_max_y <= t_max_z)
    {
      current.y += step_y;
      t_max_y += t_delta_y;
    }
    else
    {
      current.z += step_z;
      t_max_z += t_delta_z;
    }

    if (current == end_key)
    {
      break;
    }

    free_voxels.insert(current);
    ++step_count;
  }
}

std::vector<DynamicObjectFilter::Cluster> DynamicObjectFilter::buildClusters(
    const std::vector<VoxelKey> &dynamic_voxels,
    const VoxelSet &dynamic_lookup,
    const VoxelPointMap &occupied_points,
    const std::vector<V3D> &world_points) const
{
  std::vector<Cluster> clusters;
  if (dynamic_voxels.empty())
  {
    return clusters;
  }

  VoxelSet visited;
  visited.reserve(dynamic_lookup.size());
  const int min_cluster_size = std::max(1, config_.min_cluster_size);

  for (const VoxelKey &seed : dynamic_voxels)
  {
    if (!visited.insert(seed).second)
    {
      continue;
    }

    Cluster cluster;
    std::vector<VoxelKey> stack = {seed};
    V3D centroid_sum = V3D::Zero();

    while (!stack.empty())
    {
      const VoxelKey current = stack.back();
      stack.pop_back();

      auto point_it = occupied_points.find(current);
      if (point_it != occupied_points.end())
      {
        for (int point_index : point_it->second)
        {
          cluster.point_indices.push_back(point_index);
          centroid_sum += world_points[point_index];
        }
      }

      const std::array<VoxelKey, 6> neighbors = {
          VoxelKey{current.x + 1, current.y, current.z},
          VoxelKey{current.x - 1, current.y, current.z},
          VoxelKey{current.x, current.y + 1, current.z},
          VoxelKey{current.x, current.y - 1, current.z},
          VoxelKey{current.x, current.y, current.z + 1},
          VoxelKey{current.x, current.y, current.z - 1}};

      for (const VoxelKey &neighbor : neighbors)
      {
        if (dynamic_lookup.find(neighbor) == dynamic_lookup.end())
        {
          continue;
        }
        if (visited.insert(neighbor).second)
        {
          stack.push_back(neighbor);
        }
      }
    }

    if (static_cast<int>(cluster.point_indices.size()) < min_cluster_size)
    {
      continue;
    }

    cluster.centroid = centroid_sum / static_cast<double>(cluster.point_indices.size());
    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

void DynamicObjectFilter::trackClusters(std::vector<Cluster> &clusters)
{
  struct Association
  {
    double distance;
    size_t previous_index;
    size_t current_index;
  };

  std::vector<Association> associations;
  associations.reserve(previous_tracks_.size() * clusters.size());
  for (size_t i = 0; i < previous_tracks_.size(); ++i)
  {
    for (size_t j = 0; j < clusters.size(); ++j)
    {
      associations.push_back({(previous_tracks_[i].centroid - clusters[j].centroid).norm(), i, j});
    }
  }

  std::sort(associations.begin(), associations.end(), [](const Association &lhs, const Association &rhs) {
    return lhs.distance < rhs.distance;
  });

  std::vector<bool> previous_used(previous_tracks_.size(), false);
  std::vector<bool> current_used(clusters.size(), false);

  for (const Association &association : associations)
  {
    if (association.distance > config_.max_tracking_distance)
    {
      break;
    }
    if (previous_used[association.previous_index] || current_used[association.current_index])
    {
      continue;
    }

    Cluster &cluster = clusters[association.current_index];
    cluster.track_length = previous_tracks_[association.previous_index].length + 1;
    previous_used[association.previous_index] = true;
    current_used[association.current_index] = true;
  }

  previous_tracks_.clear();
  previous_tracks_.reserve(clusters.size());
  for (const Cluster &cluster : clusters)
  {
    previous_tracks_.push_back({cluster.centroid, cluster.track_length});
  }
}

void DynamicObjectFilter::cleanupStaleVoxels()
{
  if (config_.cleanup_interval_frames <= 0 || config_.cleanup_stale_frames <= 0)
  {
    return;
  }
  if (frame_counter_ % config_.cleanup_interval_frames != 0)
  {
    return;
  }

  const int stale_before = frame_counter_ - config_.cleanup_stale_frames;
  for (auto it = voxel_states_.begin(); it != voxel_states_.end();)
  {
    if (it->second.last_update_frame < stale_before)
    {
      it = voxel_states_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void DynamicObjectFilter::filter(const StatesGroup &state,
                                 const M3D &extR,
                                 const V3D &extT,
                                 const PointCloudXYZI::Ptr &input_cloud,
                                 PointCloudXYZI::Ptr &static_cloud,
                                 PointCloudXYZI::Ptr &dynamic_cloud,
                                 PointCloudXYZI::Ptr &dynamic_debug_cloud)
{
  if (!static_cloud)
  {
    static_cloud.reset(new PointCloudXYZI());
  }
  if (!dynamic_cloud)
  {
    dynamic_cloud.reset(new PointCloudXYZI());
  }
  if (!dynamic_debug_cloud)
  {
    dynamic_debug_cloud.reset(new PointCloudXYZI());
  }

  static_cloud->clear();
  dynamic_cloud->clear();
  dynamic_debug_cloud->clear();

  if (!input_cloud)
  {
    return;
  }

  for (const auto &cloud : {static_cloud, dynamic_cloud, dynamic_debug_cloud})
  {
    cloud->header = input_cloud->header;
    cloud->sensor_origin_ = input_cloud->sensor_origin_;
    cloud->sensor_orientation_ = input_cloud->sensor_orientation_;
    cloud->is_dense = input_cloud->is_dense;
  }

  if (!config_.enabled || input_cloud->empty())
  {
    *static_cloud = *input_cloud;
    return;
  }

  ++frame_counter_;

  std::vector<V3D> world_points(input_cloud->size(), V3D::Zero());
  std::vector<bool> candidate_mask(input_cloud->size(), false);
  std::vector<bool> dynamic_mask(input_cloud->size(), false);
  VoxelPointMap occupied_points;
  occupied_points.reserve(input_cloud->size());
  VoxelSet free_voxels;
  free_voxels.reserve(input_cloud->size());

  const V3D sensor_origin = state.rot_end * extT + state.pos_end;

  for (size_t i = 0; i < input_cloud->size(); ++i)
  {
    const PointType &point = input_cloud->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }
    if (point.z < config_.sensor_z_min || point.z > config_.sensor_z_max)
    {
      continue;
    }

    const V3D point_world = transformPointToWorld(state, extR, extT, point);
    world_points[i] = point_world;
    const double range = (point_world - sensor_origin).norm();
    if (range < config_.min_range || range > config_.max_range)
    {
      continue;
    }

    const VoxelKey voxel_key = pointToKey(point_world);
    occupied_points[voxel_key].push_back(static_cast<int>(i));
    raycastFreeVoxels(sensor_origin, point_world, voxel_key, free_voxels);
  }

  for (const VoxelKey &voxel_key : free_voxels)
  {
    if (occupied_points.find(voxel_key) != occupied_points.end())
    {
      continue;
    }

    VoxelState &voxel_state = voxel_states_[voxel_key];
    voxel_state.last_update_frame = frame_counter_;
    voxel_state.free_streak = voxel_state.last_free_frame == frame_counter_ - 1
                                  ? voxel_state.free_streak + 1
                                  : 1;
    voxel_state.last_free_frame = frame_counter_;
    if (voxel_state.free_streak >= config_.burn_in_frames &&
        voxel_state.last_occupied_frame < frame_counter_ - config_.temporal_buffer_frames)
    {
      voxel_state.ever_free = true;
    }
  }

  std::vector<VoxelKey> dynamic_voxels;
  dynamic_voxels.reserve(occupied_points.size());
  VoxelSet dynamic_lookup;
  dynamic_lookup.reserve(occupied_points.size());
  const int occupancy_reset_frames = std::max(1, config_.occupancy_reset_frames);

  for (const auto &entry : occupied_points)
  {
    const VoxelKey &voxel_key = entry.first;
    VoxelState &voxel_state = voxel_states_[voxel_key];
    voxel_state.last_update_frame = frame_counter_;
    voxel_state.occupied_streak = voxel_state.last_occupied_frame == frame_counter_ - 1
                                      ? voxel_state.occupied_streak + 1
                                      : 1;
    voxel_state.last_occupied_frame = frame_counter_;
    voxel_state.free_streak = 0;

    if (voxel_state.occupied_streak >= occupancy_reset_frames)
    {
      voxel_state.ever_free = false;
    }
    if (voxel_state.ever_free)
    {
      dynamic_voxels.push_back(voxel_key);
      dynamic_lookup.insert(voxel_key);
      for (int point_index : entry.second)
      {
        candidate_mask[point_index] = true;
      }
    }
  }

  std::vector<Cluster> clusters = buildClusters(dynamic_voxels, dynamic_lookup, occupied_points, world_points);
  trackClusters(clusters);

  for (const Cluster &cluster : clusters)
  {
    if (cluster.track_length < config_.min_track_duration)
    {
      continue;
    }
    for (int point_index : cluster.point_indices)
    {
      dynamic_mask[point_index] = true;
    }
  }

  static_cloud->reserve(input_cloud->size());
  dynamic_cloud->reserve(input_cloud->size());
  dynamic_debug_cloud->reserve(input_cloud->size());

  for (size_t i = 0; i < input_cloud->size(); ++i)
  {
    if (candidate_mask[i])
    {
      dynamic_debug_cloud->push_back(input_cloud->points[i]);
    }
    if (dynamic_mask[i])
    {
      dynamic_cloud->push_back(input_cloud->points[i]);
    }
    else
    {
      static_cloud->push_back(input_cloud->points[i]);
    }
  }

  cleanupStaleVoxels();
}
