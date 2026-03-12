/*
 * Temporal Obstacle Layer with Intensity-based Clearing Control
 */

#include "jeeves_temporal_obstacle_layer/temporal_obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(
  jeeves_temporal_obstacle_layer::TemporalObstacleLayer,
  nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace jeeves_temporal_obstacle_layer {

TemporalObstacleLayer::TemporalObstacleLayer() {}

TemporalObstacleLayer::~TemporalObstacleLayer() {}

void TemporalObstacleLayer::onInitialize() {
  // Call parent initialization
  ObstacleLayer::onInitialize();

  // Get clock from the node
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  clock_ = node->get_clock();

  // Declare and get temporal parameters
  declareParameter("max_obstacle_age", rclcpp::ParameterValue(2.0));

  double max_age;
  node->get_parameter(name_ + "." + "max_obstacle_age", max_age);
  max_obstacle_age_seconds_ = max_age;

  RCLCPP_INFO(
    logger_,
    "TemporalObstacleLayer initialized with max_obstacle_age=%.2f seconds",
    max_obstacle_age_seconds_);
}

void TemporalObstacleLayer::reset() {
  // Call parent reset
  ObstacleLayer::reset();

  // Clear our tracking maps
  obstacle_birth_time_.clear();
  cell_clearing_mode_.clear();

  RCLCPP_DEBUG(logger_, "TemporalObstacleLayer reset");
}

void TemporalObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j) {
  rclcpp::Time now = clock_->now();
  double tmp_min_x = min_i;
  double tmp_min_y = min_j;
  double tmp_max_x = max_i;
  double tmp_max_y = max_j;

  // Step 1: Apply temporal decay - clear old obstacles
  decayObstacles(&tmp_min_x, &tmp_min_y, &tmp_max_x, &tmp_max_y);

  // Step 2: Get marking observations and process them
  std::vector<nav2_costmap_2d::Observation> marking_observations;
  getMarkingObservations(marking_observations);

  unsigned char * costmap = getCharMap();

  // Process each marking observation
  for (const auto & obs : marking_observations) {
    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    // Get sensor origin in map coords
    unsigned int x0, y0;
    if (!worldToMap(obs.origin_.x, obs.origin_.y, x0, y0)) {
      continue;
    }

    // Set up iterators
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // Check for intensity field
    bool has_intensity = false;
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud, "intensity");
    try {
      if (iter_intensity != iter_intensity.end()) {
        has_intensity = true;
      }
    } catch (...) {
      has_intensity = false;
    }

    const unsigned int max_range_cells = cellDistance(obs.obstacle_max_range_);
    const unsigned int min_range_cells = cellDistance(obs.obstacle_min_range_);
    const double min_obstacle_height = min_obstacle_height_;
    const double max_obstacle_height = max_obstacle_height_;

    // Process each point
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      if (has_intensity) {
        float intensity = *iter_intensity;
        ++iter_intensity;

        // Filter by height
        if (pz < min_obstacle_height || pz > max_obstacle_height) {
          continue;
        }

        // Get map coordinates
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
          continue;
        }

        // Check range
        const int dx = static_cast<int>(mx) - static_cast<int>(x0);
        const int dy = static_cast<int>(my) - static_cast<int>(y0);
        const unsigned int dist = static_cast<unsigned int>(
          std::hypot(static_cast<double>(dx), static_cast<double>(dy)));

        if (dist > max_range_cells || dist < min_range_cells) {
          continue;
        }

        // Mark cell with timestamp and intensity info
        unsigned int cell_index = getIndex(mx, my);
        markObstacleCell(cell_index, intensity, now);

        // Update bounds
        double wx, wy;
        mapToWorld(mx, my, wx, wy);
        tmp_min_x = std::min(wx, tmp_min_x);
        tmp_min_y = std::min(wy, tmp_min_y);
        tmp_max_x = std::max(wx, tmp_max_x);
        tmp_max_y = std::max(wy, tmp_max_y);
      } else {
        // No intensity - process as before
        // Filter by height
        if (pz < min_obstacle_height || pz > max_obstacle_height) {
          continue;
        }

        // Get map coordinates
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
          continue;
        }

        // Check range
        const int dx = static_cast<int>(mx) - static_cast<int>(x0);
        const int dy = static_cast<int>(my) - static_cast<int>(y0);
        const unsigned int dist = static_cast<unsigned int>(
          std::hypot(static_cast<double>(dx), static_cast<double>(dy)));

        if (dist > max_range_cells || dist < min_range_cells) {
          continue;
        }

        // Mark cell with timestamp (no intensity, so default to delayed clearing)
        unsigned int cell_index = getIndex(mx, my);
        markObstacleCell(cell_index, 0.0f, now);

        // Update bounds
        double wx, wy;
        mapToWorld(mx, my, wx, wy);
        tmp_min_x = std::min(wx, tmp_min_x);
        tmp_min_y = std::min(wy, tmp_min_y);
        tmp_max_x = std::max(wx, tmp_max_x);
        tmp_max_y = std::max(wy, tmp_max_y);
      }
    }
  }

  // Call parent to handle clearing observations (raytracing) only
  // Parent will use our tracking info to avoid clearing obstacles within temporal window
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void TemporalObstacleLayer::markObstacleCell(
  unsigned int cell_index,
  float intensity,
  const rclcpp::Time & now) {
  unsigned char * costmap = getCharMap();

  // Determine clearing mode from intensity
  bool use_immediate_clearing = isImmediateClearingPoint(intensity);

  // Mark the cell as lethal obstacle
  costmap[cell_index] = LETHAL_OBSTACLE;

  // Track timestamp and clearing mode
  obstacle_birth_time_[cell_index] = now;
  cell_clearing_mode_[cell_index] = use_immediate_clearing;
}

bool TemporalObstacleLayer::shouldClearCell(
  unsigned int cell_index,
  const rclcpp::Time & now) {
  // Check if cell is tracked
  auto it = obstacle_birth_time_.find(cell_index);
  if (it == obstacle_birth_time_.end()) {
    // Not tracked - allow parent to clear it
    return true;
  }

  // Check clearing mode
  bool uses_immediate_clearing = false;
  auto mode_it = cell_clearing_mode_.find(cell_index);
  if (mode_it != cell_clearing_mode_.end()) {
    uses_immediate_clearing = mode_it->second;
  }

  // If uses immediate clearing, allow parent to clear it
  if (uses_immediate_clearing) {
    return true;
  }

  // Check age
  double age = (now - it->second).seconds();
  bool exceeds_max_age = (max_obstacle_age_seconds_ > 0.0) &&
                         (age > max_obstacle_age_seconds_);

  // Only allow clearing if exceeded max age
  return exceeds_max_age;
}

void TemporalObstacleLayer::decayObstacles(
  double * min_x, double * min_y, double * max_x, double * max_y) {
  if (max_obstacle_age_seconds_ <= 0.0) {
    // Temporal decay disabled
    return;
  }

  rclcpp::Time now = clock_->now();
  unsigned char * costmap = getCharMap();

  std::vector<unsigned int> cells_to_clear;

  // Check all tracked obstacles
  for (auto & [cell_index, birth_time] : obstacle_birth_time_) {
    double age = (now - birth_time).seconds();

    // Check if this cell should use delayed clearing
    bool uses_immediate_clearing = false;
    if (cell_clearing_mode_.find(cell_index) != cell_clearing_mode_.end()) {
      uses_immediate_clearing = cell_clearing_mode_[cell_index];
    }

    // Only apply temporal decay to cells with delayed clearing enabled
    if (!uses_immediate_clearing && age > max_obstacle_age_seconds_) {
      // Mark for clearing
      cells_to_clear.push_back(cell_index);
    }
  }

  // Clear aged obstacles
  for (unsigned int cell_index : cells_to_clear) {
    if (costmap[cell_index] == LETHAL_OBSTACLE) {
      costmap[cell_index] = NO_INFORMATION;

      // Update bounds
      unsigned int x, y;
      indexToCells(cell_index, x, y);
      double wx, wy;
      mapToWorld(x, y, wx, wy);
      *min_x = std::min(wx, *min_x);
      *min_y = std::min(wy, *min_y);
      *max_x = std::max(wx, *max_x);
      *max_y = std::max(wy, *max_y);
    }

    // Clean up tracking
    obstacle_birth_time_.erase(cell_index);
    cell_clearing_mode_.erase(cell_index);
  }

  if (!cells_to_clear.empty()) {
    RCLCPP_DEBUG(
      logger_,
      "Decayed %lu obstacles due to age exceeding %.2f seconds",
      cells_to_clear.size(), max_obstacle_age_seconds_);
  }
}

}  // namespace jeeves_temporal_obstacle_layer
