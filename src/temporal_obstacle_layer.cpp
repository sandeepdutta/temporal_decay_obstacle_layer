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
  // First, apply temporal decay to age out old obstacles
  double tmp_min_x = min_i;
  double tmp_min_y = min_j;
  double tmp_max_x = max_i;
  double tmp_max_y = max_j;

  decayObstacles(&tmp_min_x, &tmp_min_y, &tmp_max_x, &tmp_max_y);

  // Process marking observations to track intensity values and current marked cells
  std::vector<nav2_costmap_2d::Observation> marking_observations;
  getMarkingObservations(marking_observations);

  rclcpp::Time now = clock_->now();
  unsigned char * costmap = getCharMap();

  // Build map of cells that should be marked from current observations
  std::map<unsigned int, rclcpp::Time> current_markings;

  // Process each marking observation and track intensity
  for (const auto & obs : marking_observations) {
    markObstaclesWithIntensity(obs, &tmp_min_x, &tmp_min_y, &tmp_max_x, &tmp_max_y);
    extractMarkedCells(obs, current_markings, now);
  }

  // Then call parent updateCosts to handle clearing and normal flow (with raytracing enabled)
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

  // After parent clears via raytracing, restore obstacles within their temporal window
  // This preserves temporal decay while allowing spatial clearing via raytracing
  for (const auto & [cell_index, mark_time] : current_markings) {
    double age = (now - mark_time).seconds();
    bool exceeds_max_age = (max_obstacle_age_seconds_ > 0.0) &&
                          (age > max_obstacle_age_seconds_);

    // Re-mark cell if it hasn't exceeded max age and isn't using immediate clearing
    if (!exceeds_max_age) {
      bool uses_immediate_clearing = false;
      if (cell_clearing_mode_.find(cell_index) != cell_clearing_mode_.end()) {
        uses_immediate_clearing = cell_clearing_mode_[cell_index];
      }

      // Re-mark if using delayed clearing
      if (!uses_immediate_clearing) {
        costmap[cell_index] = LETHAL_OBSTACLE;

        // Update bounds
        unsigned int x, y;
        indexToCells(cell_index, x, y);
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        tmp_min_x = std::min(wx, tmp_min_x);
        tmp_min_y = std::min(wy, tmp_min_y);
        tmp_max_x = std::max(wx, tmp_max_x);
        tmp_max_y = std::max(wy, tmp_max_y);
      }
    }
  }

  // Track newly marked cells that weren't already tracked
  for (unsigned int i = 0; i < getSizeInCellsX() * getSizeInCellsY(); ++i) {
    if (costmap[i] == LETHAL_OBSTACLE) {
      if (obstacle_birth_time_.find(i) == obstacle_birth_time_.end()) {
        obstacle_birth_time_[i] = now;
        // Default to delayed clearing if not explicitly set
        if (cell_clearing_mode_.find(i) == cell_clearing_mode_.end()) {
          cell_clearing_mode_[i] = false;
        }
      }
    }
  }
}

void TemporalObstacleLayer::extractMarkedCells(
  const nav2_costmap_2d::Observation & obs,
  std::map<unsigned int, rclcpp::Time> & marked_cells,
  const rclcpp::Time & now) {
  const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

  // Get sensor origin in map coords
  unsigned int x0, y0;
  if (!worldToMap(obs.origin_.x, obs.origin_.y, x0, y0)) {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  const unsigned int max_range_cells = cellDistance(obs.obstacle_max_range_);
  const unsigned int min_range_cells = cellDistance(obs.obstacle_min_range_);
  const double min_obstacle_height = min_obstacle_height_;
  const double max_obstacle_height = max_obstacle_height_;

  // Process each point in the cloud
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double px = *iter_x, py = *iter_y, pz = *iter_z;

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

    // Add to marked cells
    unsigned int cell_index = getIndex(mx, my);
    marked_cells[cell_index] = now;
  }
}

void TemporalObstacleLayer::markObstaclesWithIntensity(
  const nav2_costmap_2d::Observation & obs,
  double * min_x, double * min_y, double * max_x, double * max_y) {
  const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);
  rclcpp::Time now = clock_->now();

  // Get sensor origin in map coords
  unsigned int x0, y0;
  if (!worldToMap(obs.origin_.x, obs.origin_.y, x0, y0)) {
    return;
  }

  // Check if point cloud has intensity field
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  // Try to get intensity iterator (may not exist)
  bool has_intensity = false;
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud, "intensity");
  try {
    // If iterator is valid, we have intensity
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

  // Process each point in the cloud
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double px = *iter_x, py = *iter_y, pz = *iter_z;

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

    // Determine clearing mode from intensity
    bool use_immediate_clearing = false;
    if (has_intensity) {
      float intensity = *iter_intensity;
      use_immediate_clearing = isImmediateClearingPoint(intensity);

      RCLCPP_DEBUG(
        logger_,
        "Point at (%.2f, %.2f) intensity=%.2f: %s",
        px, py, intensity,
        use_immediate_clearing ? "immediate clearing" : "delayed clearing");
    }

    // Track this cell
    unsigned int cell_index = getIndex(mx, my);
    cell_clearing_mode_[cell_index] = use_immediate_clearing;
    obstacle_birth_time_[cell_index] = now;

    if (has_intensity) {
      ++iter_intensity;
    }
  }
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
