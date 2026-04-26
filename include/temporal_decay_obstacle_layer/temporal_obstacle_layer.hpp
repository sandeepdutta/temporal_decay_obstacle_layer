/*
 * Temporal Obstacle Layer
 *
 * Extends the standard ObstacleLayer with temporal decay of obstacle markings.
 */

#ifndef TEMPORAL_DECAY_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_
#define TEMPORAL_DECAY_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <string>

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "rclcpp/time.hpp"

namespace temporal_decay_obstacle_layer {

/**
 * @class TemporalObstacleLayer
 * @brief Extends ObstacleLayer with temporal decay: obstacles are automatically
 *        cleared after max_obstacle_age seconds. Useful for camera-based obstacle
 *        detection with limited FOV.
 */
class TemporalObstacleLayer : public nav2_costmap_2d::ObstacleLayer {
public:
  TemporalObstacleLayer();
  virtual ~TemporalObstacleLayer();

  /**
   * @brief Initialize layer
   */
  virtual void onInitialize();

  /**
   * @brief Update costs with temporal decay
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Reset the layer
   */
  virtual void reset();

protected:
  /**
   * @brief Apply temporal decay to old obstacle markings
   * Clears obstacles that exceeded max_obstacle_age_seconds_
   * @param min_x, min_y, max_x, max_y Update bounds
   */
  void decayObstacles(double * min_x, double * min_y, double * max_x, double * max_y);

  /**
   * @brief Mark obstacle cell with timestamp
   * @param cell_index Cell index to mark
   * @param now Current timestamp
   */
  void markObstacleCell(unsigned int cell_index, const rclcpp::Time & now);

  /**
   * @brief Check if a cell can be cleared based on its age
   * @param cell_index Cell index to check
   * @param now Current timestamp
   * @return true if cell exceeds max age
   */
  bool shouldClearCell(unsigned int cell_index, const rclcpp::Time & now);

  /**
   * @brief Track when each cell was marked as an obstacle
   * Maps cell index -> timestamp when marked
   */
  std::map<unsigned int, rclcpp::Time> obstacle_birth_time_;

  /**
   * @brief Maximum age for obstacles (seconds)
   */
  double max_obstacle_age_seconds_;

  /**
   * @brief Clock for getting current time
   */
  rclcpp::Clock::SharedPtr clock_;

  /**
   * @brief Logger
   */
  rclcpp::Logger logger_{rclcpp::get_logger("temporal_obstacle_layer")};
};

}  // namespace temporal_decay_obstacle_layer

#endif  // TEMPORAL_DECAY_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_
