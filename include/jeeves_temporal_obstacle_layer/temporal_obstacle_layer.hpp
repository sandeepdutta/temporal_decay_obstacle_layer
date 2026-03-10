/*
 * Temporal Obstacle Layer with Intensity-based Clearing Control
 *
 * Extends the standard ObstacleLayer with:
 * - Temporal decay of obstacle markings (keep markings for N seconds)
 * - Per-point clearing control via intensity field annotation
 *   - Positive intensity: delayed clearing enabled
 *   - Negative intensity: immediate clearing (or no clearing)
 */

#ifndef JEEVES_TEMPORAL_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_
#define JEEVES_TEMPORAL_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <string>

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "rclcpp/time.hpp"

namespace jeeves_temporal_obstacle_layer {

/**
 * @class TemporalObstacleLayer
 * @brief Extends ObstacleLayer with temporal decay and intensity-based clearing control
 *
 * Features:
 * - Obstacles are automatically cleared after max_obstacle_age seconds
 * - Per-point intensity annotation:
 *   - intensity >= 0: Normal marking, temporal decay enabled
 *   - intensity < 0: Marking with immediate clearing (no temporal decay)
 *
 * Useful for camera-based obstacle detection with limited FOV.
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
   * @param min_x, min_y, max_x, max_y Update bounds
   */
  void decayObstacles(double * min_x, double * min_y, double * max_x, double * max_y);

  /**
   * @brief Mark cells from observations, tracking intensity values
   * @param obs Observation with point cloud
   * @param min_x, min_y, max_x, max_y Update bounds
   */
  void markObstaclesWithIntensity(
    const nav2_costmap_2d::Observation & obs,
    double * min_x, double * min_y, double * max_x, double * max_y);

  /**
   * @brief Extract cell indices from observation that should be marked
   * @param obs Observation with point cloud
   * @param marked_cells Output map of cell indices -> birth time
   * @param now Current time
   */
  void extractMarkedCells(
    const nav2_costmap_2d::Observation & obs,
    std::map<unsigned int, rclcpp::Time> & marked_cells,
    const rclcpp::Time & now);

  /**
   * @brief Check if intensity indicates delayed clearing should be disabled
   * @param intensity Intensity value from point cloud
   * @return true if intensity is negative (disable delayed clearing)
   */
  inline bool isImmediateClearingPoint(float intensity) const {
    return intensity < 0.0f;
  }

  /**
   * @brief Track which cells should use immediate clearing vs delayed
   * Maps cell index -> should use immediate clearing
   */
  std::map<unsigned int, bool> cell_clearing_mode_;

  /**
   * @brief Track when each cell was marked as an obstacle
   * Maps cell index -> timestamp when marked
   */
  std::map<unsigned int, rclcpp::Time> obstacle_birth_time_;

  /**
   * @brief Maximum age for obstacles with delayed clearing (seconds)
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

}  // namespace jeeves_temporal_obstacle_layer

#endif  // JEEVES_TEMPORAL_OBSTACLE_LAYER__TEMPORAL_OBSTACLE_LAYER_HPP_
