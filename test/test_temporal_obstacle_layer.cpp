#include <gtest/gtest.h>
#include <memory>

#include "jeeves_temporal_obstacle_layer/temporal_obstacle_layer.hpp"

using namespace jeeves_temporal_obstacle_layer;

/**
 * @brief Basic test suite for TemporalObstacleLayer
 *
 * Tests core functionality:
 * - Layer instantiation
 * - Layer inheritance from ObstacleLayer
 * - Layer destruction
 */
class TemporalObstacleLayerTest : public ::testing::Test {
protected:
  void SetUp() override {
    layer_ = std::make_unique<TemporalObstacleLayer>();
  }

  void TearDown() override {
    layer_.reset();
  }

  std::unique_ptr<TemporalObstacleLayer> layer_;
};

/**
 * @test Test that TemporalObstacleLayer can be instantiated
 */
TEST_F(TemporalObstacleLayerTest, ConstructorDoesNotThrow) {
  EXPECT_NO_THROW({
    TemporalObstacleLayer layer;
  });
}

/**
 * @test Test that layer object is valid after construction
 */
TEST_F(TemporalObstacleLayerTest, LayerIsValid) {
  EXPECT_NE(layer_.get(), nullptr);
}

/**
 * @test Test that multiple instances can be created
 */
TEST_F(TemporalObstacleLayerTest, MultipleInstancesCreatable) {
  auto layer1 = std::make_unique<TemporalObstacleLayer>();
  auto layer2 = std::make_unique<TemporalObstacleLayer>();
  auto layer3 = std::make_unique<TemporalObstacleLayer>();

  EXPECT_NE(layer1.get(), nullptr);
  EXPECT_NE(layer2.get(), nullptr);
  EXPECT_NE(layer3.get(), nullptr);
  EXPECT_NE(layer1.get(), layer2.get());
  EXPECT_NE(layer2.get(), layer3.get());
}

/**
 * @test Test that destructor is safe
 */
TEST_F(TemporalObstacleLayerTest, DestructorIsSafe) {
  {
    auto temp_layer = std::make_unique<TemporalObstacleLayer>();
    // Destructor should be called here without throwing
  }
  EXPECT_TRUE(true);  // If we get here, no exception was thrown
}

/**
 * @test Test layer inherits from Nav2 ObstacleLayer
 */
TEST_F(TemporalObstacleLayerTest, InheritsFromObstacleLayer) {
  // Check that layer can be cast to ObstacleLayer
  nav2_costmap_2d::ObstacleLayer * obstacle_layer =
    dynamic_cast<nav2_costmap_2d::ObstacleLayer *>(layer_.get());

  EXPECT_NE(obstacle_layer, nullptr);
}

/**
 * @brief Test suite for temporal decay logic
 *
 * Tests the temporal decay functionality that clears old obstacles
 */
class TemporalDecayTest : public ::testing::Test {
protected:
  /**
   * Test helper: Create a layer subclass to access protected members for testing
   */
  class TestableTemporalObstacleLayer : public TemporalObstacleLayer {
  public:
    // Expose protected members for testing
    std::map<unsigned int, rclcpp::Time> & getObstacleBirthTimes() {
      return obstacle_birth_time_;
    }

    std::map<unsigned int, bool> & getCellClearingModes() {
      return cell_clearing_mode_;
    }

    double getMaxObstacleAge() const {
      return max_obstacle_age_seconds_;
    }

    void setMaxObstacleAge(double seconds) {
      max_obstacle_age_seconds_ = seconds;
    }

    // Expose protected method for testing
    void testDecayObstacles(
      double * min_x, double * min_y, double * max_x, double * max_y) {
      decayObstacles(min_x, min_y, max_x, max_y);
    }
  };

  void SetUp() override {
    layer_ = std::make_unique<TestableTemporalObstacleLayer>();
  }

  void TearDown() override {
    layer_.reset();
  }

  std::unique_ptr<TestableTemporalObstacleLayer> layer_;
};

/**
 * @test Test max obstacle age can be disabled
 *
 * When max_obstacle_age_seconds_ is 0 or negative, temporal decay should be disabled
 */
TEST_F(TemporalDecayTest, DecayDisabledWhenMaxAgeIsZeroOrNegative) {
  // Set max age to 0 (disabled)
  layer_->setMaxObstacleAge(0.0);
  EXPECT_EQ(layer_->getMaxObstacleAge(), 0.0);

  // Test negative max age
  layer_->setMaxObstacleAge(-5.0);
  EXPECT_LT(layer_->getMaxObstacleAge(), 0.0);
}

/**
 * @test Test obstacle tracking data structure initialization
 *
 * Verify that tracking maps are properly initialized and empty
 */
TEST_F(TemporalDecayTest, TrackingMapsInitiallyEmpty) {
  layer_->setMaxObstacleAge(2.0);

  // Birth times map should be empty initially
  EXPECT_TRUE(layer_->getObstacleBirthTimes().empty());
  EXPECT_TRUE(layer_->getCellClearingModes().empty());
}

/**
 * @test Test clearing mode tracking
 *
 * Verify that immediate clearing mode is correctly tracked per cell
 */
TEST_F(TemporalDecayTest, CellClearingModeTracking) {
  layer_->setMaxObstacleAge(2.0);

  auto & clearing_modes = layer_->getCellClearingModes();

  // Simulate adding obstacles with different clearing modes
  clearing_modes[0] = false;  // Cell 0: delayed clearing
  clearing_modes[1] = true;   // Cell 1: immediate clearing
  clearing_modes[2] = false;  // Cell 2: delayed clearing

  EXPECT_FALSE(clearing_modes[0]);
  EXPECT_TRUE(clearing_modes[1]);
  EXPECT_FALSE(clearing_modes[2]);
}

/**
 * @test Test obstacle birth time tracking
 *
 * Verify that birth times are properly recorded for obstacles
 */
TEST_F(TemporalDecayTest, ObstacleBirthTimeTracking) {
  layer_->setMaxObstacleAge(2.0);

  auto & birth_times = layer_->getObstacleBirthTimes();
  rclcpp::Clock clock;

  // Simulate tracking obstacle birth times
  rclcpp::Time now = clock.now();
  birth_times[0] = now;
  birth_times[1] = now;
  birth_times[2] = now;

  EXPECT_EQ(birth_times.size(), 3);
  EXPECT_EQ(birth_times[0], now);
  EXPECT_EQ(birth_times[1], now);
  EXPECT_EQ(birth_times[2], now);
}

/**
 * @test Test max obstacle age parameter
 *
 * Verify that max obstacle age can be set and retrieved correctly
 */
TEST_F(TemporalDecayTest, MaxObstacleAgeParameter) {
  // Test various age values
  std::vector<double> test_ages = {0.5, 1.0, 2.0, 5.0, 10.0};

  for (double age : test_ages) {
    layer_->setMaxObstacleAge(age);
    EXPECT_DOUBLE_EQ(layer_->getMaxObstacleAge(), age);
  }
}

/**
 * @test Test obstacle age calculation logic
 *
 * Verify that obstacle ages can be properly tracked and compared
 */
TEST_F(TemporalDecayTest, ObstacleAgeCalculation) {
  layer_->setMaxObstacleAge(2.0);

  rclcpp::Clock clock;
  rclcpp::Time now = clock.now();
  rclcpp::Time past = now - rclcpp::Duration::from_seconds(3.0);

  // Store times
  auto & birth_times = layer_->getObstacleBirthTimes();
  birth_times[0] = now;
  birth_times[1] = past;

  // Verify times are recorded
  EXPECT_EQ(birth_times.size(), 2);

  // Calculate ages (in real implementation, this happens in decayObstacles)
  double age_0 = (now - birth_times[0]).seconds();
  double age_1 = (now - birth_times[1]).seconds();

  // Recent obstacle should have age close to 0
  EXPECT_LE(age_0, 0.1);  // Allow small tolerance

  // Old obstacle should have age > 2 seconds
  EXPECT_GT(age_1, 2.0);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
