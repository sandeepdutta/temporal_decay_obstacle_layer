# Temporal Obstacle Layer

A custom Nav2 costmap layer that extends the standard `ObstacleLayer` with temporal decay and intensity-based clearing control.

**Perfect for**: Camera-based obstacle detection with limited field of view (FOV).

## Features

- **Temporal Decay**: Obstacle markings automatically fade after a configurable duration
- **Intensity Annotation**: Per-point control over clearing behavior via point cloud intensity field
  - `intensity >= 0.0`: Delayed clearing (default, uses temporal decay)
  - `intensity < 0.0`: Immediate clearing (no temporal decay)
- **Drop-in Replacement**: Extends standard `ObstacleLayer`, all existing configs work

## The Problem It Solves

You have a camera with limited FOV:
1. Camera marks an obstacle ✓
2. Camera rotates/moves away, obstacle now **out of view**
3. Standard ObstacleLayer immediately **clears it via raycasting** ✗
4. Robot doesn't know the obstacle might still be there

**Solution**: Keep the obstacle marking for a few seconds even after the camera looks away.

## Installation

```bash
cd ~/workspaces/isaac_ros-dev/src/jeeves_navigation
colcon build --packages-select jeeves_temporal_obstacle_layer
```

## Quick Start

### 1. Update Costmap Config

```yaml
global_costmap:
  plugins: ["static_layer", "temporal_obstacle_layer", "inflation_layer"]

  temporal_obstacle_layer:
    plugin: jeeves_temporal_obstacle_layer/TemporalObstacleLayer
    max_obstacle_age: 2.0  # Keep obstacles for 2 seconds

    observation_sources: camera

    camera:
      topic: /camera/point_cloud
      marking: true
      clearing: false  # Don't raytrace when camera rotates
      obstacle_max_range: 5.0
```

### 2. Publish Point Cloud with Intensity Annotation

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# Create point cloud with intensity
points = np.array([
    (1.0, 2.0, 0.5, 1.0),   # (x, y, z, intensity)
    (2.0, 3.0, 1.0, -1.0),  # intensity < 0 = immediate clearing
], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])

cloud = pc2.create_cloud(
    header=Header(frame_id='map', stamp=now()),
    fields=[...],
    points=points
)
publisher.publish(cloud)
```

## How It Works

### Timeline

```
t=0.0s: Camera sees obstacle → marks cell (10,10)
t=0.5s: Camera rotates away → marking persists
t=1.0s: Still marked → robot avoids it
t=2.0s: Obstacle is > max_obstacle_age old → automatically cleared
```

### Intensity Field

| Value | Behavior |
|-------|----------|
| `>= 0.0` | **Delayed Clearing**: Uses temporal decay |
| `< 0.0` | **Immediate Clearing**: No temporal decay |

Example use cases:
- `intensity = 1.0`: Normal detection, delayed clearing
- `intensity = -1.0`: High confidence, immediate clearing
- `intensity = 0.5`: Low confidence, delayed clearing

## Configuration

### Parameters

**Layer Parameters:**
- `max_obstacle_age` (double, default: 2.0)
  - How long to keep obstacles before temporal decay (seconds)
  - Set to 0.0 to disable temporal decay
  - Typical range: 1.0-3.0 seconds

**Sensor Configuration:**

All standard ObstacleLayer parameters work:
- `observation_sources`: List of sensor topics
- `marking`: Whether to mark obstacles (true/false)
- `clearing`: Whether to use raycasting to clear (true/false)
- `obstacle_max_range`: Maximum range for marking
- `raytrace_max_range`: Maximum range for raycasting
- `observation_persistence`: How long to keep observations in buffer
- `min_obstacle_height` / `max_obstacle_height`: Height filtering

### Example: Camera + Lidar

Use a camera for marking with temporal decay, lidar for immediate clearing:

```yaml
temporal_obstacle_layer:
  max_obstacle_age: 1.5
  observation_sources: [camera, lidar]

  camera:
    topic: /camera/detections
    marking: true
    clearing: false              # No raycasting
    obstacle_max_range: 5.0
    observation_persistence: 0.0

  lidar:
    topic: /lidar/scan
    marking: true
    clearing: true               # Raytrace to clear
    raytrace_max_range: 5.0
    observation_persistence: 0.0
```

## Publishing Point Clouds with Intensity

### C++ Example

```cpp
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

void publishObstacles(const std::vector<Obstacle>& obstacles) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = "map";
  cloud->header.stamp = node->now();

  // Set fields: x, y, z, intensity
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
  modifier.resize(obstacles.size());

  // Fill points
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud, "intensity");

  for (const auto& obs : obstacles) {
    *iter_x = obs.x;
    *iter_y = obs.y;
    *iter_z = obs.z;
    *iter_intensity = obs.high_confidence ? -1.0f : 1.0f;

    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
  }

  publisher->publish(*cloud);
}
```

### Python Example

```python
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

def publish_obstacles(obstacles, publisher):
    # Create structured array with x, y, z, intensity
    points = np.array([
        (obs['x'], obs['y'], obs['z'],
         -1.0 if obs['confidence'] > 0.8 else 1.0)
        for obs in obstacles
    ], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])

    # Create PointCloud2
    cloud = pc2.create_cloud(
        header=Header(frame_id='map', stamp=publisher.node.get_clock().now().to_msg()),
        fields=[
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ],
        points=points.tobytes()
    )

    publisher.publish(cloud)
```

## Debugging

### Enable Debug Logging

```bash
ros2 launch nav2_bringup bringup_launch.py --log-level nav2_costmap_2d:=debug
```

The layer will log:
- Point intensity values and clearing modes
- When obstacles are decayed
- How many obstacles were cleared

### Check Layer Status

```bash
# Get costmap
ros2 topic echo /global_costmap/costmap

# Get layer bounds
ros2 topic echo /global_costmap/costmap_updates

# Monitor decay
ros2 launch nav2_bringup rviz_launch.py  # Visualize costmap
```

## Performance

- **Memory**: ~8 bytes per tracked obstacle (timestamp map)
- **CPU**: Linear scan through obstacles every update cycle
- **Typical overhead**: < 1% for most systems

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Obstacles disappear too quickly | Increase `max_obstacle_age` |
| Obstacles never disappear | Decrease `max_obstacle_age` or check `clearing: true` |
| Intensity not being read | Ensure PointCloud2 has `intensity` field as FLOAT32 |
| Layer not loading | Check plugin name: `jeeves_temporal_obstacle_layer/TemporalObstacleLayer` |

## Advanced: Confidence-based Clearing

Use intensity to encode confidence from perception system:

```python
# In your perception node
if detection.confidence > 0.9:
    intensity = -1.0  # High confidence: clear immediately
elif detection.confidence > 0.5:
    intensity = 1.0   # Medium confidence: temporal decay
else:
    intensity = 2.0   # Low confidence: longer decay (not implemented yet)
```

## Future Enhancements

- [ ] Gradient decay (cost decreases over time instead of sudden removal)
- [ ] Per-cell max_age based on intensity value
- [ ] Decay filtering layer for smooth obstacle fade-out

## License

BSD-3-Clause

## Contributing

Issues and PRs welcome!
