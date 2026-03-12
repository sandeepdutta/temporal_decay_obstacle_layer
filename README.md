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
3. Standard ObstacleLayer immediately **if the clearing : true** ✗
4. Robot doesn't know the obstacle might still be there

**Solution**: Keep the obstacle marking for a few seconds even after the camera looks away.

## Installation

```bash
colcon build --packages-select jeeves_temporal_obstacle_layer
```

## How It Works

Obstacles marked with `intensity >= 0.0` persist for `max_obstacle_age` seconds, then auto-clear.
Obstacles with `intensity < 0.0` clear immediately (no temporal decay).

## Configuration

- `max_obstacle_age` (double, default: 2.0): Seconds before temporal decay
- All standard ObstacleLayer parameters apply: `observation_sources`, `marking`, `clearing`, `obstacle_max_range`, etc.


## Debugging

Enable debug logging:
```bash
ros2 launch nav2_bringup bringup_launch.py --log-level nav2_costmap_2d:=debug
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

## License

BSD-3-Clause

## Contributing

Issues and PRs welcome!
