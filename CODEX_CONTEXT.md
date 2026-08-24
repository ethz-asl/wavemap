# Codex Context: Layered Wavemap Work

## Repository State

Workspace: `/home/ci/catkin_ws/src/wavemap`

The current work extends Wavemap with a layered map architecture for digital-twin style voxel data:

- continuous data stored in a templated wavelet octree;
- discrete/categorical data stored in compressed discrete layers;
- `.lwvmp` persistence for full layered maps;
- ROS1 server support for loading/saving/publishing layered maps;
- RViz support for visualizing layered maps from topic or file.

Original Wavemap behavior should remain available and should not be broken. The layered functionality is additive.

## Main Design Decisions

### Continuous Data

Continuous layers stay inside a wavelet map. The current example type is based on:

- occupancy;
- RGB/color;
- traversability.

Continuous fields must support the arithmetic needed by the Haar/wavelet transform:

- addition;
- subtraction;
- scalar multiplication;
- threshold/clamp/pruning traits.

This is why continuous float-like data makes sense, while categorical labels do not.

### Discrete Data

Discrete data is stored separately from the continuous wavelet octree.

Current example discrete layers:

- `semantic`: integer label;
- `changed`: boolean flag.

The goal is not to hardcode these forever. They are example layers. The intended user-facing pattern is:

```cpp
struct MyDiscreteLayers {
  DiscreteLayer<int> semantic;
  DiscreteLayer<bool> changed;
};
```

Discrete data uses a compressed parent/exceptions representation. It does not use Haar arithmetic.

### Full Layered Map

The intended top-level structure is:

```text
LayeredMap
  continuous_map      // wavelet octree with occupancy + continuous fields
  discrete_layers     // templated discrete layer struct
```

Naming decisions:

- `ContinuousWaveletMap`: continuous wavelet map alias;
- `LayeredMap`: full map aggregation;
- `LayeredMapConfig`: full config;
- `ExampleLayeredMap`: current example config/type.

## Implemented Components

### Core / Library

Implemented or modified:

- `library/cpp/include/wavemap/layered/layered_map.h`
- `library/cpp/include/wavemap/layered/layered_map_io.h`
- `library/cpp/include/wavemap/layered/layered_map_schema.h`

Current functionality:

- full `LayeredMap` abstraction;
- continuous + discrete access;
- `.lwvmp` save/load;
- schema metadata stored in `.lwvmp`;
- schema compatibility checks.

### Example Config

Implemented or modified:

- `examples/cpp/digital_twin/common/layered_voxel_config.h`
- `examples/cpp/digital_twin/common/example_layered_map_config.h`
- `examples/cpp/digital_twin/common/example_layered_map_ros_config.h`
- `examples/cpp/digital_twin/common/layered_ros_converter.h`

Current example layers:

```text
continuous:
  occupancy
  color / RGB
  traversability

discrete:
  semantic
  changed
```

### ROS Server

Implemented or modified:

- `interfaces/ros1/wavemap_ros/include/wavemap_ros/layered_ros_server_extension.h`
- `interfaces/ros1/wavemap_ros/include/wavemap_ros/ros_server.h`
- `interfaces/ros1/wavemap_ros/src/ros_server.cc`
- `interfaces/ros1/wavemap_ros/CMakeLists.txt`

The layered extension is attached to the existing `RosServer`.

It provides:

```bash
/wavemap/layered_map
/wavemap/load_layered_map
/wavemap/save_layered_map
/wavemap/layered_map_request_full
```

Confirmed runtime log:

```text
Layered map extension enabled. Advertising layered map topic and services.
```

The original Wavemap services/topics remain separate:

```bash
/wavemap/map
/wavemap/load_map
/wavemap/save_map
/wavemap/map_request_full
```

Important caveat:

- the original occupancy map and the layered map are currently separate internal maps;
- `/wavemap/load_map` loads `.wvmp`;
- `/wavemap/load_layered_map` loads `.lwvmp`.

### ROS / RViz Conversions

Implemented or modified:

- `interfaces/ros1/wavemap_ros_conversions/include/wavemap_ros_conversions/discrete_layer_marker_conversions.h`
- layered ROS conversion helpers in example/common config.

### RViz Plugin

Implemented or modified:

- `interfaces/ros1/wavemap_rviz_plugin/include/wavemap_rviz_plugin/layered_map_display.h`
- `interfaces/ros1/wavemap_rviz_plugin/src/layered_map_display.cc`
- `interfaces/ros1/wavemap_rviz_plugin/include/wavemap_rviz_plugin/common.h`
- `interfaces/ros1/wavemap_rviz_plugin/include/wavemap_rviz_plugin/visuals/voxel_visual.h`
- `interfaces/ros1/wavemap_rviz_plugin/src/visuals/voxel_visual.cc`
- `interfaces/ros1/wavemap_rviz_plugin/include/wavemap_rviz_plugin/wavemap_map_display.h`
- `interfaces/ros1/wavemap_rviz_plugin/src/wavemap_map_display.cc`
- `interfaces/ros1/wavemap_rviz_plugin/CMakeLists.txt`
- `interfaces/ros1/wavemap_rviz_plugin/plugin_description.xml`

Current RViz functionality:

- new `LayeredMap` display;
- source can be topic or file;
- topic default: `/wavemap/layered_map`;
- file loading supports `.lwvmp`;
- dynamic layer dropdown, not hardcoded to only semantic/changed;
- continuous visualization uses typed reconstruction/factory path;
- discrete visualization supports integer and bool layers;
- viewport legend/card overlay works;
- legend uses generated 1x1 textures for color swatches.

Known RViz notes:

- fixed frame should usually be `map`;
- if RViz says there is no TF data, set global fixed frame correctly;
- if launching inside container gives `qt.qpa.xcb: could not connect to display`, this is a display/X issue, not necessarily plugin failure.

### Example / Experiment Files

Implemented or modified:

- `examples/cpp/digital_twin/CMakeLists.txt`
- `examples/cpp/digital_twin/io/create_large_layered_map_experiment.cc`
- `examples/cpp/digital_twin/io/load_existing_maps_experiment.cc`
- `examples/cpp/digital_twin/io/layered_map_save_load_failure_experiment.cc`
- `examples/cpp/digital_twin/ros/publish_structured_layered_map_experiment.cc`

Useful generated test map:

```text
/home/ci/data/maps/layered_map_large.lwvmp
```

Regenerate it with:

```bash
cd ~/catkin_ws/src/wavemap
source /opt/ros/noetic/setup.bash
cmake --build examples/cpp/build --target create_large_layered_map_experiment
./examples/cpp/build/digital_twin/create_large_layered_map_experiment
```

Recent output from the large map example:

```text
populated continuous voxels: 307200
populated discrete voxels: 19128
continuous blocks before prune: 4
continuous blocks after prune: 4
continuous nodes before prune: 43912
continuous nodes after prune: 43912
semantic parents: 456 exceptions: 1474 observed: 19128
changed parents: 456 exceptions: 616 observed: 19128
file size bytes: 5850724
```

## Useful Commands

### Build RViz Plugin

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build wavemap_rviz_plugin --no-status
```

Last known result: build succeeded with no warnings.

### Launch Server

The correct dt architecture config file is:

```text
/home/ci/catkin_ws/src/dt_architecture/load_config_file.yaml
```

Launch:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch wavemap_ros wavemap_server.launch   param_file:=/home/ci/catkin_ws/src/dt_architecture/load_config_file.yaml
```

If the launch also starts RViz and RViz crashes, the server may still be fine. Check for:

```text
Layered map extension enabled. Advertising layered map topic and services.
```

### Check Layered Topics/Services

In another terminal:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice list | grep layered
rostopic list | grep layered
```

Expected:

```bash
/wavemap/load_layered_map
/wavemap/save_layered_map
/wavemap/layered_map_request_full
/wavemap/layered_map
```

### Load Layered Map Through Server

```bash
rosservice call /wavemap/load_layered_map "file_path: '/home/ci/data/maps/layered_map_large.lwvmp'"
rosservice call /wavemap/layered_map_request_full "{}"
rostopic echo -n 1 /wavemap/layered_map
```

### Open RViz Manually

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rviz
```

Then add the `LayeredMap` display and select:

```text
Topic: /wavemap/layered_map
```

Or use file mode and select:

```text
/home/ci/data/maps/layered_map_large.lwvmp
```

## Current Git Commit Recommendation

If not already committed, commit all current layered-map pipeline changes as one checkpoint:

```bash
cd ~/catkin_ws/src/wavemap
git add .
git commit -m "Add layered map ROS and RViz pipeline"
```

A previous stable commit exists:

```text
0e940190 Add layered map support
```

## What Is Still Missing

The main missing piece is the live update pipeline for extra layers.

Currently implemented:

```text
create/load LayeredMap
save/load .lwvmp
publish /wavemap/layered_map
visualize in RViz
```

Still missing:

```text
incoming live layer observations -> update LayeredMap -> publish updated /wavemap/layered_map
```

Since extra data comes per voxel, the planned path is:

1. Add ROS message(s) for layered voxel updates.
2. Add converter/helper from update message to `LayeredMap` update.
3. Add subscriber in `LayeredRosServerExtension`.
4. Update continuous layers by replacement for now.
5. Update discrete layers with `DiscreteLayer<T>::set(...)` or equivalent.
6. Publish `/wavemap/layered_map` after update.
7. Add publisher experiment for live update testing.
8. Test in RViz.

Suggested first update message, example-specific for now:

```text
LayeredVoxelUpdate
  geometry_msgs/Point position
  float32 r
  float32 g
  float32 b
  float32 traversability
  int32 semantic
  bool changed

LayeredVoxelUpdateArray
  std_msgs/Header header
  LayeredVoxelUpdate[] updates
```

Important design choice:

- first version should require update positions to already be in `world_frame`;
- TF transformation can be added later;
- original occupancy integration should remain untouched.

Live behavior target:

```text
original Wavemap ray integration -> occupancy map
layered voxel update topic       -> continuous/discrete extra layers
server publishes                 -> /wavemap/layered_map
```

Potential future work:

- sync occupancy from original occupancy map into `LayeredMap` continuous map;
- generic schema-driven update messages instead of example-specific messages;
- richer RViz color policies configured by user;
- validation with real bag data.

## Bag / External Drive Context

The user has a bag outside the environment on an external drive:

```text
/media/guilhermecabaco/CABACO_02/rosbag
```

It may require mounting/bind-mounting into the container/environment. The user does not want to copy the bag because it is too large.

Previous attempts to bind mount to `/home/ci/data/external_bag` or `/tmp/external_bag` failed with permission denied. If the environment is restarted with the bind mount configured, this Codex session may be lost. This file exists to make the work resumable.
