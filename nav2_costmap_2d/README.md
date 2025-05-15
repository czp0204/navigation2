# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors. A plugin interface allows for the layers to be combined into the costmap and finally inflated via an inflation radius based on the robot footprint. The nav2 version of the costmap_2d package is mostly a direct ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) for additional parameter descriptions for the costmap and its included plugins. The [tutorials](https://navigation.ros.org/tutorials/index.html) and [first-time setup guides](https://navigation.ros.org/setup_guides/index.html) also provide helpful context for working with the costmap 2D package and its layers. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) is also provided to explain how to create costmap plugins.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## To visualize the voxels in RVIZ:
- Make sure `publish_voxel_map` in `voxel_layer` param's scope is set to `True`.
- Open a new terminal and run:
  ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```
    Here you can change `my_marker` to any topic name you like for the markers to be published on.

- Then add `my_marker` to RVIZ using the GUI.


### Errata:
- To see the markers in 3D, you will need to change the _view_ in RVIZ to a 3 dimensional view (e.g. orbit) from the RVIZ GUI.
- Currently due to some bug in rviz, you need to set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from a saved bag file while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## Costmap Filters

### Overview

Costmap Filters - is a costmap layer-based instrument which provides an ability to apply to map spatial-dependent raster features named as filter-masks. These features are used in plugin algorithms when filling costmaps in order to allow robots to change their trajectory, behavior or speed when a robot enters/leaves an area marked in a filter masks. Examples of costmap filters include keep-out/safety zones where robots will never enter, speed restriction areas, preferred lanes for robots moving in industries and warehouses. More information about design, architecture of the feature and how it works could be found on Nav2 website: https://navigation.ros.org.

## 设计模式

Nav2 Costmap 2D 采用以下设计模式：

1. **分层模式**：将环境表示分为多个独立的图层，每层处理不同类型的信息
2. **插件模式**：使用 pluginlib 实现可扩展的图层系统
3. **组合模式**：多个图层组合成最终的代价地图
4. **观察者模式**：传感器数据更新通知图层更新
5. **生命周期模式**：遵循 ROS2 生命周期管理

## 代码框架

### 核心组件

1. **Costmap2DROS 类**：
   - 主要接口类，管理代价地图更新和生命周期
   - 提供与 ROS2 集成的功能
   - 管理坐标变换和机器人位姿

2. **LayeredCostmap 类**：
   - 管理多个图层并将它们组合成完整的代价地图
   - 处理地图更新和边界计算
   - 管理机器人足迹 (footprint) 设置

3. **Costmap2D 类**：
   - 核心代价地图数据结构
   - 提供代价值存储和访问
   - 实现地图坐标变换功能

### 图层系统

代价地图由多个插件图层组成，每个图层继承自 `Layer` 基类：

1. **StaticLayer**：
   - 加载静态地图数据
   - 提供永久性障碍物信息

2. **ObstacleLayer**：
   - 处理来自传感器的障碍物观测
   - 动态更新障碍物信息

3. **VoxelLayer**：
   - 三维体素表示的障碍物层
   - 支持三维传感器数据

4. **InflationLayer**：
   - 基于机器人足迹膨胀障碍物
   - 创建平滑的代价梯度

5. **RangeSensorLayer**：
   - 处理距离传感器数据
   - 支持声纳等特殊传感器

### 过滤器系统

代价地图还支持过滤器插件，用于特殊区域的处理：

1. **KeepoutFilter**：禁止区域过滤器
2. **SpeedFilter**：速度限制区域过滤器
3. **PreferredLanesFilter**：首选路径过滤器

## 实现原理

### 1. 多层组合机制

LayeredCostmap 通过以下流程组合多个图层：

```cpp
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // 重置主代价地图
  if (!size_locked_) {
    primary_costmap_.resetMap(0, 0, primary_costmap_.getSizeInCellsX(), primary_costmap_.getSizeInCellsY());
  }

  // 更新每个插件图层
  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
  }

  // 如果有更新的区域，处理图层叠加
  if (minx_ < maxx_ && miny_ < maxy_) {
    // 更新边界和主代价地图
    primary_costmap_.updateOrigin(robot_x - primary_costmap_.getSizeInMetersX() / 2,
                             robot_y - primary_costmap_.getSizeInMetersY() / 2);
    
    // 处理每个图层的更新
    for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
      (*plugin)->updateCosts(primary_costmap_, minx_, miny_, maxx_, maxy_);
    }

    // 处理过滤器
    for (auto filter = filters_.begin(); filter != filters_.end(); ++filter) {
      (*filter)->updateCosts(primary_costmap_, minx_, miny_, maxx_, maxy_);
    }

    // 确保主代价地图和组合代价地图同步
    combined_costmap_.copyMap(primary_costmap_);
  }
}
```

### 2. 机器人足迹处理

代价地图使用机器人足迹进行碰撞检测和膨胀：

```cpp
void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;
  
  // 计算外接圆半径和内切圆半径
  calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  // 通知所有图层足迹已更改
  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    (*plugin)->onFootprintChanged();
  }
}
```

### 3. 传感器数据整合

ObstacleLayer 使用观测缓冲区处理传感器数据：

```cpp
void ObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // 处理激光雷达射线追踪
  // 标记自由空间和障碍物
  // 更新边界
}
```

## 关键参数

1. **全局参数**：
   - `global_frame`：代价地图的全局坐标系
   - `robot_base_frame`：机器人基座坐标系
   - `transform_tolerance`：坐标变换容忍度
   - `update_frequency`：地图更新频率
   - `publish_frequency`：地图发布频率

2. **滚动窗口参数**：
   - `rolling_window`：是否使用滚动窗口
   - `width`, `height`：地图尺寸
   - `resolution`：地图分辨率

3. **图层参数**：
   - `plugins`：启用的图层列表
   - `inflation_radius`：膨胀半径
   - `cost_scaling_factor`：代价缩放因子

## 使用案例

1. **导航场景**：
   - 全局路径规划使用全局代价地图
   - 局部控制器使用局部代价地图
   - 恢复行为使用特定区域的代价地图

2. **特殊区域处理**：
   - 使用过滤器定义禁入区域
   - 创建速度限制区域
   - 设置首选导航通道

3. **多传感器融合**：
   - 结合激光雷达数据更新障碍物
   - 使用体素层处理3D传感器数据
   - 整合多种传感器信息构建完整环境表示

通过这些组件和功能，Nav2 Costmap 2D 提供了一个灵活且强大的环境表示框架，为导航系统提供必要的障碍物和空间信息。
