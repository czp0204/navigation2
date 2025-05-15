# Nav2 Planner

The Nav2 planner is a Task Server in Nav2 that implements the `nav2_behavior_tree::ComputePathToPose` interface.

A planning module implementing the `nav2_behavior_tree::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It loads a map of potential planner plugins to do the path generation in different user-defined situations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-planner-server.html) for additional parameter descriptions and a [tutorial about writing planner plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html).

## 设计模式

Nav2 Planner 采用以下设计模式：

1. **插件模式**：使用 pluginlib 动态加载和管理不同的全局路径规划器插件
2. **工厂模式**：根据配置创建和管理多个规划器实例
3. **生命周期模式**：遵循 ROS2 的生命周期管理模式
4. **动作服务器模式**：通过 ROS2 Action 提供规划服务

## 代码框架

### 1. 核心组件

- **PlannerServer 类**：主要服务器类，实现 ROS2 生命周期节点
  - 管理规划器插件的加载和使用
  - 提供 Action 服务来响应路径规划请求
  - 处理从起点到终点的单目标规划
  - 处理多路径点序列的规划

### 2. 接口

- **全局规划器插件接口**：`nav2_core::GlobalPlanner`
  - 所有规划器插件必须实现的接口
  - 定义了配置、激活、停用等生命周期方法
  - 定义了核心规划功能 `createPlan`

### 3. 动作服务

- **ComputePathToPose**：单目标规划动作服务
- **ComputePathThroughPoses**：多路径点规划动作服务
- **IsPathValid**：路径有效性检查服务

## 实现原理

### 1. 插件管理

PlannerServer 使用 pluginlib 加载和管理全局规划器插件：

```cpp
pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
PlannerMap planners_;  // 规划器映射表
```

这允许用户配置多个规划器并在不同情况下选择合适的规划器。

### 2. 规划流程

1. **配置阶段**：
   - 加载配置的规划器插件
   - 初始化代价地图
   - 设置相关参数

2. **规划过程**：
   ```cpp
   nav_msgs::msg::Path getPlan(
     const geometry_msgs::msg::PoseStamped & start,
     const geometry_msgs::msg::PoseStamped & goal,
     const std::string & planner_id);
   ```
   
   - 接收起点和目标点
   - 选择适当的规划器插件
   - 调用插件的 `createPlan` 方法
   - 返回规划路径

3. **验证路径**：
   - 检查路径的有效性
   - 确保路径包含有效点
   - 发布路径以供可视化

### 3. 多点规划

为了支持通过多个路径点的规划，PlannerServer 实现了以下策略：

1. **顺序规划**：依次规划相邻点之间的路径
2. **路径合并**：将各段路径合并为完整路径
3. **平滑处理**：确保路径点之间的连续性

## 关键参数

- **`planner_plugins`**：要加载的规划器插件列表
- **`planner_plugin_ids`**：规划器插件 ID 列表
- **`expected_planner_frequency`**：规划器的预期频率
- **`max_planner_duration`**：规划操作的最大持续时间

## 使用案例

1. **单目标导航**：使用 `ComputePathToPose` 动作计算从起点到终点的路径
2. **多点导航**：使用 `ComputePathThroughPoses` 动作计算经过多个路径点的路径
3. **路径验证**：使用 `IsPathValid` 服务验证现有路径的有效性

通过这种设计，Nav2 Planner 提供了一个灵活且可扩展的框架，允许用户选择和配置不同的全局路径规划算法。
