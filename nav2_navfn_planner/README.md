# Navfn Planner

The NavfnPlanner is a global planner plugin for the Nav2 Planner server. It implements the Navigation Function planner with either A\* or Dijkstra expansions. It is largely equivalent to its counterpart in ROS 1 Navigation. The Navfn planner assumes a circular robot (or a robot that can be approximated as circular for the purposes of global path planning) and operates on a weighted costmap.

## 设计模式

NavfnPlanner 采用以下设计模式：

1. **插件模式**：实现了 `nav2_core::GlobalPlanner` 接口，作为一个全局规划器插件
2. **生命周期模式**：遵循 ROS2 的生命周期管理（configure/activate/deactivate/cleanup）
3. **策略模式**：支持 A* 和 Dijkstra 两种路径规划算法的切换
4. **单例模式**：每个规划器实例维护自己的代价地图和参数

## 代码框架

主要包含两个核心类：

### 1. NavFn 类 (navfn.hpp/cpp)
- 核心算法实现
- 负责路径规划的底层计算
- 主要成员：
  - `costarr`: 2D配置空间中的代价数组
  - `potarr`: 导航函数的势场数组
  - `gradx/grady`: 梯度数组
  - `pathx/pathy`: 路径点坐标

### 2. NavfnPlanner 类 (navfn_planner.hpp/cpp)
- 插件接口实现
- 生命周期管理
- 参数配置
- 主要接口：
  - `createPlan`: 创建路径规划
  - `makePlan`: 实际的规划实现
  - `computePotential`: 计算导航函数
  - `getPlanFromPotential`: 从势场中提取路径

## 算法实现

### 1. 导航函数计算

NavFn 使用以下两种算法之一来计算路径：

1. **Dijkstra 算法** (`calcNavFnDijkstra`)：
   - 使用广度优先搜索
   - 从目标点开始向外扩展
   - 计算到达每个点的最小代价

2. **A* 算法** (`calcNavFnAstar`)：
   - 使用启发式搜索
   - 使用欧几里得距离作为启发函数
   - 通常比 Dijkstra 更快

### 2. 路径生成过程

1. 代价地图处理：
```cpp
void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);
```

2. 势场计算：
```cpp
bool computePotential(const geometry_msgs::msg::Point & world_point);
```

3. 梯度下降：
```cpp
float gradCell(int n);  // 计算单元格的梯度
```

4. 路径平滑：
```cpp
void smoothApproachToGoal(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan);
```

## 关键参数

- `tolerance`: 目标点附近的容差范围
- `use_astar`: 是否使用 A* 算法（否则使用 Dijkstra）
- `allow_unknown`: 是否允许规划穿过未知区域
- `use_final_approach_orientation`: 是否使用最终接近姿态

## 性能考虑

1. 代价计算：
   - `COST_NEUTRAL = 50`: 开放空间的基础代价
   - `COST_FACTOR = 0.8`: 代价地图值的转换因子

2. 优化策略：
   - 使用优先级缓冲区进行扩展
   - 支持动态参数调整
   - 对路径终点进行平滑处理

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-navfn.html) for additional parameter descriptions.
