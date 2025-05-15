# Nav2 Core

This package hosts the abstract interface (virtual base classes) for plugins to be used with the following:
- global planner (e.g., `nav2_navfn_planner`)
- controller (e.g., path execution controller, e.g `nav2_dwb_controller`)
- smoother (e.g., `nav2_ceres_costaware_smoother`)
- goal checker (e.g. `simple_goal_checker`)
- behaviors (e.g. `drive_on_heading`)
- progress checker (e.g. `simple_progress_checker`)
- waypoint task executor (e.g. `take_pictures`)
- exceptions in planning and control

The purposes of these plugin interfaces are to create a separation of concern from the system software engineers and the researcher / algorithm designers. Each plugin type is hosted in a "task server" (e.g. planner, recovery, control servers) which handles requests and multiple algorithm plugin instances. The plugins are used to compute a value back to the server without having to worry about ROS 2 actions, topics, or other software utilities. A plugin designer can simply use the tools provided in the API to do their work, or create new ones if they like internally to gain additional information or capabilities.

## 设计模式

Nav2 Core 采用以下设计模式：

1. **插件模式**：定义标准接口，允许动态加载和替换算法实现
2. **策略模式**：不同算法实现相同接口，可互换使用
3. **工厂模式**：服务器使用插件加载器动态创建算法实例
4. **依赖注入模式**：通过配置函数向插件注入所需依赖
5. **生命周期模式**：所有插件遵循统一的生命周期管理

## 代码框架

### 核心接口

Nav2 Core 定义了多个关键导航组件的抽象接口：

1. **GlobalPlanner 接口**：
   - 全局路径规划器的抽象基类
   - 定义了路径创建和生命周期管理方法

2. **Controller 接口**：
   - 本地轨迹控制器的抽象基类
   - 定义了速度命令计算和路径跟踪方法

3. **Behavior 接口**：
   - 导航行为的抽象基类
   - 定义了行为执行和生命周期管理方法

4. **Smoother 接口**：
   - 路径平滑器的抽象基类
   - 定义了路径优化方法

5. **GoalChecker 接口**：
   - 目标检查器的抽象基类
   - 定义了判断机器人是否达到目标的方法

6. **ProgressChecker 接口**：
   - 进度检查器的抽象基类
   - 定义了评估机器人移动进度的方法

7. **WaypointTaskExecutor 接口**：
   - 路径点任务执行器的抽象基类
   - 定义了在路径点执行特定任务的方法

### 异常处理

Nav2 Core 还定义了导航异常类型：
- PlannerException：路径规划相关异常
- ControllerException：控制相关异常
- GoalCheckerException：目标检查相关异常

## 实现原理

### 1. 插件接口设计

所有插件接口都遵循一个统一的模式，包含以下通用方法：

```cpp
// 配置插件
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

// 清理资源
virtual void cleanup() = 0;

// 激活插件
virtual void activate() = 0;

// 停用插件
virtual void deactivate() = 0;
```

除了生命周期方法外，每个接口还定义了特定于其功能的方法：

### 2. 全局规划器接口

```cpp
/**
 * @brief 创建从起点到终点的全局路径
 * @param start 机器人的起始位姿
 * @param goal 目标位姿
 * @return 从起点到终点的路径点序列
 */
virtual nav_msgs::msg::Path createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) = 0;
```

### 3. 控制器接口

```cpp
/**
 * @brief 设置全局路径
 * @param path 全局路径
 */
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

/**
 * @brief 计算速度命令
 * @param pose 当前机器人位姿
 * @param velocity 当前机器人速度
 * @param goal_checker 目标检查器指针
 * @return 最佳速度命令
 */
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;
```

### 4. 行为接口

行为接口提供了一个简单的抽象基类，允许插件开发者根据需要实现具体行为。行为插件通常使用 TimedBehavior 模板类进一步细化此接口。

## 插件加载机制

在导航系统中，任务服务器使用 pluginlib 来加载插件：

```cpp
// 创建插件加载器
pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader_(
  "nav2_core", "nav2_core::GlobalPlanner");

// 加载插件实例
pluginlib::UniquePtr<nav2_core::GlobalPlanner> planner_ =
  planner_loader_.createUniqueInstance(planner_id);

// 配置插件
planner_->configure(node, planner_id, tf_, costmap_ros_);
```

## 使用案例

### 1. 创建自定义全局规划器

```cpp
namespace my_planner
{

class MyPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // 实现特定成员
};

}  // namespace my_planner

// 注册插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav2_core::GlobalPlanner)
```

### 2. 创建自定义控制器

```cpp
namespace my_controller
{

class MyController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // 实现特定成员
};

}  // namespace my_controller

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, nav2_core::Controller)
```

### 3. 创建自定义行为

```cpp
namespace my_behavior
{

class MyBehavior : public nav2_core::Behavior
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 行为特定方法
};

}  // namespace my_behavior

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_behavior::MyBehavior, nav2_core::Behavior)
```

通过这些接口和插件机制，Nav2 Core 包为导航系统提供了高度可扩展和模块化的架构，使研究人员和开发者能够轻松替换或扩展导航功能，而无需深入了解 ROS2 的通信和生命周期管理的复杂性。
