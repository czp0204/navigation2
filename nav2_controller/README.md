# Nav2 Controller

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available controller plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

## 设计模式

Nav2 Controller 采用以下设计模式：

1. **插件模式**：使用 pluginlib 动态加载和管理不同的控制器和检查器插件
2. **策略模式**：允许不同的控制器实现不同的导航策略
3. **生命周期模式**：遵循 ROS2 的生命周期管理模式
4. **组合模式**：将控制器、进度检查器和目标检查器组合成一个完整的控制系统

## 代码框架

### 1. 核心组件

- **ControllerServer 类**：主要服务器类，实现 ROS2 生命周期节点
  - 管理控制器、进度检查器和目标检查器插件
  - 提供 FollowPath 动作服务
  - 协调路径执行过程

### 2. 插件接口

- **Controller 接口**：`nav2_core::Controller`
  - 所有路径跟随控制器插件必须实现的接口
  - 定义了配置、激活、停用等生命周期方法
  - 定义了核心控制计算方法 `computeVelocityCommands`

- **GoalChecker 接口**：`nav2_core::GoalChecker`
  - 检查机器人是否已达到目标
  - 支持不同类型的目标到达判定策略

- **ProgressChecker 接口**：`nav2_core::ProgressChecker`
  - 监控机器人沿路径的进度
  - 检测机器人是否被卡住或进度不足

### 3. 默认插件实现

- **SimpleProgressChecker**：简单的进度检查器实现
- **SimpleGoalChecker**：基于位置距离的目标检查器
- **StoppedGoalChecker**：检查机器人是否停在目标位置
- **PoseProgressChecker**：基于姿态变化的进度检查器

## 实现原理

### 1. 控制流程

ControllerServer 的核心控制逻辑实现在 `computeControl` 方法中：

```cpp
void ControllerServer::computeControl()
{
  // 获取当前路径
  // 循环直到达到目标或取消
  while (rclcpp::ok()) {
    // 检查路径更新
    updateGlobalPath();
    
    // 获取当前机器人位姿
    geometry_msgs::msg::PoseStamped pose;
    if (!getRobotPose(pose)) {
      // 处理错误
      continue;
    }
    
    // 计算控制命令
    computeAndPublishVelocity();
    
    // 检查是否达到目标
    if (isGoalReached()) {
      // 处理达到目标的情况
      break;
    }
    
    // 检查进度
    if (!progress_checker_->check(pose)) {
      // 处理卡住的情况
      break;
    }
  }
}
```

### 2. 控制命令计算

控制命令的计算由选定的控制器插件实现：

```cpp
void ControllerServer::computeAndPublishVelocity()
{
  // 计算速度命令
  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  
  try {
    cmd_vel_stamped = controllers_[current_controller_]->computeVelocityCommands(
      current_pose_,
      current_velocity_);
    
    // 应用速度限制
    // 发布速度命令
    publishVelocity(cmd_vel_stamped);
  } catch (nav2_core::PlannerException & e) {
    // 处理异常
  }
}
```

### 3. 目标和进度检查

目标检查和进度检查由相应的插件实现：

```cpp
bool ControllerServer::isGoalReached()
{
  // 使用目标检查器确定是否达到目标
  return goal_checkers_[current_goal_checker_]->isGoalReached(
    current_pose_,
    end_pose_,
    current_velocity_);
}
```

## 关键参数

- **`controller_frequency`**：控制循环频率
- **`controller_plugins`**：要加载的控制器插件列表
- **`goal_checker_plugins`**：要加载的目标检查器插件列表
- **`progress_checker_plugin`**：要加载的进度检查器插件
- **`min_x/y/theta_velocity_threshold`**：速度阈值参数
- **`failure_tolerance`**：控制失败容忍度

## 使用案例

1. **路径跟随**：使用不同控制器（DWB、Pure Pursuit、TEB 等）跟随全局规划器生成的路径
2. **特定运动模型**：针对不同类型机器人（差分驱动、全向、阿克曼等）使用专门的控制器插件
3. **特定目标行为**：使用不同的目标检查器实现不同的到达行为（例如需要在目标点停止或允许连续移动）

通过这种灵活的设计，Nav2 Controller 允许用户根据具体需求选择不同的控制策略，并能够轻松扩展添加新的控制器实现。
