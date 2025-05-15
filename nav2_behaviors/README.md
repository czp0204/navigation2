# Behaviors

The `nav2_behaviors` package implements a task server for executing behaviors.

The package defines:
- A `TimedBehavior` template which is used as a base class to implement specific timed behavior action server - but not required.
- The  `Backup`, `DriveOnHeading`, `Spin` and `Wait` behaviors.

The only required class a behavior must derive from is the `nav2_core/behavior.hpp` class, which implements the pluginlib interface the behavior server will use to dynamically load your behavior. The `nav2_behaviors/timed_behavior.hpp` derives from this class and implements a generic action server for a timed behavior behavior (e.g. calls an implmentation function on a regular time interval to compute a value) but **this is not required** if it is not helpful. A behavior does not even need to be an action if you do not wish, it may be a service or other interface. However, most motion and behavior primitives are probably long-running and make sense to be modeled as actions, so the provided `timed_behavior.hpp` helps in managing the complexity to simplify new behavior development, described more below.

The value of the centralized behavior server is to **share resources** amongst several behaviors that would otherwise be independent nodes. Subscriptions to TF, costmaps, and more can be quite heavy and add non-trivial compute costs to a robot system. By combining these independent behaviors into a single server, they may share these resources while retaining complete independence in execution and interface.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-behavior-server.html) for additional parameter descriptions and a [tutorial about writing behaviors](https://navigation.ros.org/plugin_tutorials/docs/writing_new_behavior_plugin.html).

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins.

## 设计模式

Nav2 Behaviors 采用以下设计模式：

1. **工厂模式**：通过插件系统动态加载不同行为
2. **模板方法模式**：使用 TimedBehavior 模板定义行为的基本骨架
3. **策略模式**：不同行为实现不同的运动策略
4. **动作模式**：使用 ROS2 Action 接口实现长时间运行的行为
5. **生命周期模式**：遵循 ROS2 生命周期管理

## 代码框架

### 核心组件

1. **BehaviorServer 类**：
   - 行为服务器主类，继承自 nav2_util::LifecycleNode
   - 管理行为插件的加载和生命周期
   - 提供共享资源供行为使用

2. **TimedBehavior 模板类**：
   - 基础行为模板，实现 nav2_core::Behavior 接口
   - 提供 ROS2 Action 接口
   - 实现周期性执行机制

3. **Behavior 插件**：
   - **BackUp**：后退行为
   - **DriveOnHeading**：沿指定方向行驶
   - **Spin**：旋转行为
   - **Wait**：等待行为
   - **AssistedTeleop**：辅助遥控行为

### 插件系统

BehaviorServer 利用 pluginlib 动态加载行为插件：

```cpp
pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;
std::vector<pluginlib::UniquePtr<nav2_core::Behavior>> behaviors_;
```

## 实现原理

### 1. 行为执行流程

TimedBehavior 通过以下步骤执行行为：

```cpp
void execute()
{
  // 行为初始化
  if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
    action_server_->terminate_current();
    return;
  }

  // 周期性执行
  rclcpp::WallRate loop_rate(cycle_frequency_);
  while (rclcpp::ok()) {
    // 检查取消请求
    if (action_server_->is_cancel_requested()) {
      stopRobot();
      action_server_->terminate_current();
      return;
    }

    // 检查暂停请求
    if (action_server_->is_preempt_requested()) {
      // 处理新目标
      action_server_->accept_pending();
      if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
        action_server_->terminate_current();
        return;
      }
    }

    // 执行一次行为周期
    Status status = onCycleUpdate();
    if (status == Status::SUCCEEDED) {
      // 行为完成，成功
      stopRobot();
      action_server_->succeeded_current();
      onActionCompletion();
      return;
    } else if (status == Status::FAILED) {
      // 行为完成，失败
      stopRobot();
      action_server_->terminate_current();
      onActionCompletion();
      return;
    }

    loop_rate.sleep();
  }
}
```

### 2. 行为插件实现示例 - DriveOnHeading

DriveOnHeading 行为实现了沿特定方向行驶的功能：

```cpp
Status onCycleUpdate() override
{
  // 计算已行驶距离
  double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);

  // 检查是否到达目标距离
  if (distance >= std::fabs(command_x_)) {
    this->stopRobot();
    return Status::SUCCEEDED;
  }

  // 生成速度命令
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = command_speed_;
  
  // 处理加速度限制
  // 检查是否需要减速以避免越过目标
  
  // 碰撞检测
  if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {
    this->stopRobot();
    return Status::FAILED;
  }

  // 发布速度命令
  this->vel_pub_->publish(std::move(cmd_vel));
  return Status::RUNNING;
}
```

### 3. 碰撞检测

行为通过 CostmapTopicCollisionChecker 进行碰撞检测：

```cpp
bool isCollisionFree(
  const double & distance,
  geometry_msgs::msg::Twist * cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // 在当前速度和位置下模拟机器人未来位置
  // 检查未来位置是否会发生碰撞
  
  // 如果碰撞即将发生，返回 false
  return collision_free;
}
```

## 关键参数

### 全局参数

- `cycle_frequency`：行为执行的循环频率
- `global_frame`：全局坐标系
- `robot_base_frame`：机器人基座坐标系
- `transform_tolerance`：坐标变换容忍度
- `simulate_ahead_time`：碰撞检测前视时间

### 行为特定参数

1. **Backup**：
   - `speed`：后退速度
   - `acceleration_limit`：加速度限制
   - `deceleration_limit`：减速度限制
   - `distance`：后退距离

2. **Spin**：
   - `spin_dist`：旋转角度
   - `angular_vel_percent`：角速度百分比
   - `time_allowance`：最大时间允许值

3. **DriveOnHeading**：
   - `speed`：行驶速度
   - `distance`：行驶距离
   - `time_allowance`：最大时间允许值

## 使用案例

### 1. 基本行为执行

```cpp
// 后退行为目标
auto goal = BackUpAction::Goal();
goal.target.x = -0.5;  // 后退0.5米
goal.speed = 0.1;      // 0.1 m/s的速度

// 发送目标到行为服务器
auto future = backup_action_client_->async_send_goal(goal);
```

### 2. 行为组合

行为可以通过行为树组合成更复杂的行为序列：

```xml
<BehaviorTree ID="RecoveryActions">
  <Sequence name="Recover">
    <Spin spin_dist="1.57" time_allowance="5"/>
    <Wait wait_duration="2"/>
    <BackUp backup_dist="0.3" backup_speed="0.1"/>
  </Sequence>
</BehaviorTree>
```

### 3. 自主恢复

当导航失败时，可以使用行为服务器执行恢复动作：

```cpp
// 导航失败时的恢复策略
void recoverFromFailure()
{
  // 先尝试旋转以搜索新路径
  auto spin_goal = SpinAction::Goal();
  spin_goal.target_yaw = 6.28;  // 360度
  spin_client_->async_send_goal(spin_goal);
  
  // 如果仍然失败，尝试后退
  auto backup_goal = BackUpAction::Goal();
  backup_goal.target.x = -0.5;
  backup_client_->async_send_goal(backup_goal);
}
```

### 4. 辅助遥控

AssistedTeleop 行为可以帮助操作员安全地控制机器人，避免碰撞：

```cpp
// 创建辅助遥控目标
auto teleop_goal = AssistedTeleopAction::Goal();
teleop_goal.time_allowance = rclcpp::Duration(0.0, 0);  // 无时间限制

// 激活辅助遥控模式
teleop_client_->async_send_goal(teleop_goal);
```

通过这些组件和机制，Nav2 Behaviors 提供了一个灵活且可扩展的框架，用于实现各种导航行为原语，可以单独使用或组合成更复杂的机器人行为。
