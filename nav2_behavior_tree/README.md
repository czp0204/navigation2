# nav2_behavior_tree

This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees for either navigation or autonomy systems. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing.

The nav2_behavior_tree module provides:
* A C++ template class for easily integrating ROS2 actions and services into Behavior Trees,
* Navigation-specific behavior tree nodes, and
* a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes for navigation or higher-level autonomy applications.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html) for additional parameter descriptions and a list of XML nodes made available in this package. Also review the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html) is also provided to explain how to create a simple BT plugin.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## The bt_action_node Template and the Behavior Tree Engine

The [bt_action_node template](include/nav2_behavior_tree/bt_action_node.hpp) allows one to easily integrate a ROS2 action into a BehaviorTree. To do so, one derives from the BtActionNode template, providing the action message type. For example,

```C++
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
    ...
};
```

The resulting node must be registered with the factory in the Behavior Tree engine in order to be available for use in Behavior Trees executed by this engine.

```C++
BehaviorTreeEngine::BehaviorTreeEngine()
{
    ...

  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");

    ...
}
```

Once a new node is registered with the factory, it is now available to the BehaviorTreeEngine and can be used in Behavior Trees. For example, the following simple XML description of a BT shows the FollowPath node in use:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ComputePathToPose goal="${goal}"/>
      <FollowPath path="${path}" controller_property="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```
The BehaviorTree engine has a run method that accepts an XML description of a BT for execution:

```C++
  BtStatus run(
    BT::Blackboard::Ptr & blackboard,
    const std::string & behavior_tree_xml,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));
```

See the code in the [BT Navigator](../nav2_bt_navigator/src/bt_navigator.cpp) for an example usage of the BehaviorTreeEngine.

For more information about the behavior tree nodes that are available in the default BehaviorTreeCPP library, see documentation here: https://www.behaviortree.dev/docs/3.8/learn-the-basics/BT_basics

## 设计模式

Nav2 Behavior Tree 采用以下设计模式：

1. **工厂模式**：通过 BehaviorTreeFactory 动态创建行为树节点
2. **模板方法模式**：使用 BtActionNode 和 BtServiceNode 模板类定义与 ROS2 交互的基本骨架
3. **组合模式**：行为树本身是组合模式的典型应用，将简单行为组合成复杂行为
4. **装饰器模式**：使用装饰器节点修改其他节点的行为
5. **访问者模式**：通过 TreeNode 的遍历和访问机制

## 代码框架

### 核心组件

1. **BehaviorTreeEngine 类**：
   - 行为树引擎，负责创建和执行行为树
   - 提供从文件或文本加载行为树的功能
   - 管理行为树的执行循环

2. **BtActionNode 模板类**：
   - 连接 ROS2 Action 和行为树节点的桥梁
   - 提供异步操作的状态管理
   - 处理 Action 的发送、取消和结果处理

3. **BtServiceNode 模板类**：
   - 连接 ROS2 Service 和行为树节点的桥梁
   - 提供服务调用的封装

4. **BtActionServer 类**：
   - 将行为树与 ROS2 Action Server 集成
   - 管理行为树的生命周期

### 插件系统

Nav2 Behavior Tree 包含丰富的插件，分为四类：

1. **动作节点（Action）**：
   - 计算路径（ComputePathToPose, ComputePathThroughPoses）
   - 跟随路径（FollowPath）
   - 导航到位置（NavigateToPose, NavigateThroughPoses）
   - 特殊行为（Spin, BackUp, Wait）

2. **条件节点（Condition）**：
   - 目标检查（GoalReached, IsStuck）
   - 路径有效性检查（IsPathValid）
   - 距离检查（DistanceTraveled）

3. **控制节点（Control）**：
   - 序列（Sequence）- 按顺序执行子节点
   - 选择（Selector）- 尝试子节点直到一个成功
   - 并行（Parallel）- 同时执行子节点
   - 恢复（Recovery）- 实现恢复行为

4. **装饰器节点（Decorator）**：
   - 重试（Retry）- 失败时重试子节点
   - 速率控制器（RateController）- 控制子节点执行频率
   - 超时（Timeout）- 为子节点设置超时

## 实现原理

### 1. 行为树节点与 ROS2 集成

BtActionNode 和 BtServiceNode 通过以下方式将 ROS2 功能集成到行为树中：

```cpp
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  BtActionNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf)
  {
    // 创建 Action 客户端
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    
    // 使用模板类型参数创建对应的 Action 客户端
    action_client_ = rclcpp_action::create_client<ActionT>(node_, xml_tag_name);
    
    // 设置初始状态
    goal_updated_ = false;
    goal_result_available_ = false;
    goal_aborted_ = false;
  }
  
  // 实现行为树节点的核心方法
  BT::NodeStatus tick() override;
  void halt() override;
};
```

### 2. 行为树执行机制

BehaviorTreeEngine 实现了行为树的执行循环：

```cpp
BtStatus BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // 确保行为树是有效的
  if (!tree) {
    return BtStatus::FAILED;
  }

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // 循环执行行为树
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    // 执行用户提供的循环回调
    if (onLoop) {
      onLoop();
    }

    // 检查是否请求取消
    if (cancelRequested && cancelRequested()) {
      // 取消所有正在执行的动作节点
      haltAllActions(tree->rootNode());
      return BtStatus::CANCELED;
    }

    // 执行一次行为树 tick
    result = tree->tickRoot();

    // 防止CPU占用过高
    std::this_thread::sleep_for(loopTimeout);
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}
```

### 3. 黑板机制

行为树中的节点通过黑板共享数据：

```cpp
// 黑板初始化示例
BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout);
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);

// 节点间数据传递示例 - 路径规划节点输出路径，供路径跟踪节点使用
blackboard->set("path", path_msg);  // ComputePathToPose节点
auto path = blackboard->get<nav_msgs::msg::Path>("path");  // FollowPath节点
```

## 使用案例

1. **导航序列**：计算路径 → 跟随路径

```xml
<Sequence name="NavigateToGoal">
  <ComputePathToPose goal="${goal}" path="${path}" planner_id="GridBased"/>
  <FollowPath path="${path}" controller_id="DWB"/>
</Sequence>
```

2. **带恢复机制的导航**：

```xml
<RecoveryNode number_of_retries="3">
  <Sequence name="NavigateToGoal">
    <ComputePathToPose goal="${goal}" path="${path}"/>
    <FollowPath path="${path}"/>
  </Sequence>
  <Sequence name="RecoveryActions">
    <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    <Spin spin_dist="1.57"/>
  </Sequence>
</RecoveryNode>
```

3. **条件控制的导航**：

```xml
<Sequence name="NavigateWithDistanceCheck">
  <ComputePathToPose goal="${goal}" path="${path}"/>
  <Fallback>
    <DistanceTraveled distance="10.0" />
    <Sequence>
      <BackUp backup_dist="0.2" backup_speed="0.05"/>
      <FollowPath path="${path}"/>
    </Sequence>
  </Fallback>
</Sequence>
```

通过这些组件和机制，Nav2 Behavior Tree 提供了一个灵活且强大的框架，用于构建复杂的导航和自主行为。
