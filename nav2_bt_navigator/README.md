# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://docs.nav2.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

## 设计模式

Nav2 BT Navigator 采用以下设计模式：

1. **策略模式**：使用行为树作为可配置的导航策略
2. **组合模式**：行为树本身是一个组合模式的典型应用，将简单行为组合成复杂行为
3. **模板方法模式**：通过 Navigator 模板类定义导航行为的基本骨架
4. **生命周期模式**：遵循 ROS2 生命周期管理

## 代码框架

### 核心组件

1. **BtNavigator 类**：
   - 主要导航节点类，实现 nav2_util::LifecycleNode 接口
   - 管理导航器插件和生命周期

2. **Navigator 类**：
   - 导航器接口的模板类，作为所有基于行为树的导航动作插件的基类
   - 提供配置、激活、任务接收和完成等功能

3. **NavigateToPose 和 NavigateThroughPoses**：
   - 两种主要的导航器实现
   - 分别处理单目标点和多目标点导航任务

### 插件系统

- **BtActionServer**：提供与行为树交互的动作服务器
- **插件加载器**：动态加载行为树节点插件
- **NavigatorMuxer**：确保一次只有一个导航任务在执行

## 实现原理

### 1. 行为树导航流程

BT Navigator 使用行为树来组织和执行导航任务的各个步骤：

1. **目标接收**：
   ```cpp
   bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
   {
     // 检查是否有其他导航任务在执行
     if (plugin_muxer_) {
       plugin_muxer_->startNavigating(getName());
     }
     // 处理目标
     return goalReceived(goal);
   }
   ```

2. **行为树执行**：
   - 读取行为树XML文件
   - 创建和配置行为树
   - 执行行为树直到完成或取消

3. **任务完成**：
   ```cpp
   void onCompletion(
     typename ActionT::Result::SharedPtr result,
     const nav2_behavior_tree::BtStatus final_bt_status)
   {
     // 处理完成状态
     goalCompleted(result, final_bt_status);
     // 释放导航器状态
     if (plugin_muxer_) {
       plugin_muxer_->stopNavigating(getName());
     }
   }
   ```

### 2. 行为树组件

行为树由不同类型的节点组成：

1. **控制节点**：决定子节点的执行顺序（序列、选择、并行等）
2. **条件节点**：检查条件（如路径是否有效、是否接近目标等）
3. **动作节点**：执行具体行动（如计算路径、执行控制等）
4. **装饰节点**：修改子节点的行为（如重试、超时等）

### 3. 导航灵活性

BT Navigator 提供了高度灵活性：

1. **可配置行为**：通过修改XML配置文件改变导航行为
2. **多种导航模式**：支持单点导航和多点导航
3. **任务特定行为树**：每个导航任务可以使用不同的行为树
4. **动态加载**：可以在运行时加载和切换不同的行为树

## 使用案例

1. **简单导航**：从当前位置到目标位置
2. **多点导航**：按顺序经过多个路径点
3. **特殊行为组合**：在导航过程中执行特定行为（如避让、暂停等）
4. **恢复行为**：当导航失败时尝试不同的恢复策略

通过行为树的灵活性，BT Navigator 可以实现比传统状态机更复杂和适应性更强的导航行为。
