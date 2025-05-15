# Nav2 Lifecycle Manager

### Background on lifecycle enabled nodes
Using ROS2's managed/lifecycle nodes feature allows the system startup to ensure that all required nodes have been instantiated correctly before they begin their execution. Using lifecycle nodes also allows nodes to be restarted or replaced on-line. More details about managed nodes can be found on [ROS2 Design website](https://design.ros2.org/articles/node_lifecycle.html). Several nodes in Nav2, such as map_server, planner_server, and controller_server, are lifecycle enabled. These nodes provide the required overrides of the lifecycle functions: ```on_configure()```, ```on_activate()```, ```on_deactivate()```, ```on_cleanup()```, ```on_shutdown()```, and ```on_error()```.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) for additional parameter descriptions.

### nav2_lifecycle_manager
Nav2's lifecycle manager is used to change the states of the lifecycle nodes in order to achieve a controlled _startup_, _shutdown_, _reset_, _pause_, or _resume_ of the navigation stack. The lifecycle manager presents a ```lifecycle_manager/manage_nodes``` service, from which clients can invoke the startup, shutdown, reset, pause, or resume functions. Based on this service request, the lifecycle manager calls the necessary lifecycle services in the lifecycle managed nodes. Currently, the RVIZ panel uses this ```lifecycle_manager/manage_nodes``` service when user presses the buttons on the RVIZ panel (e.g.,startup, reset, shutdown, etc.), but it is meant to be called on bringup through a production system application.

In order to start the navigation stack and be able to navigate, the necessary nodes must be configured and activated. Thus, for example when _startup_ is requested from the lifecycle manager's manage_nodes service, the lifecycle managers calls _configure()_ and _activate()_ on the lifecycle enabled nodes in the node list. These are all transitioned in ordered groups for bringup transitions, and reverse ordered groups for shutdown transitions.

The lifecycle manager has a default nodes list for all the nodes that it manages. This list can be changed using the lifecycle manager's _"node_names"_ parameter.

The diagram below shows an _example_ of a list of managed nodes, and how it interfaces with the lifecycle manager.
<img src="./doc/diagram_lifecycle_manager.JPG" title="" width="100%" align="middle">

The UML diagram below shows the sequence of service calls once the _startup_ is requested from the lifecycle manager.

<img src="./doc/uml_lifecycle_manager.JPG" title="Lifecycle manager UML diagram" width="100%" align="middle">

## 设计模式

Nav2 Lifecycle Manager 采用以下设计模式：

1. **服务-客户端模式**：提供服务让客户端可以控制节点的生命周期状态
2. **观察者模式**：使用 Bond 连接监控被管理节点的健康状态
3. **组合模式**：管理多个生命周期节点作为一个整体系统
4. **命令模式**：封装各种生命周期转换操作为统一的命令接口

## 代码框架

主要包含两个核心类：

### 1. LifecycleManager 类 (lifecycle_manager.hpp/cpp)
- 核心管理器实现
- 提供服务接口控制节点生命周期
- 主要成员与功能：
  - `node_map_`：被管理的节点映射表
  - `bond_map_`：与节点的 Bond 连接映射表
  - `startup()`：启动节点
  - `shutdown()`：关闭节点
  - `pause()`：暂停节点
  - `resume()`：恢复节点
  - `reset()`：重置节点
  - `changeStateForAllNodes()`：改变所有节点状态

### 2. LifecycleManagerClient 类 (lifecycle_manager_client.hpp/cpp)
- 客户端接口实现
- 提供简便的 API 控制生命周期
- 主要接口：
  - `startup()`：启动系统
  - `shutdown()`：关闭系统
  - `pause()`：暂停系统
  - `resume()`：恢复系统
  - `reset()`：重置系统
  - `is_active()`：检查系统状态

## 实现原理

### 1. 生命周期管理流程

生命周期管理器实现了以下主要流程：

1. **系统启动 (startup)**：
   ```cpp
   bool LifecycleManager::startup()
   {
     message("Starting managed nodes bringup...");
     if (!changeStateForAllNodes(TRANSITION_CONFIGURE) ||
         !changeStateForAllNodes(TRANSITION_ACTIVATE))
     {
       return false;
     }
     return true;
   }
   ```

2. **系统关闭 (shutdown)**：
   ```cpp
   bool LifecycleManager::shutdown()
   {
     message("Shutting down managed nodes...");
     if (!changeStateForAllNodes(TRANSITION_DEACTIVATE) ||
         !changeStateForAllNodes(TRANSITION_CLEANUP) ||
         !changeStateForAllNodes(TRANSITION_UNCONFIGURED_SHUTDOWN))
     {
       return false;
     }
     return true;
   }
   ```

### 2. 故障监控与恢复

Lifecycle Manager 使用 Bond 机制监控所有管理的节点：

1. **创建 Bond 连接**：
   - 为每个受管节点创建 Bond 连接
   - 定期检查连接状态
   
2. **故障处理**：
   - 当 Bond 断开时，表示节点不再响应
   - 可以配置自动重连或发出诊断警告

3. **诊断功能**：
   - 提供系统活动状态的诊断信息
   - 使用 ROS2 诊断框架报告健康状态

## 关键参数

- `node_names`：要管理的节点名称列表
- `autostart`：是否自动启动所有节点
- `attempt_respawn_reconnection`：断开连接时是否尝试重新连接
- `bond_timeout`：Bond 超时时间

通过配置这些参数，可以控制 Nav2 系统的启动行为和故障恢复策略。