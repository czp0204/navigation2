# Map Server

The `Map Server` provides maps to the rest of the Nav2 system using both topic and
service interfaces. Map server will expose maps on the node bringup, but can also change maps using a `load_map` service during run-time, as well as save maps using a `save_map` server.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-map-server.html) for additional parameter descriptions.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework.

Currently map server divides into tree parts:

- `map_server`
- `map_saver`
- `map_io` library

`map_server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`map_saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`map_io` - is a map input-output library. The library is designed to be an object-independent
in order to allow easily save/load map from external code just by calling necessary function.
This library is also used by `map_loader` and `map_saver` to work. Currently it contains
OccupancyGrid saving/loading functions moved from the rest part of map server code.
It is designed to be replaceable for a new IO library (e.g. for library with new map encoding
method or any other library supporting costmaps, multifloor maps, etc...).

### CLI-usage

#### Map Server

The `Map Server` is a composable ROS2 node. By default, there is a `map_server` executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1.
With ROS1, one invoked the map server and passing the map YAML filename, like this:

```
$ map_server map.yaml
```

Where the YAML file specified contained the various map metadata, such as:

```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Nav2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a
level of indirection to get the map yaml filename. For example, for a node named 'map_server',
the parameter file would look like this:

```
# map_server_params.yaml
map_server:
    ros__parameters:
        yaml_filename: "map.yaml"
```

One can invoke the map service executable directly, passing the params file on the command line,
like this:

```
$ map_server __params:=map_server_params.yaml
```

There is also possibility of having multiple map server nodes in a single process, where the parameters file would separate the parameters by node name, like this:

```
# combined_params.yaml
map_server1:
    ros__parameters:
        yaml_filename: "some_map.yaml"

map_server2:
    ros__parameters:
        yaml_filename: "another_map.yaml"
```

Then, one would invoke this process with the params file that contains the parameters for both nodes:

```
$ process_with_multiple_map_servers __params:=combined_params.yaml
```


The parameter for the initial map (yaml_filename) has to be set, but an empty string can be used if no initial map should be loaded. In this case, no map is loaded during
on_configure or published during on_activate. The _load_map_-service should the be used to load and publish a map. 


#### Map Saver

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

## Currently Supported Map Types

- Occupancy grid (nav_msgs/msg/OccupancyGrid)

## MapIO library

`MapIO` library contains following API functions declared in `map_io.hpp` to work with
OccupancyGrid maps:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the image from map file and generate an OccupancyGrid
- loadMapFromYaml(): Load the map YAML, image from map file and generate an OccupancyGrid
- saveMapToFile(): Write OccupancyGrid map to file

## Services

As in ROS navigation, the `map_server` node provides a "map" service to get the map. See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, `map_server` also now provides a "load_map" service and `map_saver` -
a "save_map" service. See nav2_msgs/srv/LoadMap.srv and nav2_msgs/srv/SaveMap.srv for details.

For using these services `map_server`/`map_saver` should be launched as a continuously running
`nav2::LifecycleNode` node. In addition to the CLI, `Map Saver` has a functionality of server
handling incoming services. To run `Map Saver` in a server mode
`nav2_map_server/launch/map_saver_server.launch.py` launch-file could be used.

Service usage examples:

```
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

## 设计模式

Nav2 Map Server 采用以下设计模式：

1. **单一职责原则**：将地图服务功能拆分为加载和保存两个独立组件
2. **服务模式**：通过 ROS2 服务接口提供地图加载和保存功能
3. **发布-订阅模式**：地图数据通过话题发布
4. **策略模式**：支持多种地图格式和保存模式
5. **生命周期模式**：遵循 ROS2 生命周期管理

## 代码框架

### 核心组件

1. **MapServer 类**：
   - 地图服务器主类，实现 nav2_util::LifecycleNode 接口
   - 加载地图并通过话题发布
   - 提供地图获取和加载服务

2. **MapSaver 类**：
   - 地图保存器，保存当前地图到文件
   - 提供命令行和服务两种接口

3. **MapIO 库**：
   - 独立的地图输入输出库
   - 提供地图文件解析和生成功能
   - 可扩展支持新的地图格式

### 文件结构

- **map_server.hpp/cpp**：地图服务器实现
- **map_saver.hpp/cpp**：地图保存器实现
- **map_io.hpp/cpp**：地图输入输出库
- **map_mode.hpp**：定义地图模式（二元、三元、比例尺）

## 实现原理

### 1. 地图加载流程

MapServer 通过以下步骤加载地图：

```cpp
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map)
{
  try {
    // 加载 YAML 文件获取地图参数
    LoadParameters load_parameters = loadMapYaml(yaml_file);
    // 根据参数加载地图图像并转换为 OccupancyGrid
    loadMapFromFile(load_parameters, map);
    return LOAD_MAP_SUCCESS;
  } catch (YAML::Exception & e) {
    // 处理异常
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    // 处理异常
    return INVALID_MAP_DATA;
  }
}
```

### 2. 地图发布机制

MapServer 在激活状态下发布地图：

```cpp
nav2_util::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  
  // 初始化消息头
  updateMsgHeader();
  
  // 如果有地图可用，发布地图
  if (map_available_) {
    occ_pub_->publish(msg_);
  }
  
  // 激活发布器
  occ_pub_->on_activate();
  
  return nav2_util::CallbackReturn::SUCCESS;
}
```

### 3. 地图保存功能

通过 MapIO 库保存地图到文件：

```cpp
bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  // 检查参数有效性
  if (save_parameters.map_file_name.empty()) {
    return false;
  }
  
  // 根据地图数据创建图像
  // 将图像保存到文件
  // 生成并保存 YAML 元数据
  
  return true;
}
```

## 关键参数

1. **地图服务器参数**：
   - `yaml_filename`：地图 YAML 文件路径
   - `frame_id`：地图坐标系 ID

2. **地图加载参数**：
   - `resolution`：地图分辨率（米/像素）
   - `origin`：地图原点 [x, y, theta]
   - `occupied_thresh`：占用阈值
   - `free_thresh`：空闲阈值
   - `negate`：是否反转像素值

3. **地图保存参数**：
   - `map_file_name`：保存的地图文件名
   - `image_format`：图像格式（pgm, png）
   - `map_mode`：地图模式（trinary, scale, raw）

## 使用案例

### 1. 基本导航场景

- 启动地图服务器加载静态地图
- 导航系统使用该地图进行定位和规划

```bash
# 启动带有地图服务器的导航系统
$ ros2 launch nav2_bringup bringup.launch.py map:=/path/to/map.yaml
```

### 2. 动态切换地图

- 在运行时通过服务切换到新地图
- 用于多楼层导航或环境变化情况

```bash
# 在运行时加载新地图
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /path/to/new_map.yaml}"
```

### 3. 地图构建和保存

- 使用 SLAM 构建地图
- 通过地图保存器保存当前地图

```bash
# 保存当前地图
$ ros2 run nav2_map_server map_saver_cli -f my_new_map
```

### 4. 地图过滤器信息服务器

除了基本的地图服务功能外，Nav2 Map Server 还支持通过 `CostmapFilterInfoServer` 提供地图过滤器信息，用于特殊区域导航行为控制（如速度限制区域、禁止区域等）。

通过这些组件和功能，Nav2 Map Server 提供了灵活且可扩展的地图服务，满足不同的导航需求。

