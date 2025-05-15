# AMCL
Adaptive Monte Carlo Localization (AMCL) is a probabilistic localization module which estimates the position and orientation (i.e. Pose) of a robot in a given known map using a 2D laser scanner. This is largely a refactored port from ROS 1 without any algorithmic changes.

See the [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-amcl.html) for more details about configurable settings and their meanings.

## 设计模式

Nav2 AMCL 采用以下设计模式：

1. **策略模式**：通过插件系统支持不同的运动模型和传感器模型
2. **观察者模式**：使用消息过滤器处理传感器数据
3. **单例模式**：粒子滤波器作为单一实例
4. **生命周期模式**：遵循 ROS2 生命周期管理

## 代码框架

### 核心组件

1. **AmclNode 类**：
   - 主要节点类，实现 nav2_util::LifecycleNode 接口
   - 协调粒子滤波器、传感器和运动模型
   - 管理 ROS2 接口和坐标变换

2. **粒子滤波器 (pf_t)**：
   - 核心定位算法实现
   - 管理粒子集合和重采样
   - 计算位姿估计和协方差

3. **传感器模型**：
   - 激光雷达传感器模型
   - 评估观测与地图的匹配程度

4. **运动模型**：
   - 差分驱动运动模型
   - 全向运动模型
   - 预测机器人的运动

### 插件系统

AMCL 使用 pluginlib 加载不同的运动模型：

```cpp
pluginlib::ClassLoader<nav2_amcl::MotionModel> plugin_loader_{"nav2_amcl",
  "nav2_amcl::MotionModel"};
```

支持的运动模型包括：
- 差分驱动模型 (Differential)
- 全向驱动模型 (Omni)

## 实现原理

### 1. 蒙特卡洛定位算法

AMCL 的核心是自适应蒙特卡洛定位算法，通过以下步骤进行定位：

1. **粒子初始化**：
   - 全局定位：在整个地图上均匀分布粒子
   - 局部定位：根据初始位姿和协方差初始化粒子

2. **预测步骤**：
   ```cpp
   void updateFilter(
     const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
     const pf_vector_t & pose)
   {
     // 应用运动模型，预测粒子的新位置
     // 根据里程计数据更新粒子
   }
   ```

3. **更新步骤**：
   ```cpp
   void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data)
   {
     // 使用传感器数据更新粒子权重
     // 评估每个粒子与观测的匹配程度
   }
   ```

4. **重采样**：
   ```cpp
   void pf_update_resample(pf_t * pf, void * random_pose_data)
   {
     // 根据权重重采样粒子
     // 适应性调整采样策略
   }
   ```

5. **位姿估计**：
   ```cpp
   bool getMaxWeightHyp(
     std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps)
   {
     // 计算最可能的位姿假设
     // 估计位姿和协方差
   }
   ```

### 2. 自适应采样

AMCL 使用自适应采样机制调整粒子数量：

```cpp
// 快速和慢速权重更新
pf->w_fast = pf->w_fast + pf->alpha_fast * (set->weight - pf->w_fast);
pf->w_slow = pf->w_slow + pf->alpha_slow * (set->weight - pf->w_slow);

// 基于KLD计算所需粒子数
if (pf->w_fast < pf->w_slow) {
  // 如果快速平均权重低于慢速平均权重，增加粒子数量
  // 表明机器人可能被"绑架"或快速移动
  sample_count = pf->max_samples;
} else {
  // 位置良好估计，可以减少粒子数量
  sample_count = pf->min_samples;
}
```

### 3. 坐标变换发布

AmclNode 发布地图到里程计的变换：

```cpp
void sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  // 计算地图到里程计的变换
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::impl::transform2stamped(latest_tf_, tmp_tf_stamped);
  
  // 发布变换
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}
```

## 关键参数

1. **粒子滤波器参数**：
   - `min_particles`：最小粒子数
   - `max_particles`：最大粒子数
   - `alpha_slow`：慢速平均权重更新系数
   - `alpha_fast`：快速平均权重更新系数

2. **运动模型参数**：
   - `alpha1-5`：运动模型噪声参数
   - `robot_model_type`：机器人运动模型类型

3. **传感器模型参数**：
   - `laser_model_type`：激光雷达模型类型
   - `laser_max_range`：激光雷达最大范围
   - `z_hit/z_rand/z_short/z_max`：传感器模型权重

4. **帧和坐标参数**：
   - `global_frame_id`：全局坐标系 ID
   - `odom_frame_id`：里程计坐标系 ID
   - `base_frame_id`：机器人基座坐标系 ID

## 使用案例

1. **机器人初始化定位**：
   - 设置初始位姿和协方差
   - 局部粒子云定位

2. **全局定位**：
   - 在未知初始位置的情况下定位
   - 粒子均匀分布在地图上

3. **跟踪定位**：
   - 实时跟踪机器人位置
   - 适应性调整粒子数量

4. **自动恢复**：
   - 检测到"绑架"（机器人被移动）时增加粒子数
   - 重新收敛到正确位置

通过这些组件和机制，Nav2 AMCL 提供了一个高效且可靠的机器人定位解决方案，能够在已知地图中准确估计机器人的位置和朝向。
