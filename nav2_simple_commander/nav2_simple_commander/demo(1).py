#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import math
import os
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import py_trees
from .arm_control import ArmCameraGuidedGrasp, create_camera_guided_grasp_tree
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import tf2_ros
from tf2_ros import TransformException
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

class CustomNavigator(BasicNavigator):
    def __init__(self, node_name='custom_navigator', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()

        # 销毁原有的amcl_pose订阅
        if hasattr(self, 'localization_pose_sub'):
            self.destroy_subscription(self.localization_pose_sub)

        # 创建新的位姿话题订阅
        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # 订阅自定义定位话题
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/fastlo2/lio_odom',
            self._custom_pose_callback,
            pose_qos,
            callback_group=self.callback_group)

        # 设置初始位姿为未接收
        self.initial_pose_received = False
        self.current_pose = None
        self._goal_handle = None
        self._result_future = None
        self._navigation_result = TaskResult.UNKNOWN

        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 设置初始位姿的定时器
        self.create_timer(1.0, self._try_get_initial_pose)

        # 创建导航动作客户端
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer=None):
        """等待导航系统就绪，不检查AMCL"""
        self._waitForNodeToActivate(navigator)
        self.info('Nav2系统已准备就绪!')
        return

    def _waitForNodeToActivate(self, node_name):
        """等待节点激活"""
        self.info(f'等待 {node_name} 节点激活...')
        while not self._is_node_active(node_name):
            time.sleep(0.1)
        self.info(f'{node_name} 节点已激活!')

    def _is_node_active(self, node_name):
        """检查节点是否激活"""
        try:
            # 尝试获取节点状态
            node_info = self.get_node_names_and_namespaces()
            for name, namespace in node_info:
                if node_name in name:
                    return True
            return False
        except Exception as e:
            self.warn(f'检查节点状态时出错: {str(e)}')
            return False

    def _custom_pose_callback(self, msg):
        """处理自定义位姿消息"""
        self.initial_pose_received = True
        self.current_pose = msg.pose.pose
        self.debug(f'收到位姿消息: /fastlo2/lio_odom, 位置: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')

    def _try_get_initial_pose(self):
        """尝试从TF获取当前位姿并设置为初始位姿"""
        if self.initial_pose_received and self.current_pose:
            return

        try:
            # 尝试获取从map到base_link的变换
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )

            # 创建初始位姿
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = trans.transform.translation.x
            initial_pose.pose.position.y = trans.transform.translation.y
            initial_pose.pose.position.z = trans.transform.translation.z
            initial_pose.pose.orientation = trans.transform.rotation

            # 设置初始位姿
            self.setInitialPose(initial_pose)
            self.current_pose = initial_pose.pose
            self.info(f'已从TF设置初始位姿: x={initial_pose.pose.position.x:.2f}, y={initial_pose.pose.position.y:.2f}')

        except TransformException as ex:
            self.warn(f'无法获取TF变换: {ex}')

    def get_current_pose(self):
        """获取当前位姿"""
        if self.current_pose:
            return self.current_pose
        return None

    def goToPose(self, pose):
        """发送导航目标并等待结果"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.error('导航动作服务器未就绪')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        feedback = feedback_msg.feedback
        self.debug(f'导航反馈: 剩余距离={feedback.distance_remaining:.2f}米')

    def _goal_response_callback(self, future):
        """处理目标响应"""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.error('导航目标被拒绝')
            self._navigation_result = TaskResult.FAILED
            return

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """处理导航结果"""
        result = future.result().result
        if result.error_code == 0:
            self.info('导航成功完成')
            self._navigation_result = TaskResult.SUCCEEDED
        else:
            self.error(f'导航失败，错误码: {result.error_code}')
            self._navigation_result = TaskResult.FAILED

    def getResult(self):
        """获取导航结果"""
        return self._navigation_result

    def isTaskComplete(self):
        """检查任务是否完成"""
        if self._goal_handle is None:
            return False
        return self._goal_handle.is_done()

    def cancelTask(self):
        """取消导航任务"""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._navigation_result = TaskResult.CANCELED

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.publisher = self.create_publisher(String, 'demo_topic', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'facing_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Demo节点已启动')

        # 创建自定义导航器
        self.navigator = CustomNavigator()

        # 等待导航系统准备就绪
        self.navigator.waitUntilNav2Active()

        # 设置固定目标位姿
        self.goal_fixed = Pose()
        self.goal_fixed.position.x = 1.0
        self.goal_fixed.position.y = 0.0
        self.goal_fixed.position.z = 0.0
        self.goal_fixed.orientation.x = 0.0
        self.goal_fixed.orientation.y = 0.0
        self.goal_fixed.orientation.z = 0.0
        self.goal_fixed.orientation.w = 1.0
        self.get_logger().info('已设置固定目标位姿')

        # 读取shelfMap.yaml文件
        self.shelf_pose = self.read_shelf_pose()
        if self.shelf_pose:
            self.get_logger().info('成功读取货架位置信息')
            self.facing_pose = self.calculate_facing_pose(self.shelf_pose)
            self.get_logger().info(f'正对货架的位姿: x={self.facing_pose.position.x:.2f}, y={self.facing_pose.position.y:.2f}, z={self.facing_pose.position.z:.2f}')

            # 发送导航目标
            self.send_navigation_goal()

        # 初始化抓取相关变量
        self.grasp_tree = None
        self.grasp_started = False
        self.grasp_completed = False
        self.navigation_completed = False

    def send_navigation_goal(self):
        """发送导航目标并等待结果"""
        # 创建目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.goal_fixed  # 使用固定目标位姿

        # 打印目标位姿信息
        self.get_logger().info(f'目标位姿 - 位置: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}, z={goal_pose.pose.position.z:.2f}')
        self.get_logger().info(f'目标位姿 - 朝向: x={goal_pose.pose.orientation.x:.4f}, y={goal_pose.pose.orientation.y:.4f}, z={goal_pose.pose.orientation.z:.4f}, w={goal_pose.pose.orientation.w:.4f}')

        # 发送导航目标
        self.get_logger().info('发送导航目标...')
        nav_success = self.navigator.goToPose(goal_pose)

        if not nav_success:
            self.get_logger().error('导航目标被拒绝！请检查导航系统状态')
            return

        # 等待导航结果
        self.monitor_navigation_progress()

    def monitor_navigation_progress(self):
        """监控导航进度"""
        start_time = time.time()
        last_feedback_time = start_time
        feedback_timeout = 10.0  # 10秒无反馈超时
        total_timeout = 300.0  # 总超时时间（秒）
        last_distance = float('inf')
        stuck_threshold = 0.05  # 5厘米
        stuck_time = 0
        stuck_timeout = 30.0  # 30秒卡住超时

        while not self.navigator.isTaskComplete():
            # 检查总超时
            if time.time() - start_time > total_timeout:
                self.get_logger().error('导航总时间超过5分钟，取消任务')
                self.navigator.cancelTask()
                break

            # 获取反馈
            feedback = self.navigator.getFeedback()
            current_pose = self.navigator.get_current_pose()

            if feedback and hasattr(feedback, 'distance_remaining'):
                remaining = feedback.distance_remaining
                self.get_logger().info(f'导航进度: 剩余{remaining:.2f}米')
                
                # 检查是否卡住
                if abs(remaining - last_distance) < stuck_threshold:
                    stuck_time += 0.5
                    if stuck_time > stuck_timeout:
                        self.get_logger().error('机器人可能卡住，取消导航')
                        self.navigator.cancelTask()
                        break
                else:
                    stuck_time = 0
                
                last_distance = remaining
                last_feedback_time = time.time()

                # 打印当前位置
                if current_pose:
                    self.get_logger().info(f'当前位置: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}')
            else:
                if time.time() - last_feedback_time > feedback_timeout:
                    self.get_logger().warn('导航反馈超时，但任务仍在进行...')
                    last_feedback_time = time.time()

            time.sleep(0.5)

        # 获取最终结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功完成！')
            self.navigation_completed = True
            self.start_grasp_operation()
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')
        else:
            self.get_logger().error(f'未知导航结果: {result}')

    def start_grasp_operation(self):
        """启动抓取操作"""
        if not self.grasp_started:
            self.get_logger().info('开始抓取操作...')
            # 创建抓取行为树
            self.grasp_tree = create_camera_guided_grasp_tree(
                name="GraspOperation",
                node=self,
                april_code="5",
                prompt="silver metal sheet"
            )
            self.grasp_started = True
            # 开始执行抓取行为树
            self.execute_grasp_tree()

    def execute_grasp_tree(self):
        """执行抓取行为树"""
        if self.grasp_tree:
            # 执行行为树
            status = self.grasp_tree.tick()
            
            # 检查执行状态
            if status == py_trees.common.Status.SUCCESS:
                self.get_logger().info('抓取操作成功完成！')
                self.grasp_completed = True
            elif status == py_trees.common.Status.FAILURE:
                self.get_logger().error('抓取操作失败！')
                self.grasp_completed = True
            elif status == py_trees.common.Status.RUNNING:
                # 继续执行
                self.create_timer(0.1, self.execute_grasp_tree)  # 每100ms检查一次
            else:
                self.get_logger().error(f'未知抓取状态: {status}')
                self.grasp_completed = True

    def read_shelf_pose(self):
        try:
            workspace_root = os.path.expanduser('~/ir100_ws')
            yaml_path = os.path.join(workspace_root, 'src/slam/shelf_dection/config/shelfMap.yaml')
            
            self.get_logger().info(f'尝试读取YAML文件: {yaml_path}')
            
            if not os.path.exists(yaml_path):
                self.get_logger().error(f'YAML文件不存在: {yaml_path}')
                return None
                
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                
            for shelf in data['shelves']:
                if shelf['id'] == 5:
                    pose = Pose()
                    pose.position = Point(
                        x=shelf['pose']['position']['x'],
                        y=shelf['pose']['position']['y'],
                        z=shelf['pose']['position']['z']
                    )
                    pose.orientation = Quaternion(
                        x=shelf['pose']['orientation']['x'],
                        y=shelf['pose']['orientation']['y'],
                        z=shelf['pose']['orientation']['z'],
                        w=shelf['pose']['orientation']['w']
                    )
                    return pose
                    
            self.get_logger().error('未找到id为5的货架')
            return None
            
        except Exception as e:
            self.get_logger().error(f'读取YAML文件时出错: {str(e)}')
            return None

    def calculate_facing_pose(self, shelf_pose):
        """计算正对货架的位姿"""
        facing_pose = Pose()
        
        # 从四元数获取欧拉角
        qx, qy, qz, qw = shelf_pose.orientation.x, shelf_pose.orientation.y, shelf_pose.orientation.z, shelf_pose.orientation.w
        
        # 计算偏航角（yaw）
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 打印原始角度
        self.get_logger().info(f'标签原始偏航角: {math.degrees(yaw):.2f}度')
        
        # 计算正对位姿
        facing_pose.position.x = shelf_pose.position.x - 0.8  # 在x方向后退0.8米
        facing_pose.position.y = shelf_pose.position.y
        facing_pose.position.z = 0.0
        
        # 计算新的朝向（旋转180度）
        facing_yaw = yaw + math.pi
        facing_pose.orientation.x = 0.0
        facing_pose.orientation.y = 0.0
        facing_pose.orientation.z = math.sin(facing_yaw / 2.0)
        facing_pose.orientation.w = math.cos(facing_yaw / 2.0)
        
        return facing_pose

    def timer_callback(self):
        """定时器回调函数"""
        msg = String()
        if self.shelf_pose:
            msg.data = f'货架位置: x={self.shelf_pose.position.x:.2f}, y={self.shelf_pose.position.y:.2f}, z={self.shelf_pose.position.z:.2f}'
        else:
            msg.data = '未找到货架位置信息'
        self.publisher.publish(msg)
        
        if hasattr(self, 'facing_pose'):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose = self.facing_pose
            self.pose_publisher.publish(pose_msg)
            
            status_info = []
            if self.navigation_completed:
                status_info.append("导航已完成")
            if self.grasp_started:
                status_info.append("抓取已开始")
            if self.grasp_completed:
                status_info.append("抓取已完成")
            if status_info:
                self.get_logger().info(f"当前状态: {', '.join(status_info)}")

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 