#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import math
import os
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import py_trees
from .arm_control import ArmCameraGuidedGrasp, create_camera_guided_grasp_tree

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.publisher = self.create_publisher(String, 'demo_topic', 10)
        # 添加位姿发布者
        self.pose_publisher = self.create_publisher(PoseStamped, 'facing_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Demo node has been started')
        
        # 创建导航器
        self.navigator = BasicNavigator()
        
        # 读取shelfMap.yaml文件
        self.shelf_pose = self.read_shelf_pose()
        if self.shelf_pose:
            self.get_logger().info('成功读取货架位置信息')
            # 计算正对货架的位姿
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
        # 等待导航系统准备就绪
        # self.navigator.waitUntilNav2Active()
        self.navigator.initial_pose_received = True
        
        # 创建目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.facing_pose
        
        # 发送导航目标
        self.get_logger().info('发送导航目标...')
        self.navigator.goToPose(goal_pose)
        
        
        # 等待导航结果
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'导航进度: {feedback.distance_remaining:.2f}米')
            
        # 获取最终结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功完成！')
            self.navigation_completed = True
            # 导航完成后启动抓取操作
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
            # 使用工作空间根目录作为基准
            workspace_root = os.path.expanduser('~/ir100_ws')
            yaml_path = os.path.join(workspace_root, 'src/slam/shelf_dection/config/shelfMap.yaml')
            
            self.get_logger().info(f'尝试读取YAML文件: {yaml_path}')
            
            if not os.path.exists(yaml_path):
                self.get_logger().error(f'YAML文件不存在: {yaml_path}')
                return None
                
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                
            # 查找id为5的货架
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
        """
        计算正对AprilTag的位姿
        1. z轴位置设为0
        2. 在标签前方0.5米处（x方向）
        3. 只考虑水平面旋转（偏航角），x,y设为0
        """
        facing_pose = Pose()
        
        # 从四元数获取欧拉角
        qx, qy, qz, qw = shelf_pose.orientation.x, shelf_pose.orientation.y, shelf_pose.orientation.z, shelf_pose.orientation.w
        
        # 计算偏航角（yaw）
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 打印原始角度
        self.get_logger().info(f'标签原始偏航角: {math.degrees(yaw):.2f}度')
        
        # 简化位置计算：只在x方向调整0.5米，z设为0
        facing_pose.position.x = shelf_pose.position.x - 0.8  # 在x方向后退0.8米
        facing_pose.position.y = shelf_pose.position.y  # y保持不变
        facing_pose.position.z = 0.0  # z设为0
        
        # 只考虑水平面旋转，x,y设为0
        facing_pose.orientation.x = 0.0
        facing_pose.orientation.y = 0.0
        
        # 计算新的z、w分量（旋转180度）
        facing_yaw = yaw + math.pi
        facing_pose.orientation.z = math.sin(facing_yaw / 2.0)
        facing_pose.orientation.w = math.cos(facing_yaw / 2.0)
        
        # 打印最终位姿信息
        self.get_logger().info(f'正对位姿 - 位置: x={facing_pose.position.x:.2f}, y={facing_pose.position.y:.2f}, z={facing_pose.position.z:.2f}')
        self.get_logger().info(f'正对位姿 - 朝向: x={facing_pose.orientation.x:.4f}, y={facing_pose.orientation.y:.4f}, z={facing_pose.orientation.z:.4f}, w={facing_pose.orientation.w:.4f}')
        
        return facing_pose

    def timer_callback(self):
        """定时器回调函数"""
        # 发布字符串消息
        msg = String()
        if self.shelf_pose:
            msg.data = f'货架位置: x={self.shelf_pose.position.x:.2f}, y={self.shelf_pose.position.y:.2f}, z={self.shelf_pose.position.z:.2f}'
        else:
            msg.data = '未找到货架位置信息'
        self.publisher.publish(msg)
        
        # 发布位姿消息
        if hasattr(self, 'facing_pose'):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose = self.facing_pose
            self.pose_publisher.publish(pose_msg)
            
            # 输出当前状态信息
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