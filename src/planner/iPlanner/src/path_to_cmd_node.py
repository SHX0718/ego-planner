#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Path to PositionCommand Bridge Node
将 iPlanner 输出的 nav_msgs/Path 转换为 quadrotor_msgs/PositionCommand
用于与 EGO-Planner 仿真器的 so3_control 控制器对接
"""

import rospy
import numpy as np
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
from quadrotor_msgs.msg import PositionCommand

class PathToCmdBridge:
    def __init__(self):
        rospy.init_node('path_to_cmd_bridge', anonymous=False)
        
        # Parameters
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 0.5)
        self.max_speed = rospy.get_param('~max_speed', 2.0)
        self.max_acc = rospy.get_param('~max_acc', 3.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        self.cmd_freq = rospy.get_param('~cmd_freq', 50.0)
        self.path_frame = rospy.get_param('~path_frame', 'base')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.target_height = rospy.get_param('~target_height', 1.0)  # 目标高度
        self.takeoff_height = rospy.get_param('~takeoff_height', 1.0)  # 起飞高度
        
        # State variables
        self.current_path = None
        self.world_path = None  # 世界坐标系下的路径
        self.current_odom = None
        self.path_index = 0
        self.last_cmd_time = rospy.Time.now()
        self.is_goal_reached = True
        self.has_taken_off = False
        self.takeoff_target = None
        self.pending_goal = None  # 待处理的目标点
        self.last_yaw = 0.0  # 上次的 yaw 角
        self.path_receive_pos = None  # 收到路径时的机器人位置
        self.path_receive_yaw = 0.0  # 收到路径时的机器人航向
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Wait for TF to be ready
        
        # Subscribers
        rospy.Subscriber('/path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/visual_slam/odom', Odometry, self.odom_callback, queue_size=1)
        
        # 目标点转换: RViz PoseStamped -> iPlanner PointStamped
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback, queue_size=1)
        self.goal_point_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        # Timer for publishing commands
        self.cmd_timer = rospy.Timer(rospy.Duration(1.0 / self.cmd_freq), self.cmd_timer_callback)
        
        rospy.loginfo("[PathToCmdBridge] Node started.")
        rospy.loginfo("[PathToCmdBridge] Use RViz '2D Nav Goal' to set goal point.")
        rospy.loginfo("[PathToCmdBridge] Will auto-takeoff to %.1fm when goal is set." % self.takeoff_height)
    
    def goal_pose_callback(self, msg):
        """将 RViz 的 PoseStamped 转换为 PointStamped 发给 iPlanner"""
        # 检查是否需要先起飞
        if self.current_odom is not None:
            current_z = self.current_odom.pose.pose.position.z
            if current_z < self.takeoff_height * 0.8:  # 还没起飞
                rospy.loginfo("[PathToCmdBridge] Taking off first... current height: %.2f" % current_z)
                # 保存目标点，等起飞完成后再发送
                self.pending_goal = msg
                self.has_taken_off = False
                # 设置起飞目标
                current_x = self.current_odom.pose.pose.position.x
                current_y = self.current_odom.pose.pose.position.y
                self.takeoff_target = np.array([current_x, current_y, self.takeoff_height])
                return
        
        # 已经起飞，直接发送目标
        self.send_goal_to_iplanner(msg)
    
    def send_goal_to_iplanner(self, msg):
        """发送目标点给 iPlanner"""
        goal_point = PointStamped()
        goal_point.header = msg.header
        goal_point.point.x = msg.pose.position.x
        goal_point.point.y = msg.pose.position.y
        goal_point.point.z = self.target_height  # 使用固定高度
        
        self.goal_point_pub.publish(goal_point)
        rospy.loginfo("[PathToCmdBridge] Goal sent to iPlanner: (%.2f, %.2f, %.2f)" % 
                      (goal_point.point.x, goal_point.point.y, goal_point.point.z))
    
    def path_callback(self, msg):
        """接收 iPlanner 发布的路径"""
        if len(msg.poses) < 2:
            return
        
        if self.current_odom is None:
            return
        
        # 保存收到路径时的机器人位姿
        self.path_receive_pos = self.get_robot_position()
        self.path_receive_yaw = self.get_robot_yaw()
        
        # 立即转换到世界坐标系并保存
        self.current_path = msg
        self.world_path = self.transform_path_to_world(msg, self.path_receive_pos, self.path_receive_yaw)
        self.path_index = 0
        self.is_goal_reached = False
        rospy.loginfo_throttle(2.0, "[PathToCmdBridge] Received path with %d points" % len(msg.poses))
    
    def odom_callback(self, msg):
        """接收里程计"""
        self.current_odom = msg
    
    def get_robot_position(self):
        """获取机器人当前位置（世界坐标系）"""
        if self.current_odom is None:
            return None
        pos = self.current_odom.pose.pose.position
        return np.array([pos.x, pos.y, pos.z])
    
    def get_robot_yaw(self):
        """获取机器人当前航向角"""
        if self.current_odom is None:
            return 0.0
        q = self.current_odom.pose.pose.orientation
        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def transform_path_to_world(self, path, robot_pos, robot_yaw):
        """将路径从机器人坐标系转换到世界坐标系"""
        if robot_pos is None:
            return None
        
        world_path = []
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)
        
        for pose in path.poses:
            # Path is in robot frame, transform to world frame
            local_x = pose.pose.position.x
            local_y = pose.pose.position.y
            local_z = pose.pose.position.z
            
            world_x = robot_pos[0] + cos_yaw * local_x - sin_yaw * local_y
            world_y = robot_pos[1] + sin_yaw * local_x + cos_yaw * local_y
            world_z = self.target_height  # 使用固定高度
            
            world_path.append(np.array([world_x, world_y, world_z]))
        
        return world_path
    
    def find_lookahead_point(self, robot_pos, world_path):
        """找到前视点"""
        if len(world_path) == 0:
            return None, -1
        
        # Find the closest point on path that is at least lookahead_dist away
        for i in range(self.path_index, len(world_path)):
            dist = np.linalg.norm(world_path[i] - robot_pos)
            if dist >= self.lookahead_dist:
                return world_path[i], i
        
        # If no point found, return the last point
        return world_path[-1], len(world_path) - 1
    
    def cmd_timer_callback(self, event):
        """定时发布位置指令"""
        if self.current_odom is None:
            return
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
        
        # 检查是否在起飞阶段
        if self.takeoff_target is not None and not self.has_taken_off:
            dist_to_takeoff = np.linalg.norm(self.takeoff_target - robot_pos)
            if dist_to_takeoff < 0.2:  # 到达起飞高度
                self.has_taken_off = True
                rospy.loginfo("[PathToCmdBridge] Takeoff complete! Height: %.2f" % robot_pos[2])
                # 发送之前保存的目标点
                if self.pending_goal is not None:
                    rospy.sleep(0.5)  # 稍等稳定
                    self.send_goal_to_iplanner(self.pending_goal)
                    self.pending_goal = None
            else:
                # 继续起飞
                velocity = (self.takeoff_target - robot_pos)
                velocity = velocity / np.linalg.norm(velocity) * min(1.0, dist_to_takeoff)
                self.publish_cmd(self.takeoff_target, velocity, np.zeros(3))
                return
        
        if self.current_path is None or self.world_path is None:
            return
        
        if self.is_goal_reached:
            return
        
        # 使用已经转换好的世界坐标系路径
        world_path = self.world_path
        if world_path is None or len(world_path) == 0:
            return
        
        # Check if goal reached
        goal_pos = world_path[-1]
        dist_to_goal = np.linalg.norm(goal_pos - robot_pos)
        if dist_to_goal < self.goal_tolerance:
            self.is_goal_reached = True
            rospy.loginfo("[PathToCmdBridge] Goal reached!")
            # Publish stop command
            self.publish_cmd(robot_pos, np.zeros(3), np.zeros(3))
            return
        
        # Find lookahead point
        target_pos, new_idx = self.find_lookahead_point(robot_pos, world_path)
        if target_pos is None:
            return
        
        if new_idx > self.path_index:
            self.path_index = new_idx
        
        # Calculate velocity direction
        direction = target_pos - robot_pos
        dist = np.linalg.norm(direction)
        if dist < 0.01:
            return
        
        direction = direction / dist
        
        # Speed control: slow down near goal
        speed = self.max_speed
        if dist_to_goal < 2.0:
            speed = max(0.3, self.max_speed * dist_to_goal / 2.0)
        
        velocity = direction * speed
        
        # Simple acceleration (towards target velocity)
        acceleration = np.zeros(3)
        
        # Publish command
        self.publish_cmd(target_pos, velocity, acceleration)
    
    def publish_cmd(self, position, velocity, acceleration):
        """发布位置指令"""
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.world_frame
        
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = 1
        
        cmd.position.x = position[0]
        cmd.position.y = position[1]
        cmd.position.z = position[2]
        
        cmd.velocity.x = velocity[0]
        cmd.velocity.y = velocity[1]
        cmd.velocity.z = velocity[2]
        
        cmd.acceleration.x = acceleration[0]
        cmd.acceleration.y = acceleration[1]
        cmd.acceleration.z = acceleration[2]
        
        # Calculate yaw from velocity direction (with smoothing)
        if np.linalg.norm(velocity[:2]) > 0.1:
            target_yaw = np.arctan2(velocity[1], velocity[0])
            # 平滑 yaw 变化，避免打转
            yaw_diff = target_yaw - self.last_yaw
            # 处理 -pi 到 pi 的跨越
            while yaw_diff > np.pi:
                yaw_diff -= 2 * np.pi
            while yaw_diff < -np.pi:
                yaw_diff += 2 * np.pi
            # 限制 yaw 变化速率
            max_yaw_rate = 0.5  # rad per cmd
            yaw_diff = np.clip(yaw_diff, -max_yaw_rate, max_yaw_rate)
            cmd.yaw = self.last_yaw + yaw_diff
            self.last_yaw = cmd.yaw
        else:
            cmd.yaw = self.last_yaw
        cmd.yaw_dot = 0.0
        
        self.cmd_pub.publish(cmd)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PathToCmdBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass
