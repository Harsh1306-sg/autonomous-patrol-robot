#!/usr/bin/env python3

"""
Robot Navigator Helper Class
Provides utilities for robot navigation with Nav2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


class BasicNavigator(Node):
    """
    Basic Navigator class to interface with Nav2
    Provides simplified methods for navigation tasks
    """
    
    def __init__(self, node_name='basic_navigator'):
        super().__init__(node_name)
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        
        self.get_logger().info('Basic Navigator initialized')
    
    def wait_for_nav2(self, timeout_sec=10.0):
        """Wait for Nav2 action server to become available"""
        self.get_logger().info('Waiting for Nav2 action server...')
        
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Nav2 action server not available!')
            return False
        
        self.get_logger().info('Nav2 action server is ready')
        return True
    
    def goToPose(self, pose, behavior_tree=''):
        """
        Navigate to a pose
        
        Args:
            pose: PoseStamped message with target pose
            behavior_tree: Optional behavior tree XML (not used in basic implementation)
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree
        
        self.get_logger().info(f'Navigating to pose: '
                              f'x={pose.pose.position.x:.2f}, '
                              f'y={pose.pose.position.y:.2f}')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedbackCallback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False
        
        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def _feedbackCallback(self, msg):
        """Store feedback from navigation"""
        self.feedback = msg.feedback
        
        # Log distance remaining periodically
        distance = self.feedback.distance_remaining
        if distance > 0.1:  # Only log if significant distance remains
            self.get_logger().info(
                f'Distance remaining: {distance:.2f}m',
                throttle_duration_sec=5.0  # Log every 5 seconds
            )
    
    def isTaskComplete(self):
        """Check if navigation task is complete"""
        if self.result_future is None:
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
        
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                return True
            elif self.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn('Goal was aborted!')
                return True
            elif self.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Goal was canceled!')
                return True
            else:
                return False
        else:
            return False
    
    def getFeedback(self):
        """Get current navigation feedback"""
        return self.feedback
    
    def getResult(self):
        """Get navigation result"""
        if self.result_future is None:
            return None
        return self.result_future.result()
    
    def cancelTask(self):
        """Cancel current navigation task"""
        if self.goal_handle is not None:
            self.get_logger().info('Canceling navigation task...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            return True
        return False
    
    def createPoseStamped(self, x, y, yaw_degrees):
        """
        Create a PoseStamped message
        
        Args:
            x: X coordinate in map frame
            y: Y coordinate in map frame  
            yaw_degrees: Orientation in degrees
            
        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quaternion = self.euler_to_quaternion(0.0, 0.0, math.radians(yaw_degrees))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        return pose
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            List [qx, qy, qz, qw]
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    
    @staticmethod
    def quaternion_to_euler(qx, qy, qz, qw):
        """
        Convert quaternion to Euler angles
        
        Args:
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            List [roll, pitch, yaw] in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]


def main():
    """Test the BasicNavigator"""
    rclpy.init()
    
    navigator = BasicNavigator()
    
    # Wait for Nav2
    if not navigator.wait_for_nav2():
        navigator.get_logger().error('Failed to connect to Nav2')
        return
    
    # Create a test pose
    test_pose = navigator.createPoseStamped(2.0, 2.0, 45.0)
    
    # Navigate to pose
    if navigator.goToPose(test_pose):
        navigator.get_logger().info('Navigation started')
        
        # Wait for completion
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)
        
        navigator.get_logger().info('Navigation complete')
    
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
