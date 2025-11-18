#!/usr/bin/env python3

"""
Simple Standalone Patrol Robot Script
Just run this after launching TurtleBot3 with Nav2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import os
from datetime import datetime
from pathlib import Path
import math


class SimplePatrol(Node):
    def __init__(self):
        super().__init__('simple_patrol')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Simple waypoints for TurtleBot3 empty world
        self.waypoints = [
            [0.0, 0.0, 0.0],      # Start
            [1.5, 0.0, 90.0],     # Point 1
            [1.5, 1.5, 180.0],    # Point 2
            [0.0, 1.5, -90.0],    # Point 3
            [0.0, 0.0, 0.0],      # Back to start
        ]
        
        self.current_index = 0
        self.patrol_cycle = 0
        self.max_cycles = 2
        
        # Setup logging
        self.log_dir = os.path.expanduser('~/patrol_logs')
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)
        self.log_file = os.path.join(
            self.log_dir,
            f'patrol_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        )
        self.logs = []
        
        self.get_logger().info('🤖 Simple Patrol Robot Started!')
        self.get_logger().info(f'📍 Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'🔄 Cycles to complete: {self.max_cycles}')
        
    def euler_to_quaternion(self, yaw_deg):
        yaw = math.radians(yaw_deg)
        return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    
    def create_pose(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = 0.0
        
        quat = self.euler_to_quaternion(waypoint[2])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def log_visit(self, index, status):
        wp = self.waypoints[index]
        entry = {
            'time': datetime.now().isoformat(),
            'cycle': self.patrol_cycle,
            'waypoint': index,
            'x': wp[0],
            'y': wp[1],
            'yaw': wp[2],
            'status': status
        }
        self.logs.append(entry)
        
        with open(self.log_file, 'w') as f:
            json.dump(self.logs, f, indent=2)
        
        self.get_logger().info(
            f'✅ [{self.patrol_cycle}] WP{index}: ({wp[0]:.1f}, {wp[1]:.1f}) - {status}'
        )
    
    def send_goal(self, index):
        wp = self.waypoints[index]
        self.get_logger().info(f'🎯 Going to waypoint {index}: ({wp[0]:.1f}, {wp[1]:.1f})')
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 not available!')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(wp)
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response)
        return True
    
    def goal_response(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected')
            self.log_visit(self.current_index, 'REJECTED')
            self.next_waypoint()
            return
        
        self.get_logger().info('⏳ Goal accepted, moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)
    
    def goal_result(self, future):
        status = future.result().status
        
        if status == 4:  # SUCCESS
            self.get_logger().info(f'✅ Reached waypoint {self.current_index}!')
            self.log_visit(self.current_index, 'SUCCESS')
        else:
            self.get_logger().warn(f'⚠️  Failed with status {status}')
            self.log_visit(self.current_index, 'FAILED')
        
        self.next_waypoint()
    
    def next_waypoint(self):
        self.current_index += 1
        
        if self.current_index >= len(self.waypoints):
            self.patrol_cycle += 1
            
            if self.patrol_cycle >= self.max_cycles:
                self.finish()
                return
            
            self.get_logger().info(f'🔄 Starting cycle {self.patrol_cycle}')
            self.current_index = 0
        
        self.send_goal(self.current_index)
    
    def finish(self):
        self.get_logger().info('🎉 PATROL COMPLETE!')
        self.get_logger().info(f'📊 Total waypoints visited: {len(self.logs)}')
        self.get_logger().info(f'📁 Log saved: {self.log_file}')
        
        # Print summary
        success = sum(1 for log in self.logs if log['status'] == 'SUCCESS')
        self.get_logger().info(f'✅ Success rate: {success}/{len(self.logs)}')
    
    def start(self):
        self.get_logger().info('🚀 Starting patrol in 3 seconds...')
        self.get_logger().info('⚠️  Make sure you set the initial pose in RViz!')
        
        # Wait a bit then start
        import time
        time.sleep(3)
        
        self.send_goal(self.current_index)


def main():
    rclpy.init()
    patrol = SimplePatrol()
    
    try:
        patrol.start()
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        patrol.get_logger().info('⛔ Stopped by user')
        patrol.finish()
    finally:
        patrol.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
