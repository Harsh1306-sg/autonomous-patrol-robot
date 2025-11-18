#!/usr/bin/env python3

"""
ROS2 Patrolling Robot Script
Patrols predefined waypoints, dynamically adjusts path based on obstacles,
returns to start position, and maintains patrol logs.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
import json
import os
from datetime import datetime
from pathlib import Path


class PatrolRobot(Node):
    def __init__(self):
        super().__init__('patrol_robot')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Patrol parameters
        self.waypoints = self.define_waypoints()
        self.starting_position = self.waypoints[0]  # First waypoint is starting position
        self.current_waypoint_index = 0
        self.patrol_cycle = 0
        self.max_patrol_cycles = 3  # Number of complete patrol cycles before stopping
        
        # Logging setup
        self.log_directory = os.path.expanduser('~/patrol_logs')
        Path(self.log_directory).mkdir(parents=True, exist_ok=True)
        self.log_file = os.path.join(
            self.log_directory, 
            f'patrol_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        )
        self.patrol_log = {
            'start_time': datetime.now().isoformat(),
            'cycles': []
        }
        
        self.get_logger().info('Patrol Robot Node Initialized')
        self.get_logger().info(f'Log file: {self.log_file}')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        
    def define_waypoints(self):
        """Define patrol waypoints - modify these coordinates for your environment"""
        waypoints = [
            # Format: [x, y, yaw_in_degrees]
            [0.0, 0.0, 0.0],      # Starting position
            [3.5, 0.5, 90.0],     # Waypoint 1
            [3.5, 3.0, 180.0],    # Waypoint 2
            [1.0, 3.0, -90.0],    # Waypoint 3
            [1.0, 1.0, 0.0],      # Waypoint 4
        ]
        return waypoints
    
    def euler_to_quaternion(self, yaw_degrees):
        """Convert yaw angle in degrees to quaternion"""
        import math
        yaw = math.radians(yaw_degrees)
        
        # For yaw-only rotation (roll=0, pitch=0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        return [qx, qy, qz, qw]
    
    def create_pose_stamped(self, waypoint):
        """Create a PoseStamped message from waypoint coordinates"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = 0.0
        
        quaternion = self.euler_to_quaternion(waypoint[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        return pose
    
    def log_waypoint_visit(self, waypoint_index, status, additional_info=""):
        """Log waypoint visit with timestamp and coordinates"""
        waypoint = self.waypoints[waypoint_index]
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'cycle': self.patrol_cycle,
            'waypoint_index': waypoint_index,
            'coordinates': {
                'x': waypoint[0],
                'y': waypoint[1],
                'yaw': waypoint[2]
            },
            'status': status,
            'additional_info': additional_info
        }
        
        if self.patrol_cycle >= len(self.patrol_log['cycles']):
            self.patrol_log['cycles'].append([])
        
        self.patrol_log['cycles'][self.patrol_cycle].append(log_entry)
        
        # Save to file
        with open(self.log_file, 'w') as f:
            json.dump(self.patrol_log, f, indent=2)
        
        self.get_logger().info(
            f'[Cycle {self.patrol_cycle}] Waypoint {waypoint_index}: '
            f'({waypoint[0]:.2f}, {waypoint[1]:.2f}) - {status}'
        )
    
    def send_goal(self, waypoint_index):
        """Send navigation goal to Nav2"""
        waypoint = self.waypoints[waypoint_index]
        goal_pose = self.create_pose_stamped(waypoint)
        
        self.get_logger().info(
            f'Navigating to waypoint {waypoint_index}: '
            f'x={waypoint[0]:.2f}, y={waypoint[1]:.2f}, yaw={waypoint[2]:.2f}°'
        )
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigate to pose action server not available!')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback"""
        feedback = feedback_msg.feedback
        # You can access feedback.distance_remaining, feedback.navigation_time, etc.
        # This allows dynamic path adjustment monitoring
        pass
    
    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.log_waypoint_visit(
                self.current_waypoint_index, 
                'REJECTED', 
                'Goal was rejected by Nav2'
            )
            self.handle_navigation_failure()
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback when navigation result is received"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint_index}')
            self.log_waypoint_visit(self.current_waypoint_index, 'SUCCESS')
            self.move_to_next_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.log_waypoint_visit(
                self.current_waypoint_index, 
                'FAILED', 
                f'Nav2 status code: {status}'
            )
            self.handle_navigation_failure()
    
    def handle_navigation_failure(self):
        """Handle navigation failure - retry or skip waypoint"""
        self.get_logger().warn(
            f'Handling navigation failure at waypoint {self.current_waypoint_index}'
        )
        
        # Simple strategy: skip to next waypoint after logging failure
        # You can implement more sophisticated recovery behaviors here
        self.move_to_next_waypoint()
    
    def move_to_next_waypoint(self):
        """Move to the next waypoint in the patrol sequence"""
        self.current_waypoint_index += 1
        
        # Check if we completed a full patrol cycle
        if self.current_waypoint_index >= len(self.waypoints):
            self.complete_patrol_cycle()
            return
        
        # Navigate to next waypoint
        self.send_goal(self.current_waypoint_index)
    
    def complete_patrol_cycle(self):
        """Complete current patrol cycle and decide next action"""
        self.get_logger().info(f'Completed patrol cycle {self.patrol_cycle}')
        
        # Log cycle completion
        completion_log = {
            'timestamp': datetime.now().isoformat(),
            'cycle': self.patrol_cycle,
            'status': 'CYCLE_COMPLETE',
            'waypoints_visited': self.current_waypoint_index
        }
        self.patrol_log['cycles'][self.patrol_cycle].append(completion_log)
        
        with open(self.log_file, 'w') as f:
            json.dump(self.patrol_log, f, indent=2)
        
        self.patrol_cycle += 1
        
        # Check if we should continue patrolling
        if self.patrol_cycle < self.max_patrol_cycles:
            self.get_logger().info(f'Starting patrol cycle {self.patrol_cycle}')
            self.current_waypoint_index = 1  # Start from first waypoint (skip starting position)
            self.send_goal(self.current_waypoint_index)
        else:
            self.return_to_start()
    
    def return_to_start(self):
        """Return to starting position and end patrol"""
        self.get_logger().info('Returning to starting position...')
        
        # Navigate to starting position (waypoint 0)
        goal_pose = self.create_pose_stamped(self.starting_position)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.return_home_response_callback)
    
    def return_home_response_callback(self, future):
        """Callback for return to start navigation"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Return to start goal rejected')
            self.finalize_patrol()
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.return_home_result_callback)
    
    def return_home_result_callback(self, future):
        """Callback when return to start is complete"""
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Successfully returned to starting position')
            self.log_waypoint_visit(0, 'RETURNED_HOME', 'Patrol mission complete')
        else:
            self.get_logger().warn(f'Failed to return to start. Status: {status}')
        
        self.finalize_patrol()
    
    def finalize_patrol(self):
        """Finalize patrol mission and save final logs"""
        self.patrol_log['end_time'] = datetime.now().isoformat()
        self.patrol_log['total_cycles'] = self.patrol_cycle
        self.patrol_log['status'] = 'COMPLETE'
        
        with open(self.log_file, 'w') as f:
            json.dump(self.patrol_log, f, indent=2)
        
        self.get_logger().info('=== PATROL MISSION COMPLETE ===')
        self.get_logger().info(f'Total cycles completed: {self.patrol_cycle}')
        self.get_logger().info(f'Logs saved to: {self.log_file}')
        
        # Print log summary
        self.print_patrol_summary()
    
    def print_patrol_summary(self):
        """Print summary of patrol mission"""
        self.get_logger().info('=== PATROL SUMMARY ===')
        
        total_waypoints = 0
        successful = 0
        failed = 0
        
        for cycle_idx, cycle in enumerate(self.patrol_log['cycles']):
            cycle_waypoints = [entry for entry in cycle if 'waypoint_index' in entry]
            total_waypoints += len(cycle_waypoints)
            
            for entry in cycle_waypoints:
                if entry['status'] == 'SUCCESS':
                    successful += 1
                elif entry['status'] in ['FAILED', 'REJECTED']:
                    failed += 1
        
        self.get_logger().info(f'Total waypoints visited: {total_waypoints}')
        self.get_logger().info(f'Successful navigations: {successful}')
        self.get_logger().info(f'Failed navigations: {failed}')
        
        if total_waypoints > 0:
            success_rate = (successful / total_waypoints) * 100
            self.get_logger().info(f'Success rate: {success_rate:.1f}%')
    
    def start_patrol(self):
        """Start the patrol mission"""
        self.get_logger().info('=== STARTING PATROL MISSION ===')
        self.get_logger().info(f'Patrol cycles to complete: {self.max_patrol_cycles}')
        self.get_logger().info(f'Waypoints per cycle: {len(self.waypoints)}')
        
        # Log mission start
        self.patrol_log['cycles'].append([])
        
        # Start from first waypoint after starting position
        self.current_waypoint_index = 1
        self.send_goal(self.current_waypoint_index)


def main(args=None):
    rclpy.init(args=args)
    
    patrol_robot = PatrolRobot()
    
    try:
        # Start patrol
        patrol_robot.start_patrol()
        
        # Spin to process callbacks
        rclpy.spin(patrol_robot)
        
    except KeyboardInterrupt:
        patrol_robot.get_logger().info('Patrol interrupted by user')
        patrol_robot.finalize_patrol()
    finally:
        patrol_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
