#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.duration import Duration


class PersonNavigator(Node):
    def __init__(self):
        super().__init__('person_navigator')
        
        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to person distance
        self.distance_sub = self.create_subscription(
            Float32,
            '/person/distance',
            self.distance_callback,
            10
        )
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State variables
        self.latest_distance = None
        self.navigation_triggered = False
        self.is_navigating = False
        self.person_goal_position = None  # Store person's position in map frame
        self.safe_margin = 0.01  # Stop 0.01m away from person (safe distance)
        self.min_navigation_distance = 0.35  # Only navigate if person is >0.35m away (avoids repeated tiny movements)
        
        self.get_logger().info('Person Navigator node started!')
        self.get_logger().info(f'Will navigate to persons >={self.min_navigation_distance}m away')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')
    
    def distance_callback(self, msg):
        """When person is detected, calculate and send navigation goal ONCE"""
        self.latest_distance = msg.data
        
        # Don't send new goals while already navigating
        if self.is_navigating:
            return
        
        # Check if person detected and far enough away
        if self.latest_distance > self.min_navigation_distance and not self.navigation_triggered:
            # Person detected and far enough - calculate goal position ONCE
            self.get_logger().info(f'Person detected at {self.latest_distance:.2f}m - calculating goal position...')
            success = self.send_navigation_goal(self.latest_distance)
            if success:
                self.navigation_triggered = True
                self.is_navigating = True
        elif self.latest_distance > 0 and self.latest_distance <= self.min_navigation_distance:
            # Person is close enough - no navigation needed
            if not self.navigation_triggered:
                self.get_logger().info(
                    f'Person is close enough ({self.latest_distance:.2f}m), no navigation needed.',
                    throttle_duration_sec=3.0
                )
                self.navigation_triggered = True
    
    def send_navigation_goal(self, distance):
        """Calculate person's position in map frame and send navigation goal ONCE"""
        try:
            # Wait for map->base_footprint transform (means localization is ready)
            if not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                self.get_logger().warn('Map frame not available yet. Please set initial pose in RViz (2D Pose Estimate)')
                return False
            
            # Get current robot pose in map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            # Calculate target position: stop at safe_margin away from person
            # max(0.05, ...) prevents robot from going too close (collision)
            target_distance = max(0.05, distance - self.safe_margin)
            
            # Create goal pose in base_footprint frame (relative to robot)
            goal_pose_base = PoseStamped()
            goal_pose_base.header.frame_id = 'base_footprint'
            goal_pose_base.header.stamp = self.get_clock().now().to_msg()
            goal_pose_base.pose.position.x = target_distance  # Forward direction
            goal_pose_base.pose.position.y = 0.0
            goal_pose_base.pose.position.z = 0.0
            goal_pose_base.pose.orientation.w = 1.0
            
            # Transform goal to map frame - this gives us the ABSOLUTE position
            goal_pose_map = tf2_geometry_msgs.do_transform_pose_stamped(goal_pose_base, transform)
            
            # Store the goal position
            self.person_goal_position = goal_pose_map
            
            # Create Nav2 action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose_map
            
            self.get_logger().info(
                f' Target calculated: Person at {distance:.2f}m, moving to {target_distance:.2f}m away'
            )
            self.get_logger().info(
                f' Goal position in map: ({goal_pose_map.pose.position.x:.2f}, {goal_pose_map.pose.position.y:.2f})'
            )
            self.get_logger().info(' Sending goal to Nav2 - will complete even if person disappears from view')
            
            # Send goal to Nav2
            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation goal: {str(e)}')
            return False
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(' Navigation goal rejected by Nav2!')
            self.is_navigating = False
            self.navigation_triggered = False
            return
        
        self.get_logger().info(' Goal accepted! Navigating to person...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        self.is_navigating = False
        self.navigation_triggered = False  # Reset to allow new detection
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(' Reached destination! Arrived near person.')
        elif status == 5:  # CANCELED
            self.get_logger().warn('  Navigation was canceled')
        elif status == 6:  # ABORTED  
            self.get_logger().warn('  Navigation aborted (obstacle blocking path or invalid goal)')
        else:
            self.get_logger().error(f' Navigation failed with status: {status}')
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Optionally log progress
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PersonNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
