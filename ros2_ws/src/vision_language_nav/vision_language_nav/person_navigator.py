#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.duration import Duration
import math
import time


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
        
        # Publisher for direct velocity control (for rotation)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State variables
        self.latest_distance = None
        self.navigation_triggered = False
        self.is_navigating = False
        self.person_goal_position = None  # Store person's position in map frame
        self.safe_margin = 0.01  # Stop 0.01m away from person (safe distance)
        self.min_navigation_distance = 0.35  # Only navigate if person is >0.35m away (avoids repeated tiny movements)
        
        # Search behavior variables
        self.last_person_time = None
        self.person_lost_timeout = 5.0  # If no person for 5 seconds, start search
        self.is_searching = False
        self.search_attempts = 0
        self.max_search_attempts = 3  # Try 3 times: rotate, move forward, rotate, move forward, rotate
        self.person_detected_during_search = False
        self.rotation_start_time = None
        self.rotation_timer = None
        self.move_start_time = None
        self.move_timer = None
        
        self.get_logger().info('Person Navigator node started!')
        self.get_logger().info(f'Will navigate to persons >={self.min_navigation_distance}m away')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')
        
        # Timer to check if person is lost
        self.create_timer(1.0, self.check_person_lost)
    
    def distance_callback(self, msg):
        """When person is detected, calculate and send navigation goal ONCE"""
        self.latest_distance = msg.data
        self.last_person_time = time.time()  # Update last seen time
        
        # If person detected during search, mark it
        if self.is_searching:
            self.person_detected_during_search = True
            self.get_logger().info(' Person found during search! Stopping search...')
            self.stop_search()
        
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
    
    def check_person_lost(self):
        """Timer callback to check if person has been lost for too long"""
        if self.is_searching or self.is_navigating:
            return  # Don't start search if already searching or navigating
        
        if self.last_person_time is None:
            return  # Person never detected yet
        
        time_since_person = time.time() - self.last_person_time
        
        if time_since_person > self.person_lost_timeout:
            self.get_logger().warn(f'  Person lost for {time_since_person:.1f}s - starting search behavior...')
            self.start_search()
    
    def start_search(self):
        """Start the search behavior: rotate 360°, if not found move forward and repeat"""
        self.is_searching = True
        self.search_attempts = 0
        self.person_detected_during_search = False
        self.perform_search_cycle()
    
    def perform_search_cycle(self):
        """Perform one search cycle: rotate 360°, check if found, if not move forward"""
        if not self.is_searching:
            return
        
        if self.search_attempts >= self.max_search_attempts:
            self.get_logger().warn(' Search failed after 3 attempts. Stopping search.')
            self.stop_search()
            return
        
        self.search_attempts += 1
        self.get_logger().info(f' Search attempt {self.search_attempts}/{self.max_search_attempts}: Rotating 360°...')
        
        # Perform 360° rotation
        self.rotate_360()
        
    def rotate_360(self):
        """Rotate robot 360 degrees in place using timer"""
        rotation_speed = 0.5  # rad/s
        rotation_time = (2 * math.pi) / rotation_speed  # ~12.6 seconds
        
        self.get_logger().info(f' Rotating 360° (will take ~{rotation_time:.1f}s)...')
        
        self.rotation_start_time = time.time()
        
        # Create timer to publish rotation commands at 20Hz
        self.rotation_timer = self.create_timer(0.05, lambda: self.rotation_step(rotation_speed, rotation_time))
    
    def rotation_step(self, rotation_speed, rotation_time):
        """Timer callback for rotation"""
        if not self.is_searching or self.person_detected_during_search:
            if self.person_detected_during_search:
                self.get_logger().info(' Person detected during rotation!')
            self.stop_rotation()
            if self.person_detected_during_search:
                self.stop_search()
            else:
                self.after_rotation()
            return
        
        elapsed = time.time() - self.rotation_start_time
        
        if elapsed >= rotation_time:
            self.stop_rotation()
            self.after_rotation()
            return
        
        # Publish rotation command
        twist = Twist()
        twist.angular.z = rotation_speed
        self.cmd_vel_pub.publish(twist)
    
    def stop_rotation(self):
        """Stop rotation and cleanup timer"""
        if self.rotation_timer:
            self.rotation_timer.cancel()
            self.rotation_timer = None
        
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
    
    def after_rotation(self):
        """Called after rotation completes"""
        time.sleep(0.3)
        
        if self.person_detected_during_search:
            self.stop_search()
            return
        
        # Person not found - move forward if we have more attempts
        if self.search_attempts < self.max_search_attempts:
            self.get_logger().info(' Person not found. Moving forward 0.5m...')
            self.move_forward(0.5)
        else:
            self.get_logger().warn(' Search failed after 3 attempts. Stopping search.')
            self.stop_search()
    
    def move_forward(self, distance):
        """Move robot forward by specified distance using timer"""
        move_speed = 0.15  # m/s
        move_time = distance / move_speed  # time to cover distance
        
        self.move_start_time = time.time()
        
        # Create timer to publish move commands at 20Hz
        self.move_timer = self.create_timer(0.05, lambda: self.move_step(move_speed, move_time))
    
    def move_step(self, move_speed, move_time):
        """Timer callback for forward movement"""
        if not self.is_searching or self.person_detected_during_search:
            self.stop_movement()
            if self.person_detected_during_search:
                self.stop_search()
            else:
                self.after_movement()
            return
        
        elapsed = time.time() - self.move_start_time
        
        if elapsed >= move_time:
            self.stop_movement()
            self.after_movement()
            return
        
        # Publish forward command
        twist = Twist()
        twist.linear.x = move_speed
        self.cmd_vel_pub.publish(twist)
    
    def stop_movement(self):
        """Stop forward movement and cleanup timer"""
        if self.move_timer:
            self.move_timer.cancel()
            self.move_timer = None
        
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
    
    def after_movement(self):
        """Called after forward movement completes"""
        time.sleep(0.5)
        
        if self.person_detected_during_search:
            self.stop_search()
        else:
            # Continue search with next rotation
            self.perform_search_cycle()
    
    def stop_search(self):
        """Stop the search behavior"""
        self.is_searching = False
        self.person_detected_during_search = False
        
        # Cancel any active timers
        if self.rotation_timer:
            self.rotation_timer.cancel()
            self.rotation_timer = None
        if self.move_timer:
            self.move_timer.cancel()
            self.move_timer = None
        
        # Make sure robot is stopped
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(' Search behavior stopped.')


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
