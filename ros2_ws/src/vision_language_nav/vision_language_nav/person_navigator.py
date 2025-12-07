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
        
        self.latest_distance = None
        self.navigation_triggered = False
        
        self.get_logger().info('Person Navigator node started!')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')
    
    def distance_callback(self, msg):
        """When person is detected, calculate and send navigation goal"""
        self.latest_distance = msg.data
        
        # Only trigger navigation once when person is first detected
        if self.latest_distance > 0 and not self.navigation_triggered:
            success = self.send_navigation_goal(self.latest_distance)
            if success:
                self.navigation_triggered = True
    
    def send_navigation_goal(self, distance):
        """Send goal to Nav2 to approach the person"""
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
            
            # Calculate target position in base_footprint frame
            safe_margin = 0.5  # Stop 0.5m away from person
            target_distance = max(0.3, distance - safe_margin)
            
            # Create goal pose in base_footprint frame first
            goal_pose_base = PoseStamped()
            goal_pose_base.header.frame_id = 'base_footprint'
            goal_pose_base.header.stamp = self.get_clock().now().to_msg()
            goal_pose_base.pose.position.x = target_distance
            goal_pose_base.pose.position.y = 0.0
            goal_pose_base.pose.position.z = 0.0
            goal_pose_base.pose.orientation.w = 1.0
            
            # Transform to map frame using tf2_geometry_msgs
            goal_pose_map = tf2_geometry_msgs.do_transform_pose_stamped(goal_pose_base, transform)
            
            # Create action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose_map
            
            self.get_logger().info(
                f'Sending navigation goal: move {target_distance:.2f}m forward '
                f'(person at {distance:.2f}m, safe margin: {safe_margin}m)'
            )
            self.get_logger().info(
                f'Goal in map frame: x={goal_pose_map.pose.position.x:.2f}, '
                f'y={goal_pose_map.pose.position.y:.2f}'
            )
            
            # Send goal
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
            self.get_logger().error('Navigation goal rejected!')
            return
        
        self.get_logger().info('Navigation goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed!')
        self.navigation_triggered = False  # Allow new navigation
    
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
