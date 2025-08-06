#!/usr/bin/env python3

import sys
import math
import random
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList, ZoneList, ItemHolder, ItemHolders
from auro_interfaces.srv import ItemRequest

# Simple function to convert quaternion to euler angles
def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# Constants - ADJUSTED FOR BETTER COLLECTION
LINEAR_VELOCITY = 0.08  # Slower for more precise movement
ANGULAR_VELOCITY = 0.3
COLLECTION_DISTANCE = 0.25  # Reduced - get closer before attempting collection
ZONE_APPROACH_DISTANCE = 0.5
OBSTACLE_THRESHOLD = 0.15
APPROACH_ATTEMPTS_MAX = 3  # Maximum attempts to approach an item

# States for the finite state machine
class RobotState(Enum):
    SEARCHING = 0
    APPROACHING_ITEM = 1
    COLLECTING_ITEM = 2
    SEARCHING_ZONE = 3
    APPROACHING_ZONE = 4
    DEPOSITING_ITEM = 5
    AVOIDING_OBSTACLE = 6
    FINE_APPROACH = 7  # New state for final approach to item

class ItemCollectorRobot(Node):
    def __init__(self):
        super().__init__('item_collector_robot')
        
        # Get robot name from parameter
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.get_logger().info(f'Starting robot controller for {self.robot_name}')
        
        # State management
        self.state = RobotState.SEARCHING
        self.previous_state = None
        
        # Robot pose and navigation
        self.pose = Pose()
        self.yaw = 0.0
        self.target_item = None
        self.target_zone = None
        self.held_item_colour = None
        
        # Item approach tracking
        self.approach_attempts = 0
        self.last_item_position = None
        self.collection_timeout = 0
        
        # Sensor data
        self.scan_data = []
        self.items = []
        self.zones = []
        self.item_holders = []
        
        # Obstacle avoidance
        self.obstacle_detected = False
        self.avoidance_start_yaw = 0.0
        self.avoidance_turn_angle = 0.0
        
        # Performance tracking
        self.items_collected = 0
        self.search_start_time = self.get_clock().now()
        
        # Callback groups for concurrent execution
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Publishers - use relative topics (will be namespaced)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers - use relative topics (will be namespaced)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10, 
            callback_group=self.timer_cb_group
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=self.timer_cb_group
        )
        
        self.items_sub = self.create_subscription(
            ItemList, 'items', self.items_callback, 10,
            callback_group=self.timer_cb_group
        )
        
        self.zones_sub = self.create_subscription(
            ZoneList, 'zone', self.zones_callback, 10,
            callback_group=self.timer_cb_group
        )
        
        # Subscribe to global item holders topic
        self.item_holders_sub = self.create_subscription(
            ItemHolders, '/item_holders', self.item_holders_callback, 10,
            callback_group=self.timer_cb_group
        )
        
        # Service clients - use global service names
        self.pick_up_client = self.create_client(
            ItemRequest, '/pick_up_item',
            callback_group=self.client_cb_group
        )
        
        self.offload_client = self.create_client(
            ItemRequest, '/offload_item',
            callback_group=self.client_cb_group
        )
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        if not self.pick_up_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Pick up service not available')
        if not self.offload_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Offload service not available')
        
        # Control loop timer
        self.timer = self.create_timer(
            0.1, self.control_loop,
            callback_group=self.timer_cb_group
        )
        
        self.get_logger().info(f'Item Collector Robot {self.robot_name} initialized')
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.pose = msg.pose.pose
        _, _, self.yaw = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
    
    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        if len(msg.ranges) == 0:
            return
            
        self.scan_data = msg.ranges
        
        # Only check very close front sectors for obstacles (walls, not items)
        front_angles = list(range(355, 360)) + list(range(0, 5))
        front_distances = [msg.ranges[i] for i in front_angles if i < len(msg.ranges)]
        
        # Only detect as obstacle if very close and multiple readings confirm it
        close_readings = [d for d in front_distances if not math.isinf(d) and d > 0.05 and d < OBSTACLE_THRESHOLD]
        
        # Need at least 3 close readings to consider it an obstacle
        self.obstacle_detected = len(close_readings) >= 3
    
    def items_callback(self, msg):
        """Update visible items list"""
        self.items = msg.data
    
    def zones_callback(self, msg):
        """Update visible zones list"""
        self.zones = msg.data
    
    def item_holders_callback(self, msg):
        """Track which robot is holding which item"""
        self.item_holders = msg.data
        
        # Check if we're holding an item
        for holder in self.item_holders:
            if holder.robot_id == self.robot_name and holder.holding_item:
                self.held_item_colour = holder.item_colour
                break
        else:
            self.held_item_colour = None
    
    def control_loop(self):
        """Main control loop implementing the state machine"""
        # Handle obstacle avoidance with priority, but NOT during item approach/collection
        if (self.obstacle_detected and 
            self.state not in [RobotState.AVOIDING_OBSTACLE, 
                              RobotState.DEPOSITING_ITEM, 
                              RobotState.FINE_APPROACH,  # Don't avoid during fine approach
                              RobotState.COLLECTING_ITEM]):
            self.previous_state = self.state
            self.state = RobotState.AVOIDING_OBSTACLE
            self.avoidance_start_yaw = self.yaw
            self.avoidance_turn_angle = random.uniform(30, 60) * random.choice([-1, 1])
            self.get_logger().info(f'Wall obstacle detected! Switching to AVOIDING_OBSTACLE from {self.previous_state}')
        
        # Execute state-specific behavior
        if self.state == RobotState.SEARCHING:
            self.search_for_items()
        elif self.state == RobotState.APPROACHING_ITEM:
            self.approach_item()
        elif self.state == RobotState.FINE_APPROACH:
            self.fine_approach_item()
        elif self.state == RobotState.COLLECTING_ITEM:
            self.collect_item()
        elif self.state == RobotState.SEARCHING_ZONE:
            self.search_for_zone()
        elif self.state == RobotState.APPROACHING_ZONE:
            self.approach_zone()
        elif self.state == RobotState.DEPOSITING_ITEM:
            self.deposit_item()
        elif self.state == RobotState.AVOIDING_OBSTACLE:
            self.avoid_obstacle()
    
    def search_for_items(self):
        """Search pattern for finding items"""
        if self.items and not self.held_item_colour:
            # Found an item, switch to approaching
            self.target_item = self.select_best_item()
            if self.target_item:
                self.state = RobotState.APPROACHING_ITEM
                self.approach_attempts = 0
                self.get_logger().info(f'Found {self.target_item.colour} item, approaching')
                return
        
        # Implement spiral search pattern
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY
        twist.angular.z = 0.2  # Gentle spiral pattern
        self.cmd_vel_pub.publish(twist)
    
    def approach_item(self):
        """Navigate towards the target item - initial rough approach"""
        if not self.target_item or not self.items:
            self.state = RobotState.SEARCHING
            return
        
        # Find current position of target item
        current_item = None
        for item in self.items:
            if item.colour == self.target_item.colour:
                current_item = item
                break
        
        if not current_item:
            # Item disappeared, try again or search for new one
            self.approach_attempts += 1
            if self.approach_attempts < APPROACH_ATTEMPTS_MAX:
                self.get_logger().warn(f'Item disappeared, attempt {self.approach_attempts}')
                # Try to continue moving forward briefly
                twist = Twist()
                twist.linear.x = LINEAR_VELOCITY * 0.5
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.get_logger().warn('Item lost after multiple attempts, searching for new item')
                self.state = RobotState.SEARCHING
                return
        
        # Improved distance estimation
        estimated_distance = self.estimate_distance_to_item(current_item)
        
        # If reasonably close, switch to fine approach mode
        if estimated_distance <= COLLECTION_DISTANCE * 1.5:
            self.state = RobotState.FINE_APPROACH
            self.collection_timeout = 20  # 2 seconds timeout for fine approach
            self.get_logger().info('Switching to fine approach mode')
            return
        
        # Rough approach - move towards item
        twist = Twist()
        
        # More conservative approach speed
        if estimated_distance > 1.0:
            twist.linear.x = LINEAR_VELOCITY
        else:
            twist.linear.x = LINEAR_VELOCITY * 0.7
            
        # Proportional control for turning - more sensitive
        twist.angular.z = -current_item.x * 0.004
        self.cmd_vel_pub.publish(twist)
    
    def fine_approach_item(self):
        """Fine approach to item - slower, more precise movement"""
        if not self.target_item or not self.items:
            self.state = RobotState.SEARCHING
            return
        
        # Timeout mechanism
        self.collection_timeout -= 1
        if self.collection_timeout <= 0:
            self.get_logger().warn('Fine approach timeout, trying collection anyway')
            self.state = RobotState.COLLECTING_ITEM
            return
        
        # Find current position of target item
        current_item = None
        for item in self.items:
            if item.colour == self.target_item.colour:
                current_item = item
                break
        
        if not current_item:
            self.get_logger().warn('Item lost during fine approach, attempting collection')
            self.state = RobotState.COLLECTING_ITEM
            return
        
        estimated_distance = self.estimate_distance_to_item(current_item)
        
        # If very close or item appears large, try collection
        if estimated_distance <= COLLECTION_DISTANCE or current_item.diameter > 60:
            self.state = RobotState.COLLECTING_ITEM
            return
        
        # Fine movement towards item
        twist = Twist()
        
        # Very slow approach
        twist.linear.x = LINEAR_VELOCITY * 0.4
        
        # Fine angular adjustment
        if abs(current_item.x) > 10:  # Only adjust if significantly off-center
            twist.angular.z = -current_item.x * 0.002
        
        self.cmd_vel_pub.publish(twist)
    
    def estimate_distance_to_item(self, item):
        """Improved distance estimation based on item diameter"""
        # Base estimation - adjust these values based on your camera setup
        if item.diameter > 80:
            return 0.15
        elif item.diameter > 60:
            return 0.25
        elif item.diameter > 40:
            return 0.4
        elif item.diameter > 20:
            return 0.8
        else:
            return 1.5
    
    def collect_item(self):
        """Attempt to pick up the item"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Attempting to collect item...')
        
        # Call pick up service
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        
        future = self.pick_up_client.call_async(request)
        future.add_done_callback(self.pick_up_callback)
    
    def pick_up_callback(self, future):
        """Handle pick up service response"""
        try:
            response = future.result()
            if response.success:
                self.items_collected += 1
                self.get_logger().info(f'Successfully collected item! Total: {self.items_collected}')
                self.state = RobotState.SEARCHING_ZONE
                self.approach_attempts = 0
            else:
                self.get_logger().warn(f'Failed to collect item: {response.message}')
                self.approach_attempts += 1
                
                if self.approach_attempts < APPROACH_ATTEMPTS_MAX:
                    # Try moving forward a bit more and then retry
                    self.get_logger().info(f'Trying to get closer, attempt {self.approach_attempts}')
                    self.state = RobotState.FINE_APPROACH
                    self.collection_timeout = 10
                else:
                    self.get_logger().warn('Max approach attempts reached, searching for new item')
                    self.state = RobotState.SEARCHING
                    self.approach_attempts = 0
                    
        except Exception as e:
            self.get_logger().error(f'Pick up service call failed: {e}')
            self.state = RobotState.SEARCHING
    
    def search_for_zone(self):
        """Search for appropriate zone to deposit item"""
        if not self.held_item_colour:
            self.state = RobotState.SEARCHING
            return
        
        if self.zones:
            # Find matching zone
            self.target_zone = self.select_best_zone()
            if self.target_zone:
                self.state = RobotState.APPROACHING_ZONE
                self.get_logger().info(f'Found zone, approaching')
                return
        
        # Rotate to search for zones
        twist = Twist()
        twist.angular.z = ANGULAR_VELOCITY * 0.8
        self.cmd_vel_pub.publish(twist)
    
    def approach_zone(self):
        """Navigate towards the target zone"""
        if not self.target_zone or not self.zones:
            self.state = RobotState.SEARCHING_ZONE
            return
        
        # Find current position of target zone
        current_zone = None
        for zone in self.zones:
            if zone.zone == self.target_zone.zone:
                current_zone = zone
                break
        
        if not current_zone:
            self.state = RobotState.SEARCHING_ZONE
            return
        
        # Check if we're over the zone (size indicates coverage)
        if current_zone.size > 0.15:  # Lower threshold for easier zone detection
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.state = RobotState.DEPOSITING_ITEM
            return
        
        # Move towards zone
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY * 0.6  # Slower approach to zone
        twist.angular.z = -current_zone.x * 0.003  # Proportional control
        self.cmd_vel_pub.publish(twist)
    
    def deposit_item(self):
        """Attempt to offload the item in the zone"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Call offload service
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        
        future = self.offload_client.call_async(request)
        future.add_done_callback(self.offload_callback)
    
    def offload_callback(self, future):
        """Handle offload service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deposited item!')
                self.held_item_colour = None
                self.state = RobotState.SEARCHING
            else:
                self.get_logger().warn(f'Failed to deposit item: {response.message}')
                # Back up and try different zone
                self.backup_from_zone()
        except Exception as e:
            self.get_logger().error(f'Offload service call failed: {e}')
            self.state = RobotState.SEARCHING_ZONE
    
    def avoid_obstacle(self):
        """Execute obstacle avoidance maneuver"""
        twist = Twist()
        
        # Calculate how much we've turned
        angle_turned = normalize_angle(self.yaw - self.avoidance_start_yaw)
        
        if abs(angle_turned) >= abs(math.radians(self.avoidance_turn_angle)):
            # Finished turning, resume previous state
            self.state = self.previous_state or RobotState.SEARCHING
            self.get_logger().info(f'Obstacle avoidance complete, returning to {self.state}')
        else:
            # Continue turning
            twist.angular.z = ANGULAR_VELOCITY if self.avoidance_turn_angle > 0 else -ANGULAR_VELOCITY
        
        self.cmd_vel_pub.publish(twist)
    
    def select_best_item(self):
        """Select the best item to collect based on visibility and value"""
        if not self.items:
            return None
        
        # Prefer items that are centered, larger (closer), and higher value
        best_item = None
        best_score = float('-inf')
        
        for item in self.items:
            # Score based on centering (higher is better when centered)
            centering_score = 1.0 - abs(item.x) / 320.0
            
            # Size score (larger diameter = closer = better)
            size_score = min(item.diameter / 100.0, 1.0)
            
            # Value score
            value_score = item.value / 20.0
            
            # Total score with adjusted weights
            total_score = centering_score * 2.0 + size_score * 1.5 + value_score
            
            if total_score > best_score:
                best_score = total_score
                best_item = item
        
        return best_item
    
    def select_best_zone(self):
        """Select appropriate zone for depositing item"""
        if not self.zones or not self.held_item_colour:
            return None
        
        # For initial deposit, prefer closer/larger zones
        best_zone = None
        best_score = float('-inf')
        
        for zone in self.zones:
            # Score based on size (proximity) and centering
            size_score = zone.size * 2.0  # Prioritize larger (closer) zones
            centering_score = 1.0 - abs(zone.x) / 320.0
            
            total_score = size_score + centering_score
            
            if total_score > best_score:
                best_score = total_score
                best_zone = zone
        
        return best_zone
    
    def backup_from_zone(self):
        """Back up from zone if deposit failed"""
        twist = Twist()
        twist.linear.x = -LINEAR_VELOCITY * 0.5
        self.cmd_vel_pub.publish(twist)
        
        # Create a one-shot timer to switch state after backing up
        def switch_to_search():
            self.state = RobotState.SEARCHING_ZONE
        
        self.create_timer(1.0, switch_to_search)
    
    def destroy_node(self):
        """Clean shutdown"""
        # Stop the robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    robot = ItemCollectorRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()