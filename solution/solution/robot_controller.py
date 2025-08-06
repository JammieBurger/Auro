#!/usr/bin/env python3

import sys
import math
import random
import time
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

# Constants - IMPROVED FOR BETTER PERFORMANCE
LINEAR_VELOCITY = 0.12  # Slightly faster movement
ANGULAR_VELOCITY = 0.4  # Faster turning
COLLECTION_DISTANCE = 0.4  # Slightly more forgiving collection distance
ZONE_APPROACH_DISTANCE = 0.5
OBSTACLE_THRESHOLD = 0.2  # Less sensitive to avoid false wall detection
APPROACH_ATTEMPTS_MAX = 5  # More attempts before giving up
STARTUP_WAIT_TIME = 30  # Wait 3 seconds for items to spawn

# States for the finite state machine
class RobotState(Enum):
    WAITING_FOR_ITEMS = 0
    SEARCHING = 1
    APPROACHING_ITEM = 2
    COLLECTING_ITEM = 3
    WAITING_FOR_COLLECTION = 4  # New state to prevent multiple service calls
    SEARCHING_ZONE = 5
    APPROACHING_ZONE = 6
    DEPOSITING_ITEM = 7
    WAITING_FOR_DEPOSIT = 8  # New state for deposit too
    AVOIDING_OBSTACLE = 9
    FINE_APPROACH = 10

class ItemCollectorRobot(Node):
    def __init__(self):
        super().__init__('item_collector_robot')
        
        # Get robot name from parameter
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.get_logger().info(f'Starting robot controller for {self.robot_name}')
        
        # State management
        self.state = RobotState.WAITING_FOR_ITEMS
        self.previous_state = None
        self.startup_counter = STARTUP_WAIT_TIME  # Wait for items to spawn
        
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
        
        # Obstacle avoidance - IMPROVED
        self.obstacle_detected = False
        self.avoidance_start_yaw = 0.0
        self.obstacle_clear_counter = 0  # Count how long obstacle has been clear
        
        # Search pattern improvements
        self.search_mode = 'forward'  # 'forward', 'turn_left', 'turn_right'
        self.search_timer = 0
        
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
        
        self.get_logger().info(f'Item Collector Robot {self.robot_name} initialized - waiting for items to spawn')
    
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
        
        # Check front sectors for obstacles - IMPROVED to detect walls not items
        front_ranges = []
        
        # Check front 40 degrees (20 degrees each side) - narrower to avoid detecting items
        for i in range(340, 360):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        for i in range(0, 20):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        
        # Only detect obstacle if multiple very close readings (walls, not items)
        very_close_readings = [d for d in front_ranges if d < 0.15]  # Very close for walls
        self.obstacle_detected = len(very_close_readings) >= 8  # Need many readings to confirm wall
    
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
        
        # Handle startup waiting period
        if self.state == RobotState.WAITING_FOR_ITEMS:
            self.startup_counter -= 1
            if self.startup_counter <= 0 or len(self.items) > 0:
                self.get_logger().info('Items detected or startup wait complete - beginning search')
                self.state = RobotState.SEARCHING
            else:
                # Stay still during startup
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                return
        
        # Handle obstacle avoidance - ONLY when truly necessary
        if (self.obstacle_detected and 
            self.state == RobotState.SEARCHING and  # Only avoid obstacles when searching
            not self.items):
            self.previous_state = self.state
            self.state = RobotState.AVOIDING_OBSTACLE
            self.obstacle_clear_counter = 0  # Reset clear counter
            self.get_logger().info(f'Wall detected while searching with no items visible - avoiding')
        
        # Execute state-specific behavior
        if self.state == RobotState.SEARCHING:
            self.search_for_items()
        elif self.state == RobotState.APPROACHING_ITEM:
            self.approach_item()
        elif self.state == RobotState.FINE_APPROACH:
            self.fine_approach_item()
        elif self.state == RobotState.COLLECTING_ITEM:
            self.collect_item()
        elif self.state == RobotState.WAITING_FOR_COLLECTION:
            # Just wait - do nothing until callback changes state
            pass
        elif self.state == RobotState.SEARCHING_ZONE:
            self.search_for_zone()
        elif self.state == RobotState.APPROACHING_ZONE:
            self.approach_zone()
        elif self.state == RobotState.DEPOSITING_ITEM:
            self.deposit_item()
        elif self.state == RobotState.WAITING_FOR_DEPOSIT:
            # Just wait - do nothing until callback changes state
            pass
        elif self.state == RobotState.AVOIDING_OBSTACLE:
            self.avoid_obstacle()
    
    def search_for_items(self):
        """IMPROVED search pattern for finding items"""
        if self.items and not self.held_item_colour:
            # Found an item, switch to approaching
            self.target_item = self.select_best_item()
            if self.target_item:
                self.state = RobotState.APPROACHING_ITEM
                self.approach_attempts = 0
                self.get_logger().info(f'Found {self.target_item.colour} item, approaching')
                return
        
        # Implement better search pattern
        twist = Twist()
        self.search_timer += 1
        
        if self.search_mode == 'forward':
            twist.linear.x = LINEAR_VELOCITY
            if self.search_timer > 30:  # Move forward for 3 seconds
                self.search_mode = 'turn_left' if random.random() > 0.5 else 'turn_right'
                self.search_timer = 0
        elif self.search_mode == 'turn_left':
            twist.angular.z = ANGULAR_VELOCITY
            if self.search_timer > 15:  # Turn for 1.5 seconds
                self.search_mode = 'forward'
                self.search_timer = 0
        elif self.search_mode == 'turn_right':
            twist.angular.z = -ANGULAR_VELOCITY
            if self.search_timer > 15:  # Turn for 1.5 seconds
                self.search_mode = 'forward'
                self.search_timer = 0
        
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
                self.get_logger().warn('Item lost after attempts, searching for new item')
                self.state = RobotState.SEARCHING
                return
        
        # Improved distance estimation
        estimated_distance = self.estimate_distance_to_item(current_item)
        
        # If reasonably close, switch to fine approach mode
        if estimated_distance <= COLLECTION_DISTANCE * 2.0:  # Switch much earlier now that estimates are realistic
            self.state = RobotState.FINE_APPROACH
            self.get_logger().info(f'Switching to fine approach, estimated distance: {estimated_distance:.2f}')
            return
        
        # Rough approach - move towards item with steering
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY * 0.8
        
        # FIXED: Add steering correction towards the item
        if abs(current_item.x) > 10:  # If item is off-center
            # Convert pixel offset to angular velocity
            # current_item.x is positive when item is to the left of center
            # We want to turn left (positive angular.z) when item is to the left
            twist.angular.z = current_item.x * 0.002  # Proportional steering
            # Limit angular velocity
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.5, 
                                min(ANGULAR_VELOCITY * 0.5, twist.angular.z))
        
        self.cmd_vel_pub.publish(twist)
    
    def fine_approach_item(self):
        """Fine approach to item - slower, more precise movement"""
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
            self.get_logger().warn('Item lost during fine approach, attempting collection')
            self.state = RobotState.COLLECTING_ITEM
            return
        
        estimated_distance = self.estimate_distance_to_item(current_item)
        
        # If very close or item appears very large, try collection
        if estimated_distance < 0.35 and current_item.diameter > 170:
            self.get_logger().info(f'Close enough for collection! Distance: {estimated_distance:.2f}, Diameter: {current_item.diameter}')
            self.state = RobotState.COLLECTING_ITEM
            return
        
        # Fine movement towards item
        twist = Twist()
        
        # FIXED: More aggressive forward movement to get really close
        if abs(current_item.x) < 5:  # Very well centered
            twist.linear.x = LINEAR_VELOCITY * 0.8  # More aggressive forward motion when centered
        elif abs(current_item.x) < 20:  # Reasonably centered
            twist.linear.x = LINEAR_VELOCITY * 0.6  # Moderate forward motion
        else:
            twist.linear.x = LINEAR_VELOCITY * 0.2  # Slower when not centered
        
        # FIXED: Enable fine angular adjustment - this was the main problem!
        if abs(current_item.x) > 3:  # Fine adjustment threshold
            # More sensitive steering for fine approach
            twist.angular.z = current_item.x * 0.003
            # Limit angular velocity for fine movements
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.3, 
                                min(ANGULAR_VELOCITY * 0.3, twist.angular.z))
        
        self.cmd_vel_pub.publish(twist)
    
    def estimate_distance_to_item(self, item):
        """VERY REALISTIC distance estimation - robot must be extremely close"""
        # Based on feedback: need diameter >170 and distance <0.35m for collection
        # Much more conservative estimates
        if item.diameter > 180:  # Extremely large = extremely close
            return 0.15
        elif item.diameter > 170:  # Very large (your threshold)
            return 0.30  # Just under your 0.35 threshold
        elif item.diameter > 160:
            return 0.40
        elif item.diameter > 150:
            return 0.50
        elif item.diameter > 140:
            return 0.60
        elif item.diameter > 130:
            return 0.75
        elif item.diameter > 110:
            return 1.0
        elif item.diameter > 90:
            return 1.3
        elif item.diameter > 70:
            return 1.7
        else:
            return 2.5
    
    def collect_item(self):
        """Attempt to pick up the item - FIXED to prevent multiple calls"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # CRITICAL FIX: Immediately change state to prevent repeated calls!
        self.state = RobotState.WAITING_FOR_COLLECTION
        
        # Log current item information for debugging
        if self.items:
            for item in self.items:
                if item.colour == self.target_item.colour:
                    self.get_logger().info(f'Attempting collection - Item diameter: {item.diameter}, x-offset: {item.x}')
                    break
        
        self.get_logger().info(f'Attempting to collect item...')
        
        # Call pick up service (only once now!)
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        
        future = self.pick_up_client.call_async(request)
        future.add_done_callback(self.pick_up_callback)
    
    def pick_up_callback(self, future):
        """Handle pick up service response - FIXED to avoid complex retry logic"""
        try:
            response = future.result()
            if response.success:
                self.items_collected += 1
                self.get_logger().info(f'Successfully collected item! Total: {self.items_collected}')
                self.state = RobotState.SEARCHING_ZONE
                self.approach_attempts = 0
            else:
                # Log the failure with current item info for calibration
                if self.items and self.target_item:
                    for item in self.items:
                        if item.colour == self.target_item.colour:
                            estimated_dist = self.estimate_distance_to_item(item)
                            self.get_logger().warn(f'Collection FAILED - Diameter: {item.diameter}, '
                                                 f'Estimated distance: {estimated_dist:.2f}m, '
                                                 f'X-offset: {item.x}, Message: {response.message}')
                            break
                
                self.approach_attempts += 1
                
                if self.approach_attempts < APPROACH_ATTEMPTS_MAX:
                    # MUCH SIMPLER: Just go back to fine approach to get closer
                    self.get_logger().info(f'Returning to fine approach, attempt {self.approach_attempts}')
                    self.state = RobotState.FINE_APPROACH
                else:
                    self.get_logger().warn('Max approach attempts reached, searching for new item')
                    self.state = RobotState.SEARCHING
                    self.approach_attempts = 0
                    
        except Exception as e:
            self.get_logger().error(f'Pick up service call failed: {e}')
            self.state = RobotState.SEARCHING
    
    def retry_collection(self):
        """Retry collection after moving closer"""
        self.state = RobotState.COLLECTING_ITEM
    
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
        twist.angular.z = ANGULAR_VELOCITY * 0.6
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
        if current_zone.size > 0.12:  # Lower threshold for easier zone detection
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.state = RobotState.DEPOSITING_ITEM
            return
        
        # Move towards zone with steering correction
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY * 0.6  # Slower approach to zone
        
        # Add steering towards zone
        if abs(current_zone.x) > 10:
            twist.angular.z = current_zone.x * 0.003  # Proportional control
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.4, 
                                min(ANGULAR_VELOCITY * 0.4, twist.angular.z))
        
        self.cmd_vel_pub.publish(twist)
    
    def deposit_item(self):
        """Attempt to offload the item in the zone - FIXED to prevent multiple calls"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # CRITICAL FIX: Immediately change state to prevent repeated calls!
        self.state = RobotState.WAITING_FOR_DEPOSIT
        
        # Call offload service (only once now!)
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        
        future = self.offload_client.call_async(request)
        future.add_done_callback(self.offload_callback)
    
    def offload_callback(self, future):
        """Handle offload service response - FIXED"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deposited item!')
                self.held_item_colour = None
                self.state = RobotState.SEARCHING
            else:
                self.get_logger().warn(f'Failed to deposit item: {response.message}')
                # Back up and try different zone
                self.state = RobotState.SEARCHING_ZONE  # Much simpler than backup logic
        except Exception as e:
            self.get_logger().error(f'Offload service call failed: {e}')
            self.state = RobotState.SEARCHING_ZONE
    
    def avoid_obstacle(self):
        """SMART obstacle avoidance - turn until item found or obstacle clear"""
        twist = Twist()
        
        # Check if we can see any items while turning
        if self.items and not self.held_item_colour:
            # Found an item! Switch to approaching it
            self.target_item = self.select_best_item()
            if self.target_item:
                self.get_logger().info('Item spotted during avoidance - switching to approach!')
                self.state = RobotState.APPROACHING_ITEM
                self.approach_attempts = 0
                return
        
        # Check if obstacle is clear
        if not self.obstacle_detected:
            self.obstacle_clear_counter += 1
        else:
            self.obstacle_clear_counter = 0
        
        # If obstacle has been clear for 20 iterations (2 seconds), we're good
        if self.obstacle_clear_counter >= 20:
            self.get_logger().info('Obstacle cleared - resuming previous activity')
            self.state = self.previous_state or RobotState.SEARCHING
            self.obstacle_clear_counter = 0
            return
        
        # Keep turning (alternate direction occasionally to avoid spinning forever)
        turn_direction = 1 if (self.get_clock().now().nanoseconds // 1000000000) % 4 < 2 else -1
        twist.linear.x = -LINEAR_VELOCITY * 0.3
        twist.angular.z = ANGULAR_VELOCITY * 0.7 * turn_direction
        
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
            size_score = min(item.diameter / 80.0, 1.0)
            
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