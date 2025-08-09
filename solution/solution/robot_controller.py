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
OBSTACLE_THRESHOLD = 0.35  # Distance to detect obstacles
APPROACH_ATTEMPTS_MAX = 5  # More attempts before giving up
STARTUP_WAIT_TIME = 30  # Wait for items to spawn

# IMPROVED Zone definitions based on assessment world
# These are the ACTUAL zone center positions from the world file
ZONE_POSITIONS = {
    'CYAN': (-3.42, 2.5),     # Bottom left zone (cyan)
    'PURPLE': (-3.42, -2.46), # Bottom right zone (purple) 
    'PINK': (2.57, 2.5),      # Top left zone (pink/deeppink)
    'GREEN': (2.57, -2.46)    # Top right zone (green/seagreen)
}

# Map zone colors from camera to world positions
ZONE_COLOR_MAP = {
    1: 'CYAN',    # ZONE_CYAN
    2: 'PURPLE',  # ZONE_PURPLE  
    3: 'GREEN',   # ZONE_GREEN
    4: 'PINK'     # ZONE_PINK
}

# States for the finite state machine
class RobotState(Enum):
    WAITING_FOR_ITEMS = 0
    SEARCHING = 1
    APPROACHING_ITEM = 2
    COLLECTING_ITEM = 3
    WAITING_FOR_COLLECTION = 4
    SEARCHING_ZONE = 5
    APPROACHING_ZONE = 6
    DEPOSITING_ITEM = 7
    WAITING_FOR_DEPOSIT = 8
    AVOIDING_OBSTACLE = 9
    FINE_APPROACH = 10
    NAVIGATING_TO_ZONE = 11  # New state for better zone navigation

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
        self.startup_counter = STARTUP_WAIT_TIME
        
        # Robot pose and navigation
        self.pose = Pose()
        self.yaw = 0.0
        self.target_item = None
        self.target_zone = None
        self.target_zone_position = None  # World coordinates of target zone
        self.held_item_colour = None
        
        # Item approach tracking
        self.approach_attempts = 0
        self.last_item_position = None
        self.collection_timeout = 0
        
        # Zone navigation improvements
        self.zone_search_angle = 0.0  # Track search rotation
        self.zone_approach_phase = 'seeking'  # 'seeking', 'centering', 'approaching'
        
        # Sensor data
        self.scan_data = []
        self.items = []
        self.zones = []
        self.item_holders = []
        
        # Obstacle avoidance
        self.obstacle_detected = False
        self.avoidance_start_yaw = 0.0
        self.obstacle_clear_counter = 0
        
        # Search pattern improvements
        self.search_mode = 'forward'
        self.search_timer = 0
        
        # Performance tracking
        self.items_collected = 0
        self.search_start_time = self.get_clock().now()
        
        # Callback groups for concurrent execution
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
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
        
        self.item_holders_sub = self.create_subscription(
            ItemHolders, '/item_holders', self.item_holders_callback, 10,
            callback_group=self.timer_cb_group
        )
        
        # Service clients
        self.pick_up_client = self.create_client(
            ItemRequest, '/pick_up_item',
            callback_group=self.client_cb_group
        )
        
        self.offload_client = self.create_client(
            ItemRequest, '/offload_item',
            callback_group=self.client_cb_group
        )
        
        # Wait for services
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
        
        # Check front sectors for obstacles
        front_ranges = []
        
        # Check front 40 degrees (20 degrees each side)
        for i in range(340, 360):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        for i in range(0, 20):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        
        # Detect obstacle if multiple close readings
        close_readings = [d for d in front_ranges if d < OBSTACLE_THRESHOLD]
        self.obstacle_detected = len(close_readings) >= 5
    
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
        holding_item = False
        for holder in self.item_holders:
            if holder.robot_id == self.robot_name and holder.holding_item:
                self.held_item_colour = holder.item_colour
                holding_item = True
                break
        else:
            self.held_item_colour = None
    
    def get_distance_to_position(self, target_x, target_y):
        """Calculate distance from current position to target world coordinates"""
        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_position(self, target_x, target_y):
        """Calculate angle from current position to target world coordinates"""
        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        return math.atan2(dy, dx)
    
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
        
        # Handle obstacle avoidance - can trigger during multiple states
        if (self.obstacle_detected and 
            self.state in [RobotState.SEARCHING, RobotState.APPROACHING_ZONE, 
                          RobotState.NAVIGATING_TO_ZONE, RobotState.SEARCHING_ZONE]):
            
            # Only avoid obstacles if we're not seeing items (when searching) or if we're navigating to zones
            should_avoid = False
            if self.state == RobotState.SEARCHING and not self.items:
                should_avoid = True
            elif self.state in [RobotState.APPROACHING_ZONE, RobotState.NAVIGATING_TO_ZONE, RobotState.SEARCHING_ZONE]:
                should_avoid = True
                
            if should_avoid:
                self.previous_state = self.state
                self.state = RobotState.AVOIDING_OBSTACLE
                self.obstacle_clear_counter = 0
                self.get_logger().info(f'Obstacle detected during {self.previous_state.name} - avoiding')
        
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
            pass
        elif self.state == RobotState.SEARCHING_ZONE:
            self.search_for_zone()
        elif self.state == RobotState.NAVIGATING_TO_ZONE:
            self.navigate_to_zone()
        elif self.state == RobotState.APPROACHING_ZONE:
            self.approach_zone()
        elif self.state == RobotState.DEPOSITING_ITEM:
            self.deposit_item()
        elif self.state == RobotState.WAITING_FOR_DEPOSIT:
            pass
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
        
        # Implement search pattern
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
        if estimated_distance <= COLLECTION_DISTANCE * 2.0:
            self.state = RobotState.FINE_APPROACH
            self.get_logger().info(f'Switching to fine approach, estimated distance: {estimated_distance:.2f}')
            return
        
        # Rough approach - move towards item with steering
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY * 0.8
        
        # Add steering correction towards the item
        if abs(current_item.x) > 10:  # If item is off-center
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
        
        # More aggressive forward movement to get really close
        if abs(current_item.x) < 5:  # Very well centered
            twist.linear.x = LINEAR_VELOCITY * 0.8
        elif abs(current_item.x) < 20:  # Reasonably centered
            twist.linear.x = LINEAR_VELOCITY * 0.6
        else:
            twist.linear.x = LINEAR_VELOCITY * 0.2
        
        # Fine angular adjustment
        if abs(current_item.x) > 3:
            twist.angular.z = current_item.x * 0.003
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.3, 
                                min(ANGULAR_VELOCITY * 0.3, twist.angular.z))
        
        self.cmd_vel_pub.publish(twist)
    
    def estimate_distance_to_item(self, item):
        """Realistic distance estimation based on item diameter"""
        if item.diameter > 290:
            return 0.15
        elif item.diameter > 270:
            return 0.30
        elif item.diameter > 250:
            return 0.40
        elif item.diameter > 240:
            return 0.50
        elif item.diameter > 220:
            return 0.60
        elif item.diameter > 210:
            return 0.75
        elif item.diameter > 200:
            return 1.0
        elif item.diameter > 190:
            return 1.3
        elif item.diameter > 160:
            return 1.7
        else:
            return 2.5
    
    def collect_item(self):
        """Attempt to pick up the item"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Immediately change state to prevent repeated calls
        self.state = RobotState.WAITING_FOR_COLLECTION
        
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
                # Start zone navigation with known zone positions
                self.state = RobotState.NAVIGATING_TO_ZONE
                self.approach_attempts = 0
                self.zone_approach_phase = 'seeking'
                
                # Set target zone based on held item color (this will be updated from item_holders_callback)
                self.select_target_zone_position()
                
            else:
                self.approach_attempts += 1
                
                if self.approach_attempts < APPROACH_ATTEMPTS_MAX:
                    self.get_logger().info(f'Collection failed, returning to fine approach, attempt {self.approach_attempts}')
                    self.state = RobotState.FINE_APPROACH
                else:
                    self.get_logger().warn('Max approach attempts reached, searching for new item')
                    self.state = RobotState.SEARCHING
                    self.approach_attempts = 0
                    
        except Exception as e:
            self.get_logger().error(f'Pick up service call failed: {e}')
            self.state = RobotState.SEARCHING
    
    def select_target_zone_position(self):
        """Select target zone world position based on any available zone (for initial deposit)"""
        # For the first deposit, just pick the closest zone
        min_distance = float('inf')
        closest_zone_pos = None
        
        for zone_name, position in ZONE_POSITIONS.items():
            distance = self.get_distance_to_position(position[0], position[1])
            if distance < min_distance:
                min_distance = distance
                closest_zone_pos = position
        
        self.target_zone_position = closest_zone_pos
        self.get_logger().info(f'Selected zone at position: {self.target_zone_position}')
    
    def search_for_zone(self):
        """Search for appropriate zone using camera"""
        if not self.held_item_colour:
            self.state = RobotState.SEARCHING
            return
        
        if self.zones:
            # Found a zone, switch to camera-based approach
            self.target_zone = self.select_best_zone()
            if self.target_zone:
                self.state = RobotState.APPROACHING_ZONE
                self.get_logger().info(f'Found zone using camera, approaching')
                return
        
        # Rotate to search for zones
        twist = Twist()
        twist.angular.z = ANGULAR_VELOCITY * 0.6
        self.cmd_vel_pub.publish(twist)
    
    def navigate_to_zone(self):
        """Navigate to target zone using world coordinates (more reliable than camera)"""
        if not self.held_item_colour or not self.target_zone_position:
            self.state = RobotState.SEARCHING_ZONE
            return
        
        target_x, target_y = self.target_zone_position
        distance = self.get_distance_to_position(target_x, target_y)
        
        # If we're close enough, switch to camera-based zone approach
        if distance < 1.5:
            self.get_logger().info('Close to zone area, switching to camera-based approach')
            self.state = RobotState.SEARCHING_ZONE
            return
        
        # Navigate towards zone using world coordinates
        target_angle = self.get_angle_to_position(target_x, target_y)
        angle_diff = normalize_angle(target_angle - self.yaw)
        
        twist = Twist()
        
        # Turn towards target if not aligned
        if abs(angle_diff) > 0.2:
            twist.angular.z = ANGULAR_VELOCITY if angle_diff > 0 else -ANGULAR_VELOCITY
            self.get_logger().debug(f'Turning towards zone, angle diff: {angle_diff:.2f}')
        else:
            # Move forward when aligned
            twist.linear.x = LINEAR_VELOCITY
            twist.angular.z = angle_diff * 1.0  # Small correction
            self.get_logger().debug(f'Moving towards zone, distance: {distance:.2f}m')
        
        self.cmd_vel_pub.publish(twist)
    
    def approach_zone(self):
        """Navigate towards the target zone using camera feedback"""
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
        
        # Check for obstacles specifically during zone approach
        if self.obstacle_detected:
            # If we're very close to the zone, it might be a false positive from the zone boundary
            if current_zone.size > 0.10:
                self.get_logger().info('Close to zone, ignoring obstacle detection (might be zone boundary)')
            else:
                # Real obstacle, let the main control loop handle it
                return
        
        # Check if we're over the zone (large size means we're close/over it)
        if current_zone.size > 0.99:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.state = RobotState.DEPOSITING_ITEM
            self.get_logger().info(f'Over zone (size: {current_zone.size:.3f}), attempting deposit')
            return
        
        # Improved zone approach with better centering
        twist = Twist()
        
        # Zone centering logic
        if abs(current_zone.x) > 5:  # Not centered
            # Turn to center the zone
            twist.angular.z = current_zone.x * 0.004  # Proportional turning
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.5, 
                                min(ANGULAR_VELOCITY * 0.5, twist.angular.z))
            twist.linear.x = LINEAR_VELOCITY * 0.5  # Slow forward movement while centering
            self.get_logger().debug(f'Centering on zone, x-offset: {current_zone.x}')
        else:
            # Zone is centered, move forward
            twist.linear.x = LINEAR_VELOCITY * 0.9
            self.get_logger().debug(f'Zone centered, moving forward, size: {current_zone.size:.3f}')
        
        self.cmd_vel_pub.publish(twist)
    
    def deposit_item(self):
        """Attempt to offload the item in the zone"""

        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Immediately change state to prevent repeated calls
        self.state = RobotState.WAITING_FOR_DEPOSIT
        
        self.get_logger().info('Attempting to deposit item in zone')
        
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
                self.target_zone_position = None
                self.state = RobotState.SEARCHING
            else:
                self.get_logger().warn(f'Failed to deposit item: {response.message}')
                # Try to find another zone
                self.state = RobotState.SEARCHING_ZONE
        except Exception as e:
            self.get_logger().error(f'Offload service call failed: {e}')
            self.state = RobotState.SEARCHING_ZONE
    
    def avoid_obstacle(self):
        """Smart obstacle avoidance that works for different states"""
        twist = Twist()
        
        # Check if we can see any items while avoiding (only relevant when searching for items)
        if self.items and not self.held_item_colour and self.previous_state == RobotState.SEARCHING:
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
        
        # If obstacle has been clear for sufficient time, resume previous activity
        if self.obstacle_clear_counter >= 20:
            if self.previous_state:
                self.get_logger().info(f'Obstacle cleared - resuming {self.previous_state.name}')
                self.state = self.previous_state
            else:
                self.get_logger().info('Obstacle cleared - resuming search')
                self.state = RobotState.SEARCHING
            self.obstacle_clear_counter = 0
            return
        
        # Obstacle avoidance maneuver
        # Back up and turn - direction depends on the situation
        turn_direction = 1
        
        # If we were navigating to a zone, try to turn in a direction that doesn't take us too far from target
        if (self.previous_state in [RobotState.NAVIGATING_TO_ZONE, RobotState.APPROACHING_ZONE] 
            and self.target_zone_position):
            
            target_x, target_y = self.target_zone_position
            target_angle = self.get_angle_to_position(target_x, target_y)
            angle_diff = normalize_angle(target_angle - self.yaw)
            
            # Turn towards the target zone direction if possible
            if abs(angle_diff) > 0.5:
                turn_direction = 1 if angle_diff > 0 else -1
                self.get_logger().debug(f'Avoiding obstacle while turning towards zone')
        
        twist.linear.x = -LINEAR_VELOCITY * 0.5  # Back up slightly
        twist.angular.z = ANGULAR_VELOCITY * 0.9 * turn_direction
        
        self.cmd_vel_pub.publish(twist)
    
    def select_best_item(self):
        """Select the best item to collect based on visibility and value"""
        if not self.items:
            return None
        
        best_item = None
        best_score = float('-inf')
        
        for item in self.items:
            # Score based on centering (higher is better when centered)
            centering_score = 1.0 - abs(item.x) / 320.0
            
            # Size score (larger diameter = closer = better)
            size_score = min(item.diameter / 80.0, 1.0)
            
            # Value score
            value_score = item.value / 20.0
            
            # Total score
            total_score = centering_score * 2.0 + size_score * 1.5 + value_score
            
            if total_score > best_score:
                best_score = total_score
                best_item = item
        
        return best_item
    
    def select_best_zone(self):
        """Select the best visible zone based on size and centering"""
        if not self.zones:
            return None
            
        best_zone = None
        best_score = float('-inf')
        
        for zone in self.zones:
            # Score based on size (proximity) and centering
            size_score = zone.size * 2.0  # Prioritize larger (closer) zones
            centering_score = 1.0 - abs(zone.x) / 320.0
            
            # Total score - zones are our target, no penalties!
            total_score = size_score + centering_score
            
            if total_score > best_score:
                best_score = total_score
                best_zone = zone
        
        return best_zone
    
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