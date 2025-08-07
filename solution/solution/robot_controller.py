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
OBSTACLE_THRESHOLD = 0.35  # INCREASED from 0.2 to see obstacles from further away
APPROACH_ATTEMPTS_MAX = 5  # More attempts before giving up
STARTUP_WAIT_TIME = 30  # Wait 3 seconds for items to spawn

# Corner WALL positions (not the zones themselves!)
# These are the actual wall intersections where robots get stuck
CORNER_WALL_POSITIONS = [
    (-4.0, 3.0),   # Bottom left wall corner
    (-4.0, -3.0),  # Bottom right wall corner  
    (3.0, 3.0),    # Top left wall corner
    (3.0, -3.0),   # Top right wall corner
]
WALL_BLACKLIST_RADIUS = 0.4  # Distance from walls to avoid (much smaller radius)

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
        
        # Obstacle avoidance - ENHANCED
        self.obstacle_detected = False
        self.avoidance_start_yaw = 0.0
        self.obstacle_clear_counter = 0  # Count how long obstacle has been clear
        
        # Wall blacklist tracking - NEW (only walls, not zones!)
        self.wall_blacklist_active = False  # Only active when carrying item
        
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
        
        # Check front sectors for obstacles - ENHANCED to see obstacles from further away
        front_ranges = []
        
        # Check front 40 degrees (20 degrees each side)
        for i in range(340, 360):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        for i in range(0, 20):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0.05:
                front_ranges.append(msg.ranges[i])
        
        # Detect obstacle if multiple close readings (walls or other obstacles)
        close_readings = [d for d in front_ranges if d < OBSTACLE_THRESHOLD]  # Increased threshold
        self.obstacle_detected = len(close_readings) >= 5  # Reduced requirement for detection
    
    def items_callback(self, msg):
        """Update visible items list"""
        self.items = msg.data
    
    def zones_callback(self, msg):
        """Update visible zones list"""
        self.zones = msg.data
    
    def item_holders_callback(self, msg):
        """Track which robot is holding which item"""
        self.item_holders = msg.data
        
        # Check if we're holding an item and update corner blacklist
        holding_item = False
        for holder in self.item_holders:
            if holder.robot_id == self.robot_name and holder.holding_item:
                self.held_item_colour = holder.item_colour
                holding_item = True
                break
        else:
            self.held_item_colour = None
            
        # Activate wall blacklist only when carrying an item
        self.wall_blacklist_active = holding_item
        
        if self.wall_blacklist_active:
            self.get_logger().debug('Wall blacklist ACTIVE - carrying item, avoiding corner walls')
        else:
            self.get_logger().debug('Wall blacklist INACTIVE - not carrying item')
    
    def is_near_corner_walls(self, x=None, y=None):
        """Check if current position (or given position) is too close to corner WALLS (not zones)"""
        if not self.wall_blacklist_active:
            return False  # No blacklist when not carrying item
            
        # Use current position if not specified
        if x is None:
            x = self.pose.position.x
        if y is None:
            y = self.pose.position.y
            
        # Check distance to each corner WALL intersection
        for wall_x, wall_y in CORNER_WALL_POSITIONS:
            distance = math.sqrt((x - wall_x)**2 + (y - wall_y)**2)
            if distance < WALL_BLACKLIST_RADIUS:
                return True
        return False
    
    def should_avoid_wall_direction(self, target_x, target_y):
        """Check if moving towards target would lead too close to corner walls"""
        if not self.wall_blacklist_active:
            return False
            
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        
        # Project position if we move towards target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 0:
            # Normalize direction
            dx /= distance
            dy /= distance
            
            # Project forward by some distance
            projection_distance = min(0.3, distance)  # Shorter projection for walls
            future_x = current_x + dx * projection_distance
            future_y = current_y + dy * projection_distance
            
            return self.is_near_corner_walls(future_x, future_y)
        
        return False
    
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
        
        # Handle obstacle avoidance with wall awareness (but still allow zone access!)
        if (self.obstacle_detected and 
            self.state == RobotState.SEARCHING and
            not self.items):
            
            # If too close to corner walls, try to escape even if obstacle detected
            if self.is_near_corner_walls():
                self.get_logger().info('Too close to corner walls - attempting escape despite obstacle')
                self.escape_corner_walls()
                return
            else:
                self.previous_state = self.state
                self.state = RobotState.AVOIDING_OBSTACLE
                self.obstacle_clear_counter = 0
                self.get_logger().info('Obstacle detected while searching - avoiding')
        
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
    
    def escape_corner_walls(self):
        """Escape from being too close to corner walls (not the zones!)"""
        twist = Twist()
        
        # Find direction away from nearest corner wall
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        
        # Find nearest corner wall
        nearest_wall = None
        min_distance = float('inf')
        for wall_x, wall_y in CORNER_WALL_POSITIONS:
            distance = math.sqrt((current_x - wall_x)**2 + (current_y - wall_y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_wall = (wall_x, wall_y)
        
        if nearest_wall:
            wall_x, wall_y = nearest_wall
            
            # Calculate direction away from wall corner
            away_x = current_x - wall_x
            away_y = current_y - wall_y
            
            # Normalize
            distance = math.sqrt(away_x**2 + away_y**2)
            if distance > 0:
                away_x /= distance
                away_y /= distance
                
                # Calculate desired heading (away from wall)
                desired_heading = math.atan2(away_y, away_x)
                heading_error = normalize_angle(desired_heading - self.yaw)
                
                # Turn towards escape direction
                if abs(heading_error) > 0.1:
                    twist.angular.z = ANGULAR_VELOCITY if heading_error > 0 else -ANGULAR_VELOCITY
                else:
                    # Move forward when heading is correct
                    twist.linear.x = LINEAR_VELOCITY
                    twist.angular.z = heading_error * 2.0  # Proportional correction
        
        self.cmd_vel_pub.publish(twist)
    
    def search_for_items(self):
        """IMPROVED search pattern for finding items with wall avoidance (but allowing zone access)"""
        if self.items and not self.held_item_colour:
            # Found an item, switch to approaching
            self.target_item = self.select_best_item()
            if self.target_item:
                self.state = RobotState.APPROACHING_ITEM
                self.approach_attempts = 0
                self.get_logger().info(f'Found {self.target_item.colour} item, approaching')
                return
        
        # Check if we're too close to corner walls and need to escape
        if self.is_near_corner_walls():
            self.get_logger().info('Too close to corner walls during search - escaping')
            self.escape_corner_walls()
            return
        
        # Implement search pattern with wall avoidance (but zones are OK!)
        twist = Twist()
        self.search_timer += 1
        
        if self.search_mode == 'forward':
            # Check if forward movement would lead too close to walls
            future_x = self.pose.position.x + math.cos(self.yaw) * 0.3
            future_y = self.pose.position.y + math.sin(self.yaw) * 0.3
            
            if self.should_avoid_wall_direction(future_x, future_y):
                # Switch to turning mode to avoid walls
                self.search_mode = 'turn_left' if random.random() > 0.5 else 'turn_right'
                self.search_timer = 0
                self.get_logger().info('Avoiding potential wall during forward search')
            else:
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
        """Realistic distance estimation"""
        if item.diameter > 180:
            return 0.15
        elif item.diameter > 170:
            return 0.30
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
        """Attempt to pick up the item"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Immediately change state to prevent repeated calls
        self.state = RobotState.WAITING_FOR_COLLECTION
        
        # Log current item information for debugging
        if self.items:
            for item in self.items:
                if item.colour == self.target_item.colour:
                    self.get_logger().info(f'Attempting collection - Item diameter: {item.diameter}, x-offset: {item.x}')
                    break
        
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
                    self.get_logger().info(f'Returning to fine approach, attempt {self.approach_attempts}')
                    self.state = RobotState.FINE_APPROACH
                else:
                    self.get_logger().warn('Max approach attempts reached, searching for new item')
                    self.state = RobotState.SEARCHING
                    self.approach_attempts = 0
                    
        except Exception as e:
            self.get_logger().error(f'Pick up service call failed: {e}')
            self.state = RobotState.SEARCHING
    
    def search_for_zone(self):
        """Search for appropriate zone to deposit item (zones are in corners - that's OK!)"""
        if not self.held_item_colour:
            self.state = RobotState.SEARCHING
            return
        
        # Only avoid walls, NOT zones! Zones are supposed to be in corners
        if self.is_near_corner_walls():
            self.get_logger().info('Too close to corner walls while searching for zone - escaping')
            self.escape_corner_walls()
            return
        
        if self.zones:
            # Find matching zone - zones in corners are GOOD!
            self.target_zone = self.select_best_zone()
            if self.target_zone:
                self.state = RobotState.APPROACHING_ZONE
                self.get_logger().info(f'Found zone, approaching (zones in corners are OK!)')
                return
        
        # Rotate to search for zones, but avoid walls
        twist = Twist()
        twist.angular.z = ANGULAR_VELOCITY * 0.6
        self.cmd_vel_pub.publish(twist)
    
    def approach_zone(self):
        """Navigate towards the target zone (zones are in corners and that's the goal!)"""
        if not self.target_zone or not self.zones:
            self.state = RobotState.SEARCHING_ZONE
            return
        
        # Only escape walls, not zones! We WANT to go to zones even if they're in corners
        if self.is_near_corner_walls():
            self.get_logger().info('Too close to corner walls while approaching zone - escaping')
            self.escape_corner_walls()
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
        
        # Check if we're over the zone
        if current_zone.size > 0.12:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.state = RobotState.DEPOSITING_ITEM
            return
        
        # Move towards zone with steering correction - zones are the goal!
        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY * 0.6
        
        # Add steering towards zone
        if abs(current_zone.x) > 10:
            twist.angular.z = current_zone.x * 0.003
            twist.angular.z = max(-ANGULAR_VELOCITY * 0.4, 
                                min(ANGULAR_VELOCITY * 0.4, twist.angular.z))
        
        self.cmd_vel_pub.publish(twist)
    
    def deposit_item(self):
        """Attempt to offload the item in the zone"""
        # Stop moving
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Immediately change state to prevent repeated calls
        self.state = RobotState.WAITING_FOR_DEPOSIT
        
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
                self.wall_blacklist_active = False  # Deactivate wall blacklist
                self.state = RobotState.SEARCHING
            else:
                self.get_logger().warn(f'Failed to deposit item: {response.message}')
                self.state = RobotState.SEARCHING_ZONE
        except Exception as e:
            self.get_logger().error(f'Offload service call failed: {e}')
            self.state = RobotState.SEARCHING_ZONE
    
    def avoid_obstacle(self):
        """Smart obstacle avoidance with corner awareness"""
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
        
        # Keep turning but avoid walls when carrying item (zones are OK!)
        turn_direction = 1 if (self.get_clock().now().nanoseconds // 1000000000) % 4 < 2 else -1
        
        # If carrying item and would turn too close to walls, reverse turn direction
        if self.wall_blacklist_active:
            future_yaw = self.yaw + (ANGULAR_VELOCITY * 0.7 * turn_direction * 0.1)
            future_x = self.pose.position.x + math.cos(future_yaw) * 0.2
            future_y = self.pose.position.y + math.sin(future_yaw) * 0.2
            
            if self.is_near_corner_walls(future_x, future_y):
                turn_direction *= -1  # Reverse turn direction
                self.get_logger().info('Reversing turn direction to avoid walls (zones are OK)')
        
        twist.linear.x = -LINEAR_VELOCITY * 0.3  # Back up slightly
        twist.angular.z = ANGULAR_VELOCITY * 0.7 * turn_direction
        
        self.cmd_vel_pub.publish(twist)
    
    def select_best_item(self):
        """Select the best item to collect based on visibility, value (no zone penalties!)"""
        if not self.items:
            return None
        
        # Prefer items that are centered, larger (closer), and higher value
        # NO penalties for zones since zones are where we WANT to go!
        best_item = None
        best_score = float('-inf')
        
        for item in self.items:
            # Score based on centering (higher is better when centered)
            centering_score = 1.0 - abs(item.x) / 320.0
            
            # Size score (larger diameter = closer = better)
            size_score = min(item.diameter / 80.0, 1.0)
            
            # Value score
            value_score = item.value / 20.0
            
            # Total score with adjusted weights (no corner penalties for zones!)
            total_score = centering_score * 2.0 + size_score * 1.5 + value_score
            
            if total_score > best_score:
                best_score = total_score
                best_item = item
        
        return best_item
    
    def select_best_zone(self):
        
        # For deposit, prefer closer/larger zones - zones in corners are exactly what we want!
        best_zone = None
        best_score = float('-inf')
        
        for zone in self.zones:
            # Score based on size (proximity) and centering
            size_score = zone.size * 2.0  # Prioritize larger (closer) zones
            centering_score = 1.0 - abs(zone.x) / 320.0
            
            # NO penalty for zones! Zones are supposed to be in corners!
            total_score = size_score + centering_score
            
            if total_score > best_score:
                best_score = total_score
                best_zone = zone
        
        # For initial deposit, prefer closer/larger zones that aren't in corners
        best_zone = None
        best_score = float('-inf')
        
        for zone in self.zones:
            # Score based on size (proximity) and centering
            size_score = zone.size * 2.0  # Prioritize larger (closer) zones
            centering_score = 1.0 - abs(zone.x) / 320.0
            
            # Corner avoidance score - penalty if zone is in corner
            corner_penalty = 0.0
            if self.corner_blacklist_active:
                # Estimate zone position in world coordinates (very rough)
                zone_world_x = self.pose.position.x + (zone.x / 100.0)  # Very rough approximation
                zone_world_y = self.pose.position.y
                
                if self.is_in_blacklisted_corner(zone_world_x, zone_world_y):
                    corner_penalty = -1.0  # Penalty for zones in corners (but less than items)
            
            total_score = size_score + centering_score + corner_penalty
            
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
        