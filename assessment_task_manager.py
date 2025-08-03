#!/usr/bin/env python3

"""
Assessment Task Manager Node for AURO Coursework
This node manages the simulated world for the item retrieval task.
It spawns items, tracks collection/return events, and manages zones.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import EntityState
from tf2_ros import TransformListener, Buffer
import random
import math
import json
import time
from typing import Dict, List, Tuple
import yaml

class ItemType:
    """Enumeration of item types"""
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    YELLOW = "yellow"

class Zone:
    """Represents a collection zone"""
    def __init__(self, name: str, position: Point, item_type: str, radius: float = 1.0):
        self.name = name
        self.position = position
        self.item_type = item_type
        self.radius = radius
        self.collected_items = 0

class Item:
    """Represents an item in the world"""
    def __init__(self, item_id: str, item_type: str, position: Point):
        self.item_id = item_id
        self.item_type = item_type
        self.position = position
        self.collected = False
        self.collected_by = None
        self.spawn_time = time.time()

class AssessmentTaskManager(Node):
    """
    Main node for managing the assessment task.
    Handles item spawning, collection tracking, and zone management.
    """
    
    def __init__(self):
        super().__init__('assessment_task_manager')
        
        # Parameters
        self.declare_parameter('world_config_file', 'worlds/assessment_world.yaml')
        self.declare_parameter('max_items_per_type', 3)
        self.declare_parameter('item_spawn_radius', 8.0)
        self.declare_parameter('respawn_delay', 5.0)
        
        # Load world configuration
        self.world_config = self.load_world_config()
        
        # Initialize data structures
        self.zones: Dict[str, Zone] = {}
        self.items: Dict[str, Item] = {}
        self.robot_states: Dict[str, Dict] = {}
        self.item_counter = 0
        
        # Publishers
        self.item_status_pub = self.create_publisher(String, '/assessment/item_status', 10)
        self.zone_status_pub = self.create_publisher(String, '/assessment/zone_status', 10)
        self.task_score_pub = self.create_publisher(Int32, '/assessment/score', 10)
        
        # Subscribers
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.collect_item_sub = self.create_subscription(
            String, '/collect_item', self.collect_item_callback, 10)
        self.return_item_sub = self.create_subscription(
            String, '/return_item', self.return_item_callback, 10)
        
        # Services
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.update_task)
        
        # Initialize world
        self.setup_world()
        
        self.get_logger().info("Assessment Task Manager initialized")
    
    def load_world_config(self) -> Dict:
        """Load world configuration from YAML file"""
        config_file = self.get_parameter('world_config_file').value
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().warn(f"Config file {config_file} not found, using defaults")
            return self.get_default_config()
    
    def get_default_config(self) -> Dict:
        """Return default world configuration"""
        return {
            'zones': [
                {'name': 'red_zone', 'position': [8.0, 8.0, 0.0], 'item_type': 'red', 'radius': 1.5},
                {'name': 'blue_zone', 'position': [-8.0, 8.0, 0.0], 'item_type': 'blue', 'radius': 1.5},
                {'name': 'green_zone', 'position': [8.0, -8.0, 0.0], 'item_type': 'green', 'radius': 1.5},
                {'name': 'yellow_zone', 'position': [-8.0, -8.0, 0.0], 'item_type': 'yellow', 'radius': 1.5}
            ],
            'obstacles': [
                {'position': [0.0, 0.0, 0.0], 'size': [2.0, 2.0, 1.0]},
                {'position': [4.0, 4.0, 0.0], 'size': [1.0, 1.0, 1.0]},
                {'position': [-4.0, -4.0, 0.0], 'size': [1.0, 1.0, 1.0]}
            ],
            'spawn_areas': [
                {'center': [0.0, 0.0], 'radius': 6.0}
            ]
        }
    
    def setup_world(self):
        """Initialize zones and spawn initial items"""
        # Create zones
        for zone_config in self.world_config['zones']:
            pos = Point(x=zone_config['position'][0], 
                       y=zone_config['position'][1], 
                       z=zone_config['position'][2])
            zone = Zone(zone_config['name'], pos, zone_config['item_type'], zone_config['radius'])
            self.zones[zone.name] = zone
        
        # Spawn initial items
        for item_type in [ItemType.RED, ItemType.BLUE, ItemType.GREEN, ItemType.YELLOW]:
            for _ in range(self.get_parameter('max_items_per_type').value):
                self.spawn_item(item_type)
    
    def spawn_item(self, item_type: str):
        """Spawn a new item in the world"""
        self.item_counter += 1
        item_id = f"item_{item_type}_{self.item_counter}"
        
        # Generate random position
        position = self.generate_spawn_position()
        
        # Create item
        item = Item(item_id, item_type, position)
        self.items[item_id] = item
        
        # Spawn in Gazebo (this would be implemented with actual Gazebo service calls)
        self.spawn_item_in_gazebo(item)
        
        # Publish status
        self.publish_item_status()
        
        self.get_logger().info(f"Spawned {item_type} item at ({position.x:.2f}, {position.y:.2f})")
    
    def generate_spawn_position(self) -> Point:
        """Generate a random spawn position within valid areas"""
        spawn_area = self.world_config['spawn_areas'][0]  # Use first spawn area
        center = spawn_area['center']
        radius = spawn_area['radius']
        
        # Generate random position within circle
        angle = random.uniform(0, 2 * math.pi)
        r = random.uniform(0, radius)
        
        x = center[0] + r * math.cos(angle)
        y = center[1] + r * math.sin(angle)
        
        return Point(x=x, y=y, z=0.0)
    
    def spawn_item_in_gazebo(self, item: Item):
        """Spawn item in Gazebo simulation"""
        # This would implement actual Gazebo spawning
        # For now, we'll just log the action
        self.get_logger().debug(f"Spawning {item.item_type} item {item.item_id} in Gazebo")
    
    def robot_pose_callback(self, msg: PoseStamped):
        """Handle robot pose updates"""
        robot_id = msg.header.frame_id
        if robot_id not in self.robot_states:
            self.robot_states[robot_id] = {}
        
        self.robot_states[robot_id]['pose'] = msg.pose
        self.robot_states[robot_id]['last_update'] = time.time()
        
        # Check for item collection opportunities
        self.check_item_collection(robot_id, msg.pose.position)
    
    def check_item_collection(self, robot_id: str, robot_pos: Point):
        """Check if robot is close enough to collect any items"""
        collection_radius = 0.5  # meters
        
        for item_id, item in self.items.items():
            if item.collected:
                continue
            
            distance = math.sqrt((robot_pos.x - item.position.x)**2 + 
                               (robot_pos.y - item.position.y)**2)
            
            if distance < collection_radius:
                self.get_logger().info(f"Robot {robot_id} can collect item {item_id}")
                # Publish collection opportunity
                msg = String()
                msg.data = json.dumps({
                    'robot_id': robot_id,
                    'item_id': item_id,
                    'item_type': item.item_type,
                    'distance': distance
                })
                self.item_status_pub.publish(msg)
    
    def collect_item_callback(self, msg: String):
        """Handle item collection requests"""
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            item_id = data['item_id']
            
            if item_id in self.items and not self.items[item_id].collected:
                self.items[item_id].collected = True
                self.items[item_id].collected_by = robot_id
                
                # Update robot state
                if robot_id not in self.robot_states:
                    self.robot_states[robot_id] = {}
                self.robot_states[robot_id]['holding_item'] = item_id
                
                # Remove from Gazebo
                self.delete_item_from_gazebo(item_id)
                
                self.get_logger().info(f"Item {item_id} collected by robot {robot_id}")
                self.publish_item_status()
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid collect_item message: {e}")
    
    def return_item_callback(self, msg: String):
        """Handle item return requests"""
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            zone_name = data['zone_name']
            
            if robot_id in self.robot_states and 'holding_item' in self.robot_states[robot_id]:
                item_id = self.robot_states[robot_id]['holding_item']
                item = self.items[item_id]
                
                # Check if correct zone
                if zone_name in self.zones:
                    zone = self.zones[zone_name]
                    if zone.item_type == item.item_type:
                        # Successful return
                        zone.collected_items += 1
                        del self.items[item_id]
                        del self.robot_states[robot_id]['holding_item']
                        
                        # Spawn replacement item
                        self.create_timer(self.get_parameter('respawn_delay').value, 
                                        lambda: self.spawn_item(item.item_type))
                        
                        self.get_logger().info(f"Item {item_id} returned to {zone_name}")
                        self.publish_zone_status()
                        self.publish_score()
                    else:
                        self.get_logger().warn(f"Wrong zone: {item.item_type} item to {zone.item_type} zone")
                        
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid return_item message: {e}")
    
    def delete_item_from_gazebo(self, item_id: str):
        """Remove item from Gazebo simulation"""
        self.get_logger().debug(f"Deleting item {item_id} from Gazebo")
    
    def publish_item_status(self):
        """Publish current item status"""
        status = {
            'items': {}
        }
        
        for item_id, item in self.items.items():
            status['items'][item_id] = {
                'type': item.item_type,
                'position': [item.position.x, item.position.y, item.position.z],
                'collected': item.collected,
                'collected_by': item.collected_by
            }
        
        msg = String()
        msg.data = json.dumps(status)
        self.item_status_pub.publish(msg)
    
    def publish_zone_status(self):
        """Publish current zone status"""
        status = {
            'zones': {}
        }
        
        for zone_name, zone in self.zones.items():
            status['zones'][zone_name] = {
                'position': [zone.position.x, zone.position.y, zone.position.z],
                'item_type': zone.item_type,
                'collected_items': zone.collected_items
            }
        
        msg = String()
        msg.data = json.dumps(status)
        self.zone_status_pub.publish(msg)
    
    def publish_score(self):
        """Publish current task score"""
        total_score = sum(zone.collected_items for zone in self.zones.values())
        msg = Int32()
        msg.data = total_score
        self.task_score_pub.publish(msg)
    
    def update_task(self):
        """Periodic task update"""
        # Check for stale robot states
        current_time = time.time()
        stale_robots = []
        
        for robot_id, state in self.robot_states.items():
            if current_time - state.get('last_update', 0) > 10.0:  # 10 seconds timeout
                stale_robots.append(robot_id)
        
        for robot_id in stale_robots:
            del self.robot_states[robot_id]
            self.get_logger().warn(f"Robot {robot_id} state expired")
        
        # Publish periodic status updates
        self.publish_item_status()
        self.publish_zone_status()
        self.publish_score()

def main(args=None):
    rclpy.init(args=args)
    
    node = AssessmentTaskManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
