#!/usr/bin/env python3

"""
Item Detector Node for AURO Coursework
This node processes camera images and detects colored items in the environment.
Students must use this node without modification as per assessment requirements.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
from typing import List, Tuple, Dict, Optional

class ItemDetection:
    """Class to represent a detected item"""
    def __init__(self, item_type: str, pixel_coords: Tuple[int, int], 
                 confidence: float, bounding_box: Tuple[int, int, int, int]):
        self.item_type = item_type
        self.pixel_coords = pixel_coords  # (x, y) in image coordinates
        self.confidence = confidence
        self.bounding_box = bounding_box  # (x, y, width, height)
        self.world_coords = None  # Will be set if depth information available

class ItemDetector(Node):
    """
    Node that processes camera images to detect colored items.
    This node must be used without modification as per assessment requirements.
    """
    
    def __init__(self):
        super().__init__('item_detector')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detection_topic', '/item_detections')
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('max_contour_area', 50000)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('robot_frame', 'base_link')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Color ranges for item detection (HSV)
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([170, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'green': [
                (np.array([40, 100, 100]), np.array([80, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([30, 255, 255]))
            ]
        }
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String, self.get_parameter('detection_topic').value, 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/item_detector/debug_image', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.get_parameter('camera_topic').value, 
            self.image_callback, 10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        
        # Store latest depth image
        self.latest_depth_image = None
        
        # Statistics
        self.detection_count = 0
        self.false_positive_count = 0
        
        self.get_logger().info("Item Detector Node initialized")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Process camera info to extract calibration parameters"""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
        # Only log once
        if self.camera_matrix is not None:
            self.get_logger().info("Camera calibration parameters received")
    
    def depth_callback(self, msg: Image):
        """Store latest depth image for 3D coordinate calculation"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")
    
    def image_callback(self, msg: Image):
        """Main image processing callback"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect items in the image
            detections = self.detect_items(cv_image)
            
            # Create debug image
            debug_image = self.create_debug_image(cv_image, detections)
            
            # Publish detections
            self.publish_detections(detections, msg.header)
            
            # Publish debug image
            self.publish_debug_image(debug_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def detect_items(self, image: np.ndarray) -> List[ItemDetection]:
        """Detect colored items in the image"""
        detections = []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        hsv_blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        # Detect each color
        for color_name, color_ranges in self.color_ranges.items():
            mask = self.create_color_mask(hsv_blurred, color_ranges)
            color_detections = self.find_contours_and_classify(mask, color_name, image)
            detections.extend(color_detections)
        
        return detections
    
    def create_color_mask(self, hsv_image: np.ndarray, color_ranges: List[Tuple]) -> np.ndarray:
        """Create a mask for the specified color ranges"""
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in color_ranges:
            color_mask = cv2.inRange(hsv_image, lower, upper)
            mask = cv2.bitwise_or(mask, color_mask)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def find_contours_and_classify(self, mask: np.ndarray, color_name: str, 
                                  original_image: np.ndarray) -> List[ItemDetection]:
        """Find contours in the mask and classify them as items"""
        detections = []
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        min_area = self.get_parameter('min_contour_area').value
        max_area = self.get_parameter('max_contour_area').value
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < min_area or area > max_area:
                continue
            
            # Calculate bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate center point
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Calculate confidence based on contour properties
            confidence = self.calculate_confidence(contour, area, original_image[y:y+h, x:x+w])
            
            # Only accept detections above threshold
            if confidence >= self.get_parameter('confidence_threshold').value:
                detection = ItemDetection(
                    item_type=color_name,
                    pixel_coords=(center_x, center_y),
                    confidence=confidence,
                    bounding_box=(x, y, w, h)
                )
                
                # Add 3D coordinates if depth information available
                if self.latest_depth_image is not None and self.camera_matrix is not None:
                    world_coords = self.pixel_to_world(center_x, center_y)
                    if world_coords is not None:
                        detection.world_coords = world_coords
                
                detections.append(detection)
                self.detection_count += 1
        
        return detections
    
    def calculate_confidence(self, contour: np.ndarray, area: float, roi_image: np.ndarray) -> float:
        """Calculate confidence score for a detection"""
        # Base confidence from contour area (normalized)
        area_confidence = min(area / 5000.0, 1.0)  # Normalize to max area
        
        # Confidence from contour shape (how circular/square it is)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            shape_confidence = min(circularity * 2, 1.0)  # Boost circular shapes
        else:
            shape_confidence = 0.0
        
        # Confidence from color saturation in the region
        hsv_roi = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        saturation = np.mean(hsv_roi[:, :, 1])
        saturation_confidence = min(saturation / 255.0, 1.0)
        
        # Combine confidences
        total_confidence = (area_confidence * 0.4 + 
                           shape_confidence * 0.3 + 
                           saturation_confidence * 0.3)
        
        return total_confidence
    
    def pixel_to_world(self, pixel_x: int, pixel_y: int) -> Optional[Tuple[float, float, float]]:
        """Convert pixel coordinates to world coordinates using depth information"""
        if (self.latest_depth_image is None or 
            self.camera_matrix is None or 
            pixel_y >= self.latest_depth_image.shape[0] or
            pixel_x >= self.latest_depth_image.shape[1]):
            return None
        
        # Get depth value
        depth = self.latest_depth_image[pixel_y, pixel_x]
        
        # Check for valid depth
        if np.isnan(depth) or depth <= 0:
            return None
        
        # Convert to normalized coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Calculate 3D coordinates relative to camera
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        return (x, y, z)
    
    def create_debug_image(self, original_image: np.ndarray, 
                          detections: List[ItemDetection]) -> np.ndarray:
        """Create debug image with detection overlays"""
        debug_image = original_image.copy()
        
        # Color map for drawing
        color_map = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255)
        }
        
        for detection in detections:
            # Draw bounding box
            x, y, w, h = detection.bounding_box
            color = color_map.get(detection.item_type, (255, 255, 255))
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            
            # Draw center point
            center_x, center_y = detection.pixel_coords
            cv2.circle(debug_image, (center_x, center_y), 5, color, -1)
            
            # Draw label
            label = f"{detection.item_type} ({detection.confidence:.2f})"
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw 3D coordinates if available
            if detection.world_coords is not None:
                coord_text = f"({detection.world_coords[0]:.2f}, {detection.world_coords[1]:.2f}, {detection.world_coords[2]:.2f})"
                cv2.putText(debug_image, coord_text, (x, y + h + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Add detection count
        cv2.putText(debug_image, f"Detections: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_image
    
    def publish_detections(self, detections: List[ItemDetection], header: Header):
        """Publish detection results"""
        detection_data = {
            'header': {
                'stamp': {
                    'sec': header.stamp.sec,
                    'nanosec': header.stamp.nanosec
                },
                'frame_id': header.frame_id
            },
            'detections': []
        }
        
        for detection in detections:
            detection_dict = {
                'item_type': detection.item_type,
                'pixel_coords': detection.pixel_coords,
                'confidence': detection.confidence,
                'bounding_box': detection.bounding_box
            }
            
            if detection.world_coords is not None:
                detection_dict['world_coords'] = detection.world_coords
            
            detection_data['detections'].append(detection_dict)
        
        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(detection_data)
        self.detection_pub.publish(msg)
        
        if len(detections) > 0:
            self.get_logger().debug(f"Published {len(detections)} item detections")
    
    def publish_debug_image(self, debug_image: np.ndarray):
        """Publish debug image with detection overlays"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ItemDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()