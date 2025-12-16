#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
import base64
import numpy as np
import time
import json
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import math
from server_wrapper import send_request

class GSClient:
    def __init__(self, port:int = 12185):
        self.url = f"http://localhost:{port}/gdsam"

    def detections(self, image: np.ndarray, target_prompt: str):
        print(f"GSClient.detect_and_segment: {image.shape}, {target_prompt}" )
        response = send_request(self.url, image=image, target_prompt=target_prompt)
        return response

class Controller(Node):
    def __init__(self):
        super().__init__('go2_controller')

        # target object
        self.prompt = "plant"
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/unitree_go2/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, 
            '/unitree_go2/front_cam/color_image', 
            self.image_callback, 
            10
        )
        self.depth_image_sub = self.create_subscription(
            Image, 
            '/unitree_go2/front_cam/depth_image',
            self.depth_image_callback, 
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/unitree_go2/pose',
            self.pose_callback,
            10
        )

        self.current_pose = None
        self.initial_position = None
        self.target_distance = None
        self.is_moving_forward = False
        self.forward_velocity = 0.8  

        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # VLLM server configuration
        self.vllm_url = "http://localhost:8000/v1/chat/completions"
        
        # Grounding SAM for object detection and segmentation
        self.grounding_sam = GSClient()

        # Control variables
        self.current_image = None
        self.current_depth_image = None
        self.object_detected = False
        self.is_moving = False
        self.bbox = None
        
        # Movement parameters
        self.angular_velocity = 0.5  # rad/s for rotation
        self.detection_interval = 2.0  # seconds between detections
        
        # Create output directory for saved images
        self.output_dir = "detections"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Timer for periodic object detection
        self.detection_timer = self.create_timer(
            self.detection_interval, 
            self.check_for_object
        )
        
        # Timer for movement control
        self.movement_timer = self.create_timer(0.1, self.movement_control)
        
        self.get_logger().info("Go-2 Controller initialized")
        self.get_logger().info("Starting rotation to search for {}...".format(self.prompt))

    def pose_callback(self, msg):
        """Callback for receiving robot pose"""
        self.current_pose = msg
    
    def calculate_travel_distance(self, initial_pos, current_pos):
        """Calculate 2D distance traveled from initial position"""
        if initial_pos is None or current_pos is None:
            return 0.0
        
        dx = current_pos.x - initial_pos.x
        dy = current_pos.y - initial_pos.y
        return math.sqrt(dx*dx + dy*dy)
        
    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Convert ROS image to OpenCV format
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def depth_image_callback(self, msg):
        """Callback for receiving depth images"""
        try:
            # Convert ROS depth image to OpenCV format (typically 16-bit or 32-bit)
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
    
    def calculate_distance(self, bbox_info, depth_image):
        """Calculate average distance to object using depth image and bounding box"""
        if depth_image is None or bbox_info is None:
            return None
        
        try:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = bbox_info['x1'], bbox_info['y1'], bbox_info['x2'], bbox_info['y2']
            
            # Extract depth region using bounding box as mask
            depth_roi = depth_image[y1:y2, x1:x2].copy()
            
            # Filter out invalid depth values (0 or very large values)
            valid_mask = (depth_roi > 0) & (depth_roi < 10000)  # Adjust max distance as needed
            valid_depths = depth_roi[valid_mask]
            
            if len(valid_depths) == 0:
                self.get_logger().warn("No valid depth values in bounding box")
                return None

            # Use clustering or simple thresholding to separate object from background
            # Method 1: Use median as threshold (simpler approach)
            # median_depth = np.median(valid_depths)
            ret, thresh = cv2.threshold(valid_depths, np.min(valid_depths), np.max(valid_depths), cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self.get_logger().info(f"Thresh depth in bounding box: {ret:.2f} mm")
            # self.get_logger().info(f"Depth image value range: min={np.min(depth_image)}, max={np.max(depth_image)}")

            # Assume object is closer than background
            # Take depths that are closer than median (foreground)
            object_depths = valid_depths[valid_depths <= ret]

            # Method 2: More sophisticated - use k-means clustering (optional)
            # from sklearn.cluster import KMeans
            # kmeans = KMeans(n_clusters=2, random_state=0)
            # clusters = kmeans.fit_predict(valid_depths.reshape(-1, 1))
            # # Take the cluster with smaller centroid (closer objects)
            # closer_cluster = np.argmin(kmeans.cluster_centers_.flatten())
            # object_depths = valid_depths[clusters == closer_cluster]

            # Draw bounding box on depth image for visualization
            depth_image_clean = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
            depth_image_vis = cv2.normalize(depth_image_clean, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_vis = depth_image_vis.astype(np.uint8)
            depth_image_vis = cv2.cvtColor(depth_image_vis, cv2.COLOR_GRAY2BGR)
            depth_image_vis = self.draw_bounding_box(depth_image_vis, bbox_info)

            # Save the depth image with bounding box
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            depth_filename = f"object_depth_bbox_{timestamp}.jpg"
            depth_filepath = os.path.join(self.output_dir, depth_filename)
            cv2.imwrite(depth_filepath, depth_image_vis)
            self.get_logger().info(f"Depth image with bounding box saved: {depth_filepath}")

            if len(object_depths) > 0:
                # Calculate average distance of closest region
                average_distance = np.mean(object_depths)

                # Convert from depth units to meters (adjust based on your camera specs)
                # Typical conversion: if depth is in millimeters, divide by 1000
                distance_meters = average_distance / 1 # Adjust conversion factor
                
                return distance_meters
            else:
                self.get_logger().warn("Could not separate object from background")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error calculating distance: {e}")
            return None

    def encode_image_to_base64(self, image):
        """Convert OpenCV image to base64 string for VLLM API"""
        try:
            # Resize image to reduce payload size (optional)
            height, width = image.shape[:2]
            if width > 800:
                scale = 800 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Encode image as JPEG
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Convert to base64
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return f"data:image/jpeg;base64,{image_base64}"
            
        except Exception as e:
            self.get_logger().error(f"Error encoding image: {e}")
            return None
    
    def detect_obj_with_vllm(self, image):
        """Use VLLM server to detect object in image"""
        try:
            # Encode image
            image_base64 = self.encode_image_to_base64(image)
            if not image_base64:
                return False
            
            # Prepare the request payload
            payload = {
                "model": "llava-hf/llava-1.5-7b-hf",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": "Look at this image carefully. Is there a {} visible in this image? Answer with only 'YES' if you can see a {}, or 'NO' if you cannot see a {}.".format(self.prompt, self.prompt, self.prompt)
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": image_base64
                                }
                            }
                        ]
                    }
                ],
                "max_tokens": 10,
                "temperature": 0.1
            }
            
            # Send request to VLLM server
            response = requests.post(
                self.vllm_url,
                headers={"Content-Type": "application/json"},
                json=payload,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                answer = result['choices'][0]['message']['content'].strip().upper()
                
                self.get_logger().info(f"VLLM Response: {answer}")

                # Check if object is detected
                return "YES" in answer
            else:
                self.get_logger().error(f"VLLM server error: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Request to VLLM server failed: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error in object detection: {e}")
            return False

    def detect_obj_with_grounding_sam(self, image):
        """Use Grounding SAM to detect object and get bounding box"""
        annotated_image = image.copy()
        response = self.grounding_sam.detections(image=np.array(image), target_prompt=self.prompt)
        scores = response["response"]["scores"]
        bboxes = response["response"]["boxes"]
        if len(scores) > 0 and max(scores) > 0.7:
            max_index = scores.index(max(scores))
            bbox = bboxes[max_index]
            bbox_info = {
                'x1': int(bbox[0]),
                'y1': int(bbox[1]),
                'x2': int(bbox[2]),
                'y2': int(bbox[3]),
                'confidence': float(max(scores)),
                'class': self.prompt,
                'center_x': int((bbox[0] + bbox[2]) / 2),
                'center_y': int((bbox[1] + bbox[3]) / 2),
                'width': int(bbox[2] - bbox[0]),
                'height': int(bbox[3] - bbox[1])
            }
            x1, y1, x2, y2 = map(int, bbox)
                        
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_image, f"{scores[max_index]:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"{self.output_dir}/input_{ts}.png", annotated_image)
            return bbox_info
        return None

    def draw_bounding_box(self, image, bbox_info):
        """Draw bounding box on image"""
        if bbox_info is None:
            return image
        
        # Make a copy of the image
        annotated_image = image.copy()
        
        # Extract coordinates
        x1, y1, x2, y2 = bbox_info['x1'], bbox_info['y1'], bbox_info['x2'], bbox_info['y2']
        confidence = bbox_info['confidence']
        class_name = bbox_info['class']
        
        # Draw bounding box
        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw center point
        center_x, center_y = bbox_info['center_x'], bbox_info['center_y']
        cv2.circle(annotated_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Add text label
        label = f"{class_name}: {confidence:.2f}"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        
        # Draw background for text
        cv2.rectangle(annotated_image, 
                     (x1, y1 - label_size[1] - 10), 
                     (x1 + label_size[0], y1), 
                     (0, 255, 0), -1)
        
        # Draw text
        cv2.putText(annotated_image, label, 
                   (x1, y1 - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return annotated_image
    
    def save_detection_image(self, image, bbox_info):
        """Save the image with bounding box"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"detection_{timestamp}.jpg"
            filepath = os.path.join(self.output_dir, filename)
            
            # Draw bounding box on image
            annotated_image = self.draw_bounding_box(image, bbox_info)
            
            # Save the image
            cv2.imwrite(filepath, image)
            
            self.get_logger().info(f"Detection image saved: {filepath}")
            return filepath
            
        except Exception as e:
            self.get_logger().error(f"Error saving detection image: {e}")
            return None
    
    def print_bbox_coordinates(self, bbox_info):
        """Print detailed bounding box information"""
        if bbox_info is None:
            return
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("BOUNDING BOX COORDINATES:")
        self.get_logger().info(f"  Class: {bbox_info['class']}")
        self.get_logger().info(f"  Confidence: {bbox_info['confidence']:.3f}")
        self.get_logger().info(f"  Top-left corner (x1, y1): ({bbox_info['x1']}, {bbox_info['y1']})")
        self.get_logger().info(f"  Bottom-right corner (x2, y2): ({bbox_info['x2']}, {bbox_info['y2']})")
        self.get_logger().info(f"  Center point (x, y): ({bbox_info['center_x']}, {bbox_info['center_y']})")
        self.get_logger().info(f"  Width x Height: {bbox_info['width']} x {bbox_info['height']} pixels")
        self.get_logger().info("=" * 50)
    
    def check_for_object(self):
        """Periodic check for object in current image"""
        if self.current_image is not None and not self.object_detected:
            self.get_logger().info("Checking for {}...".format(self.prompt))
            
            # First use VLLM for initial detection
            if self.detect_obj_with_vllm(self.current_image):
                self.get_logger().info("{} DETECTED by VLLM! Stopping robot immediately...".format(self.prompt))
                
                # STOP ROBOT FIRST to prevent further rotation
                self.stop_robot()
                
                # Wait a moment for robot to fully stop
                time.sleep(0.5)
                
                # Capture a fresh image after stopping
                detection_image = self.current_image.copy() if self.current_image is not None else None
                
                if detection_image is not None:
                    self.get_logger().info("Running LLMDet for bounding box detection...")

                    # Use Grounding SAM to get bounding box on the stopped image
                    bbox_info = self.detect_obj_with_grounding_sam(detection_image)
                    
                    if bbox_info is not None:
                        self.bbox = bbox_info

                        # Calculate distance using depth image
                        if self.current_depth_image is not None:
                            distance = self.calculate_distance(bbox_info, self.current_depth_image)
                            if distance is not None:
                                self.get_logger().info(f"{self.prompt} DISTANCE: {distance:.2f} meters")
                                # Store distance in bbox_info for later use
                                bbox_info['distance_meters'] = distance
                            else:
                                self.get_logger().warn("Could not calculate object distance")
                        else:
                            self.get_logger().warn("No depth image available for distance calculation")
                        
                        self.print_bbox_coordinates(bbox_info)

                        # Save current position and start forward movement
                        if self.current_pose is not None and distance is not None:
                            # Save initial position for distance tracking
                            self.initial_position = self.current_pose.pose.position
                            self.target_distance = distance - 0.5  # Stop 0.5m before object (safety margin)
                            self.is_moving_forward = True
                            
                            self.get_logger().info(f"Starting forward movement to cover {self.target_distance:.2f} meters")
                            self.get_logger().info(f"Initial position: x={self.initial_position.x:.2f}, y={self.initial_position.y:.2f}")
                        else:
                            self.get_logger().warn("Cannot start forward movement - missing pose or distance data")
                        
                        # Save image with bounding box
                        saved_path = self.save_detection_image(detection_image, bbox_info)
                        if saved_path:
                            self.get_logger().info(f"📸 Detection image saved to: {saved_path}")
                        
                        self.object_detected = True
                        self.get_logger().info("{} detection and bounding box analysis complete!".format(self.prompt))
                    else:
                        self.get_logger().warn("VLLM detected {} but Grounding SAM couldn't find bounding box.".format(self.prompt))
                        self.get_logger().warn("This might be a false positive. Resuming search...")
                        # Reset detection flag to continue searching
                        self.object_detected = False
                else:
                    self.get_logger().error("No image available after stopping robot")
                    self.object_detected = False
            else:
                self.get_logger().info("No object detected, continuing search...")

    def movement_control(self):
        """Control robot movement"""
        cmd = Twist()
        if self.is_moving_forward and self.object_detected:
            # Moving forward towards object
            if self.current_pose is not None and self.initial_position is not None:
                # Calculate distance traveled
                travel_distance = self.calculate_travel_distance(
                    self.initial_position, 
                    self.current_pose.pose.position
                )
                
                self.get_logger().info(f"Traveled: {travel_distance:.2f}m / Target: {self.target_distance:.2f}m")
                
                # Check if target distance reached
                if travel_distance >= self.target_distance:
                    self.get_logger().info("Target distance reached! Stopping robot.")
                    self.is_moving_forward = False
                    self.object_detected = True  # Mark as complete
                    self.stop_robot()
                    return
                
                # Continue moving forward
                cmd.linear.x = self.forward_velocity
                
            else:
                self.get_logger().warn("No pose data available for forward movement")
                cmd.linear.x = self.forward_velocity
                
        elif not self.object_detected and not self.is_moving_forward:
            # Rotate to search for object
            cmd.angular.z = self.angular_velocity
            
            if not self.is_moving:
                self.get_logger().info("Rotating to search for {}...".format(self.prompt))
                self.is_moving = True
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot completely"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        # Send stop command multiple times to ensure it's received
        for _ in range(10):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        
        self.get_logger().info("Robot stopped successfully!")
        self.is_moving = False
        self.is_moving_forward = False  # Reset forward movement flag
        
        # Print final summary
        if self.initial_position is not None and self.current_pose is not None:
            final_distance = self.calculate_travel_distance(
                self.initial_position, 
                self.current_pose.pose.position
            )
            self.get_logger().info(f"Final position reached after traveling {final_distance:.2f} meters")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create controller node
        controller = Controller()
        
        # Spin the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.stop_robot()
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()