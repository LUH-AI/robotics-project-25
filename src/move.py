#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time

class Go2SequenceController(Node):
    def __init__(self):
        super().__init__('go2_sequence_controller')
        
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,
            '/unitree_go2/cmd_vel',
            10
        )
        
        # Movement parameters (adjust these based on your robot's capabilities)
        self.forward_speed = 1.5      # m/s - forward movement speed
        self.turn_speed = 0.7        # rad/s - turning speed
        self.strafe_speed = 0.3       # m/s - strafing speed
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        self.get_logger().info('Go2 Sequence Controller initialized')
        self.get_logger().info('Starting movement sequence...')
        
    def send_velocity_command(self, linear_vel, angular_vel):
        """Send velocity command to the robot"""
        twist_msg = Twist()
        
        # Set linear velocity
        twist_msg.linear.x = float(linear_vel[0])
        twist_msg.linear.y = float(linear_vel[1])
        twist_msg.linear.z = float(linear_vel[2])
        
        # Set angular velocity
        twist_msg.angular.x = float(angular_vel[0])
        twist_msg.angular.y = float(angular_vel[1])
        twist_msg.angular.z = float(angular_vel[2])
        
        # Publish the command
        self.publisher.publish(twist_msg)
        
    def execute_movement(self, linear_vel, angular_vel, duration, description):
        """Execute a movement for a specific duration"""
        self.get_logger().info(f'Executing: {description} for {duration}s')
        
        start_time = time.time()
        
        # Send commands at 10Hz while the movement duration hasn't elapsed
        while (time.time() - start_time) < duration:
            self.send_velocity_command(linear_vel, angular_vel)
            time.sleep(0.1)  # 10Hz update rate
            
        # Stop movement after duration
        self.send_velocity_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.get_logger().info(f'Completed: {description}')
        time.sleep(0.5)  # Small pause between movements
        
    def execute_sequence(self):
        """Execute the complete movement sequence"""
        try:
            # 1. Move forward 10 seconds
            self.execute_movement(
                linear_vel=[self.forward_speed, 0.0, 0.0],
                angular_vel=[0.0, 0.0, 0.0],
                duration=10.0,
                description="Move forward"
            )
            
            # 2. Turn left 5 seconds
            self.execute_movement(
                linear_vel=[0.0, 0.0, 0.0],
                angular_vel=[0.0, 0.0, self.turn_speed],  # Positive Z = left turn
                duration=6.0,
                description="Turn left"
            )
            
            # 3. Move forward 20 seconds
            self.execute_movement(
                linear_vel=[self.forward_speed, 0.0, 0.0],
                angular_vel=[0.0, 0.0, 0.0],
                duration=10.0,
                description="Move forward (long)"
            )
            
            # 4. Turn left 5 seconds
            self.execute_movement(
                linear_vel=[0.0, 0.0, 0.0],
                angular_vel=[0.0, 0.0, self.turn_speed],  # Positive Z = left turn
                duration=6.0,
                description="Turn left"
            )
            
            # 5. Move forward 10 seconds
            self.execute_movement(
                linear_vel=[self.forward_speed, 0.0, 0.0],
                angular_vel=[0.0, 0.0, 0.0],
                duration=10.0,
                description="Move forward"
            )
            
            # 6. Strafe left 5 seconds
            # self.execute_movement(
            #     linear_vel=[0.0, self.strafe_speed, 0.0],  # Positive Y = left strafe
            #     angular_vel=[0.0, 0.0, 0.0],
            #     duration=5.0,
            #     description="Strafe left"
            # )
            
            # # 7. Move forward 10 seconds
            # self.execute_movement(
            #     linear_vel=[self.forward_speed, 0.0, 0.0],
            #     angular_vel=[0.0, 0.0, 0.0],
            #     duration=10.0,
            #     description="Move forward (final)"
            # )
            
            # 8. Stop (already stopped, but make it explicit)
            self.get_logger().info('Movement sequence completed - Robot stopped')
            
        except KeyboardInterrupt:
            self.get_logger().info('Sequence interrupted by user')
            self.send_velocity_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        
        except Exception as e:
            self.get_logger().error(f'Error during sequence: {str(e)}')
            self.send_velocity_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the controller
    controller = Go2SequenceController()
    
    try:
        # Execute the movement sequence
        controller.execute_sequence()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        # Ensure robot is stopped
        controller.send_velocity_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        controller.get_logger().info('Final stop command sent')
        
        # Clean up
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()