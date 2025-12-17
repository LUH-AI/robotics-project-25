#!/usr/bin/env python3
"""
Go2 Agent Status GUI - Shows current operational mode
Displays: Frontier Exploration → Object Detection → Object Pursuit
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import tkinter as tk
from threading import Thread
import signal
import sys


class Go2StatusGUI(Node):
    def __init__(self, root):
        super().__init__('go2_status_gui')
        self.root = root
        
        # Current state
        self.current_mode = "SEARCHING"
        self.object_detected = False
        self.detection_count = 0
        
        # Subscribe to detection topic
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/go2/object_detections',
            self.detection_callback,
            10
        )
        
        # Subscribe to mode changes (published by go2_object_goal_manager)
        self.mode_sub = self.create_subscription(
            String,
            '/go2/agent_mode',
            self.mode_callback,
            10
        )
        
        # Publisher for SAM3 prompt updates
        self.prompt_pub = self.create_publisher(
            String,
            '/go2/object_detection/prompt',
            10
        )
        
        # Publisher for exploration enable/disable
        from std_msgs.msg import Bool
        self.exploration_pub = self.create_publisher(
            Bool,
            '/go2/exploration/enable',
            10
        )
        
        # Track current prompt and exploration state
        self.current_prompt = "green cube."
        self.exploration_enabled = False  # Start paused
        
        self.get_logger().info("Go2 Status GUI initialized")
        
        # Update GUI every 100ms
        self.update_gui()
    
    def detection_callback(self, msg):
        """Handle incoming detections"""
        if msg.detections:
            self.object_detected = True
            self.detection_count = len(msg.detections)
        else:
            self.object_detected = False
            self.detection_count = 0
    
    def mode_callback(self, msg):
        """Handle mode changes from the agent"""
        self.current_mode = msg.data.upper()
        self.get_logger().info(f"Mode changed to: {self.current_mode}")
    
    def update_prompt(self, new_prompt):
        """Publish new prompt to SAM3 detector"""
        if new_prompt and new_prompt.strip():
            msg = String()
            msg.data = new_prompt.strip()
            self.prompt_pub.publish(msg)
            self.current_prompt = new_prompt.strip()
            self.get_logger().info(f"Updated detection prompt to: '{self.current_prompt}'")
    
    def toggle_exploration(self, enabled):
        """Enable or disable frontier exploration"""
        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = enabled
        self.exploration_pub.publish(msg)
        self.exploration_enabled = enabled
        state = "ENABLED" if enabled else "PAUSED"
        self.get_logger().info(f"Exploration {state}")
    
    def update_gui(self):
        """Update the GUI with current status"""
        # Determine display text and color
        if self.current_mode == "PURSUIT":
            status_text = "✓ SEARCH COMPLETE!"
            bg_color = "#00AA00"  # Green
            detail_text = f"Target found and reached!\nReady for new search"
            button_text = "Start New Search"
        elif self.current_mode == "PURSUING":
            status_text = "→ PURSUING OBJECT"
            bg_color = "#FFA500"  # Orange
            detail_text = f"Navigating to detected object..."
            button_text = "Update"
        elif self.object_detected:
            status_text = "✓ OBJECT DETECTED"
            bg_color = "#FFA500"  # Orange
            detail_text = f"Found {self.detection_count} object(s)\nSwitching to pursuit..."
            button_text = "Update"
        else:
            status_text = "🔍 SEARCHING MODE"
            bg_color = "#0066CC"  # Blue
            detail_text = "Frontier Exploration Active"
            button_text = "Update"
        
        # Update labels
        mode_label.config(text=status_text, bg=bg_color)
        detail_label.config(text=detail_text)
        update_button.config(text=button_text)
        
        # Schedule next update
        self.root.after(100, self.update_gui)


def ros_spin(node):
    """ROS2 spin in separate thread"""
    rclpy.spin(node)


def on_closing(root, node):
    """Handle window close"""
    node.get_logger().info("Shutting down GUI...")
    root.quit()
    node.destroy_node()


if __name__ == '__main__':
    # Initialize ROS2
    rclpy.init()
    
    # Create GUI window
    root = tk.Tk()
    root.title("Go2 Agent Status")
    root.geometry("500x380")
    root.configure(bg='#1a1a1a')
    
    # Make window always on top
    root.attributes('-topmost', True)
    
    # Create status label
    mode_label = tk.Label(
        root,
        text="🔍 INITIALIZING...",
        font=("Arial", 24, "bold"),
        fg="white",
        bg="#0066CC",
        pady=30
    )
    mode_label.pack(fill=tk.BOTH, expand=True)
    
    # Create detail label
    detail_label = tk.Label(
        root,
        text="Starting up...",
        font=("Arial", 14),
        fg="white",
        bg="#1a1a1a",
        pady=20
    )
    detail_label.pack(fill=tk.BOTH, expand=True)
    
    # Create prompt editor section
    prompt_frame = tk.Frame(root, bg='#1a1a1a', pady=10)
    prompt_frame.pack(fill=tk.X, padx=20)
    
    prompt_label = tk.Label(
        prompt_frame,
        text="SAM3 Search Prompt:",
        font=("Arial", 10),
        fg="white",
        bg="#1a1a1a"
    )
    prompt_label.pack(side=tk.LEFT, padx=(0, 10))
    
    prompt_entry = tk.Entry(
        prompt_frame,
        font=("Arial", 12),
        bg="#2a2a2a",
        fg="white",
        insertbackground="white",
        width=25
    )
    prompt_entry.insert(0, "green cube.")
    prompt_entry.pack(side=tk.LEFT, padx=(0, 10), ipady=5)
    
    def on_update_prompt():
        new_prompt = prompt_entry.get()
        gui_node.update_prompt(new_prompt)
    
    update_button = tk.Button(
        prompt_frame,
        text="Update",
        font=("Arial", 10, "bold"),
        bg="#0066CC",
        fg="white",
        activebackground="#0088EE",
        activeforeground="white",
        command=on_update_prompt,
        padx=15,
        pady=5
    )
    update_button.pack(side=tk.LEFT)

    # Create exploration control section
    exploration_frame = tk.Frame(root, bg='#1a1a1a', pady=10)
    exploration_frame.pack(fill=tk.X, padx=20)
    
    exploration_var = tk.BooleanVar(value=False)  # Start paused
    
    def on_toggle_exploration():
        enabled = exploration_var.get()
        gui_node.toggle_exploration(enabled)
    
    exploration_check = tk.Checkbutton(
        exploration_frame,
        text="Enable Frontier Exploration",
        font=("Arial", 12, "bold"),
        bg="#1a1a1a",
        fg="white",
        selectcolor="#2a2a2a",
        activebackground="#1a1a1a",
        activeforeground="white",
        variable=exploration_var,
        command=on_toggle_exploration
    )
    exploration_check.pack(side=tk.LEFT, padx=10)
    
    # Create ROS2 node
    gui_node = Go2StatusGUI(root)
    
    # Handle window close
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root, gui_node))
    
    # Handle Ctrl+C
    def signal_handler(sig, frame):
        gui_node.get_logger().info("Interrupted, shutting down...")
        on_closing(root, gui_node)
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start ROS2 spin in separate thread
    ros_thread = Thread(target=ros_spin, args=(gui_node,), daemon=True)
    ros_thread.start()
    
    # Run GUI main loop
    gui_node.get_logger().info("Status GUI running")
    root.mainloop()
    
    # Cleanup
    rclpy.shutdown()
