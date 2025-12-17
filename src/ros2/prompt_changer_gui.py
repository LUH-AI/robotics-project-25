#!/usr/bin/env python3
"""Simple GUI for changing SAM3 detection prompt on-the-fly."""

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PromptChanger(Node):
    def __init__(self, root):
        super().__init__('prompt_changer_gui')
        self.root = root
        self.publisher = self.create_publisher(String, '/go2/object_detection/prompt', 10)
        
        # Create GUI
        root.title("SAM3 Prompt Changer")
        root.geometry("400x120")
        root.configure(bg="#1e1e1e")
        
        # Label
        label = tk.Label(
            root,
            text="Detection Prompt:",
            font=("Arial", 11),
            bg="#1e1e1e",
            fg="white"
        )
        label.pack(pady=(15, 5))
        
        # Entry field
        self.entry = tk.Entry(
            root,
            font=("Arial", 12),
            bg="#2a2a2a",
            fg="white",
            insertbackground="white",
            width=30
        )
        self.entry.insert(0, "green cube.")
        self.entry.pack(pady=5)
        self.entry.bind('<Return>', lambda e: self.update_prompt())
        
        # Button
        button = tk.Button(
            root,
            text="Update Prompt",
            font=("Arial", 10, "bold"),
            bg="#0066CC",
            fg="white",
            activebackground="#0088EE",
            activeforeground="white",
            command=self.update_prompt,
            padx=20,
            pady=8
        )
        button.pack(pady=5)
        
    def update_prompt(self):
        prompt = self.entry.get().strip()
        if prompt:
            msg = String()
            msg.data = prompt
            self.publisher.publish(msg)
            self.get_logger().info(f"Updated prompt to: '{prompt}'")


def main():
    rclpy.init()
    
    root = tk.Tk()
    node = PromptChanger(root)
    
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.01)
        root.after(10, spin_once)
    
    spin_once()
    
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
