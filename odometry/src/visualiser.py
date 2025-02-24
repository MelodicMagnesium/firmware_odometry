#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray
from nav_msgs.msg import Odometry
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class WheelVisualizer(Node):
    def __init__(self):
        super().__init__('wheel_visualizer')
        # Subscriber for PWM values (as a string) on /chatter
        self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        # Subscriber for wheels_spin (Int64MultiArray) on /wheels_spin
        self.create_subscription(Int64MultiArray, 'wheels_spin', self.wheels_spin_callback, 10)
        # Data storage:
        # pwm_data: dictionary with keys: LF, LR, RF, RR
        self.pwm_data = {"LF": 0, "LR": 0, "RF": 0, "RR": 0}
        # wheels_spin_data: list with 4 elements in desired order: [LF, LR, RF, RR]
        self.wheels_spin_data = [0, 0, 0, 0]
        
    def chatter_callback(self, msg):
        # Expected format: "LF: 123, LR: 123, RF: 234, RR: 234"
        matches = re.findall(r'(\w+):\s*(-?\d+)', msg.data)
        for motor, value in matches:
            self.pwm_data[motor] = int(value)
        self.get_logger().info(f"PWM: {self.pwm_data}")
    
    def wheels_spin_callback(self, msg):
        # The publisher sends wheels_spin with order [RF, LF, LR, RR].
        # We want to display in order: LF, LR, RF, RR.
        if len(msg.data) >= 4:
            self.wheels_spin_data = [msg.data[1], msg.data[2], msg.data[0], msg.data[3]]
        self.get_logger().info(f"Wheels Spin: {self.wheels_spin_data}")

def update_plot(frame, node, bars_pwm, bars_spin):
    # Get PWM values in order: LF, LR, RF, RR
    pwm_list = [node.pwm_data.get(m, 0) for m in ['LF', 'LR', 'RF', 'RR']]
    for bar, val in zip(bars_pwm, pwm_list):
        bar.set_height(val)
    # Get wheels spin data (already ordered as [LF, LR, RF, RR])
    spin_list = node.wheels_spin_data
    for bar, val in zip(bars_spin, spin_list):
        bar.set_height(val)
    return bars_pwm + bars_spin

def main(args=None):
    rclpy.init(args=args)
    node = WheelVisualizer()
    
    # Set up a matplotlib figure with two subplots (side by side)
    fig, (ax_pwm, ax_spin) = plt.subplots(1, 2, figsize=(10, 5))
    motors = ['LF', 'LR', 'RF', 'RR']
    
    # PWM subplot
    initial_pwm = [node.pwm_data[m] for m in motors]
    bars_pwm = ax_pwm.bar(motors, initial_pwm, color='blue')
    ax_pwm.set_ylim(-300, 300)  # Adjust based on expected PWM range
    ax_pwm.set_title("Motor PWM Values")
    ax_pwm.set_ylabel("PWM")
    
    # Wheels Spin subplot
    initial_spin = node.wheels_spin_data  # [LF, LR, RF, RR]
    bars_spin = ax_spin.bar(motors, initial_spin, color='red')
    ax_spin.set_ylim(-100, 100)  # Adjust based on expected tick difference range
    ax_spin.set_title("Wheel Spin (Tick Delta)")
    ax_spin.set_ylabel("Ticks")
    
    # Spin ROS in a separate thread so that matplotlib can run on the main thread.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Animate the plot updating every 200ms
    ani = animation.FuncAnimation(fig, update_plot, fargs=(node, bars_pwm, bars_spin), interval=200)
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
