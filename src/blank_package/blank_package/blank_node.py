#!/usr/bin/python3
import os
import rclpy
import math 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Message imports
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern, WheelEncoderStamped

class SkeletonNode(Node):
    def __init__(self):
        super().__init__('skeleton_node')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        # DEBUG: Print startup info
        print(f'========================================')
        print(f'VEHICLE_NAME: {self.vehicle_name}')
        print(f'Mode: Obstacle Width Calculation')
        print(f'========================================')

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers and Publishers
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, sensor_qos)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)  
        self.tick_sub = self.create_subscription(WheelEncoderStamped, f'/{self.vehicle_name}/tick', self.tick_callback, 10)

        # State machine
        # States: 'normal', 'scanning', 'avoiding_turn_right', 'avoiding_forward', 'avoiding_turn_left'
        self.state = 'normal'
        self.avoidance_timer = None
        
        # Scanning variables
        self.scan_start_distance = 0.0 # Distance when first detected
        self.scan_prev_distance = 0.0  # Distance from the previous loop
        self.obstacle_width = 0.0      # Calculated result
        
        # Encoder ticks
        self.left_ticks = 0
        self.right_ticks = 0
        self.target_ticks = 0
        self.initialized = False 

        # Timers
        self.target_timer = self.create_timer(1.0, self.increment_target)
        self.control_timer = self.create_timer(0.1, self.control_loop) 

    def control_loop(self):
        # Only track encoders to move straight in normal mode
        if self.state == 'normal':
            self.adjust_with_target()

    def increment_target(self):
        if self.state == 'normal':
            self.target_ticks += 100

    def adjust_with_target(self):
        left_error = self.target_ticks - self.left_ticks
        right_error = self.target_ticks - self.right_ticks
        kp = 0.005
        left_speed = 0.5 + (kp * left_error)
        right_speed = 0.5 + (kp * right_error)
        left_speed = max(0.0, min(1.0, left_speed))
        right_speed = max(0.0, min(1.0, right_speed))
        self.run_wheels('target_tracking', left_speed, right_speed)

    def tick_callback(self, msg):
        if 'left' in msg.header.frame_id:
            self.left_ticks = msg.data
        elif 'right' in msg.header.frame_id:
            self.right_ticks = msg.data

        if not self.initialized and self.left_ticks > 0 and self.right_ticks > 0:
            self.target_ticks = max(self.left_ticks, self.right_ticks)
            self.initialized = True

    def check_range(self, msg):
        current_distance = msg.range
        
        # 1. Normal State: Watch for obstacle
        if self.state == 'normal':
            # If we see something close (between 2cm and 15cm)
            if current_distance < 0.15 and current_distance > 0.02:
                print(f"Obstacle detected at {current_distance:.3f}m. Starting Scan.")
                self.start_scanning(current_distance)

        # 2. Scanning State: Rotate and Measure
        elif self.state == 'scanning':
            # Check if there is a drastic change (Edge Detection)
            # If current distance jumped by more than 0.3m compared to previous, we fell off the edge
            if current_distance > (self.scan_prev_distance + 0.3):
                self.calculate_width_and_avoid()
            else:
                # If no jump, update previous value and keep rotating
                self.scan_prev_distance = current_distance
                # Keep rotating right to scan
                self.turn_right_slow()

    def start_scanning(self, distance):
        self.state = 'scanning'
        self.scan_start_distance = distance
        self.scan_prev_distance = distance
        self.set_leds_yellow()
        self.stop() 

    def calculate_width_and_avoid(self):
        self.stop()
        
        # --- PYTHAGORAS CALCULATION ---
        # a^2 + b^2 = c^2  ->  width^2 + start^2 = edge^2
        # width = sqrt(edge^2 - start^2)
        
        edge_dist = self.scan_prev_distance
        start_dist = self.scan_start_distance
        
        try:
            # Calculate the square difference
            sq_diff = (edge_dist**2) - (start_dist**2)
            
            if sq_diff > 0:
                self.obstacle_width = math.sqrt(sq_diff)
            else:
                self.obstacle_width = 0.1 # Fallback if measurement was noisy
                
        except Exception as e:
            print(f"Math Error: {e}")
            self.obstacle_width = 0.2

        print(f"Scan Complete.")
        print(f"Start Dist: {start_dist:.3f}m, Edge Dist: {edge_dist:.3f}m")
        print(f"CALCULATED WIDTH: {self.obstacle_width:.3f}m")
        
        self.start_avoidance_maneuver()

    def start_avoidance_maneuver(self):
        self.get_logger().info('Starting avoidance maneuver...')
        self.set_leds_red()
        
        # We are already rotated to the right from scanning, so we can skip the first turn
        # or just align slightly more. Let's move directly to "forward" to pass the object.
        
        self.state = 'avoiding_forward'
        self.move_forward()
        
        # DYNAMIC TIMING: Calculate how long to drive forward based on width
        # Time = Distance / Speed. Let's assume speed is ~0.2m/s
        # We add 0.2m extra margin
        drive_time = (self.obstacle_width + 0.3) / 0.2
        
        # Cap the time between 2.0s and 5.0s for safety
        drive_time = max(2.0, min(5.0, drive_time))
        
        print(f"Driving forward for {drive_time:.2f} seconds based on width.")
        self.avoidance_timer = self.create_timer(drive_time, self.avoidance_step_turn_left)

    def avoidance_step_turn_left(self):
        """Turn back to path"""
        self.avoidance_timer.cancel()
        self.get_logger().info('Turning left...')
        self.state = 'avoiding_turn_left'
        self.turn_left()
        self.avoidance_timer = self.create_timer(0.8, self.avoidance_complete)

    def avoidance_complete(self):
        self.avoidance_timer.cancel()
        self.get_logger().info('Resuming normal operation.')
        self.set_leds_green()
        self.target_ticks = max(self.left_ticks, self.right_ticks)
        self.state = 'normal'

    # --- Motor Helpers ---
    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        wheel_msg.header.stamp = self.get_clock().now().to_msg()
        wheel_msg.header.frame_id = frame_id
        wheel_msg.vel_left = float(vel_left)
        wheel_msg.vel_right = float(vel_right)
        self.wheel_pub.publish(wheel_msg)

    def move_forward(self):
        self.run_wheels('forward', 0.5, 0.5)

    def turn_left(self):
        self.run_wheels('left', -0.5, 0.5)

    def turn_right(self):
        self.run_wheels('right', 0.5, -0.5)

    def turn_right_slow(self):
        # Slower turn for better scanning resolution
        self.run_wheels('scan_right', 0.25, -0.25)

    def stop(self):
        self.run_wheels('stop', 0.0, 0.0)

    # --- LED Helpers ---
    def set_leds_red(self):
        self.publish_color(1.0, 0.0, 0.0)

    def set_leds_green(self):
        self.publish_color(0.0, 1.0, 0.0)

    def set_leds_yellow(self):
        self.publish_color(1.0, 1.0, 0.0)

    def publish_color(self, r, g, b):
        msg = LEDPattern()
        pattern = ColorRGBA(r=r, g=g, b=b, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

def main():
    rclpy.init()
    node = SkeletonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()