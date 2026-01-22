#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from enum import Enum

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern, WheelEncoderStamped

class State(Enum):
    NORMAL = 1
    AVOID_TURN_OUT = 2
    AVOID_DRIVE_PAST = 3
    AVOID_TURN_IN = 4

class EncoderAvoidanceNode(Node):
    def __init__(self):
        super().__init__('encoder_avoidance_node')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        # Subscribers
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.tick_sub = self.create_subscription(WheelEncoderStamped, f'/{self.vehicle_name}/tick', self.tick_callback, 10)
        
        # Publishers
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)

        # Variables
        self.state = State.NORMAL
        self.left_ticks = 0
        self.right_ticks = 0
        self.target_left = 0
        self.target_right = 0
        self.initialized = False

        # Config
        self.TURN_TICKS = 400 #Ticks to turn
        self.DRIVE_TICKS = 600 #Ticks forwards to drive past obstacle
        self.NORMAL_SPEED = 20 #Ticks to add per loop in normal mode
        self.TOLERANCE = 30
        # Loop
        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if not self.initialized: return

        #  STATE MACHINE
        if self.state == State.NORMAL:
            self.set_leds(0, 1, 0) # Green

            self.target_left += self.NORMAL_SPEED
            self.target_right += self.NORMAL_SPEED

        elif self.state == State.AVOID_TURN_OUT:
            self.set_leds(1, 0, 0) # Red
            
            if self.has_arrived():
                self.get_logger().info("TMoving Forward")
                self.target_left += self.DRIVE_TICKS
                self.target_right += self.DRIVE_TICKS
                self.state = State.AVOID_DRIVE_PAST

        elif self.state == State.AVOID_DRIVE_PAST:
            self.set_leds(1, 1, 0) # Yellow
            if self.has_arrived():
                
                self.get_logger().info("Turning Back")
                
                self.target_left += self.TURN_TICKS 
                self.state = State.AVOID_TURN_IN

        elif self.state == State.AVOID_TURN_IN:
            self.set_leds(1, 1, 0) # Yellow
            if self.has_arrived():
                
                self.get_logger().info("Normal Op")
                self.state = State.NORMAL

        
        self.drive_wheels()

    def check_range(self, msg):
        # Only trigger if we are in normal mode
        if self.state == State.NORMAL and msg.range < 0.2 and msg.range > 0.01:
            self.get_logger().info("Turning Left...")
            self.state = State.AVOID_TURN_OUT
            
            self.target_right += self.TURN_TICKS

    def has_arrived(self):
        #small tolerence because encoders are rarely exact
        diff_l = abs(self.target_left - self.left_ticks)
        diff_r = abs(self.target_right - self.right_ticks)
        return diff_l < self.TOLERANCE and diff_r < self.TOLERANCE

    def drive_wheels(self):
        # kP contiroller
        kp = 0.02
        
        err_l = self.target_left - self.left_ticks
        err_r = self.target_right - self.right_ticks

        cmd_l = max(min(err_l * kp, 0.8), -0.8)
        cmd_r = max(min(err_r * kp, 0.8), -0.8)

        self.publish_cmd(cmd_l, cmd_r)

    def tick_callback(self, msg):
        if 'left' in msg.header.frame_id: self.left_ticks = msg.data
        if 'right' in msg.header.frame_id: self.right_ticks = msg.data
        
        if not self.initialized and self.left_ticks > 0 and self.right_ticks > 0:
            self.target_left = self.left_ticks
            self.target_right = self.right_ticks
            self.initialized = True

    def publish_cmd(self, l, r):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = float(l)
        msg.vel_right = float(r)
        self.wheel_pub.publish(msg)

    def set_leds(self, r, g, b):
        msg = LEDPattern()
        color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        msg.rgb_vals = [color] * 5
        self.led_pub.publish(msg)

def main():
    rclpy.init()
    node = EncoderAvoidanceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()