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
    AVOID_STEP_1 = 2
    AVOID_STEP_2 = 3

class EncoderAvoidanceNode(Node):
    def __init__(self):
        super().__init__('encoder_avoidance_node')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)  
        self.tick_sub = self.create_subscription(WheelEncoderStamped, f'/{self.vehicle_name}/tick', self.tick_callback, 10)

        #state
        self.current_state = State.NORMAL
        
        #encorers
        self.left_ticks = 0
        self.right_ticks = 0
        
        self.target_left = 0
        self.target_right = 0
        
        self.initialized = False 

        #config 
        self.ticks_to_turn = 400     
        self.ticks_step_forward = 100 #
        self.kp = 0.02               


        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # turn timer counter
        self.state_timer_counter = 0

    def control_loop(self):

        
        # STATE MACHINE
        if self.current_state == State.NORMAL:
            # drive forward
            self.set_leds(0.0, 1.0, 0.0) # Green
            self.target_left += self.ticks_step_forward
            self.target_right += self.ticks_step_forward

        elif self.current_state == State.AVOID_STEP_1:
            # wai
            self.set_leds(1.0, 0.0, 0.0) # Red
            self.state_timer_counter += 1
            
            # If 20 cycles have passed (20 * 0.1s = 2.0 seconds)
            if self.state_timer_counter >= 20:
                self.transition_to_step_2()

        elif self.current_state == State.AVOID_STEP_2:
            self.set_leds(1.0, 1.0, 0.0) # Yellow
            self.state_timer_counter += 1
            
            # Wait another 2 seconds, then go back to normal
            if self.state_timer_counter >= 20:
                self.current_state = State.NORMAL
                self.get_logger().info("Back to Normal")

        # --- 2. PID Control (Position Control) ---
        # Drive wheels to match the targets set above
        self.drive_to_targets()

    def drive_to_targets(self):
        if not self.initialized:
            return

        # Calculate Error
        err_l = self.target_left - self.left_ticks
        err_r = self.target_right - self.right_ticks

        # Calculate Command (P-Controller)
        cmd_l = self.kp * err_l
        cmd_r = self.kp * err_r

        # Clamp speeds between -1.0 and 1.0
        cmd_l = max(min(cmd_l, 1.0), -1.0)
        cmd_r = max(min(cmd_r, 1.0), -1.0)

        # Publish
        self.run_wheels(cmd_l, cmd_r)
        
        # DEBUG
        # print(f"T_L:{self.target_left} | T_R:{self.target_right} | Err_L:{err_l} | Err_R:{err_r}")

    def check_range(self, msg):
        distance = msg.range
        
        # Only trigger avoidance if we are currently in NORMAL mode
        if self.current_state == State.NORMAL:
            if distance < 0.20 and distance > 0.01:
                self.start_avoidance()

    def start_avoidance(self):
        self.get_logger().info("OBSTACLE! Starting Avoidance Step 1")
        self.current_state = State.AVOID_STEP_1
        self.state_timer_counter = 0
        
        # LOGIC: Add ticks to RIGHT wheel only. 
        # Right wheel moves forward, Left wheel waits.
        # This causes a LEFT TURN.
        self.target_right += self.ticks_to_turn

    def transition_to_step_2(self):
        self.get_logger().info("Step 1 Done. Starting Step 2")
        self.current_state = State.AVOID_STEP_2
        self.state_timer_counter = 0
        
        # LOGIC: Add ticks to LEFT wheel only.
        # Left wheel moves forward to catch up.
        # This causes a RIGHT TURN (realigning).
        self.target_left += self.ticks_to_turn

    def tick_callback(self, msg):
        # Update current tick counts
        if 'left' in msg.header.frame_id:
            self.left_ticks = msg.data
        elif 'right' in msg.header.frame_id:
            self.right_ticks = msg.data

        # Initialize targets on first run
        if not self.initialized and self.left_ticks > 0 and self.right_ticks > 0:
            self.target_left = self.left_ticks
            self.target_right = self.right_ticks
            self.initialized = True
            self.get_logger().info(f'Initialized Targets: {self.target_left}')

    def run_wheels(self, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        wheel_msg.header.stamp = self.get_clock().now().to_msg()
        wheel_msg.vel_left = float(vel_left)
        wheel_msg.vel_right = float(vel_right)
        self.wheel_pub.publish(wheel_msg)

    def set_leds(self, r, g, b):
        msg = LEDPattern()
        pattern = ColorRGBA(r=r, g=g, b=b, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def stop(self):
        self.run_wheels(0.0, 0.0)

def main():
    rclpy.init()
    node = EncoderAvoidanceNode()
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