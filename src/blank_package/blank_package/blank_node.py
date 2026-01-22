#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

# Fill in something for msg type imports
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range, Imu
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern, WheelEncoderStamped


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


class SkeletonNode(Node):
    def __init__(self):
        super().__init__('blank_node')
        #Create publishers and subscribers in init, use callback
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        # DEBUG: Print startup info
        print(f'========================================')
        print(f'VEHICLE_NAME: {self.vehicle_name}')
        print(f'Subscribing to: /{self.vehicle_name}/range')
        print(f'Subscribing to: /{self.vehicle_name}/imu_data')
        print(f'Publishing to: /{self.vehicle_name}/wheels_cmd')
        print(f'========================================')

        # QoS for sensor data (RELIABLE to match tof_driver_node publisher)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, sensor_qos)
        self.imu_sub = self.create_subscription(Imu, f'/{self.vehicle_name}/imu_data', self.imu_callback, sensor_qos)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)  
        self.tick_sub = self.create_subscription(WheelEncoderStamped,f'/{self.vehicle_name}/tick',self.tick_callback,10)
        
        # IMU data
        self.current_yaw = 0.0
        self.target_yaw = None
        self.turn_complete = False
        
        # State machine for obstacle avoidance
        # States: 'normal', 'avoiding_turn_right', 'avoiding_forward', 'avoiding_turn_left'
        self.state = 'normal'
        self.avoidance_timer = None
        
        #Encoder ticks
        self.left_ticks = 0
        self.right_ticks = 0
        self.target_ticks = 0
        self.initialized = False  # Flag to initialize target from first encoder reading

        self.target_timer = self.create_timer(1.0, self.increment_target)
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # Timer for checking turn completion
        self.turn_check_timer = None

    def imu_callback(self, msg):
        """Extract yaw angle from IMU quaternion"""
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        self.current_yaw = yaw
        # DEBUG: Uncomment to see yaw updates
        # print(f'Current yaw: {math.degrees(self.current_yaw):.1f}°')

    def control_loop(self):
        if self.state == 'normal':
            self.adjust_with_target()

    def increment_target(self):
        self.target_ticks += 100

    def adjust_with_target(self):
        # error calculation
        left_error = self.target_ticks - self.left_ticks
        right_error = self.target_ticks - self.right_ticks

        kp = 0.005

        # Speed calculation
        left_speed = 0.5 + (kp * left_error)
        right_speed = 0.5 + (kp * right_error)

        # Limits
        left_speed = max(0.0, min(1.0, left_speed))
        right_speed = max(0.0, min(1.0, right_speed))

        self.run_wheels('target_tracking', left_speed, right_speed)

    def tick_callback(self, msg):
        # Check the frame_id
        if 'left' in msg.header.frame_id:
            self.left_ticks = msg.data
        elif 'right' in msg.header.frame_id:
            self.right_ticks = msg.data

        # Initialize target to current encoder position on first reading
        if not self.initialized and self.left_ticks > 0 and self.right_ticks > 0:
            self.target_ticks = max(self.left_ticks, self.right_ticks)
            self.initialized = True

    def check_range(self, msg):
        distance = msg.range
        # DEBUG: Print every ToF reading (comment out after testing)
        print(f'ToF reading: {distance:.3f}m | State: {self.state}')

        if self.state == 'normal':
            if distance < 0.1 and distance > 0.02:
                self.start_avoidance()

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_to_angle(self, target_angle_degrees, turn_direction='right'):
        """
        Turn the robot to a specific angle relative to current orientation
        target_angle_degrees: degrees to turn (e.g., 90)
        turn_direction: 'right' or 'left'
        """
        # Convert to radians
        angle_to_turn = math.radians(target_angle_degrees)
        
        # Calculate target yaw
        if turn_direction == 'right':
            self.target_yaw = self.normalize_angle(self.current_yaw - angle_to_turn)
        else:  # left
            self.target_yaw = self.normalize_angle(self.current_yaw + angle_to_turn)
        
        self.turn_complete = False
        
        print(f'Starting turn: Current={math.degrees(self.current_yaw):.1f}°, Target={math.degrees(self.target_yaw):.1f}°')
        
        # Start turning
        if turn_direction == 'right':
            self.turn_right()
        else:
            self.turn_left()
        
        # Create timer to check if turn is complete
        self.turn_check_timer = self.create_timer(0.05, lambda: self.check_turn_complete(turn_direction))

    def check_turn_complete(self, turn_direction):
        """Check if the robot has reached the target yaw angle"""
        if self.target_yaw is None:
            return
        
        # Calculate angular difference
        angle_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
        angle_diff_degrees = abs(math.degrees(angle_diff))
        
        # Tolerance of 5 degrees
        ANGLE_TOLERANCE = 5.0
        
        if angle_diff_degrees < ANGLE_TOLERANCE:
            print(f'Turn complete! Current yaw: {math.degrees(self.current_yaw):.1f}°')
            self.stop()
            self.turn_complete = True
            self.target_yaw = None
            if self.turn_check_timer:
                self.turn_check_timer.cancel()
                self.turn_check_timer = None
    
    def move_forward(self):
        self.run_wheels('forward_callback', 0.5, 0.5)

    def turn_left(self):
        self.run_wheels('left_callback', -0.5, 0.5)

    def turn_right(self):
        self.run_wheels('right_callback', 0.5, -0.5)

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

    def start_avoidance(self):
        self.get_logger().info('Obstacle detected! Starting avoidance maneuver...')
        self.set_leds_red()    # Turn the LEDS RED
        self.state = 'avoiding_turn_right'
        self.stop()
        
        # Step 1: Turn right 90 degrees using IMU
        self.turn_to_angle(90, 'right')
        
        # Wait for turn to complete, then move forward
        self.avoidance_timer = self.create_timer(0.1, self.wait_for_turn_right)

    def wait_for_turn_right(self):
        """Wait for right turn to complete, then move forward"""
        if self.turn_complete:
            self.avoidance_timer.cancel()
            self.avoidance_step_forward()

    def avoidance_step_forward(self):
        """Step 2: Go forward for 2 seconds"""
        self.get_logger().info('Going forward...')
        self.set_leds_yellow() # Turn the LEDs yellow when it is in the process of avoiding
        self.state = 'avoiding_forward'
        self.move_forward()
        self.avoidance_timer = self.create_timer(2.0, self.avoidance_step_turn_left)

    def avoidance_step_turn_left(self):
        """Step 3: Turn left 90 degrees using IMU"""
        self.avoidance_timer.cancel()
        self.get_logger().info('Turning left...')
        self.state = 'avoiding_turn_left'
        
        # Turn left 90 degrees
        self.turn_to_angle(90, 'left')
        
        # Wait for turn to complete, then finish
        self.avoidance_timer = self.create_timer(0.1, self.wait_for_turn_left)

    def wait_for_turn_left(self):
        """Wait for left turn to complete, then finish avoidance"""
        if self.turn_complete:
            self.avoidance_timer.cancel()
            self.avoidance_complete()

    def avoidance_complete(self):
        self.get_logger().info('Avoidance complete, resuming normal operation.')
        self.set_leds_green()
        # Reset target to current encoder position so we start fresh
        self.target_ticks = max(self.left_ticks, self.right_ticks)
        self.state = 'normal'

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheel_pub.publish(wheel_msg)

    def set_leds_red(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def set_leds_green(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def set_leds_yellow(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)


def main():
    print('In main')
    rclpy.init()
    node = SkeletonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()