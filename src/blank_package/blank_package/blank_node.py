#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

# Fill in something for msg type imports
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped

class SkeletonNode(Node):
    def __init__(self):
        super().__init__('blank_node')
        #Create publishers and subscribers in init, use callback
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        # self.timer = self.create_timer(0.1, self.move_forward_callback)
        # self.get_logger().info('Forward')


    def check_range(self, msg):
        distance = msg.range
        if distance >= 0.1:
            self.move_forward()
        else:
            self.stop()

    
    def move_forward(self):
        self.run_wheels('forward_callback', 0.5, 0.5)

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheel_pub.publish(wheel_msg)
    

    # def move_forward_callback(self):
    #     # Create the wheel command message
    #     msg = WheelsCmdStamped()
    #     msg.header.stamp = self.get_clock().now().to_msg()
        

    #     msg.vel_left = 0.3  
    #     msg.vel_right = 0.3
        
    #     # Publish
    #     self.wheel_pub.publish(msg)
    #     self.get_logger().info(f'Publishing: left={msg.vel_left}, right={msg.vel_right}')
    


def main():
    print('In main')
    rclpy.init()
    node = SkeletonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
