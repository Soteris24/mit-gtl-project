#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

# Fill in something for msg type imports
from duckietown_msgs.msg import WheelsCmdStamped

class SkeletonNode(Node):
    def __init__(self):
        super().__init__('blank_node')
        #Create publishers and subscribers in init, use callback
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.timer = self.create_timer(0.1, self.move_forward_callback)
        self.get_logger().info('Forward')
    

    def move_forward_callback(self):
        # Create the wheel command message
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        

        msg.vel_left = 0.3  
        msg.vel_right = 0.3
        
        # Publish
        self.wheel_pub.publish(msg)
        self.get_logger().info(f'Publishing: left={msg.vel_left}, right={msg.vel_right}')
    #Define callback functions here


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
