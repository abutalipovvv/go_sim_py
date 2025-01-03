#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotVelocity

class RobotVelocityHandler(Node):
    def __init__(self):
        super().__init__('robot_velocity_handler')

        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel',
            self.robot_velocity_callback, 
            10)

        self.publisher_ = self.create_publisher(RobotVelocity, 'robot_velocity', 10)

        self.get_logger().info("Node started: RobotVelocityHandler")

    def robot_velocity_callback(self, msg: Twist):
        self.get_logger().info(
            f"Received Twist: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
            f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})"
        )

        new_msg = RobotVelocity()

        new_msg.robot_id = 1

        new_msg.cmd_vel.linear.x = self.multiply_and_limit(msg.linear.x, 0.035, -1.0, 1.0)
        new_msg.cmd_vel.linear.y = self.multiply_and_limit(msg.linear.y, 0.012, -1.0, 1.0)
        new_msg.cmd_vel.linear.z = msg.linear.z

        new_msg.cmd_vel.angular.x = msg.angular.x
        new_msg.cmd_vel.angular.y = msg.angular.y
        new_msg.cmd_vel.angular.z = self.limit(msg.angular.z, -1.0, 1.0)

        self.publisher_.publish(new_msg)

        self.get_logger().info(
            f"Published RobotVelocity: robot_id={new_msg.robot_id}, "
            f"linear=({new_msg.cmd_vel.linear.x}, {new_msg.cmd_vel.linear.y}, {new_msg.cmd_vel.linear.z}), "
            f"angular=({new_msg.cmd_vel.angular.x}, {new_msg.cmd_vel.angular.y}, {new_msg.cmd_vel.angular.z})"
        )
        
    def multiply_and_limit(self, value, scale_factor, min_limit, max_limit):
        scaled_value = value * scale_factor
        return self.limit_value(scaled_value, min_limit, max_limit)

    def limit_value(self, value, min_limit, max_limit):
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

    def limit(self, value, min_limit, max_limit):
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

def main(args=None):
    rclpy.init(args=args)
    node = RobotVelocityHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
