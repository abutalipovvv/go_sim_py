#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from quadropted_msgs.msg import RobotModeCommand
import logging

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(RobotModeCommand, '/robot1/robot_mode', 10)
        self.get_logger().info("RobotController node has been initialized")

    def publish_mode(self, mode, robot_id):
        msg = RobotModeCommand()
        msg.mode = mode
        msg.robot_id = robot_id
        self.publisher_.publish(msg)
        log_message = f'Published mode: {mode} for robot_id: {robot_id}'
        self.get_logger().info(log_message)
        print(log_message)

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        while True:
            print("Введите команду (1: TROT, 2: REST, 3: CRAWL, 4: STAND, q: выйти):")
            command = input().strip()
            if command == '1':
                robot_controller.publish_mode('TROT', 1)
            elif command == '2':
                robot_controller.publish_mode('REST', 1)
            elif command == '3':
                robot_controller.publish_mode('CRAWL', 1)
            elif command == '4':
                robot_controller.publish_mode('STAND', 1)
            elif command.lower() == 'q':
                print("Выход")
                break
            else:
                print("Неверная команда. Попробуйте снова.")
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()