#!/usr/bin/env python3
# Author: lnotspotl
# Modified for ros2 by: abutalipovvv
import numpy as np
import tf_transformations  # Используем tf2 вместо tf
from .StateCommand import State, Command, BehaviorState
from .RestController import RestController
from .TrotGaitController import TrotGaitController
from .CrawlGaitController import CrawlGaitController
from .StandController import StandController
from geometry_msgs.msg import Twist
from robot_msgs.msg import RobotModeCommand, RobotVelocity
class Robot:
    def __init__(self, node, body, legs, imu, robot_id):
        self.body = body
        self.legs = legs
        self.node = node  # Сохраняем ссылку на ноду
        self.robot_id = robot_id  # Уникальный идентификатор робота

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.02
        self.x_shift_back = -0.0
        self.default_height = 0.25

        self.trotGaitController = TrotGaitController(
            self.node, self.default_stance,
            stance_time=0.04, swing_time=0.18, time_step=0.02,
            use_imu=imu
        )

        self.crawlGaitController = CrawlGaitController(
            self.default_stance,
            stance_time=0.55, swing_time=0.45, time_step=0.02
        )

        self.standController = StandController(self.node, self.default_stance)  # Передаем node
        self.restController = RestController(self.default_stance)
        self.currentController = self.restController

        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)


        # Подписчики для режимов и скоростей
        self.node.create_subscription(RobotModeCommand, '/robot_mode', self.mode_callback, 10)
        self.node.create_subscription(RobotVelocity, '/robot_velocity', self.velocity_callback, 10)

    def mode_callback(self, msg):
        # Проверяем, относится ли сообщение к текущему роботу
        if msg.robot_id == self.robot_id:
            if self.node.verbose:
                self.node.get_logger().info(f"Received mode command: {msg.mode} for robot_id: {msg.robot_id}")
            if msg.mode == "REST":
                self.command.rest_event = True
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "TROT":
                self.command.rest_event = False
                self.command.trot_event = True
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "CRAWL":
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = True
                self.command.stand_event = False
            elif msg.mode == "STAND":
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = True
            
            # Изменяем текущий контроллер
            self.change_controller()

    def velocity_callback(self, msg):
        # Проверяем, относится ли сообщение к текущему роботу
        if msg.robot_id == self.robot_id:
            self.command.velocity = np.array([
                msg.cmd_vel.linear.x,
                msg.cmd_vel.linear.y,
                msg.cmd_vel.linear.z
            ])  # Изменено на numpy массив [x, y, z]
            
            self.command.yaw_rate = np.array([
                msg.cmd_vel.angular.x,
                msg.cmd_vel.angular.y,
                msg.cmd_vel.angular.z
            ])  # Изменено на numpy массив [roll, pitch, yaw]
            
            if self.node.verbose:
                self.node.get_logger().info(
                    f"Velocity command updated: linear={self.command.velocity}, angular={self.command.yaw_rate}"
                )

    def change_controller(self):
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False
            self.node.get_logger().info("Switched to TROT controller")
        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False
            self.node.get_logger().info("Switched to CRAWL controller")
        elif self.command.stand_event:
            if self.state.behavior_state != BehaviorState.STAND:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
                self.state.body_local_position[2] = 0.08 * 0.25
                self.node.get_logger().info("Switched to STAND controller")
            self.command.stand_event = False
        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False
            self.node.get_logger().info("Switched to REST controller")

    def run(self):
        # Возвращаем данные текущего контроллера
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([
            [self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front, -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [0, 0, 0, 0]
        ])

