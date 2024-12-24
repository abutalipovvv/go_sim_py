#!/usr/bin/env python3
# Author: lnotspotl
# Modified for ros2 by: abutalipovvv
import numpy as np

class StandController:
    def __init__(self, node, default_stance):
        self.node = node  # Ссылка на ноду для логирования
        self.def_stance = default_stance
        self.max_reach = 0.065

        # Параметры для управления скоростью
        self.body_velocity_scale = 0.01  # Масштаб для линейных скоростей
        self.body_angular_scale = 0.005  # Масштаб для угловой скорости

        # Ограничения скорости
        self.max_linear_velocity = 0.05  # [m/s]
        self.max_angular_velocity = 0.1   # [rad/s]

    @property
    def default_stance(self):
        # Возвращаем копию дефолтного положения ног
        return np.copy(self.def_stance)

    def run(self, state, command):
        # Копируем дефолтное положение ног
        temp = self.default_stance.copy()
        temp[2] = [command.robot_height] * 4

        # Обработка команд скорости для корректировки положения тела
        linear_vel = command.velocity  # Ожидается numpy массив [x, y, z]
        angular_vel = command.yaw_rate  # Ожидается numpy массив [roll, pitch, yaw]

        # Ограничение линейной скорости
        linear_vel = np.clip(linear_vel, 
                             [-self.max_linear_velocity, -self.max_linear_velocity, -self.max_linear_velocity],
                             [self.max_linear_velocity, self.max_linear_velocity, self.max_linear_velocity])
        
        # Ограничение угловой скорости
        angular_vel = np.clip(angular_vel, 
                              [-self.max_angular_velocity, -self.max_angular_velocity, -self.max_angular_velocity],
                              [self.max_angular_velocity, self.max_angular_velocity, self.max_angular_velocity])

        # Корректируем положение тела
        state.body_local_position[0] += linear_vel[0] * self.body_velocity_scale  # x
        state.body_local_position[1] += linear_vel[1] * self.body_velocity_scale  # y
        state.body_local_position[2] += linear_vel[2] * self.body_velocity_scale  # z (подъем/опускание)

        # Корректируем ориентацию тела
        state.body_local_orientation[0] += angular_vel[0] * self.body_angular_scale  # Roll
        state.body_local_orientation[1] += angular_vel[1] * self.body_angular_scale  # Pitch
        state.body_local_orientation[2] += angular_vel[2] * self.body_angular_scale  # Yaw

        # Логирование изменений, если включен verbose
        if self.node.verbose:
            self.node.get_logger().info(f"Updated body position: {state.body_local_position}")
            self.node.get_logger().info(f"Updated body orientation: {state.body_local_orientation}")

        # Обновляем состояние ног (при необходимости можно добавлять корректировки ног для поддержания баланса)
        state.foot_locations = temp
        return state.foot_locations
