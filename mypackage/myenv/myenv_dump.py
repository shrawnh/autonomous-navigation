import math
import numpy as np
from numpy.typing import NDArray
from typing import Any
import gymnasium as gym
from controller import Supervisor, device

RIGHT_WHEEL = "right wheel motor"
LEFT_WHEEL = "left wheel motor"
DS_NAMES = ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7"]
MAX_SPEED = 6.28
GRID_SIZE = 3
SENSETIVITY = 0.03
OBSTACLE_SIZE = 0.4
ROBOT_RADIUS = 0.225
RED = [1, 0, 0]
GREEN = [0.75, 1, 0.75]
SENSOR_THRESHOLD = 1023.0
HIGH_OBSERVATION_SPACE = np.array(
    [SENSOR_THRESHOLD] * len(DS_NAMES) + [MAX_SPEED, MAX_SPEED]
)
LOW_OBSERVATION_SPACE = np.array([0] * len(DS_NAMES) + [-MAX_SPEED, -MAX_SPEED])
STATE_SIZE = HIGH_OBSERVATION_SPACE.shape[0]


class CollisionDetector(Supervisor):
    def __init__(self, wooden_boxes_data) -> None:
        # super().__init__()
        self.wooden_boxes_data = wooden_boxes_data

    def _calculate_distance(self, position1: list, position2: list[list[int]]):
        distances = []
        for position in position2:
            # distances.append(
            #     math.sqrt(
            #         (position1[0] - position[0]) ** 2
            #         + (position1[1] - position[1]) ** 2
            #     )
            # )
            x_closest = max(
                position[0] - OBSTACLE_SIZE / 2,
                min(position1[0], position[0] + OBSTACLE_SIZE / 2),
            )
            y_closest = max(
                position[1] - OBSTACLE_SIZE / 2,
                min(position1[1], position[1] + OBSTACLE_SIZE / 2),
            )
            distances.append(
                ((x_closest - position1[0]) ** 2 + (y_closest - position1[1]) ** 2)
                ** 0.5
            )
        return distances

    def _change_robot_color(self, color: list[int]) -> None:
        robot_field = self.robot.getField("color")
        robot_field.setSFColor(color)

    def _is_out_of_bounds(self, position: list[int]) -> bool:
        bound = (GRID_SIZE / 2) - SENSETIVITY
        return abs(position[0]) > bound or abs(position[1]) > bound

    def detect_collision(self):
        wooden_box_nodes = [
            self.getFromDef(self.wooden_boxes_data[box]["name"])
            for box in self.wooden_boxes_data
        ]
        robot_position = np.array(self.getSelf().getPosition())
        wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]
        distances = self._calculate_distance(robot_position, wooden_box_positions)
        is_collision = self._is_out_of_bounds(robot_position) or any(
            distance < ROBOT_RADIUS for distance in distances
        )
        if is_collision:
            self._change_robot_color(RED)
            return True

        self._change_robot_color(GREEN)
        return False


class MyRobot(Supervisor):
    def __init__(self) -> None:
        super().__init__()
        self.right_motor_device: device.Device = self.getDevice(RIGHT_WHEEL)
        self.left_motor_device: device.Device = self.getDevice(LEFT_WHEEL)
        self.distance_sensors: list[device.Device] = []

    def initialise_sensor(self) -> None:
        for name in DS_NAMES:
            sensor = self.getDevice(name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)

    def initialise_motors(self) -> None:
        self.left_motor_device.setPosition(float("inf"))
        self.right_motor_device.setPosition(float("inf"))
        self.left_motor_device.setVelocity(0.0)
        self.right_motor_device.setVelocity(0.0)

    def set_speed(self, left_speed: float, right_speed: float) -> None:
        self.left_motor_device.setVelocity(left_speed)
        self.right_motor_device.setVelocity(right_speed)
        self.right_speed = right_speed
        self.left_speed = left_speed


class EnvSetup(Supervisor, gym.Env):
    def __init__(self, timestep, robot_instance: MyRobot):
        super().__init__()
        self.myrobot = robot_instance
        self.timestep = timestep

    def reset(self, *args, **kwargs):
        self.simulationResetPhysics()  # This should call the robot class
        self.simulationReset()
        super().step(self.timestep)
        print("\nResetting the environment...\n")
        self.myrobot.initialise_motors()
        self.myrobot.initialise_sensor()
        super().step(self.timestep)

        # Open AI Gym generic: observation, info
        return np.zeros(11).astype(np.float32), {}
