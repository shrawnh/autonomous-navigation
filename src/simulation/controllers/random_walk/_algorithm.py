import numpy as np
from numpy.typing import NDArray


class RandomWalk:
    def __init__(self, seed: int = 42, obstacle_threshold: int = 650):
        self.seed = seed
        self.random = np.random.RandomState(seed)
        self.avoidObstacleCounter = 0
        self.current_action = np.array([1, 1])

        self.max_speed = 0.5
        self.obstacle_threshold = obstacle_threshold
        self.mu_duration, self.sigma_duration = 10, 5  # Mean and variance for duration
        self.mu_angle, self.sigma_angle = 0, 45  # Mean and variance for turning angle
        self.num_of_sensor = 8
        self.prev_action = None
        self.prev_angle = None
        self.noise_count = 0

    def decide_movement(self, sensor_readings):
        self.num_of_sensor = len(sensor_readings)
        print(sensor_readings, self.noise_count, self.prev_action, self.prev_angle)
        if any(sr > self.obstacle_threshold for sr in sensor_readings):
            closest_sensor_idx = np.argmax(sensor_readings)
            turn_angle = self.calculate_turn_angle(closest_sensor_idx)
            movement_duration = np.random.normal(self.mu_duration, self.sigma_duration)
            self.prev_action = "turn"
            self.prev_angle = turn_angle
            self.noise_count = 3
            return "turn", turn_angle, movement_duration
        else:
            movement_duration = np.random.normal(self.mu_duration, self.sigma_duration)

            # Add a random element to the forward movement
            # 10% chance to turn instead of moving forward
            if self.random.random() < 0.4:  # noqa: E203
                turn_angle = self.random.uniform(-90, 90)  # noqa: E203
                return "turn", turn_angle, movement_duration
            else:
                return "forward", None, movement_duration

    def calculate_turn_angle(self, sensor_idx):
        angle_offset = 180 / self.num_of_sensor  # Angle between each sensor
        turn_angle = sensor_idx * angle_offset
        if turn_angle > 90:
            turn_angle -= 180
        return turn_angle

    def get_motor_speeds(self, action, angle):
        if action == "forward":
            return np.array([self.max_speed, self.max_speed])
        else:  # Turning logic
            if angle < 0:
                return np.array([-self.max_speed, self.max_speed])
            else:
                return np.array([self.max_speed, -self.max_speed])

    def get_action(self, sensor_readings: NDArray) -> np.ndarray:
        if self.noise_count > 0:
            print("Noise = ", self.noise_count)
            self.noise_count -= 1
            return self.get_motor_speeds(self.prev_action, self.prev_angle)
        action, angle, _ = self.decide_movement(sensor_readings)
        return self.get_motor_speeds(action, angle)
