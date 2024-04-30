import numpy as np
from numpy.typing import NDArray


class SidesRuleBasedAlgo:
    def __init__(self, seed: int = 42, obstacle_threshold: int = 750):
        self.seed = seed
        self.random = np.random.RandomState(seed)
        self.avoidObstacleCounter = 0
        self.current_action = np.array([1, 1])

        self.max_speed = 0.25
        self.obstacle_threshold = obstacle_threshold
        self.mu_duration, self.sigma_duration = 10, 5  # Mean and variance for duration
        self.mu_angle, self.sigma_angle = 0, 45  # Mean and variance for turning angle
        self.num_of_sensor = 8
        self.prev_action = None
        self.prev_angle = None
        self.noise_count = 0

    # def decide_movement(self, sensor_readings):
    #     # print("Sensor readings: ", sensor_readings)
    #     self.num_of_sensor = len(sensor_readings) // 2
    #     left_sensors = sensor_readings[: self.num_of_sensor]
    #     right_sensors = sensor_readings[self.num_of_sensor :]
    #     # Check for obstacles using both left and right sensors
    #     if any(sr > self.obstacle_threshold for sr in left_sensors):
    #         closest_sensor_idx = np.argmax(left_sensors)
    #         turn_angle = self.calculate_turn_angle(closest_sensor_idx)
    #         movement_duration = np.random.normal(self.mu_duration, self.sigma_duration)
    #         self.prev_action = "turn"
    #         self.prev_angle = turn_angle
    #         self.noise_count = 3
    #         return "turn", turn_angle, movement_duration
    #     elif any(sr > self.obstacle_threshold for sr in right_sensors):
    #         closest_sensor_idx = np.argmax(right_sensors)
    #         turn_angle = self.calculate_turn_angle(closest_sensor_idx)
    #         movement_duration = np.random.normal(self.mu_duration, self.sigma_duration)
    #         self.prev_action = "turn"
    #         self.prev_angle = turn_angle
    #         self.noise_count = 3
    #         return "turn", turn_angle, movement_duration
    #     else:
    #         movement_duration = np.random.normal(self.mu_duration, self.sigma_duration)
    #         # Add a random element to the forward movement
    #         # 10% chance to turn instead of moving forward
    #         if self.random.random() < 0.4:  # noqa: E203
    #             turn_angle = self.random.uniform(-90, 90)  # noqa: E203
    #             return "turn", turn_angle, movement_duration
    #         else:
    #             return "forward", None, movement_duration

    def decide_movement(self, sensor_readings):
        if any(sensor_readings > self.obstacle_threshold):
            # Split sensor readings into left and right
            left_sensors = sensor_readings[:3]
            right_sensors = sensor_readings[3:]

            # Get the index of the closest sensor and its side
            closest_left_sensor_idx = np.argmax(left_sensors)
            closest_right_sensor_idx = np.argmax(right_sensors)

            if (
                left_sensors[closest_left_sensor_idx]
                > right_sensors[closest_right_sensor_idx]
            ):
                closest_sensor_idx = closest_left_sensor_idx
                side = "left"
            else:
                closest_sensor_idx = closest_right_sensor_idx
                side = "right"

            # Decide movement based on the index of the closest sensor
            if closest_sensor_idx == 0:  # Front sensor
                turn_angle = self.calculate_turn_angle(closest_sensor_idx)
                return (
                    "turn",
                    -turn_angle if side == "left" else turn_angle,
                    self.mu_duration,
                )
            elif closest_sensor_idx == 1:  # Middle sensor
                return "forward", None, self.mu_duration
            elif closest_sensor_idx == 2:  # Rear sensor
                turn_angle = self.calculate_turn_angle(closest_sensor_idx)
                return (
                    "turn",
                    -turn_angle if side == "left" else turn_angle,
                    self.mu_duration,
                )
        else:
            return "forward", None, self.mu_duration

    def calculate_turn_angle(self, sensor_idx, turn_left=True):
        angle_offset = 45
        turn_angle = sensor_idx * angle_offset
        if not turn_left:
            turn_angle -= 135
        return turn_angle

    def get_motor_speeds(self, action, angle):
        if action == "forward":
            return np.array([self.max_speed, self.max_speed])
        elif action == "reverse":
            return np.array([-self.max_speed, -self.max_speed])
        else:  # Turning logic
            if angle < 0:
                return np.array([-self.max_speed, self.max_speed])
            else:
                return np.array([self.max_speed, -self.max_speed])

    def get_action(self, sensor_readings: NDArray) -> np.ndarray:
        if self.noise_count > 0:
            # # print("Noise = ", self.noise_count)
            self.noise_count -= 1
            return self.get_motor_speeds(self.prev_action, self.prev_angle)
        action, angle, _ = self.decide_movement(sensor_readings)
        return self.get_motor_speeds(action, angle)
