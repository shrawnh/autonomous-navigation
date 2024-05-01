import numpy as np
from numpy.typing import NDArray


class SidesRuleBasedAlgo:
    def __init__(
        self, seed: int = 42, obstacle_threshold: int = 750, turn_interval: int = 10
    ):
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
        self.turn_interval = turn_interval
        self.time_counter = 0
        self.in_turning_mode = False
        self.turn_count = 0
        self.turn_direction = 1

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

            # Increase reaction based on proximity
            closest_distance = min(
                left_sensors[closest_left_sensor_idx],
                right_sensors[closest_right_sensor_idx],
            )
            if (
                closest_distance > self.obstacle_threshold * 0.8
            ):  # Very close to the obstacle
                # Reverse slightly before turning
                return "reverse", None, 3  # Reverse for a short duration

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
            self.time_counter += 1
            # print("Time counter = ", self.time_counter)

            # Initiating the turn phase when the time_counter hits the turn_interval
            if self.time_counter == self.turn_interval:
                self.in_turning_mode = True  # A flag to indicate turning mode is active
                self.turn_count = 0  # A counter to track the number of turns made
                self.turn_direction = -self.turn_direction

            # Handling the turn phase
            if self.in_turning_mode:
                if self.turn_count < 45:
                    self.turn_count += 1
                    # print("Turning")
                    # return self.get_motor_speeds("turn", self.random.uniform(-90, 90))
                    # return "turn", self.random.uniform(-90, 90), self.mu_duration
                    return "turn", 90 * self.turn_direction, self.mu_duration
                else:
                    # Exit turning mode and reset counters once 10 turns are made
                    # print("Resetting counter")
                    self.in_turning_mode = False
                    self.time_counter = 0
            return "forward", None, self.mu_duration

    def calculate_turn_angle(self, sensor_idx, turn_left=True):
        angle_offset = 70
        turn_angle = sensor_idx * angle_offset
        if not turn_left:
            turn_angle -= 180
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
            # # # print("Noise = ", self.noise_count)
            self.noise_count -= 1
            return self.get_motor_speeds(self.prev_action, self.prev_angle)
        action, angle, _ = self.decide_movement(sensor_readings)
        return self.get_motor_speeds(action, angle)
