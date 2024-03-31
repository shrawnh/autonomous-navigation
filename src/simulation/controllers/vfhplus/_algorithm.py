import numpy as np


class VFHPlus:
    def __init__(self, num_bins=36, max_sensor_range=5.0, robot_radius=0.5):
        self.num_bins = num_bins
        self.max_sensor_range = max_sensor_range
        self.robot_radius = robot_radius
        self.histogram = np.zeros(num_bins)

    def update_histogram(self, sensor_angles, sensor_distances):
        self.histogram.fill(0)
        bin_width = 360 / self.num_bins

        for angle, distance in zip(sensor_angles, sensor_distances):
            if distance < self.max_sensor_range:
                bin_index = int(angle // bin_width)
                self.histogram[bin_index] = max(
                    self.histogram[bin_index], self.max_sensor_range - distance
                )

    def choose_direction(self, target_angle):
        bin_width = 360 / self.num_bins
        target_bin = int(target_angle // bin_width)
        histogram_threshold = self.robot_radius * 2

        # Filter bins that are too close to obstacles
        safe_bins = np.where(self.histogram < histogram_threshold)[0]

        if len(safe_bins) == 0:
            return None  # No safe direction found

        # Find the safe bin closest to the target direction
        closest_bin = safe_bins[np.argmin(np.abs(safe_bins - target_bin))]
        chosen_direction = closest_bin * bin_width + bin_width / 2

        return chosen_direction

    def get_motor_commands(self, chosen_direction, max_speed=1.0):
        # This is a simple example. Real implementation would depend on robot dynamics.
        if chosen_direction is None:
            return 0, 0  # Stop if no safe direction is found

        left_speed = max_speed * (1 - chosen_direction / 180)
        right_speed = max_speed * (1 - (360 - chosen_direction) / 180)

        return np.array([left_speed, right_speed])

    def get_action(self, sensor_readings):
        sensor_angles = np.linspace(0, 2 * np.pi, len(sensor_readings), endpoint=False)
        self.update_histogram(sensor_angles, sensor_readings)
        chosen_direction = self.choose_direction(target_angle=180)
        return self.get_motor_commands(chosen_direction)
