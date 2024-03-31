import numpy as np


class DynamicWindowApproach:
    def __init__(
        self,
        max_speed,
        max_yawrate,
        max_accel,
        max_dyawrate,
        velocity_resolution,
        yawrate_resolution,
        dt,
        predict_time,
        robot_radius,
    ):
        self.max_speed = max_speed
        self.max_yawrate = max_yawrate
        self.max_accel = max_accel
        self.max_dyawrate = max_dyawrate
        self.velocity_resolution = velocity_resolution
        self.yawrate_resolution = yawrate_resolution
        self.dt = dt
        self.predict_time = predict_time
        self.robot_radius = robot_radius

        self.left_speed = 0.5
        self.right_speed = 0.5

    def calculate_dynamic_window(self, v, omega):
        print(v, omega)
        Vs = [0, self.max_speed, -self.max_yawrate, self.max_yawrate]
        Vd = [
            v - self.max_accel * self.dt,
            v + self.max_accel * self.dt,
            omega - self.max_dyawrate * self.dt,
            omega + self.max_dyawrate * self.dt,
        ]
        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3]),
        ]
        print(dw)
        return dw

    def evaluate_trajectory(self, v, omega, sensor_readings):
        x, y, theta = 0.0, 0.0, 0.0  # Current position and orientation
        cost = 0.0

        for _ in range(int(self.predict_time / self.dt)):
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += omega * self.dt

            # Cost for proximity to obstacles
            for reading in sensor_readings:
                if reading >= 700:  # Threshold to consider an obstacle
                    cost += 1.0

            # Additional cost components, such as for turning, can be added here

        return cost

    def get_action(self, sensor_readings):
        current_v = (self.left_speed + self.right_speed) / 2
        print(current_v)
        current_omega = (self.right_speed - self.left_speed) / (2 * self.robot_radius)
        dynamic_window = self.calculate_dynamic_window(current_v, current_omega)
        min_cost = float("inf")
        best_v, best_omega = current_v, current_omega

        for v in np.arange(
            dynamic_window[0], dynamic_window[1], self.velocity_resolution
        ):
            for omega in np.arange(
                dynamic_window[2], dynamic_window[3], self.yawrate_resolution
            ):
                cost = self.evaluate_trajectory(v, omega, sensor_readings)
                if cost < min_cost:
                    min_cost = cost
                    best_v, best_omega = v, omega
                    print(best_v, best_omega)

        # Convert the chosen v and omega to left and right wheel speeds
        # This formula depends on the specific robot kinematics
        self.left_speed = best_v - best_omega * self.robot_radius
        self.right_speed = best_v + best_omega * self.robot_radius

        return np.array([self.left_speed, self.right_speed])
