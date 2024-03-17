import numpy as np
from numpy.typing import NDArray


class RandomWalk:
    def __init__(self, seed: int = 42):
        self.seed = seed
        self.random = np.random.RandomState(seed)
        self.avoidObstacleCounter = 0
        self.current_action = np.array([1, 1])

    def get_action(self, sensor_readings: NDArray) -> np.ndarray:
        if self.avoidObstacleCounter > 0:
            self.avoidObstacleCounter -= 1
            return self.current_action
        elif any(sensor_readings > 850):
            self.avoidObstacleCounter = 30
            choice = self.random.choice([-1, 1])
            self.current_action = np.array([choice * 0.5, choice * -0.5])
            return self.current_action
        else:
            return np.array([0.5, 0.5])
