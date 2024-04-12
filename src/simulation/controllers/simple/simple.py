from myenv.myenv import WheeledRobotEnv, run_algorithm, get_env_data_from_config
from stable_baselines3.common.env_checker import check_env
import numpy as np

# train / train_save / test
MODEL_MODE = "test"

# easy / medium / hard
ENV_MODE = "step-2"

# front / front_back / sides
ROBOT_SENSORS = "front"


class Simple:
    def __init__(self):
        pass

    def get_action(self, obs):
        return np.array([1.0, 1.0])


def main():

    goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(
        ENV_MODE, MODEL_MODE.split("_")[0], ROBOT_SENSORS
    )
    env = WheeledRobotEnv(
        goal, wooden_boxes_data, grid_size, robot_sensors, verbose=False
    )
    check_env(env, warn=True)

    run_algorithm(env, Simple(), verbose=False)


if __name__ == "__main__":
    main()
