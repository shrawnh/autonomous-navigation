from myenv.myenv import WheeledRobotEnv, run_algorithm, get_env_data_from_config
from _algorithm import VFHPlus
from stable_baselines3.common.env_checker import check_env

# train / train_save / test
MODEL_MODE = "train"

# easy / medium / hard
ENV_MODE = "train-dont-collide-2"

# front / front-back / sides
ROBOT_SENSORS = "front"


def main():

    goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(
        ENV_MODE, MODEL_MODE.split("_")[0], ROBOT_SENSORS
    )
    env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors)
    check_env(env, warn=True)

    # run_algorithm(env, VFHPlus(0.225, 700, 3.14, 36))
    run_algorithm(env, VFHPlus(), False)


if __name__ == "__main__":
    main()
