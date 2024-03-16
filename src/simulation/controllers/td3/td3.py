from myenv.myenv import WheeledRobotEnv, run_model, get_env_data_from_config
from stable_baselines3 import TD3
from stable_baselines3.common.env_checker import check_env

# train / train_save / test
MODEL_MODE = "train"

# easy / medium / hard
ENV_MODE = "easy"

# front / front_back / sides
ROBOT_SENSORS = "front"


def main():

    goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(
        ENV_MODE, MODEL_MODE.split("_")[0], ROBOT_SENSORS
    )
    env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors)
    check_env(env, warn=True)

    if MODEL_MODE == "train":
        model = TD3("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=1e5)

    elif MODEL_MODE == "train_save":
        model = TD3("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=1e5)
        model.save("td3_wheeled_robot")

    elif MODEL_MODE == "test":
        model = TD3.load("td3_wheeled_robot")

    run_model(env, model)


if __name__ == "__main__":
    main()
