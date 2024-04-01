from myenv.myenv import (
    WheeledRobotEnv,
    run_model,
    get_env_data_from_config,
    model_name_check,
)
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# train / train_save / test
MODEL_MODE = "train_save"

# alpa / ""
MODEL_VERSION = ""

MODEL_NAME = "ppo_wheeled_robot"

# world name without _test or _train
ENV_MODE = "step-1"

# front / front_back / sides
ROBOT_SENSORS = "front"


def main():

    goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(ENV_MODE, MODEL_MODE.split("_")[0], ROBOT_SENSORS)  # type: ignore
    env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors)
    check_env(env, warn=True)
    model_name = model_name_check(env, MODEL_NAME, MODEL_VERSION)
    if model_name is None:
        print("Model name is None")
        return

    if MODEL_MODE.split("_")[0] == "train":
        try:
            model = PPO.load(model_name, env, n_steps=2048, verbose=2)
        except FileNotFoundError:
            model = PPO(
                "MlpPolicy",
                env,
                n_steps=2048,
                verbose=2,
                learning_rate=0.0001,
                gamma=0.9,
            )

        model.learn(total_timesteps=1e5)

        if MODEL_MODE == "train_save":
            model.save(model_name)

    elif MODEL_MODE == "test":
        try:
            model = PPO.load(model_name)
            run_model(env, model)
        except FileNotFoundError:
            print("Model not found")


if __name__ == "__main__":
    main()
