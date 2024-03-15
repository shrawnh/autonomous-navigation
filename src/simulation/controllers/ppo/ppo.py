from myenv.myenv import WheeledRobotEnv, run_model, get_env_data_from_config
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import toml
import numpy as np

# train / train + save / test
MODEL_MODE = "test"

# easy / medium / hard
ENV_MODE = "easy"


def main():

    goal, wooden_boxes_data = get_env_data_from_config(ENV_MODE)
    env = WheeledRobotEnv(goal, wooden_boxes_data)
    check_env(env, warn=True)

    if MODEL_MODE == "train":
        model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
        model.learn(total_timesteps=1e5)

    elif MODEL_MODE == "train + save":
        model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
        model.learn(total_timesteps=1e5)
        model.save("ppo_wheeled_robot")

    elif MODEL_MODE == "test":
        model = PPO.load("ppo_wheeled_robot")

    run_model(env, model)


if __name__ == "__main__":
    main()
