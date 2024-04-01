from myenv.myenv import WheeledRobotEnv, run_model, get_env_data_from_config
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# train / train_save / test
MODEL_MODE = "train"

# 1-dont-collide / easy / medium / hard
ENV_MODE = "step-1"

# front / front_back / sides
ROBOT_SENSORS = "front"


def main():

    goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(
        ENV_MODE, MODEL_MODE.split("_")[0], ROBOT_SENSORS
    )
    env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors)
    check_env(env, warn=True)

    if MODEL_MODE.split("_")[0] == "train":
        try:
            model = PPO.load("ppo_wheeled_robot", env, n_steps=2048, verbose=2)
        except FileNotFoundError:
            model = PPO(
                "MlpPolicy",
                env,
                n_steps=2048,
                verbose=2,
                learning_rate=0.0001,
                gamma=0.99,
            )

        model.learn(total_timesteps=1e6)

        if MODEL_MODE == "train_save":
            model.save("ppo_wheeled_robot")

    elif MODEL_MODE == "test":
        model = PPO.load("ppo_wheeled_robot")

        run_model(env, model)


if __name__ == "__main__":
    main()
