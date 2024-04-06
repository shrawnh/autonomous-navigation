from myenv.myenv import (
    WheeledRobotEnv,
    run_model,
    get_env_data_from_config,
    model_name_check,
)
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# train / train_save / test
MODEL_MODE = "test"

# alpa / ""
MODEL_VERSION = ""

# load / new
VERSION_MODE = "load"

MODEL_NAME = "ppo_wheeled_robot"

# world name without _test or _train
ENV_MODE = "step-2"

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
        #################### LOAD MODEL ####################
        if VERSION_MODE == "load":
            try:
                # always load the stable version of the model, but save the alpha first
                model = PPO.load(MODEL_NAME, env, n_steps=2048, verbose=2)
                model.tensorboard_log = f"logs/{ROBOT_SENSORS}_{MODEL_VERSION}"
            except FileNotFoundError:
                model = PPO(
                    "MlpPolicy",
                    env,
                    n_steps=2048,
                    verbose=2,
                    learning_rate=0.0001,
                    gamma=0.9,
                    ent_coef=0.01,
                    tensorboard_log=f"logs/{ROBOT_SENSORS}_{MODEL_VERSION}",
                )
        #################### LOAD MODEL ####################

        #################### NEW MODEL ####################
        else:
            model = PPO(
                "MlpPolicy",
                env,
                n_steps=2048,
                verbose=2,
                # learning_rate=0.0003,
                # gamma=0.8,
                # ent_coef=0.001,
                tensorboard_log=f"logs/{ROBOT_SENSORS}_{MODEL_VERSION}",
            )
        #################### NEW MODEL ####################

        model.learn(total_timesteps=1e6, tb_log_name=ENV_MODE)

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
