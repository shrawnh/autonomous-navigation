from myenv.myenv import (
    WheeledRobotEnv,
    run_model,
    get_env_data_from_config,
    model_name_check,
)
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.monitor import Monitor
import time
import os
import glob

CONTROLLERS_PATH = (
    "/Users/shrwnh/Development/autonomous-navigation/src/simulation/controllers"
)


def find_latest_model_path(env_to_train_from, param_str, model_name, identifier):
    pattern = os.path.join(
        CONTROLLERS_PATH,
        f"{model_name}/best_models",
        f"*{env_to_train_from}_*_{identifier}_*_{param_str}*",
        # f"*{param_str}*",
    )
    folders = glob.glob(pattern)

    if not folders:
        with open(f"{CONTROLLERS_PATH}/rl/errors.txt", "a") as f:
            f.write(f"No folders found for pattern: {pattern}")
        raise ValueError(f"No folders found for pattern: {pattern}")

    for folder in folders:
        if not os.path.exists(f"{folder}/best_model.zip"):
            os.system(f"rm -rf {folder}")
            folders.remove(folder)
            print(f"Deleted {folder}")

    latest_folder_path = min(folders, key=os.path.getmtime)

    return latest_folder_path


def compress_string(s: str):
    if "_" in s:
        return "".join(part[0] for part in s.split("_"))
    else:
        return s[0]


class MyController:
    def __init__(
        self,
        model_mode: str,
        version_mode,
        env_mode,
        env_to_train_from: str = "",
        robot_sensors="front",
        verbose=True,
    ):
        """
        param:: model_mode: train / train_save / test
        param:: model_version: alpha / ""
        param:: version_mode: load / new
        param:: env_mode: world name without _test or _train
        param:: robot_sensors: front / front-back / sides / sides-6 / front-back-6
        """
        self.model_mode = model_mode
        self.version_mode = version_mode
        self.env_mode = env_mode
        self.env_to_train_from = env_to_train_from
        self.robot_sensors = robot_sensors
        self.verbose = verbose

        goal, wooden_boxes_data, grid_size, robot_sensors, checkpoints = get_env_data_from_config(env_mode, model_mode.split("_")[0], robot_sensors)  # type: ignore
        self.env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors, verbose, checkpoints)  # type: ignore
        self.env = Monitor(self.env)
        try:
            check_env(self.env, warn=True)
        except Exception as e:
            with open(f"{CONTROLLERS_PATH}/rl/errors.txt", "a") as f:
                f.write(f"Error in {env_mode}: {e}\n")
            return

    def execute(
        self,
        stable_baselines3_model: BaseAlgorithm,
        model_name: str,
        model_version: str,
        total_timesteps: int,
        model_args: dict = {},
        identifier: str = "",
        time_limit: float = 150.0,
    ):
        # self.env.time_limit = time_limit  ########
        agent_dir_path = f"{CONTROLLERS_PATH}/{model_name}"
        param_str = (
            "_".join(
                f"{compress_string(key)}={value}" for key, value in model_args.items()
            )
            .replace("[", "@@")
            .replace("]", "@@")
        )

        #################### CHECKS ####################

        current_model_name = model_name_check(self.env, model_name, model_version)  # type: ignore
        if self.env.unwrapped.world_path.split("/worlds/")[1].split(".wbt")[0] != f"{self.env_mode}_{self.model_mode.split('_')[0]}":  # type: ignore
            curr_path = self.env.unwrapped.world_path.split("/worlds/")[0]
            new_path = f"{curr_path}/worlds/{self.env_mode}_{self.model_mode.split('_')[0]}.wbt"
            time.sleep(1)  # wait for .wbt to be saved, await doesnt work with webots
            self.env.worldLoad(new_path)

        if current_model_name is None:
            with open(f"{CONTROLLERS_PATH}/rl/errors.txt", "a") as f:
                f.write("Model name is None")
            raise ValueError("Model name is None")

        if (
            self.model_mode == "train"
            and self.env_to_train_from == ""
            and self.version_mode == "load"
        ):
            raise ValueError("No environment to train from")

        if param_str == "":
            param_str = "default"

        #################### CHECKS ####################

        if self.model_mode.split("_")[0] == "train":

            #################### CALLBACKS ####################

            stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=90, min_evals=10, verbose=1)  # type: ignore
            eval_callback = EvalCallback(
                self.env,
                best_model_save_path=f"{agent_dir_path}/best_models/{self.env_mode}_{model_version}_{identifier}_{param_str}",
                log_path=f"{agent_dir_path}/logs/evals/{self.env_mode}_{model_version}_{identifier}_{param_str}",
                eval_freq=10000,
                deterministic=True,
                render=False,
                n_eval_episodes=5,
                callback_after_eval=stop_train_callback,
                verbose=2,
            )

            #################### CALLBACKS ####################

            #################### LOAD MODEL ####################
            if self.version_mode == "load":
                try:
                    # always load the stable version of the model, but save the alpha first
                    latest_model = find_latest_model_path(
                        self.env_to_train_from,
                        param_str,
                        model_name,
                        identifier.split("_")[1],  # TODO: apply a more stable way
                    )
                    print(f"Loading model: {latest_model}")
                    model = stable_baselines3_model.load(
                        f"{latest_model}/best_model",
                        self.env,
                        verbose=2,
                        **model_args,
                    )
                    model.tensorboard_log = (
                        f"{agent_dir_path}/logs/{self.robot_sensors}_{model_version}"
                    )
                except (FileNotFoundError, ValueError) as e:
                    print(f"An error occurred while loading the model: {e}")
                    print("Creating a new model")
                    with open(f"{CONTROLLERS_PATH}/rl/errors.txt", "a") as f:
                        f.write(
                            f"Error in {self.env_mode} at {identifier}: while loading the model {e}\n"
                        )
                    model = stable_baselines3_model(
                        "MlpPolicy",
                        self.env,
                        verbose=2,
                        tensorboard_log=f"{agent_dir_path}/logs/{self.robot_sensors}_{model_version}",
                        **model_args,
                    )
            #################### LOAD MODEL ####################

            #################### NEW MODEL ####################
            else:
                model = stable_baselines3_model(
                    "MlpPolicy",
                    self.env,
                    verbose=2,
                    tensorboard_log=f"{agent_dir_path}/logs/{self.robot_sensors}_{model_version}",
                    **model_args,
                )
            #################### NEW MODEL ####################

            tb_log_name = f"{self.env_mode}_{identifier}_{param_str}"

            model.learn(
                total_timesteps=total_timesteps,
                tb_log_name=tb_log_name,
                callback=eval_callback,
            )

            if self.model_mode == "train_save":
                model.save(
                    f"{agent_dir_path}/models/{current_model_name}_{self.env_mode}_{param_str}_{identifier}"
                )

        elif self.model_mode == "test":
            try:
                # model = stable_baselines3_model.load(f"{agent_dir_path}/{current_model_name}")
                model = stable_baselines3_model.load(f"{model_name}")
                run_model(
                    self.env,
                    model,
                    self.verbose,
                    250,
                    f"{model_name.split('/')[8]}-{self.env_mode}.csv",
                )
            except FileNotFoundError:
                raise FileNotFoundError(f"Model not found: {model_name}")

        self.env.reset()
        self.env.reset_env_info()
