from myenv.myenv import (
    WheeledRobotEnv,
    run_model,
    get_env_data_from_config,
    model_name_check,
)
from myenv.utils import update_controller
from stable_baselines3.common.env_checker import check_env
import time


class MyController:
    def __init__(
        self,
        model_mode: str,
        model_version,
        version_mode,
        model_name: str,
        env_mode,
        robot_sensors,
    ):
        """
        param:: model_mode: train / train_save / test
        param:: model_version: alpha / ""
        param:: version_mode: load / new
        param:: env_mode: world name without _test or _train
        param:: robot_sensors: front / front_back / sides
        """
        self.model_mode = model_mode
        self.model_version = model_version
        self.version_mode = version_mode
        self.model_name = model_name
        self.env_mode = env_mode
        self.robot_sensors = robot_sensors

        goal, wooden_boxes_data, grid_size, robot_sensors = get_env_data_from_config(env_mode, model_mode.split("_")[0], robot_sensors)  # type: ignore
        self.env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors)
        check_env(self.env, warn=True)
        self.current_model_name = model_name_check(self.env, model_name, model_version)

        if self.env.world_path.split("/worlds/")[1].split(".wbt")[0] != f"{self.env_mode}_{self.model_mode.split('_')[0]}":  # type: ignore
            curr_path = self.env.world_path.split("/worlds/")[0]
            new_path = f"{curr_path}/worlds/{self.env_mode}_{self.model_mode.split('_')[0]}.wbt"
            update_controller(new_path, self.model_name.split("_")[0])
            time.sleep(1)  # wait for .wbt to be saved, await doesnt work with webots
            self.env.worldLoad(new_path)

    def main(
        self,
        stable_baselines3_model,
        total_timesteps: int,
        model_args: dict = {},
        identifier: str = "",
    ):

        if self.current_model_name is None:
            print("Model name is None")
            return

        if self.model_mode.split("_")[0] == "train":
            #################### LOAD MODEL ####################
            if self.version_mode == "load":
                try:
                    # always load the stable version of the model, but save the alpha first
                    model = stable_baselines3_model.load(
                        self.model_name, self.env, verbose=2, **model_args
                    )
                    model.tensorboard_log = (
                        f"logs/{self.robot_sensors}_{self.model_version}"
                    )
                except FileNotFoundError:
                    model = stable_baselines3_model(
                        "MlpPolicy",
                        self.env,
                        verbose=2,
                        tensorboard_log=f"logs/{self.robot_sensors}_{self.model_version}",
                        **model_args,
                    )
            #################### LOAD MODEL ####################

            #################### NEW MODEL ####################
            else:
                model = stable_baselines3_model(
                    "MlpPolicy",
                    self.env,
                    verbose=2,
                    tensorboard_log=f"logs/{self.robot_sensors}_{self.model_version}",
                    **model_args,
                )
            #################### NEW MODEL ####################
            model.learn(total_timesteps, tb_log_name=self.env_mode)

            if self.model_mode == "train_save":
                model.save(f"{self.current_model_name}_{self.env_mode}_{identifier}")
                # model.save(self.current_model_name)

        elif self.model_mode == "test":
            try:
                model = stable_baselines3_model.load(self.current_model_name)
                run_model(self.env, model)
            except FileNotFoundError:
                print("Model not found")

        self.env.reset()
        self.env.reset_env_info()
