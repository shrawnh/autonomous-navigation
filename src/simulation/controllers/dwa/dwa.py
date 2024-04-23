from myenv.myenv import WheeledRobotEnv, run_algorithm, get_env_data_from_config
from _algorithm import DynamicWindowApproach
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

    run_algorithm(
        env,
        DynamicWindowApproach(
            max_speed=1.0,
            max_yawrate=0.5,
            max_accel=0.2,
            max_dyawrate=0.1,
            velocity_resolution=0.01,
            yawrate_resolution=0.01,
            dt=0.1,
            predict_time=3.0,
            robot_radius=0.225,
        ),
        False,
    )


if __name__ == "__main__":
    main()
