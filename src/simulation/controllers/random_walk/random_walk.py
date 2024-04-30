from myenv.myenv import WheeledRobotEnv, run_algorithm, get_env_data_from_config
from _algorithm import RandomWalk
from _front_back_algo import FrontBackRandomWalk
from _sides_algo import SidesRuleBasedAlgo
from stable_baselines3.common.env_checker import check_env
import toml
import time


def create_step_name(step: int, verision: int, robot_sensors: str) -> str:
    assert robot_sensors in ["front", "front-back", "sides", "sides-6", "front-back-6"], "Invalid robot_sensors. Must be one of: front, front-back, sides, sides-6, front-back-6."  # type: ignore
    return f"step-{step}-v{verision}-{robot_sensors}"


# train / train_save / test
MODEL_MODE = "test"

# easy / medium / hard
# ENV_MODE = "step-1-v1-front-back"

# front / front-back / sides / sides-6 / front-back-6
# ROBOT_SENSORS = "front-back"
ROBOT_SENSORS = ["front-back", "front", "front-back-6", "sides-6", "sides"]
sensor_algorithm = {
    "front": RandomWalk(),
    "front-back": FrontBackRandomWalk(),
    "sides": SidesRuleBasedAlgo(),
    "sides-6": SidesRuleBasedAlgo(),
    "front-back-6": FrontBackRandomWalk(),
}

# Load the toml file
with open("steps.toml", "r") as f:
    states = toml.load(f)
    p = states["prev_step"]
    c = states["current_step"]
    n = c + 1
    v = states["version"]
    sensor_index = states["sensor_index"]

prev_step = create_step_name(p, v, ROBOT_SENSORS[sensor_index])
if states["prev_step"] == 0:
    prev_step = "$$$"

current_step = create_step_name(c, v, ROBOT_SENSORS[sensor_index])
next_step = create_step_name(c + 1, v, ROBOT_SENSORS[sensor_index])
total_steps = states["total_steps"]


def main():
    if c <= total_steps and sensor_index < len(ROBOT_SENSORS):

        goal, wooden_boxes_data, grid_size, robot_sensors, _ = get_env_data_from_config(
            current_step, MODEL_MODE.split("_")[0], ROBOT_SENSORS[sensor_index]
        )
        env = WheeledRobotEnv(goal, wooden_boxes_data, grid_size, robot_sensors, False)
        check_env(env, warn=True)

        if env.unwrapped.world_path.split("/worlds/")[1].split(".wbt")[0] != f"{current_step}_{MODEL_MODE.split('_')[0]}":  # type: ignore
            curr_path = env.unwrapped.world_path.split("/worlds/")[0]
            new_path = (
                f"{curr_path}/worlds/{current_step}_{MODEL_MODE.split('_')[0]}.wbt"
            )
            time.sleep(1)  # wait for .wbt to be saved, await doesnt work with webots
            env.worldLoad(new_path)

        run_algorithm(
            env=env,
            algorithm=sensor_algorithm[ROBOT_SENSORS[sensor_index]],
            log_file=f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/testing_logs/random_walk_2/{current_step}.csv",
        )

        if n > total_steps:
            if sensor_index + 1 >= len(ROBOT_SENSORS):
                print("All steps and sensors completed.")
                return
            print("All steps completed.")
            states["prev_step"] = 0
            states["current_step"] = 1
            states["total_steps"] = total_steps
            states["sensor_index"] = sensor_index + 1
            next_sensor_step = create_step_name(1, v, ROBOT_SENSORS[sensor_index + 1])

            with open("steps.toml", "w") as f:
                toml.dump(states, f)

            env.worldLoad(
                f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds/{next_sensor_step}_{MODEL_MODE.split('_')[0]}.wbt"
            )

        else:
            states["prev_step"] = c
            states["current_step"] += 1
            states["total_steps"] = total_steps
            states["sensor_index"] = sensor_index

            with open("steps.toml", "w") as f:
                toml.dump(states, f)

            env.worldLoad(
                f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds/{next_step}_{MODEL_MODE.split('_')[0]}.wbt"
            )


if __name__ == "__main__":
    main()
