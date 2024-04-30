from stable_baselines3 import TD3, PPO, SAC
from myenv.mycontroller import MyController
from utils__ import check_all_ids_are_unique, create_step_name
from params import params, controllers_path
import time
import toml
import os

# ROBOT_SENSORS = ["front-back", "front", "front-back-6", "sides-6"]
ROBOT_SENSORS = ["front-back"]
# ROBOT_SENSORS = ["sides-6"]
MODE = "multiple"  # single / multiple
MODEL_MODE = "test"  # train / train_save / test

IDENTIFIER = "nectar"  # abcdefghijklmnopq

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
        if not check_all_ids_are_unique(params):
            return

        controller = MyController(
            model_mode=MODEL_MODE,
            version_mode="load",
            env_mode=current_step,
            env_to_train_from=prev_step,
            robot_sensors=ROBOT_SENSORS[sensor_index],
            verbose=True,
        )

        if MODE == "single":
            controller.execute(
                stable_baselines3_model=PPO,
                model_name=f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/controllers/ppo/testing_models/{ROBOT_SENSORS[sensor_index]}/best_model.zip",
                model_version="best",
                total_timesteps=1e6,
                model_args={
                    "learning_rate": 0.0001,
                    "gamma": 0.9,
                    "ent_coef": 0.01,
                    "clip_range": 0.1,
                    "stats_window_size": 10,
                },
                identifier="_honey",
                # time_limit=15.0,
            )

        elif MODE == "multiple":
            for index, value in enumerate(params):
                pretty_time = time.strftime("%Y-%m-%d--%H-%M-%S", time.localtime())
                try:
                    controller.execute(
                        stable_baselines3_model=value["agent"],
                        # model_name=value["name"],
                        model_name=(
                            f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/controllers/{value['name']}/testing_models/{ROBOT_SENSORS[sensor_index]}/best_model.zip"
                            if MODEL_MODE.split("-")[0] == "test"
                            else value["name"]
                        ),
                        model_version=(
                            "best" if MODEL_MODE.split("-")[0] == "test" else "alpha"
                        ),
                        total_timesteps=1e6,
                        model_args=value["args"],
                        identifier=f"_{IDENTIFIER}_{value['id']}_{index}_{pretty_time}",
                    )
                    with open(f"{controllers_path}/{value['name']}/logs/params/params__{value['id']}_{index}_{pretty_time}.txt", "w") as f:  # type: ignore
                        f.write(str(value))
                except Exception as e:
                    with open(f"{controllers_path}/{value['name']}/logs/errors.txt", "a") as f:  # type: ignore
                        f.write(f"In {value['id']}_{index} at {pretty_time}: {e}\n")
                    print(e)
                    continue

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

            controller.env.worldLoad(
                f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds/{next_sensor_step}_{MODEL_MODE.split('_')[0]}.wbt"
            )

        else:
            states["prev_step"] = c
            states["current_step"] += 1
            states["total_steps"] = total_steps
            states["sensor_index"] = sensor_index

            with open("steps.toml", "w") as f:
                toml.dump(states, f)

            controller.env.worldLoad(
                f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds/{next_step}_{MODEL_MODE.split('_')[0]}.wbt"
            )

    else:
        print("All steps and sensors completed.")


if __name__ == "__main__":
    main()
