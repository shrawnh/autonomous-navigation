from stable_baselines3 import TD3, PPO, SAC
from myenv.mycontroller import MyController
from utils__ import check_all_ids_are_unique, create_step_name
from params import params, controllers_path
import time
import toml

ROBOT_SENSORS = "front"
MODE = "multiple"  # single / multiple
MODEL_MODE = "train_save"  # train / train_save / test

IDENTIFIER = "cucumber"  # abgm

# Load the toml file
with open("steps.toml", "r") as f:
    states = toml.load(f)
    p = states["prev_step"]
    c = states["current_step"]
    n = c + 1
    v = states["version"]

prev_step = create_step_name(p, v, ROBOT_SENSORS)
if states["prev_step"] == 0:
    prev_step = ""

current_step = create_step_name(c, v, ROBOT_SENSORS)
next_step = create_step_name(c + 1, v, ROBOT_SENSORS)
total_steps = states["total_steps"]


def main():
    if c <= total_steps:
        if not check_all_ids_are_unique(params):
            return

        controller = MyController(
            model_mode=MODEL_MODE,
            version_mode="load",
            env_mode=current_step,
            env_to_train_from=prev_step,
            robot_sensors=ROBOT_SENSORS,
            verbose=True,
        )

        if MODE == "single":
            controller.execute(
                stable_baselines3_model=PPO,
                model_name="ppo",
                model_version="best",
                total_timesteps=1e6,
                model_args={},
                identifier="",
                # time_limit=15.0,
            )

        elif MODE == "multiple":
            for index, value in enumerate(params):
                pretty_time = time.strftime("%Y-%m-%d--%H-%M-%S", time.localtime())
                try:
                    controller.execute(
                        stable_baselines3_model=value["agent"],
                        model_name=value["name"],
                        model_version="alpha",
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
            print("All steps completed.")
            return

        states["prev_step"] = c
        states["current_step"] += 1
        states["total_steps"] = total_steps

        with open("steps.toml", "w") as f:
            toml.dump(states, f)

        controller.env.worldLoad(
            f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds/{next_step}_{MODEL_MODE.split('_')[0]}.wbt"
        )


if __name__ == "__main__":
    main()
