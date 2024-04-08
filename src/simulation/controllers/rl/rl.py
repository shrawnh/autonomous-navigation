from stable_baselines3 import TD3, PPO
from myenv.mycontroller import MyController
import time

params = [
    {"id": 0, "agent": TD3, "name": "td3", "args": {}},
    {"id": 1, "agent": TD3, "name": "td3", "args": {}},
    {"id": 2, "agent": PPO, "name": "ppo", "args": {}},
    {"id": 3, "agent": PPO, "name": "ppo", "args": {}},
]


def check_all_ids_are_unique(params):
    ids = [param["id"] for param in params]
    all_unique = len(ids) == len(set(ids))
    if not all_unique:
        not_unique_ids = [id for id in ids if ids.count(id) > 1]
        raise ValueError(f"Agent ids are not unique: {not_unique_ids}")
    return all_unique


def main():
    if not check_all_ids_are_unique(params):
        return
    controller = MyController(
        model_mode="train_save",
        model_version="alpha",
        version_mode="new",
        env_mode="step-1",
        robot_sensors="front",
    )

    for index, value in enumerate(params):
        try:
            controller.main(
                value["agent"],
                value["name"],
                1000,
                value["args"],
                f"_{value['id']}_{index}_{time.time()}",
            )
        except Exception as e:
            print(e)
            continue


if __name__ == "__main__":
    main()
