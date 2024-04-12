from stable_baselines3 import TD3
from myenv.mycontroller import MyController
import time

# check_params = TD3("MlpPolicy", "", verbose=1, buffer_size=10000000)

array = [
    {
        "learning_starts": 1000,
        "batch_size": 256,
        "tau": 0.005,
        "gamma": 0.99,
        "train_freq": 2,
        "gradient_steps": 1,
        "policy_delay": 2,
    },
    {
        "learning_starts": 500,
        "batch_size": 128,
        "tau": 0.02,
        "gamma": 0.98,
        "train_freq": 1,
        "gradient_steps": -1,
        "policy_delay": 2,
    },
    {
        "learning_starts": 1500,
        "batch_size": 64,
        "tau": 0.005,
        "gamma": 0.99,
        "train_freq": 5,
        "gradient_steps": 2,
        "policy_delay": 2,
    },
    {
        "learning_starts": 2000,
        "batch_size": 100,
        "tau": 0.01,
        "gamma": 0.95,
        "train_freq": 3,
        "gradient_steps": 5,
        "policy_delay": 1,
    },
    {
        "learning_starts": 1000,
        "batch_size": 256,
        "tau": 0.01,
        "gamma": 0.97,
        "train_freq": 4,
        "gradient_steps": 3,
        "policy_delay": 1,
    },
    {
        "learning_starts": 800,
        "batch_size": 200,
        "tau": 0.008,
        "gamma": 0.96,
        "train_freq": 2,
        "gradient_steps": 4,
        "policy_delay": 3,
    },
    {
        "learning_starts": 1200,
        "batch_size": 150,
        "tau": 0.005,
        "gamma": 0.98,
        "train_freq": 1,
        "gradient_steps": 6,
        "policy_delay": 2,
    },
    {
        "learning_starts": 2500,
        "batch_size": 300,
        "tau": 0.003,
        "gamma": 0.99,
        "train_freq": 1,
        "gradient_steps": 8,
        "policy_delay": 4,
        "target_policy_noise": 0.2,
        "target_noise_clip": 0.5,
    },
    {
        "learning_starts": 500,
        "batch_size": 64,
        "tau": 0.02,
        "gamma": 0.95,
        "train_freq": 3,
        "gradient_steps": -1,
        "policy_delay": 1,
        "target_policy_noise": 0.3,
        "target_noise_clip": 0.6,
    },
    {
        "learning_starts": 3000,
        "batch_size": 350,
        "tau": 0.001,
        "gamma": 0.92,
        "train_freq": 5,
        "gradient_steps": 10,
        "policy_delay": 5,
        "target_policy_noise": 0.1,
        "target_noise_clip": 0.4,
    },
]


def main():
    controller = MyController(
        model_mode="train_save",
        model_version="alpha",
        version_mode="new",
        model_name="td3_wheeled_robot",
        env_mode="step-1",
        robot_sensors="front",
    )

    # for index, value in enumerate(array):
    #     try:
    #         controller.main(TD3, 1e5, value, f"_{index}_{time.time()}")
    #     except Exception as e:
    #         print(e)
    #         continue
    controller.execute(TD3, 1e5, array[3], f"_{3}_{time.time()}")


if __name__ == "__main__":
    main()
