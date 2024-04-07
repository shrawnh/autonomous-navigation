from stable_baselines3 import TD3
from myenv.mycontroller import MyController


def main():
    # model = TD3("MlpPolicy", env, verbose=1, buffer_size=10000000)
    # TD3.learn(total_timesteps=1000, log_interval=10)
    controller = MyController(
        model_mode="train_save",
        model_version="alpha",
        version_mode="new",
        model_name="td3_wheeled_robot",
        env_mode="step-2",
        robot_sensors="front",
    )
    controller.main(TD3, 1e5, {"learning_rate": 0.002})


if __name__ == "__main__":
    main()
