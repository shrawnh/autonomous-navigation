from myenv.myenv import WheeledRobotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import toml
import numpy as np

with open("wooden_boxes.toml", "r") as toml_file:
    wooden_boxes_data = toml.load(toml_file)

goal = np.array([0.77, -0.13, 0.0])


def main():
    # Initialize the environment
    env = WheeledRobotEnv(goal, wooden_boxes_data)
    check_env(env, warn=True)
    model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
    model.learn(total_timesteps=1e5)
    # model.save("ppo_wheeled_robot")

    # Replay
    print("Training is finished, press `Y` for replay...")
    env.wait_keyboard()

    obs = env.reset()
    for _ in range(10):
        action, _states = model.predict(obs)
        obs, reward, done, info = env.step(action)
        print(obs, reward, done, info)
        if done:
            obs = env.reset()


if __name__ == "__main__":
    main()
