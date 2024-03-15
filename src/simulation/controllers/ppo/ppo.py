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

    # Train the model
    # model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
    # model.learn(total_timesteps=1e5)

    # Save the trained model
    # model.save("ppo_wheeled_robot")

    # Load the trained model
    model = PPO.load("ppo_wheeled_robot")

    # Replay
    print("Training is finished, press `A` for replay...")
    env.wait_keyboard()

    # Use the model to make predictions
    observation, _ = env.reset()
    env.keyboard.enable(env.timestep)
    while env.keyboard.getKey() != ord("S"):
        action, _states = model.predict(observation, deterministic=True)
        observation, reward, done, truncated, info = env.step(action)
        print(f"Observation: {observation}")
        print(f"Reward: {reward}")
        print(f"Done: {done}")
        print(f"Truncated: {truncated}")
        print(f"Info: {info}")
        if done:
            observation, _ = env.reset()

    env.keyboard.disable()
    env.reset()


if __name__ == "__main__":
    main()
