import gymnasium as gym
from controller import Supervisor, device
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import time


class WheeledRobotEnv(Supervisor, gym.Env):
    def __init__(self):
        super().__init__()
        # Define observation space
        self.collisions = 0
        self.time_exploring = 0
        high = np.array(
            [
                100.0,  # distance sensor 0
                100.0,  # distance sensor 1
                100.0,  # distance sensor 2
                100.0,  # distance sensor 3
                100.0,  # distance sensor 12
                100.0,  # distance sensor 13
                100.0,  # distance sensor 14
                100.0,  # distance sensor 15
            ],
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Box(high, high * 2, dtype=np.float32)
        self.state = None
        self.max_speed = 6.28
        self.turn_speed = 0.5
        self.right_motor_device: device.Device = self.getDevice("right wheel motor")
        self.left_motor_device: device.Device = self.getDevice("left wheel motor")
        self.right_speed = 0.0
        self.left_speed = 0.0
        self.spec = gym.envs.registration.EnvSpec(
            id="WebotsEnv-v0", max_episode_steps=1000
        )
        # Define action space
        self.action_space = gym.spaces.Discrete(5)
        # Environment specific
        self.__timestep = int(self.getBasicTimeStep())
        self.distance_sensors = []
        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.__timestep)

    def _set_speed(self, left_speed: float, right_speed: float) -> None:
        self.left_motor_device.setVelocity(left_speed)
        self.right_motor_device.setVelocity(right_speed)
        self.right_speed = right_speed
        self.left_speed = left_speed

    def stop(self) -> None:
        self._set_speed(0, 0)

    def move_forward(self) -> None:
        self._set_speed(
            self.max_speed * self.turn_speed, self.max_speed * self.turn_speed
        )

    def move_backward(self) -> None:
        self._set_speed(
            -self.max_speed * self.turn_speed, -self.max_speed * self.turn_speed
        )

    def turn_left(self) -> None:
        self._set_speed(
            -self.max_speed * self.turn_speed, self.max_speed * self.turn_speed
        )

    def turn_right(self) -> None:
        self._set_speed(
            self.max_speed * self.turn_speed, -self.max_speed * self.turn_speed
        )

    def wait_keyboard(self):
        while self.keyboard.getKey() != ord("y"):
            super().step(self.__timestep)

    def reset(self, *args, **kwargs):
        # Reset the state of the environment to an initial state
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.__timestep)
        print("Resetting the environment...")
        self.collisions = 0
        self.time_exploring = 0
        self.__wheels = []
        for name in ["right wheel motor", "left wheel motor"]:
            motor = self.getDevice(name)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.__wheels.append(motor)

        self.distance_sensors = []
        for name in ["ds0", "ds1", "ds2", "ds3", "ds12", "ds13", "ds14", "ds15"]:
            sensor = self.getDevice(name)
            sensor.enable(self.__timestep)
            self.distance_sensors.append(sensor)

        super().step(self.__timestep)

        # Open AI Gym generic
        return np.array([0, 0, 0, 0, 0, 0, 0, 0]).astype(np.float32), {}

    def perform_action(self, action):
        if action == 0:
            self.move_forward()
        elif action == 1:
            self.move_backward()
        elif action == 2:
            self.stop()
        elif action == 3:
            self.turn_left()
        elif action == 4:
            self.turn_right()
        else:
            raise ValueError("Invalid action")

    def step(self, action):
        # Apply the action to the robot
        self.perform_action(action)
        super().step(self.__timestep)

        # Update the environment state
        # self.robot = self.getSelf()
        self.state = np.array(
            [
                self.distance_sensors[0].getValue(),
                self.distance_sensors[1].getValue(),
                self.distance_sensors[2].getValue(),
                self.distance_sensors[3].getValue(),
                self.distance_sensors[4].getValue(),
                self.distance_sensors[5].getValue(),
                self.distance_sensors[6].getValue(),
                self.distance_sensors[7].getValue(),
                # self.robot.getVelocity()[0],  # linear velocity x
            ]
        )

        done = any(sensor > 400 for sensor in self.state[:8])
        # Calculate reward
        reward = 0 if done else 1

        # Optional comments
        # Check if the episode is done
        # Optionally return additional info
        # return observation, reward, done, info

        return self.state.astype(np.float32), reward, done, {}, {}


def main():
    # Initialize the environment
    env = WheeledRobotEnv()
    # check_env(env)
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
