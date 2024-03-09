import gymnasium as gym
from controller import Supervisor, device
import numpy as np
from stable_baselines3 import PPO, A2C
import math
import toml

with open("wooden_boxes.toml", "r") as toml_file:
    wooden_boxes_data = toml.load(toml_file)


class WheeledRobotEnv(Supervisor, gym.Env):
    def __init__(self):
        super().__init__()
        # Define observation space
        self.collision = False
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
                3.14,  # left wheel speed
                3.14,  # right wheel speed
                0,  # time exploring
            ],
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Box(high, high * 2, dtype=np.float32)
        self.state = None
        self.max_speed = 6.28  # rad/s
        self.current_speed = self.max_speed * 0.5  # 50% of the max speed
        self.right_motor_device: device.Device = self.getDevice("right wheel motor")
        self.left_motor_device: device.Device = self.getDevice("left wheel motor")
        self.right_speed = 0.0
        self.left_speed = 0.0
        self.spec = gym.envs.registration.EnvSpec(
            id="WebotsEnv-v0", max_episode_steps=1000
        )
        # Define action space
        # self.action_space = gym.spaces.Discrete(5)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )
        # Environment specific
        self.__timestep = int(self.getBasicTimeStep())
        self.distance_sensors = []
        self.goal = np.array([0.77, -0.13, 0.0])  # Define the goal position
        self.wooden_boxes = wooden_boxes_data
        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.__timestep)

    def _set_current_speed(self, speed: float) -> None:
        if speed < 0 or speed > 1:
            raise ValueError(
                "Speed must be between 0 and 1, as it represents a percentage of the max speed."
            )
        self.current_speed = self.max_speed * speed

    def _set_speed(self, left_speed: float, right_speed: float) -> None:
        self.left_motor_device.setVelocity(left_speed)
        self.right_motor_device.setVelocity(right_speed)
        self.right_speed = right_speed
        self.left_speed = left_speed

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
        return np.zeros(11).astype(np.float32), {}

    def _calculate_distance(self, position1: list, position2: list[list[int]]):
        """Calculate Euclidean distance between two 3D points."""
        # assert position1 != [], "position1 cannot be empty"
        # assert position2 != [], "position2 cannot be empty"
        distances = []
        for position in position2:
            distances.append(
                math.sqrt(
                    (position1[0] - position[0]) ** 2
                    + (position1[1] - position[1]) ** 2
                )
            )
        return distances

    def _change_robot_color(self, color: list[int]) -> None:
        robot_field = self.robot.getField("color")
        robot_field.setSFColor(color)

    def _detect_collision(self):
        """Detect collision between robot and wooden boxes."""
        wooden_box_nodes = [
            self.getFromDef(wooden_boxes_data[box]["name"]) for box in wooden_boxes_data
        ]
        robot_position = np.array(self.getSelf().getPosition())
        wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]
        distances = self._calculate_distance(robot_position, wooden_box_positions)
        if any(distance < 0.55 for distance in distances):
            self.collision = True
            self._change_robot_color([1, 0, 0])
        else:
            self.collision = False
            self._change_robot_color([0.75, 1, 0.75])

    def perform_action(self, action):
        left_speed, right_speed = self.max_speed * action
        self._set_speed(left_speed, right_speed)

    def step(self, action):
        # Apply the action to the robot
        self.perform_action(action)
        super().step(self.__timestep)

        robot_position = np.array(self.getSelf().getPosition())
        distance_to_goal = np.linalg.norm(self.goal[:2] - robot_position[:2])

        # Update the environment state
        self.robot = self.getSelf()
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
                self.left_motor_device.getVelocity(),
                self.right_motor_device.getVelocity(),
                self.time_exploring,
            ]
        )

        self._detect_collision()
        done = False
        if self.collision or self.time_exploring > 60000:
            done = True
        if distance_to_goal < 0.5:
            reward = 1000
            done = True
        elif self.collision:
            print("Collision detected!")
            reward = -100
        elif self.time_exploring > 4000 and self.time_exploring % 4000 == 0:
            increment = self.time_exploring // 4000
            print("Time: ", self.time_exploring, "Increment: ", increment)
            reward = -2 * increment
        else:
            reward = 0
        self.time_exploring += 1

        return self.state.astype(np.float32), reward, done, {}, {}


def main():
    # Initialize the environment
    env = WheeledRobotEnv()
    model = PPO("MlpPolicy", env, n_steps=2**32, verbose=1)
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
