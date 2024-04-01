import toml
import numpy as np
from numpy.typing import NDArray
from typing import Any
import gymnasium as gym
from controller import Supervisor, device, Field

RIGHT_WHEEL = "right wheel motor"
LEFT_WHEEL = "left wheel motor"
DS_NAMES = ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7"]
MAX_SPEED = 6.28
SENSETIVITY = 0.05
OBSTACLE_SIZE = 0.4
ROBOT_RADIUS = 0.225
RED = [1, 0, 0]
GREEN = [0.75, 1, 0.75]
SENSOR_THRESHOLD = 1023.0


class WheeledRobotEnv(Supervisor, gym.Env):
    def __init__(
        self,
        goal: NDArray[Any],
        wooden_boxes_data: dict[str, Any],
        grid_size: int = 3,
        ds_names: list[str] = DS_NAMES,
    ):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())
        self.goal = goal

        ### SPACES ###
        self.ds_names = ds_names
        high_obs = np.array(
            [SENSOR_THRESHOLD] * len(self.ds_names) + [MAX_SPEED, MAX_SPEED]
        )
        low_obs = np.array([0] * len(self.ds_names) + [-MAX_SPEED, -MAX_SPEED])
        self.observation_space = gym.spaces.Box(
            low=low_obs, high=high_obs, dtype=np.float32
        )
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.state = np.zeros(high_obs.shape[0])
        ### SPACES ###

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

        ### ENVIRONMENT SPECIFIC ###
        self.grid_size = grid_size
        ### ENVIRONMENT SPECIFIC ###

        ### ROBOT ###
        self.right_motor_device: device.Device = self.getDevice(RIGHT_WHEEL)
        self.left_motor_device: device.Device = self.getDevice(LEFT_WHEEL)
        self.distance_sensors: list[device.Device] = []
        self._initialise_robot()
        ### ROBOT ###

        ### COLLISION DETECTOR ###
        self.wooden_boxes_data = wooden_boxes_data
        ### COLLISION DETECTOR ###

        ### INFO ###
        self.num_goal_reached = 0
        self.num_collisions = 0
        self.start_time = self.getTime()
        self.speed = {
            "num_steps": 0,
            "total_speed": 0,
        }
        ### INFO ###

    ######################## ROBOT ###########################

    def _initialise_sensor(self) -> None:
        for name in self.ds_names:
            sensor = self.getDevice(name)
            sensor.enable(self.timestep)
            self.distance_sensors.append(sensor)

    def _initialise_motors(self) -> None:
        self.left_motor_device.setPosition(float("inf"))
        self.right_motor_device.setPosition(float("inf"))
        self.left_motor_device.setVelocity(0.0)
        self.right_motor_device.setVelocity(0.0)

    def _initialise_robot(self) -> None:
        self._initialise_motors()
        self._initialise_sensor()

    def _set_speed(self, left_speed: float, right_speed: float) -> None:
        self.left_motor_device.setVelocity(left_speed)
        self.right_motor_device.setVelocity(right_speed)
        self.right_speed = right_speed
        self.left_speed = left_speed

    ######################## ROBOT ###########################

    ################ Collision Detector ######################

    def _calculate_distance(self, position1: list, position2: list[list[int]]):
        distances = []
        for position in position2:
            # distances.append(
            #     math.sqrt(
            #         (position1[0] - position[0]) ** 2
            #         + (position1[1] - position[1]) ** 2
            #     )
            # )
            x_closest = max(
                position[0] - OBSTACLE_SIZE / 2,
                min(position1[0], position[0] + OBSTACLE_SIZE / 2),
            )
            y_closest = max(
                position[1] - OBSTACLE_SIZE / 2,
                min(position1[1], position[1] + OBSTACLE_SIZE / 2),
            )
            distances.append(
                ((x_closest - position1[0]) ** 2 + (y_closest - position1[1]) ** 2)
                ** 0.5
            )
        return distances

    def _change_robot_color(self, color: list[int]) -> None:
        robot_field = self.robot.getField("color")
        robot_field.setSFColor(color)

    def _is_out_of_bounds(self, position: list[int]) -> bool:
        bound = (self.grid_size / 2) - ROBOT_RADIUS - SENSETIVITY
        return abs(position[0]) > bound or abs(position[1]) > bound

    def _detect_collision(self):
        wooden_box_nodes = [
            self.getFromDef(self.wooden_boxes_data[box]["name"])
            for box in self.wooden_boxes_data
        ]
        robot_position = np.array(self.getSelf().getPosition())
        wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]
        distances = self._calculate_distance(robot_position, wooden_box_positions)
        is_collision = self._is_out_of_bounds(robot_position) or any(
            distance < (ROBOT_RADIUS + SENSETIVITY) for distance in distances
        )
        if is_collision:
            self._change_robot_color(RED)
            return True

        self._change_robot_color(GREEN)
        return False

    ################ Collision Detector ######################

    ############### ENVIRONMENT SPECIFIC #####################

    def _update_state(self):
        self.robot = self.getSelf()
        self.state = np.array(
            [self.distance_sensors[i].getValue() for i in range(len(self.ds_names))]
            + [
                self.left_motor_device.getVelocity(),
                self.right_motor_device.getVelocity(),
            ]
        )

    def _perform_action(self, action):
        left_speed, right_speed = MAX_SPEED * action
        self._set_speed(left_speed, right_speed)
        self.speed["num_steps"] += 1
        self.speed["total_speed"] += (left_speed + right_speed) / 2

    def wait_keyboard(self):
        self.keyboard.enable(self.timestep)
        while self.keyboard.getKey() != ord("A"):
            # print(ord("r"))
            super().step(self.timestep)
        self.keyboard.disable()

    def base_step(self, action):
        self._perform_action(action)
        super().step(self.timestep)
        robot_position = np.array(self.getSelf().getPosition())
        # distance_to_goal = np.linalg.norm(self.goal[:2] - robot_position[:2])
        distances_to_goals = [
            np.linalg.norm(goal[:2] - robot_position[:2]) for goal in self.goal
        ]
        # Find the minimum distance
        distance_to_nearest_goal = min(distances_to_goals)
        self._update_state()
        is_collision = self._detect_collision()
        done = False

        return is_collision, distance_to_nearest_goal, done

    ############### ENVIRONMENT SPECIFIC #####################

    ################## REWARD FUNCS ##########################

    def find_goal_no_collision(self, is_collision: bool, distance_to_goal: float):
        if is_collision:
            self.num_collisions += 1
            return -10, True

        elif distance_to_goal < 0.35:
            # print("Goal reached!")
            self.num_goal_reached += 1
            return 1000, True

        return 0, False

    ################## REWARD FUNCS ##########################

    ################## GYM SPECIFIC ##########################

    def reset(self, *args, **kwargs):
        self.simulationResetPhysics()  # This should call the robot class
        self.simulationReset()
        super().step(self.timestep)
        # print("\nResetting the environment...\n")
        # print(f"Number of collisions: {self.num_collisions}")
        # print(f"Number of goals reached: {self.num_goal_reached}")
        self.start_time = self.getTime()
        self._initialise_robot()
        super().step(self.timestep)

        # Open AI Gym generic: observation, info
        return np.zeros(self.state.shape[0]).astype(np.float32), {}

    def step(self, action):
        is_collision, distance_to_goal, _ = self.base_step(action)

        reward, done = self.find_goal_no_collision(is_collision, distance_to_goal)

        # if self.getTime() - self.start_time > 50:
        #     reward = -50
        #     done = True
        #     print("Time limit reached!")

        # Observation, reward, done, truncated, info
        return (
            self.state.astype(np.float32),
            reward,
            done,
            False,
            {
                "num_collisions": self.num_collisions,
                "num_goal_reached": self.num_goal_reached,
                "time_taken": self.getTime() - self.start_time,
                "avg_speed": self.speed["total_speed"] / self.speed["num_steps"],
            },
        )

    ################## GYM SPECIFIC ##########################


################################### UTILS ###################################


def run_model(env: WheeledRobotEnv, model: Any, verbose: bool = True):
    """
    Run the trained model

    env: WheeledRobotEnv
    model: the loaded trained model
    """
    print("Training is finished, press `A` for play...")
    env.wait_keyboard()

    observation, _ = env.reset()
    env.keyboard.enable(env.timestep)
    total_time = 0
    total_speed = 0
    total_episodes = 0
    while env.keyboard.getKey() != ord("S"):
        action, _states = model.predict(observation, deterministic=True)
        observation, _, done, _, info = env.step(action)
        # print(f"Observation: {observation}")
        # print(f"Reward: {reward}")
        # print(f"Done: {done}")
        # print(f"Truncated: {truncated}")
        # print(f"Info: {info}")
        if done:
            verbose and print_info(info, total_episodes, total_time, total_speed, False)
            total_time += info["time_taken"]
            total_speed += info["avg_speed"]
            total_episodes += 1
            observation, _ = env.reset()

    total_episodes != 0 and print_info(
        info, total_episodes, total_time, total_speed, True
    )
    env.keyboard.disable()
    env.reset()


def run_algorithm(env: WheeledRobotEnv, algorithm: Any, verbose: bool = True):
    observation, _ = env.reset()
    env.keyboard.enable(env.timestep)
    total_time = 0
    total_speed = 0
    total_episodes = 0
    while env.keyboard.getKey() != ord("S"):
        action = algorithm.get_action(observation[0 : len(env.ds_names)])
        observation, _, done, _, info = env.step(action)
        if done:
            verbose and print_info(info, total_episodes, total_time, total_speed, False)
            total_time += info["time_taken"]
            total_speed += info["avg_speed"]
            total_episodes += 1
            env.reset()

    total_episodes != 0 and print_info(
        info, total_episodes, total_time, total_speed, True
    )
    env.keyboard.disable()
    env.reset()


def print_info(
    info: dict[str, Any],
    total_episodes: int,
    total_time: float,
    total_speed: float,
    is_episode_done: bool,
):
    print("\n=====================================\n")
    print(f"Number of collisions: \t{info['num_collisions']}\t|")
    print(f"Number of goals reached: \t{info['num_goal_reached']}\t|")
    print(f"Time taken: \t\t\t{round(info['time_taken'], 3)}\t|")
    print(f"Average speed: \t\t{round(info['avg_speed'], 3)}\t|")
    print("\n=====================================\n")
    if is_episode_done:
        print("\n#####################################\n")
        print(f"Total episodes: \t\t{total_episodes}\t|")
        print(f"Avg time: \t\t\t{round(total_time / total_episodes, 3)}\t|")
        print(f"Average speed: \t\t{round(total_speed / total_episodes, 3)}\t|")
        print("\n#####################################\n")


def get_env_data_from_config(env_mode: str, model_mode: str, robot_sensors="front"):
    configs_path = (
        f"/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs"
    )
    with open(f"{configs_path}/{env_mode}/{model_mode}.toml", "r") as toml_file:
        data = toml.load(toml_file)
        goal = np.array(data["goal"])
        wooden_boxes_data = data["wooden_boxes"]
        grid_size = data["grid_size"]

    with open(f"{configs_path}/robot_sensors.toml", "r") as toml_file:
        robot_sensors = toml.load(toml_file)[f"{robot_sensors}"]

    return goal, wooden_boxes_data, grid_size, robot_sensors


################################### UTILS ###################################
