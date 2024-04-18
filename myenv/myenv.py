import toml
import numpy as np
from numpy.typing import NDArray
from typing import Any
import gymnasium as gym
from controller import Supervisor, device, Field
import math

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
TIME_LIMIT = 150.0


class WheeledRobotEnv(Supervisor, gym.Env):
    def __init__(
        self,
        goal: NDArray[Any],
        wooden_boxes_data: dict[str, Any],
        grid_size: int = 3,
        ds_names: list[str] = DS_NAMES,
        verbose: bool = True,
        checkpoints=None,
    ):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())
        self.goal = goal
        # self.checkpoints = checkpoints
        self.checkpoints = [
            {"coordinates": coord, "passed": False} for coord in checkpoints
        ]
        self.time_limit = TIME_LIMIT

        ### SPACES ###
        self.ds_names = ds_names
        high_obs = np.array(
            [SENSOR_THRESHOLD] + [MAX_SPEED, MAX_SPEED] + [self.time_limit]  # type: ignore
        )
        low_obs = np.array([0] + [-MAX_SPEED, -MAX_SPEED] + [0.0])  # type: ignore
        self.observation_space = gym.spaces.Box(
            low=low_obs, high=high_obs, dtype=np.float32
        )
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.state = np.zeros(high_obs.shape[0])
        ### SPACES ###

        self.unwrapped.keyboard = self.getKeyboard()
        self.unwrapped.keyboard.enable(self.unwrapped.timestep)

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
        self.verbose = verbose
        self.num_goal_reached = 0
        self.num_collisions = 0
        self.num_time_limit_reached = 0
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
            sensor.enable(self.unwrapped.timestep)
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

    ################ COLLISION DETECTOR ######################

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

    def _determine_collision_side(
        self, robot_x, robot_y, robot_orientation, obstacle_x, obstacle_y
    ):
        # Calculate angle from robot to obstacle
        angle_to_obstacle = math.atan2(obstacle_y - robot_y, obstacle_x - robot_x)
        # Normalize angles
        relative_angle = (angle_to_obstacle - robot_orientation + 2 * math.pi) % (
            2 * math.pi
        )
        if self.verbose:
            print(f"Robot position: {robot_x, robot_y}")
            print(f"Robot orientation: {robot_orientation}")
            print(f"Obstacle position: {obstacle_x, obstacle_y}")
            print(f"Angle to obstacle: {angle_to_obstacle}")
            print(f"Relative angle: {relative_angle}")

        # Determine collision side based on the relative angle
        if 0 <= relative_angle < math.pi / 4 or relative_angle > 7 * math.pi / 4:
            return "Front"
        elif math.pi / 4 <= relative_angle < 3 * math.pi / 4:
            return "Right"
        elif 3 * math.pi / 4 <= relative_angle < 5 * math.pi / 4:
            return "Rear"
        elif 5 * math.pi / 4 <= relative_angle < 7 * math.pi / 4:
            return "Left"

    def _change_robot_color(self, color: list[int]) -> None:
        robot_field = self.robot.getField("color")
        robot_field.setSFColor(color)

    def _is_out_of_bounds(self, position: list[int]) -> bool:
        bound = (self.grid_size / 2) - ROBOT_RADIUS - SENSETIVITY
        bound_position = (self.grid_size / 2) * (position[0] / abs(position[0]))
        if abs(position[0]) > bound:
            self.verbose and print("Out of bounds in x direction")
            return True, bound_position, None
        elif abs(position[1]) > bound:
            self.verbose and print("Out of bounds in y direction")
            return True, None, bound_position
        return False, None, None

    def _detect_collision(self):
        wooden_box_nodes = [
            self.getFromDef(self.wooden_boxes_data[box]["name"])
            for box in self.wooden_boxes_data
        ]
        robot_position = np.array(self.getSelf().getPosition())
        wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]
        distances = self._calculate_distance(robot_position, wooden_box_positions)
        is_out_of_bounds, x, y = self._is_out_of_bounds(robot_position)
        is_obstacle_collision = any(distance < (ROBOT_RADIUS + SENSETIVITY) for distance in distances)  # type: ignore
        if is_out_of_bounds:
            self._change_robot_color(RED)
            collision_side = self._determine_collision_side(
                robot_position[0],
                robot_position[1],
                self.getSelf().getOrientation()[3],
                x if x is not None else robot_position[0],
                y if y is not None else robot_position[1],
            )
            return True, collision_side
        elif is_obstacle_collision:
            self._change_robot_color(RED)
            collision_side = self._determine_collision_side(
                robot_position[0],
                robot_position[1],
                self.getSelf().getOrientation()[3],
                wooden_box_positions[distances.index(min(distances))][0],
                wooden_box_positions[distances.index(min(distances))][1],
            )
            return True, collision_side

        self._change_robot_color(GREEN)
        return False, None

    ################ COLLISION DETECTOR ######################

    ############### ENVIRONMENT SPECIFIC #####################

    def _update_state(self):
        self.robot = self.getSelf()
        sensor_values = [
            self.distance_sensors[i].getValue() for i in range(len(self.ds_names))
        ]
        mean_sensor_value = sum(sensor_values) / len(sensor_values)
        self.state = np.array(
            # [self.distance_sensors[i].getValue() for i in range(len(self.ds_names))]
            [mean_sensor_value]
            + [
                self.left_motor_device.getVelocity(),
                self.right_motor_device.getVelocity(),
            ]
            + [self.getTime() - self.start_time]
        )

    def _perform_action(self, action):
        left_speed, right_speed = MAX_SPEED * action
        self._set_speed(left_speed, right_speed)
        self.speed["num_steps"] += 1
        self.speed["total_speed"] += (left_speed + right_speed) / 2

    def wait_keyboard(self):
        self.unwrapped.keyboard.enable(self.unwrapped.timestep)
        while self.unwrapped.keyboard.getKey() != ord("A"):
            # print(ord("r"))
            super().step(self.unwrapped.timestep)
        self.unwrapped.keyboard.disable()

    def reset_env_info(self):
        self.num_goal_reached = 0
        self.num_collisions = 0
        self.num_time_limit_reached = 0
        self.start_time = self.getTime()
        self.speed = {
            "num_steps": 0,
            "total_speed": 0,
        }

    def base_step(self, action):
        self._perform_action(action)
        super().step(self.unwrapped.timestep)
        robot_position = np.array(self.getSelf().getPosition())
        # distance_to_goal = np.linalg.norm(self.goal[:2] - robot_position[:2])
        distances_to_goals = [
            np.linalg.norm(goal[:2] - robot_position[:2]) for goal in self.goal
        ]
        # Find the minimum distance
        distance_to_nearest_goal = min(distances_to_goals)
        self._update_state()
        is_collision, collision_side = self._detect_collision()
        done = False

        return is_collision, collision_side, distance_to_nearest_goal, done

    def advanced_step(self, action):
        self._perform_action(action)
        super().step(self.unwrapped.timestep)
        robot_position = np.array(self.getSelf().getPosition())
        # distance_to_goal = np.linalg.norm(self.goal[:2] - robot_position[:2])
        distances_to_goals = [
            np.linalg.norm(goal[:2] - robot_position[:2]) for goal in self.goal
        ]
        # Find the minimum distance
        distance_to_nearest_goal = min(distances_to_goals)
        self._update_state()
        is_collision, collision_side = self._detect_collision()
        # Reached checkpoint
        distance_to_nearest_checkpiont = {"index": -1, "value": np.inf}
        if len(self.checkpoints) > 0:
            for i, coord in enumerate(self.checkpoints):
                if coord["passed"] != False and (np.linalg.norm(coord["coordinates"][:2] - robot_position[:2]) < distance_to_nearest_checkpiont["value"]):  # type: ignore
                    distance_to_nearest_checkpiont["value"] = np.linalg.norm(
                        coord["coordinates"][:2] - robot_position[:2]
                    )
                    distance_to_nearest_checkpiont["index"] = i

            self.checkpoints[distance_to_nearest_checkpiont["index"]]["passed"] = True

        return (
            is_collision,
            collision_side,
            distance_to_nearest_goal,
            distance_to_nearest_checkpiont["value"],
        )

    ############### ENVIRONMENT SPECIFIC #####################

    ################## REWARD FUNCS ##########################

    def find_goal_no_collision(self, is_collision: bool, distance_to_goal: float):
        if is_collision:
            self.num_collisions += 1
            return -100, True

        elif distance_to_goal < 0.35:
            self.num_goal_reached += 1
            return 2000, True

        elif self.getTime() - self.start_time > self.time_limit:
            self.num_time_limit_reached += 1
            self.verbose and print("Time limit reached!")
            return -50, True

        return 0, False

    def basic_reward(
        self, is_collision: bool, distance_to_goal: float, collision_side: str
    ):
        if is_collision:
            self.num_collisions += 1
            if collision_side == "Front":
                self.verbose and print("Front collision")
                return -8, True
            self.verbose and print(f"{collision_side} collision")
            return -10, True
        elif distance_to_goal < 0.35:
            self.num_goal_reached += 1
            return 10, True
        elif self.getTime() - self.start_time > self.time_limit:
            self.num_time_limit_reached += 1
            return 0, True
        return -0.005, False

    def advanced_reward(
        self,
        is_collision: bool,
        distance_to_goal: float,
        collision_side: str,
        distance_to_nearest_checkpiont: float,
    ):
        if is_collision:
            self.num_collisions += 1
            if collision_side == "Front":
                self.verbose and print("Front collision")
                return -8, True
            self.verbose and print(f"{collision_side} collision")
            return -10, True
        elif distance_to_nearest_checkpiont < 0.1:
            self.verbose and print(f"Checkpoints {self.checkpoints}")
            return 1, False
        elif distance_to_goal < 0.35:
            self.num_goal_reached += 1
            return 10, True
        elif self.getTime() - self.start_time > self.time_limit:
            self.num_time_limit_reached += 1
            return 0, True
        return -0.005, False

    ################## REWARD FUNCS ##########################

    ################## GYM SPECIFIC ##########################

    def reset(self, *args, **kwargs):
        if self.verbose:
            print("#############################################")
            print(f"Number of collisions: {self.num_collisions}")
            print(f"Number of goals reached: {self.num_goal_reached}")
            print(f"Number of time limit reached: {self.num_time_limit_reached}")
            print("#############################################")
        self.simulationResetPhysics()  # This should call the robot class
        self.simulationReset()
        super().step(self.unwrapped.timestep)
        self.start_time = self.getTime()
        self._initialise_robot()
        super().step(self.unwrapped.timestep)

        # Open AI Gym generic: observation, info
        return np.zeros(self.state.shape[0]).astype(np.float32), {}

    def step(self, action):
        # is_collision, collision_side, distance_to_goal, _ = self.base_step(action)
        (
            is_collision,
            collision_side,
            distance_to_goal,
            distance_to_nearest_checkpiont,
        ) = self.advanced_step(action)

        if distance_to_nearest_checkpiont is None:
            reward, done = self.basic_reward(
                is_collision, distance_to_goal, collision_side
            )
        else:
            reward, done = self.advanced_reward(
                is_collision,
                distance_to_goal,
                collision_side,
                distance_to_nearest_checkpiont,
            )

        # Observation, reward, done, truncated, info
        return (
            self.state.astype(np.float32),
            reward,
            done,
            False,
            {
                "num_collisions": self.num_collisions,
                "num_goal_reached": self.num_goal_reached,
                "num_time_limit_reached": self.num_time_limit_reached,
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
    env.unwrapped.wait_keyboard()

    observation, _ = env.reset()
    env.unwrapped.keyboard.enable(env.unwrapped.timestep)
    total_time = 0
    total_speed = 0
    total_episodes = 0
    while env.unwrapped.keyboard.getKey() != ord("S"):
        action, _states = model.predict(observation)
        observation, _, done, _, info = env.step(action)
        if done:
            verbose and print_info(info, total_episodes, total_time, total_speed, False)
            total_time += info["time_taken"]
            total_speed += info["avg_speed"]
            total_episodes += 1
            observation, _ = env.reset()

    total_episodes != 0 and print_info(
        info, total_episodes, total_time, total_speed, True
    )
    env.unwrapped.keyboard.disable()
    env.reset()


def run_algorithm(env: WheeledRobotEnv, algorithm: Any, verbose: bool = True):
    observation, _ = env.reset()
    env.unwrapped.keyboard.enable(env.unwrapped.timestep)
    total_time = 0
    total_speed = 0
    total_episodes = 0
    while env.unwrapped.keyboard.getKey() != ord("S"):
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
    env.unwrapped.keyboard.disable()
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
    print(f"Time limits reached: \t\t{info['num_time_limit_reached']}\t|")
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
        checkpoints = np.array(data["checkpoints"])
        wooden_boxes_data = data["wooden_boxes"]
        grid_size = data["grid_size"]

    with open(f"{configs_path}/robot_sensors.toml", "r") as toml_file:
        robot_sensors = toml.load(toml_file)[f"{robot_sensors}"]

    return goal, wooden_boxes_data, grid_size, robot_sensors, checkpoints


def model_name_check(env: WheeledRobotEnv, model_name: str, version: str = ""):
    if version == "alpha":
        return f"{model_name}-{version}"
    elif version == "best":
        return "best_model"
    env.reset()
    env.unwrapped.keyboard.enable(env.unwrapped.timestep)
    print("Do you want to OVERWRITE the stable model? (Y/N)")
    while True:
        env.step(np.array([0.0, 0.0]))  # Keep the simulation running
        key = env.unwrapped.keyboard.getKey()
        if key == ord("Y"):
            env.unwrapped.keyboard.disable()
            return f"{model_name}"
        elif key == ord("N"):
            env.unwrapped.keyboard.disable()
            return None


################################### UTILS ###################################
