import gymnasium as gym
from _robot import MyRobot

class WebotsEnv:
    def __init__(self, robot: MyRobot):
        self.observation_space = gym.spaces.Discrete(10)
        self.action_space = gym.spaces.Discrete(4)
        self.observation_space.n = 10
        self.action_space.n = 4
        self.robot = robot

    def reset(self):
        return 0

    def step(self, action):
        match action:
            case 0:
                self.robot.move_forward()
            case 1:
                self.robot.move_backward()
            case 2:
                self.robot.turn_left()
            case 3:
                self.robot.turn_right()
        if self.robot.obstacle_detected():
            reward = -10
        else:
            reward = 1
        return 0, reward, False, 0

    def render(self):
        pass

    def close(self):
        pass