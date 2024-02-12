import gymnasium as gym

class WebotsEnv:
    def __init__(self):
        self.observation_space = gym.spaces.Discrete(10)
        self.action_space = gym.spaces.Discrete(4)
        self.observation_space.n = 10
        self.action_space.n = 4

    def reset(self):
        return 0

    def step(self, action):
        return 0, 0, False, 0

    def render(self):
        pass

    def close(self):
        pass