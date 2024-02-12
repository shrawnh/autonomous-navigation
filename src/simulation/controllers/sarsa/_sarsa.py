import numpy as np, random
from _webots_env import WebotsEnv

class MySarsa():
    def __init__(self, env: WebotsEnv, alpha=0.3, gamma=0.8, epsilon=0.9, num_episodes=1000, num_states=10, num_actions=4, actions_set=[]):
        self.env = env
        self.alpha = alpha                      # Learning rate
        self.gamma = gamma                      # Discount factor
        self.epsilon =  epsilon                 # Exploration rate
        self.num_episodes = num_episodes
        self.num_states = num_states
        self.num_actions = num_actions
        self.actions_set = actions_set
        self.rewards_table = []
        self.q_table = []
        self.total_reward = 0
        self.q_table = np.zeros((env.observation_space.n, env.action_space.n))

    def _random(self) -> float:
        return float(random.randint(0, 100) / 100)

    def _decay(self, parameter) -> float:
        # TODO why 0.98
        return float(parameter) * 0.98 
    
    def _get_state(self) -> int:
        return random.randint(0, self.num_states - 1)

    # def initialize_q_table(self) -> None:
    #     for _ in range(self.num_states):
    #         self.q_table.append([0] * self.num_actions)

    def initialize_rewards_table(self) -> None:
        temp = []
        for _ in range(self.num_states):
            temp.append([-10, -2, -1, 10])
        if len(temp) != self.num_actions:
            raise ValueError(f"rewards_table must have {self.num_actions} values")
        self.rewards_table = temp
        
    def choose_action(self, state):
        if np.random.uniform(0, 1) < self.epsilon:
            return self.env.action_space.sample()  # Explore action space
        else:
            return np.argmax(self.q_table[state])  # Exploit learned values

    def argmax(self, S):
        max_index = 0
        max_value = self.q_table[S, 0]

        for index, value in enumerate(self.q_table[S]):
            if value > max_value:
                max_index = index
                max_value = value
        return max_index
    
    def update_q(self, state, action, reward, next_state, next_action):
        predict = self.q_table[state, action]
        target = reward + self.gamma * self.q_table[next_state, next_action]
        self.q_table[state, action] += self.alpha * (target - predict)

    def train(self, num_episodes=1000):
        for _ in range(num_episodes):
            state = self.env.reset()
            action = self.choose_action(state)

            for _ in range(100):
                next_state, reward, done, _ = self.env.step(action)
                next_action = self.choose_action(next_state)
                self.update_q(state, action, reward, next_state, next_action)

                if done:
                    break

                state = next_state
                action = next_action

    def test(self):
        state = self.env.reset()
        self.env.render()
        done = False
        while not done:
            action = np.argmax(self.q_table[state])
            state, _, done, _ = self.env.step(action)
            self.env.render()
        self.env.close()
