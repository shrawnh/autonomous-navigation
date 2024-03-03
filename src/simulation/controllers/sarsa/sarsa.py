from controller import Robot
from _robot import MyRobot
from _sarsa import MySarsa
from _webots_env import WebotsEnv

TIME_STEP = 64

qfile = open("qlog.txt", "w")
rewardf = open("reward.txt", "w")
qsumfile = open("qsum.txt", "w")

robot = Robot()
_robot = MyRobot(robot)
_webots_env = WebotsEnv(robot=_robot)
_sarsa = MySarsa(_webots_env)

while robot.step(TIME_STEP) != -1:
    _sarsa.train()
    # _sarsa.test()
    # print(_sarsa.q_table)
