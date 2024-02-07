from controller import Robot

TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ["ds0", "ds1", "ds2", "ds3", "ds12", "ds13", "ds14", "ds15"]
for i in range(len(dsNames)):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
avoidObstacleCounter = 0

while robot.step(TIME_STEP) != -1:
    leftSpeed = 3.0
    rightSpeed = 3.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 3.0
        rightSpeed = -3.0
    else:  # read sensors
        if any(ds[i].getValue() > 800.0 for i in range(len(dsNames))):
            avoidObstacleCounter = 30
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
