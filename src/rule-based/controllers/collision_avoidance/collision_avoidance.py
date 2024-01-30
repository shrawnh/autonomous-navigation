from controller import Robot
import time

TIME_STEP = 32
FULLY_OUT_OF_RANGE = True

robot = Robot()


class WheelNames:
    "Class to store the names of the wheels."

    BACK_RIGHT_WHEEL = "back right wheel"
    BACK_LEFT_WHEEL = "back left wheel"
    FRONT_RIGHT_WHEEL = "front right wheel"
    FRONT_LEFT_WHEEL = "front left wheel"


class DistanceSensorNames:
    "Class to store the names of the distance sensors."

    SO_0 = "so0"
    SO_1 = "so1"
    SO_2 = "so2"
    SO_3 = "so3"
    SO_4 = "so4"
    SO_5 = "so5"
    SO_6 = "so6"
    SO_7 = "so7"
    SO_8 = "so8"
    SO_9 = "so9"
    SO_10 = "so10"
    SO_11 = "so11"
    SO_12 = "so12"
    SO_13 = "so13"
    SO_14 = "so14"
    SO_15 = "so15"


while robot.step(TIME_STEP) != -1:
    # Get time step of the current world.
    time_step = int(robot.getBasicTimeStep())

    # Set the position of the left and right wheel.
    robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setPosition(float("inf"))
    robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setPosition(float("inf"))
    robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setPosition(float("inf"))
    robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setPosition(float("inf"))

    #  Enable the distance sensors.
    # robot.getDevice(DistanceSensorNames.SO_0).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_1).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_2).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_3).enable(TIME_STEP)
    robot.getDevice(DistanceSensorNames.SO_4).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_5).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_6).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_7).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_8).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_9).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_10).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_11).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_12).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_13).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_14).enable(TIME_STEP)
    # robot.getDevice(DistanceSensorNames.SO_15).enable(TIME_STEP)

    # Get the values of the distance sensors.
    # so0 = robot.getDevice(DistanceSensorNames.SO_0).getValue()
    # so1 = robot.getDevice(DistanceSensorNames.SO_1).getValue()
    # so2 = robot.getDevice(DistanceSensorNames.SO_2).getValue()
    # so3 = robot.getDevice(DistanceSensorNames.SO_3).getValue()
    so4 = robot.getDevice(DistanceSensorNames.SO_4).getValue()
    # so5 = robot.getDevice(DistanceSensorNames.SO_5).getValue()
    # so6 = robot.getDevice(DistanceSensorNames.SO_6).getValue()
    # so7 = robot.getDevice(DistanceSensorNames.SO_7).getValue()
    # so8 = robot.getDevice(DistanceSensorNames.SO_8).getValue()
    # so9 = robot.getDevice(DistanceSensorNames.SO_9).getValue()
    # so10 = robot.getDevice(DistanceSensorNames.SO_10).getValue()
    # so11 = robot.getDevice(DistanceSensorNames.SO_11).getValue()
    # so12 = robot.getDevice(DistanceSensorNames.SO_12).getValue()
    # so13 = robot.getDevice(DistanceSensorNames.SO_13).getValue()
    # so14 = robot.getDevice(DistanceSensorNames.SO_14).getValue()
    # so15 = robot.getDevice(DistanceSensorNames.SO_15).getValue()

    so_values = [
        # so0,
        # so1,
        # so2,
        # so3,
        so4,
        # so5,
        # so6,
        # so7,
        # so8,
        # so9,
        # so10,
        # so11,
        # so12,
        # so13,
        # so14,
        # so15,
    ]

    # If the distance sensor detects an obstacle, turn the robot.
    if (
        so_values[0]
        > 400
        # so_values[0] < 80
        # or so_values[1] < 80
        # or so_values[2] < 80
        # or so_values[3] < 80
        # or so_values[4] < 80
        # or so_values[5] < 80
        # or so_values[6] < 80
        # or so_values[7] < 80
        # or so_values[8] > 80
        # or so_values[9] > 80
        # or so_values[10] > 80
        # or so_values[11] > 80
        # or so_values[12] > 80
        # or so_values[13] > 80
        # or so_values[14] > 80
        # or so_values[15] > 80
    ):
        robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setVelocity(4.0)
        robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setVelocity(-4.0)
        robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setVelocity(4.0)
        robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setVelocity(-4.0)
        # robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setVelocity(0)
        # robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setVelocity(0)
        # robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setVelocity(0)
        # robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setVelocity(0)
        FULLY_OUT_OF_RANGE = False
    elif not FULLY_OUT_OF_RANGE and so_values[0] < 400:
        for i in range(TIME_STEP):
            robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setVelocity(4.0)
            robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setVelocity(-4.0)
            robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setVelocity(4.0)
            robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setVelocity(-4.0)
        FULLY_OUT_OF_RANGE = True
    # If the distance sensor does not detect an obstacle, move the robot forward.
    else:
        robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setVelocity(4.0)
        robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setVelocity(4.0)
        robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setVelocity(4.0)
        robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setVelocity(4.0)

    # Set the velocity of the left and right wheel.
    # robot.getDevice(WheelNames.BACK_RIGHT_WHEEL).setVelocity(4.0)
    # robot.getDevice(WheelNames.BACK_LEFT_WHEEL).setVelocity(4.0)
    # robot.getDevice(WheelNames.FRONT_RIGHT_WHEEL).setVelocity(4.0)
    # robot.getDevice(WheelNames.FRONT_LEFT_WHEEL).setVelocity(4.0)
