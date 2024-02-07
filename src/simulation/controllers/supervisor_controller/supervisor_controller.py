from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get the root node of the robot
root_node = robot.getRoot()
children_field = root_node.getField("children")

i = 0

if i == 0:
    children_field.importMFNodeFromString(
        -1,
        """Pioneer2Robot {
                    translation -0.75 0 0
                    rotation 0 0 1 0
                    name "robot"
                    controller "<none>"
                }""",
    )
    pioneer2_node = robot.getFromDef("PIONEER2")

while robot.step(TIME_STEP) != -1:
    if i == 10:
        translation_field = pioneer2_node.getField("translation")
        new_value = [0.5, 0, 0]
        translation_field.setSFVec3f(new_value)  # set the initial position of BB-8

    # position = ball_node.getPosition()
    # print("Ball position: %f %f %f\n" % (position[0], position[1], position[2]))

    # if position[2] < 0.2:
    #     red_color = [1, 0, 0]
    #     color_field.setSFColor(red_color)

    i += 1
