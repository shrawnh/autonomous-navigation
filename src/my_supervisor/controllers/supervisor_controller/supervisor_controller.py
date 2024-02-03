from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
root_node = robot.getRoot()
children_field = root_node.getField("children")

children_field.importMFNodeFromString(-1, "DEF BALL Ball { translation 0 1 1 }")
ball_node = robot.getFromDef("BALL")
color_field = ball_node.getField("color")

i = 0

if i == 0:
    children_field.importMFNodeFromString(
        -1,
        """DEF BB-8 BB-8 { 
                    translation 2.5 0 -0.0415417
                    rotation 6.409104107001964e-08 -0.9999999999482442 -1.0173872596197402e-05 0.006927878769975853
                    controller "<none>"
                }""",
    )
    # bb8_node = robot.getFromTypeAndName("Robot", "BB-8")
    bb8_node = robot.getFromDef("BB-8")
    # translation_field = bb8_node.getField("translation")
    # # new_value = [2.5, 0, 0]
    # # translation_field.setSFVec3f(new_value)  # set the initial position of BB-8

while robot.step(TIME_STEP) != -1:
    # [CODE PLACEHOLDER 2]
    if i == 10:
        bb8_node.remove()

    if i == 20:
        children_field.importMFNodeFromString(-1, "Nao { translation 2.5 0 0.334 }")

    position = ball_node.getPosition()
    print("Ball position: %f %f %f\n" % (position[0], position[1], position[2]))

    if position[2] < 0.2:
        red_color = [1, 0, 0]
        color_field.setSFColor(red_color)

    i += 1
