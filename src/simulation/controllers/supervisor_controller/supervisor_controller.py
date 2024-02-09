import toml
from controller import Supervisor
import math

TIME_STEP = 32

# wooden_boxes = {
#     "wooden_box_1": {
#         "name": "WB1",
#         "position": [0.5, 0, 0.2],
#         "DEF": "WB",
#     },
#     "wooden_box_2": {
#         "name": "WB2",
#         "position": [0, 0.5, 0.2],
#         "DEF": "WB",
#     },
#     "wooden_box_3": {
#         "name": "WB3",
#         "position": [0, -0.5, 0.2],
#         "DEF": "WB",
#     },
# }

# Read from TOML file
with open("wooden_boxes.toml", "r") as toml_file:
    wooden_boxes = toml.load(toml_file)


def calculate_distance(position1, position2):
    """Calculate Euclidean distance between two 3D points."""
    distances = []
    for position in position2:
        distances.append(
            math.sqrt(
                (position1[0] - position[0]) ** 2 + (position1[1] - position[1]) ** 2
            )
        )
    return distances


def construct_wooden_box_proto(name, position):
    return f"""WoodenBoxLoc {{
        name "{name}"
        translation {position[0]} {position[1]} {position[2]}
    }}"""


robot = Supervisor()  # create Supervisor instance

# get the root node of the robot
root_node = robot.getRoot()
children_field = root_node.getField("children")


# Collision threshold (depends on the size of your robot and the wooden box)
collision_threshold = 0.5  # adjust this value based on your simulation

for box in wooden_boxes:
    children_field.importMFNodeFromString(
        -1,
        construct_wooden_box_proto(
            wooden_boxes[box]["name"], wooden_boxes[box]["position"]
        ),
    )

i = 0

if i == 0:
    children_field.importMFNodeFromString(
        -1,
        """Pioneer2Robot {
                    translation -0.75 0 0
                    rotation 0 0 1 0
                    name "robot"
                    controller "pioneer_collision_avoidance"
                }""",
    )
    pioneer2_node = robot.getFromDef("PIONEER2")
    wooden_box_nodes = [
        robot.getFromDef(wooden_boxes[box]["DEF"]) for box in wooden_boxes
    ]

while robot.step(TIME_STEP) != -1:
    if i == 10:
        translation_field = pioneer2_node.getField("translation")
        # new_value = [0.5, 0, 0]
        # translation_field.setSFVec3f(new_value)  # set the initial position of BB-8

    robot_position = pioneer2_node.getPosition()
    wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]
    # print("Ball position: %f %f %f\n" % (position[0], position[1], position[2]))

    # Calculate the distance between the robot and the wooden box
    distances = calculate_distance(robot_position, wooden_box_positions)

    # Check for collision
    if any(distance < collision_threshold for distance in distances):
        print(f"Collision detected({i}!")
    # if position[2] < 0.2:
    #     red_color = [1, 0, 0]
    #     color_field.setSFColor(red_color)

    i += 1
