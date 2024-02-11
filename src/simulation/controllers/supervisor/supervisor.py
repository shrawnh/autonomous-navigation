import toml
from controller import Supervisor
from _supervisor import MySupervisor

TIME_STEP = 32

# Load data from configuration files
with open("pioneer2.toml", "r") as toml_file:
    pioneer2_data = toml.load(toml_file)

with open("wooden_boxes.toml", "r") as toml_file:
    wooden_boxes_data = toml.load(toml_file)

robot = Supervisor()
_supervisor = MySupervisor()
_supervisor.wooden_boxes = wooden_boxes_data
root_node = robot.getRoot()
children_field = root_node.getField("children")
collision_threshold = 0.55  # adjust this value based on your simulation
_supervisor.set_robot(
    children_field,
    pioneer2_data["name"],
    pioneer2_data["position"],
    pioneer2_data["controller"],
)
_supervisor.set_wooden_boxes(children_field)

pioneer2_node = robot.getFromDef("PIONEER2")
wooden_box_nodes = [
    robot.getFromDef(wooden_boxes_data[box]["DEF"]) for box in wooden_boxes_data
]

robot_field = pioneer2_node.getField("color")

while robot.step(TIME_STEP) != -1:
    robot_position = pioneer2_node.getPosition()
    wooden_box_positions = [node.getPosition() for node in wooden_box_nodes]

    distances = _supervisor.calculate_distance(robot_position, wooden_box_positions)
    if any(distance < collision_threshold for distance in distances):
        red_color = [1, 0, 0]
        robot_field.setSFColor(red_color)
    else:
        green_color = [0.75, 1, 0.75]
        robot_field.setSFColor(green_color)
