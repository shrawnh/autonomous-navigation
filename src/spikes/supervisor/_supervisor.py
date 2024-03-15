import math
from controller import Field


class MySupervisor:
    def __init__(self):
        self.wooden_boxes = {}

    def _construct_wooden_boxes_proto(self, name, position: list[int]):
        """Construct wooden boxes PROTO."""
        assert len(position) == 3, "position must be a 3D point"
        return f"""WoodenBoxLoc {{
                name "{name}"
                translation {position[0]} {position[1]} {position[2]}
            }}"""

    def set_robot(
        self,
        children_field: Field,
        name: str,
        position: list[int],
        controller="<none>",
        rotation="0 0 1 0",
    ):
        """Set robot in the simulation."""
        assert name != "", "name cannot be empty"
        assert position != [], "position cannot be empty"
        children_field.importMFNodeFromString(
            -1,
            f"""Pioneer2Robot {{
                translation {position[0]} {position[1]} {position[2]}
                rotation {rotation}
                name "{name}"
                controller "{controller}"
            }}""",
        )

    def set_wooden_boxes(self, children_field: Field):
        """Set wooden boxes in the simulation."""
        assert self.wooden_boxes != {}, "wooden_boxes cannot be empty"
        for box in self.wooden_boxes:
            assert self.wooden_boxes[box]["name"] != "", "name cannot be empty"
            assert self.wooden_boxes[box]["position"] != [], "position cannot be empty"
            children_field.importMFNodeFromString(
                -1,
                self._construct_wooden_boxes_proto(
                    self.wooden_boxes[box]["name"],
                    self.wooden_boxes[box]["position"],
                ),
            )

    def calculate_distance(self, position1: list, position2: list[list[int]]):
        """Calculate Euclidean distance between two 3D points."""
        assert position1 != [], "position1 cannot be empty"
        assert position2 != [], "position2 cannot be empty"
        distances = []
        for position in position2:
            distances.append(
                math.sqrt(
                    (position1[0] - position[0]) ** 2
                    + (position1[1] - position[1]) ** 2
                )
            )
        return distances
