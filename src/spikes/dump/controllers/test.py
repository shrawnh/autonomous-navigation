import numpy as np


rob_pos = np.array([[2, 3]])
coords_list = np.array(
    [
        [1, 2, 3],
        [6, 2, 1],
        [2, 4, 5],
    ]
)
coords_dict = [{"coordinates": coord, "passed": False} for coord in coords_list]
nearst_checkpoint = {"index": -1, "value": np.inf}

for _ in range(2):
    for i, coord in enumerate(coords_dict):
        if (
            not coord["passed"]
            and np.linalg.norm(coord["coordinates"][:2] - rob_pos[:2])
            < nearst_checkpoint["value"]
        ):
            nearst_checkpoint["value"] = np.linalg.norm(
                coord["coordinates"][:2] - rob_pos[:2]
            )
            nearst_checkpoint["index"] = i

    coords_dict[i]["passed"] = True
    print(nearst_checkpoint["value"])
    nearst_checkpoint = {"index": -1, "value": np.inf}
