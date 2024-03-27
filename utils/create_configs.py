import os

worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

configs_dir = (
    "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs_test"
)

os.makedirs(configs_dir, exist_ok=True)

mode = input(f"Enter mode (train/test): ")
assert mode in ["train", "test"], "Mode must be either 'train' or 'test'."

for filename in os.listdir(worlds_dir):
    if filename.endswith(".wbt"):
        if os.path.exists(os.path.join(configs_dir, os.path.splitext(filename)[0])):
            print(f"Updated {os.path.splitext(filename)[0]} {mode}.toml")
            dir_name = os.path.join(configs_dir, os.path.splitext(filename)[0])
        else:
            print(f"\nCreated {os.path.splitext(filename)[0]} {mode}.toml\n")
            dir_name = os.path.join(configs_dir, os.path.splitext(filename)[0])
            os.makedirs(dir_name, exist_ok=True)

        toml_file = os.path.join(dir_name, f"{mode}.toml")

        with open(toml_file, "w") as f:
            with open(os.path.join(worlds_dir, filename)) as file:
                lines = iter(file)
                wooden_box_exist = False
                for line in lines:
                    if "DEF WB" in line:
                        wooden_box = line.split()[1]
                        position_line = next(lines)
                        position = [float(i) for i in position_line.split()[2:5]]
                        not wooden_box_exist and f.write(f"\n[wooden_boxes]\n")
                        f.write(f"{wooden_box}.name = '{wooden_box}'\n")
                        f.write(f"{wooden_box}.position = {position}\n")
                        f.write(f"{wooden_box}.DEF = '{wooden_box}'\n")
                        wooden_box_exist = True
                    elif "DEF PIONEER2" in line:
                        robot = line.split()[1]
                        position_line = next(lines)
                        position = [float(i) for i in position_line.split()[2:5]]
                        f.write(f"\n[robot]\n")
                        f.write(f"{robot}.name = '{robot}'\n")
                        f.write(f"{robot}.position = {position}\n")
                        f.write(f"{robot}.DEF = '{robot}'\n")
                    elif "DEF GoalSquare" in line:
                        position_line = next(lines)
                        goal = [float(i) for i in position_line.split()[2:5]]
                        f.write(f"\ngoal = {goal}\n")
                    elif "floorSize" in line:
                        grid_size = float(line.split()[1]) * float(line.split()[2])
                        f.write(f"\ngrid_size = {grid_size}\n")
                        f.write(
                            f"\ngrid = [{float(line.split()[1])}, {float(line.split()[2])}]\n"
                        )
