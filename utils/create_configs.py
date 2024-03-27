import os

worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

configs_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs"

os.makedirs(configs_dir, exist_ok=True)

for filename in os.listdir(worlds_dir):
    # create mode file depending on the file name if it contains test or train, then mode is test or train
    if "test" in filename:
        mode = "test"
    elif "train" in filename:
        mode = "train"
    else:
        print(
            f"\033[91mSkipping {filename} because it doesn't contain either test or train\033[0m"
        )
        continue
    if filename.endswith(".wbt"):
        if os.path.exists(os.path.join(configs_dir, os.path.splitext(filename)[0])):
            print(f"\033[92mUpdated {os.path.splitext(filename)[0]} {mode}.toml\033[0m")
            dir_name = os.path.join(configs_dir, os.path.splitext(filename)[0])
        else:
            print(f"\033[93mCreated {os.path.splitext(filename)[0]} {mode}.toml\033[0m")
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
                        position = [float(i) for i in position_line.split()[1:4]]
                        not wooden_box_exist and f.write(f"\n[wooden_boxes]\n")
                        f.write(f"{wooden_box}.name = '{wooden_box}'\n")
                        f.write(f"{wooden_box}.position = {position}\n")
                        wooden_box_exist = True
                    elif "DEF PIONEER2" in line:
                        robot = line.split()[1]
                        position_line = next(lines)
                        position = [float(i) for i in position_line.split()[1:4]]
                        f.write(f"\n[robot]\n")
                        f.write(f"{robot}.name = '{robot}'\n")
                        f.write(f"{robot}.position = {position}\n")
                    elif "DEF GoalSquare" in line:
                        position_line = next(lines)
                        goal = [float(i) for i in position_line.split()[1:4]]
                        f.write(f"\ngoal = {goal}\n")
                    elif "floorSize" in line:
                        grid_size = float(line.split()[1]) * float(line.split()[2])
                        f.write(f"\ngrid_size = {grid_size}\n")
                        f.write(
                            f"\ngrid = [{float(line.split()[1])}, {float(line.split()[2])}]\n"
                        )
