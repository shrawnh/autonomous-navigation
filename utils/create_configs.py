import os
import hashlib

worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

configs_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs"

os.makedirs(configs_dir, exist_ok=True)


def get_file_hash(file_path):
    with open(file_path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def create_configs():
    num_of_changes = 0
    files_checked = 0
    for filename in os.listdir(worlds_dir):
        if filename.endswith(".wbt"):
            world_filename = os.path.splitext(filename)[0].split("_")[0]
            # create mode file depending on the file name if it contains test or train, then mode is test or train
            if "_test" in filename:
                mode = "test"
            elif "_train" in filename:
                mode = "train"
            else:
                print(
                    f"\033[91mSkipping {world_filename} because it doesn't contain either test or train\033[0m"
                )
                continue
            if os.path.exists(
                os.path.join(configs_dir, os.path.splitext(world_filename)[0])
            ):
                dir_name = os.path.join(
                    configs_dir, os.path.splitext(world_filename)[0]
                )
            else:
                print(
                    f"\033[93mCreating {os.path.splitext(world_filename)[0]} {mode}.toml\033[0m"
                )
                dir_name = os.path.join(
                    configs_dir, os.path.splitext(world_filename)[0]
                )
                os.makedirs(dir_name, exist_ok=True)

            files_checked += 1
            toml_file = os.path.join(dir_name, f"{mode}.toml")
            try:
                before_hash = get_file_hash(toml_file)
            except FileNotFoundError:
                before_hash = None

            with open(toml_file, "w") as f:
                line_array = []
                goal_array = []
                with open(os.path.join(worlds_dir, filename)) as file:
                    lines = iter(file)
                    wooden_box_exist = False
                    for line in lines:
                        if "DEF WB" in line:
                            wooden_box = line.split()[1]
                            position_line = next(lines)
                            position = [float(i) for i in position_line.split()[1:4]]
                            not wooden_box_exist and line_array.append(
                                f"\n[wooden_boxes]\n"
                            )
                            line_array.append(f"{wooden_box}.name = '{wooden_box}'\n")
                            line_array.append(f"{wooden_box}.position = {position}\n")
                            wooden_box_exist = True
                        elif "DEF PIONEER2" in line:
                            robot = line.split()[1]
                            position_line = next(lines)
                            position = [float(i) for i in position_line.split()[1:4]]
                            line_array.append(f"\n[robot]\n")
                            line_array.append(f"{robot}.name = '{robot}'\n")
                            line_array.append(f"{robot}.position = {position}\n")
                        elif "DEF GoalSquare" in line:
                            position_line = next(lines)
                            goal = [float(i) for i in position_line.split()[1:4]]
                            # line_array.insert(0, f"\ngoal = {goal}\n")
                            goal_array.append(goal)
                        elif "floorSize" in line:
                            line_array.append(
                                f"\ngrid_size = {float(line.split()[1])}\n"
                            )

                    line_array.insert(0, f"\ngoal = {goal_array}\n")

                # this is to write the lines in specified order
                f.writelines(line_array)

            after_hash = get_file_hash(toml_file)

            if before_hash != after_hash:
                num_of_changes += 1
                print(
                    f"\033[92mUpdated {os.path.splitext(world_filename)[0]} {mode}.toml\033[0m"
                )

    print(f"Files checked: {files_checked}")
    print(f"Number of changes: {num_of_changes}")


if "__main__" == __name__:
    create_configs()
