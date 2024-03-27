import os
import toml

# Directory to search for .wbt files
worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

# Directory to create new folders and .toml files
configs_dir = (
    "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs_test"
)

# Ensure the configs directory exists
os.makedirs(configs_dir, exist_ok=True)

# Iterate over all files in the worlds directory
for filename in os.listdir(worlds_dir):
    # Check if the file has the .wbt extension
    if filename.endswith(".wbt"):
        # Create a new directory in configs with the same name as the .wbt file (without the extension)
        # Check if the directory already exists
        if os.path.exists(os.path.join(configs_dir, os.path.splitext(filename)[0])):
            print(f"Directory {os.path.splitext(filename)[0]} already exists.")
            continue
        else:
            new_dir = os.path.join(configs_dir, os.path.splitext(filename)[0])
            os.makedirs(new_dir, exist_ok=True)

        # Ask the user for input
        mode = input(f"Enter mode for {filename} (train/test): ")
        assert mode in ["train", "test"], "Mode must be either 'train' or 'test'."

        # Create a new .toml file in the new directory with the file name
        toml_file = os.path.join(new_dir, f"{mode}.toml")

        # Write the mode to the .toml file
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
