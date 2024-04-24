import os
import hashlib

worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

configs_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs"

os.makedirs(configs_dir, exist_ok=True)


def get_file_hash(file_path):
    with open(file_path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def create_new_extern_robots():
    for filename in os.listdir(worlds_dir):
        if filename.endswith(".wbt"):
            with open(os.path.join(worlds_dir, filename), "r") as file:
                lines = file.readlines()

            # Find the index of the line containing "WorldInfo"
            world_info_index = next(
                (i for i, line in enumerate(lines) if "WorldInfo" in line), None
            )

            if world_info_index is not None:
                # Overwrite all the lines before "WorldInfo" with a hard-coded set of lines

                new_lines = [
                    "#VRML_SIM R2023b utf8\n\n",
                    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"\n',
                    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n',
                    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto"\n',
                    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/appearances/protos/Parquetry.proto"\n',
                    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/factory/containers/protos/WoodenBox.proto"\n',
                    'IMPORTABLE EXTERNPROTO "../protos/Pioneer2F8.proto"\n',
                    'IMPORTABLE EXTERNPROTO "../protos/Pioneer2FB.proto"\n',
                    'IMPORTABLE EXTERNPROTO "../protos/Pioneer2FB6.proto"\n',
                    'IMPORTABLE EXTERNPROTO "../protos/Pioneer2LR.proto"\n\n',
                ]
                lines[:world_info_index] = new_lines

                # Write the modified lines back to the file
                with open(os.path.join(worlds_dir, filename), "w") as file:
                    file.writelines(lines)


sensor_configs = [
    {
        "name": "front",
        "robot": "Pioneer2F8",
    },
    {
        "name": "front-back",
        "robot": "Pioneer2FB",
    },
    {
        "name": "front-back-6",
        "robot": "Pioneer2FB6",
    },
    {
        "name": "sides",
        "robot": "Pioneer2LR",
    },
]
steps = ["step-1", "step-2", "step-3"]


def create_new_robots():
    missing_test_files = []
    missing_train_files = []
    for step in steps:
        with open(
            os.path.join(worlds_dir, f"{step}-v1-switch_test base.wbt"), "r"
        ) as base_file:
            base_test_contents = base_file.read()

        with open(
            os.path.join(worlds_dir, f"{step}-v1-switch_train base.wbt"), "r"
        ) as base_file:
            base_train_contents = base_file.read()

        # Get all files that include the name of the step
        step_files = [
            f for f in os.listdir(worlds_dir) if step in f and f.endswith(".wbt")
        ]

        # Split the files into _test and _train files
        test_files = [f for f in step_files if "_test" in f]
        train_files = [f for f in step_files if "_train" in f]

        # Check if any of the sensor config names are missing from both _test and _train files
        for config in sensor_configs:
            config_name = config["name"]
            robot_name = config["robot"]
            if not any(config_name in f for f in test_files):
                missing_test_files.append(config_name)
                with open(os.path.join(worlds_dir, f"{step}-v1-{config_name}_test.wbt"), "w") as file:  # type: ignore
                    base_contents_lines = base_test_contents.split("\n")
                    pioneer_index = next((i for i, line in enumerate(base_contents_lines) if "DEF PIONEER2" in line), None)  # type: ignore
                    if pioneer_index is not None:
                        base_contents_lines[pioneer_index] = f"DEF PIONEER2 {robot_name} {{"  # type: ignore
                    file.write("\n".join(base_contents_lines))
            if not any(config_name in f for f in train_files):
                missing_train_files.append(config_name)
                with open(os.path.join(worlds_dir, f"{step}-v1-{config_name}_train.wbt"), "w") as file:  # type: ignore
                    base_contents_lines = base_train_contents.split("\n")
                    pioneer_index = next((i for i, line in enumerate(base_contents_lines) if "DEF PIONEER2" in line), None)  # type: ignore
                    if pioneer_index is not None:
                        base_contents_lines[pioneer_index] = f"DEF PIONEER2 {robot_name} {{"  # type: ignore
                    file.write("\n".join(base_contents_lines))

    return missing_test_files, missing_train_files


if __name__ == "__main__":
    train, test = create_new_robots()
    print(f"Missing test files: {test}")
    print(f"Missing train files: {train}")
