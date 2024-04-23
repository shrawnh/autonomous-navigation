import os
import hashlib

worlds_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/worlds"

configs_dir = "/Users/shrwnh/Development/autonomous-navigation/src/simulation/configs"

os.makedirs(configs_dir, exist_ok=True)


def get_file_hash(file_path):
    with open(file_path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


# def create_new_robots():
#     num_of_changes = 0
#     files_checked = 0
#     for filename in os.listdir(worlds_dir):
#         if filename.endswith(".wbt"):
#             with open(os.path.join(worlds_dir, filename)) as file:
#                 for line in file:
#                     if "WorldInfo" in line:
#                         # i want to overwrite the previous lines with a hard coded set of lines


def create_new_robots():
    # num_of_changes = 0
    # files_checked = 0
    for filename in os.listdir(worlds_dir):
        if filename.endswith("copy.wbt"):
            with open(os.path.join(worlds_dir, filename), "r") as file:
                lines = file.readlines()

            # Find the index of the line containing "WorldInfo"
            world_info_index = next(
                (i for i, line in enumerate(lines) if "WorldInfo" in line), None
            )

            if world_info_index is not None:
                # Overwrite all the lines before "WorldInfo" with a hard-coded set of lines
                # VRML_SIM R2023b utf8

                new_lines = [
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


if __name__ == "__main__":
    create_new_robots()
