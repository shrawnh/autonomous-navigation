def update_controller(file_path, new_controller):
    try:
        with open(file_path, "r") as file:
            lines = file.readlines()

        updated = False
        for i, line in enumerate(lines):
            if "controller" in line:
                lines[i] = f'  controller "{new_controller}"\n'
                updated = True
                break

        if updated:
            with open(file_path, "w") as file:
                file.writelines(lines)
            print("File updated successfully.")
        else:
            print("No 'controller' found to update.")

    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")
