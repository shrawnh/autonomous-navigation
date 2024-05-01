import os
import pandas as pd

# Directory containing the CSV files
directory = (
    "/Users/shrwnh/Development/autonomous-navigation/src/simulation/testing_logs/sac"
)
output_csv = "sac_2_averages.csv"

# Create a DataFrame to store the aggregated data
aggregated_data = pd.DataFrame(
    columns=["step", "sensor", "avg_time_taken", "avg_speed"]
)

# List all files in the directory
for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        # Extract step number and sensor name from the filename
        parts = filename.split("-")
        step = parts[1]
        sensor = "-".join(parts[3:]).replace(".csv", "")

        # Read the CSV file
        filepath = os.path.join(directory, filename)
        df = pd.read_csv(filepath)

        # Calculate the average time taken
        if "avg_time_taken" in df.columns:
            average_time = df["avg_time_taken"].mean()
            average_time = round(average_time, 2)

        # Get the last value of avg_speed
        if "avg_speed" in df.columns:
            average_speed = df["avg_speed"].iloc[-1]
            average_speed = abs(round(average_speed, 2))

        # Append to the DataFrame using concat
        new_row = pd.DataFrame(
            {
                "step": [step],
                "sensor": [sensor],
                "avg_time_taken": [average_time],
                "avg_speed": [average_speed],
            }
        )
        aggregated_data = pd.concat([aggregated_data, new_row], ignore_index=True)

# Define the order of the sensors
sensor_order = ["front", "front-back", "front-back-6", "sides-6"]

# Convert the "step" column to int for proper sorting
aggregated_data["step"] = aggregated_data["step"].astype(int)

# Convert the "sensor" column to a category and specify the order
aggregated_data["sensor"] = pd.Categorical(
    aggregated_data["sensor"], categories=sensor_order, ordered=True
)

# Sort by "step" and then by "sensor"
aggregated_data.sort_values(by=["step", "sensor"], inplace=True)

# Reset the index
aggregated_data.reset_index(drop=True, inplace=True)

# Save the DataFrame to a new CSV file
aggregated_data.to_csv(output_csv, index=False)
print(f"Data aggregated and saved to {output_csv}")
