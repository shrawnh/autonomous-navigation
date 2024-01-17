#!/bin/bash

# Input file name (the one with the list of packages and versions)
input_file="utils/pip-list.txt"

# Output file name for the requirements
output_file="./requirements.txt"

# Check if the input file exists
if [ ! -f "$input_file" ]; then
    echo "Input file not found: $input_file"
    exit 1
fi

# Clear the output file or create it if it doesn't exist
> "$output_file"

# Read each line from the input file and write to the output file
while IFS= read -r line
do
    # Extract package name and version
    package=$(echo "$line" | awk '{print $1}')
    version=$(echo "$line" | awk '{print $2}')

    # Write to the requirements.txt file
    echo "${package}==${version}" >> "$output_file"
done < "$input_file"

# run ./utils/create_requirements.sh to run files

echo "requirements.txt has been created."
