#!/bin/bash

# Define the source file and destination
SOURCE_FILE="matt_custom.yaml"
DESTINATION="/code/catkin_ws/src/dt-core/packages/line_detector/config/line_detector_node/"

# Check if the source file exists
if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: Source file '$SOURCE_FILE' does not exist."
    exit 1
fi

# Check if the destination directory exists
if [ ! -d "$DESTINATION" ]; then
    echo "Error: Destination directory '$DESTINATION' does not exist."
    exit 1
fi

# Copy the file to the destination
echo "Copying $SOURCE_FILE to $DESTINATION..."
cp "$SOURCE_FILE" "$DESTINATION"

# Check if the copy was successful
if [ $? -eq 0 ]; then
    echo "File copied successfully!"
else
    echo "Error: Failed to copy the file."
    exit 1
fi