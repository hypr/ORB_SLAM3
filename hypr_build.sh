#!/bin/bash

# Build the hypr_slam module
cd build/
make hypr_slam
cd ..

# Link libraries
LIBS_DIR="/data/shared/ORB_SLAM3/libs"

# Check if libs directory exists
if [ -d "$LIBS_DIR" ]; then
    # Create backup with timestamp
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BACKUP_DIR="${LIBS_DIR}_backup_${TIMESTAMP}"
    echo "Creating backup of existing libs directory to: $BACKUP_DIR"
    cp -r "$LIBS_DIR" "$BACKUP_DIR"
fi

# Create libs directory if it doesn't exist
mkdir -p "$LIBS_DIR"

# Copy libraries
echo "Copying libraries to $LIBS_DIR"
cp -r lib/* "$LIBS_DIR/"

echo "Done"

