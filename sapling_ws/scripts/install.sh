#!/bin/bash

# Find the correct path
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" # Get the directory where this script is located
TARGET_DIR="$SCRIPT_DIR/../src/drivers" # Define the target drivers directory (relative to this script)
REPOS_FILE="$SCRIPT_DIR/../sapling.repos" # Define where the .repos file is (currently in sapling_ws/src/)

# Check if vcs is installed
if ! command -v vcs &> /dev/null; then
    echo "Error: vcstool is not installed - Run: sudo apt install python3-vcstool"
    exit 1
fi

echo "Installing required Github repositories into $TARGET_DIR..."

# Import the repositories
vcs import "$TARGET_DIR" < "$REPOS_FILE"

echo "Done - Repositories imported to src/drivers"
