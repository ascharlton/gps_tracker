#!/bin/bash
set -e

# Navigate to script directory
cd "$(dirname "$0")"

# Install npm dependencies (only first time or when package.json changes)
if [ ! -d "node_modules" ]; then
  echo "Installing npm dependencies..."
  npm install
fi

# Start the server
echo "Starting GPS server..."
node gps_server.js

