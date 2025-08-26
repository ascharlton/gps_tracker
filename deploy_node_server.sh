#!/bin/bash
set -e

APP_NAME="gps_tracker"
SCRIPT="gps_server.js"

echo "[INFO] Updating npm dependencies..."
npm install

# Install PM2 if missing
if ! command -v pm2 &> /dev/null; then
  echo "[INFO] Installing PM2 globally..."
  npm install -g pm2
fi

echo "[INFO] Starting app with PM2..."
pm2 start $SCRIPT --name $APP_NAME || pm2 restart $APP_NAME

echo "[INFO] Saving PM2 process list..."
pm2 save

echo "[INFO] Setting up PM2 startup..."
pm2 startup | tail -n 1 | bash

echo "[INFO] Deployment complete."
echo "Use 'pm2 logs $APP_NAME' to see logs."

