#!/bin/bash
set -e


# Install nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.5/install.sh | bash
source ~/.bashrc

# Install and use the latest LTS version
nvm install --lts
nvm use --lts

# Verify installation
node -v
npm -v

cd ~/repos/gps_tracker


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

