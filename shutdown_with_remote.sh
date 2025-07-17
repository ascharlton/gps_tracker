#!/bin/bash

# CONFIGURATION
REMOTE_USER=pi
REMOTE_HOST=192.168.2.2   # IP or hostname of the Pi Zero W
SSH_KEY="$HOME/.ssh/id_rsa" # Or omit if password-based
DELAY_BEFORE_LOCAL_SHUTDOWN=10  # seconds

echo "[INFO] Initiating remote shutdown on $REMOTE_HOST..."

# Send shutdown command to the Pi Zero
ssh -i "$SSH_KEY" "$REMOTE_USER@$REMOTE_HOST" 'sudo shutdown -h now' && \
echo "[INFO] Remote Pi Zero shutdown command sent successfully."

# Optional delay to allow remote shutdown to complete
echo "[INFO] Waiting $DELAY_BEFORE_LOCAL_SHUTDOWN seconds before shutting down host..."
sleep "$DELAY_BEFORE_LOCAL_SHUTDOWN"

# Shutdown this host
echo "[INFO] Shutting down host Raspberry Pi 4B..."
sudo shutdown -h now

