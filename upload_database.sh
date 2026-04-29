#!/bin/bash

# ──────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────
LOCAL_DB_DIR="/Volumes/Laurens SSD/BasData"
REMOTE_USER="bwingen"
REMOTE_HOST="login.delftblue.tudelft.nl"
REMOTE_DEST="/scratch/bwingen/thesis/database/"

# ──────────────────────────────────────────────────────────
# SYNC COMMAND
# ──────────────────────────────────────────────────────────

echo "🚀 Starting upload of $LOCAL_DB_DIR to $REMOTE_USER@$REMOTE_HOST:$REMOTE_DEST"
echo "This uses rsync to efficiently transfer only new or changed files."

# Ensure the remote directory exists
ssh "$REMOTE_USER@$REMOTE_HOST" "mkdir -p $REMOTE_DEST"

# Run rsync
# -a: archive mode (preserves permissions, etc.)
# -v: verbose
# -z: compress data during transfer
# --progress: show progress
# --partial: allow resuming interrupted transfers
rsync -avz --progress --partial "$LOCAL_DB_DIR/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DEST"

if [ $? -eq 0 ]; then
    echo "✅ Upload complete!"
else
    echo "❌ Upload failed. Please check your connection and NetID."
fi
