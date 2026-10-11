#!/bin/bash
# Real-time Gateway Server Log Viewer
LOG_FILE="$HOME/field_gateway_relay/gateway.log"

echo "============================================================"
echo " 📜 Viewing Real-Time Field Gateway Server Logs"
echo " Target: $LOG_FILE"
echo " (Press Ctrl+C to exit log view)"
echo "============================================================"

touch "$LOG_FILE"
tail -f -n 50 "$LOG_FILE"
