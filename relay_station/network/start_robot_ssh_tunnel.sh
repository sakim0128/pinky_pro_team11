#!/usr/bin/env bash
# =============================================================================
# Robot SSH Port Forwarding Tunnel (Auto-restart daemon)
# - Port 2205 -> 198.51.100.5:22 (Robot #1 Pinky)
# - Port 2206 -> 198.51.100.6:22 (Robot #2 Pinky)
# =============================================================================

pkill -f "socat.*2205" 2>/dev/null || true
pkill -f "socat.*2206" 2>/dev/null || true
pkill -f "socat.*2207" 2>/dev/null || true
pkill -f "socat.*8005" 2>/dev/null || true
sleep 1

echo "[Tunnel] Starting SSH and Web port forwarding for robots..."
echo " - 0.0.0.0:2205 -> 198.51.100.5:22 (Robot #1 SSH)"
echo " - 0.0.0.0:2206 -> 198.51.100.6:22 (Robot #2 SSH)"
echo " - 0.0.0.0:2207 -> 198.51.100.8:22 (Robot #3 PP25022 SSH)"
echo " - 0.0.0.0:8005 -> 198.51.100.5:80 (Robot #1 Web UI)"

socat TCP4-LISTEN:2205,fork,reuseaddr TCP4:198.51.100.5:22 &
PID_2205=$!

socat TCP4-LISTEN:2206,fork,reuseaddr TCP4:198.51.100.6:22 &
PID_2206=$!

socat TCP4-LISTEN:2207,fork,reuseaddr TCP4:198.51.100.8:22 &
PID_2207=$!

socat TCP4-LISTEN:8005,fork,reuseaddr TCP4:198.51.100.5:80 &
PID_8005=$!

echo "[Tunnel] socat started. PIDs: $PID_2205, $PID_2206, $PID_2207, $PID_8005"

# Keep alive
wait $PID_2205 $PID_2206 $PID_2207 $PID_8005
