#!/bin/bash
# Verify Rung 01: Lidar
# Expected: /scan topic publishing LaserScan messages

echo "=== Rung 01: Lidar Verification ==="

TOPIC="/scan"
TIMEOUT=5

if ros2 topic info "$TOPIC" 2>/dev/null | grep -q "Type:"; then
    echo "[PASS] $TOPIC exists"
    COUNT=$(timeout "$TIMEOUT" ros2 topic hz "$TOPIC" 2>/dev/null | head -1 | grep -oP '[0-9.]+' || echo "0")
    if [ -n "$COUNT" ]; then
        echo "[PASS] $TOPIC is publishing"
        exit 0
    else
        echo "[FAIL] $TOPIC not publishing data"
        exit 1
    fi
else
    echo "[FAIL] $TOPIC not found"
    exit 1
fi
