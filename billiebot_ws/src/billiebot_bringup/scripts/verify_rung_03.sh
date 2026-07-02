#!/bin/bash
# Verify Rung 03: EKF
# Expected: /odometry/filtered topic from robot_localization

echo "=== Rung 03: EKF Verification ==="

if ros2 topic info /odometry/filtered 2>/dev/null | grep -q "Type:"; then
    echo "[PASS] /odometry/filtered exists"
    exit 0
else
    echo "[FAIL] /odometry/filtered not found — EKF may not be running"
    exit 1
fi
