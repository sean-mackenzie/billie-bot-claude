#!/bin/bash
# Verify Rung 02: Base bridge + robot_state_publisher
# Expected: /odom, /joint_states, /battery_state, /robot_description, TF tree

echo "=== Rung 02: Base Verification ==="

PASS=0
FAIL=0

for TOPIC in /odom /joint_states /battery_state /robot_description; do
    if ros2 topic info "$TOPIC" 2>/dev/null | grep -q "Type:"; then
        echo "[PASS] $TOPIC exists"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] $TOPIC not found"
        FAIL=$((FAIL + 1))
    fi
done

# Check TF
if ros2 topic info /tf 2>/dev/null | grep -q "Type:"; then
    echo "[PASS] /tf publishing"
    PASS=$((PASS + 1))
else
    echo "[FAIL] /tf not found"
    FAIL=$((FAIL + 1))
fi

# Check e-stop service
if ros2 service list 2>/dev/null | grep -q "/e_stop"; then
    echo "[PASS] /e_stop service available"
    PASS=$((PASS + 1))
else
    echo "[FAIL] /e_stop service not found"
    FAIL=$((FAIL + 1))
fi

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
