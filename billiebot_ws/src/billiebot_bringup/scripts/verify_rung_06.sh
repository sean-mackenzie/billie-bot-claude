#!/bin/bash
# Verify Rung 06: Full Nav2 stack
# Expected: Nav2 lifecycle nodes active, navigate_to_pose action available

echo "=== Rung 06: Nav2 Verification ==="

PASS=0
FAIL=0

# Check Nav2 action servers
for ACTION in navigate_to_pose; do
    if ros2 action list 2>/dev/null | grep -q "$ACTION"; then
        echo "[PASS] /$ACTION action available"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] /$ACTION action not found"
        FAIL=$((FAIL + 1))
    fi
done

# Check costmap topics
for TOPIC in /local_costmap/costmap /global_costmap/costmap; do
    if ros2 topic info "$TOPIC" 2>/dev/null | grep -q "Type:"; then
        echo "[PASS] $TOPIC exists"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] $TOPIC not found"
        FAIL=$((FAIL + 1))
    fi
done

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
