#!/bin/bash
# Verify Rung 07: OAK-D dog detector
# Expected: /dog/detections_3d and /dog/found topics

echo "=== Rung 07: OAK-D Verification ==="

PASS=0
FAIL=0

for TOPIC in /dog/detections_3d /dog/found; do
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
