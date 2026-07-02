#!/bin/bash
# Verify Rung 12: Cognition
# Expected: /billie/state, /get_dog_state service, report server health

echo "=== Rung 12: Cognition Verification ==="

PASS=0
FAIL=0

# Check state topic
if ros2 topic info /billie/state 2>/dev/null | grep -q "Type:"; then
    echo "[PASS] /billie/state exists"
    PASS=$((PASS + 1))
else
    echo "[FAIL] /billie/state not found"
    FAIL=$((FAIL + 1))
fi

# Check service
if ros2 service list 2>/dev/null | grep -q "/get_dog_state"; then
    echo "[PASS] /get_dog_state service available"
    PASS=$((PASS + 1))
else
    echo "[FAIL] /get_dog_state service not found"
    FAIL=$((FAIL + 1))
fi

# Check report server health
if curl -s http://localhost:8080/health 2>/dev/null | grep -q '"status"'; then
    echo "[PASS] Report server healthy"
    PASS=$((PASS + 1))
else
    echo "[WARN] Report server not responding (may need fastapi/uvicorn)"
    # Don't count as failure — optional dependency
fi

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
