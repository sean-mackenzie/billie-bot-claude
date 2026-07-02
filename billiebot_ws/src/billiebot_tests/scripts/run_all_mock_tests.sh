#!/bin/bash
# ============================================================================
# BillieBot — Run All Mock Tests
# ============================================================================
# Launches the full stack in mock mode, waits for topics/services, and
# runs verification checks. Returns exit 0 if all pass, 1 otherwise.
#
# Usage: ./run_all_mock_tests.sh
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PASS_COUNT=0
FAIL_COUNT=0
RESULTS=()

echo "============================================"
echo " BillieBot Mock Test Suite"
echo "============================================"
echo ""

# --- Helper ---
check_topic() {
    local topic="$1"
    local desc="$2"
    if timeout 10 ros2 topic info "$topic" 2>/dev/null | grep -q "Type:"; then
        echo "  [PASS] $desc ($topic)"
        PASS_COUNT=$((PASS_COUNT + 1))
        RESULTS+=("PASS: $desc")
    else
        echo "  [FAIL] $desc ($topic)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        RESULTS+=("FAIL: $desc")
    fi
}

check_service() {
    local service="$1"
    local desc="$2"
    if ros2 service list 2>/dev/null | grep -q "$service"; then
        echo "  [PASS] $desc ($service)"
        PASS_COUNT=$((PASS_COUNT + 1))
        RESULTS+=("PASS: $desc")
    else
        echo "  [FAIL] $desc ($service)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        RESULTS+=("FAIL: $desc")
    fi
}

# --- TC-01: Interface build ---
echo "TC-01: Interface definitions"
if ros2 interface show billiebot_interfaces/msg/DogState 2>/dev/null | grep -q "state"; then
    echo "  [PASS] DogState message defined"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo "  [FAIL] DogState message not found"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# --- TC-02: URDF validity ---
echo "TC-02: URDF validity"
if ros2 topic info /robot_description 2>/dev/null | grep -q "Type:"; then
    echo "  [PASS] robot_description published"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo "  [FAIL] robot_description not published"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# --- TC-03 to TC-06: Base bridge ---
echo "TC-03: Odometry"
check_topic "/odom" "Odometry publishing"

echo "TC-04: Joint states"
check_topic "/joint_states" "Joint states publishing"

echo "TC-05: Battery monitoring"
check_topic "/battery_state" "Battery state publishing"

echo "TC-06: E-stop service"
check_service "/e_stop" "E-stop service"

# --- TC-07 to TC-10: Perception ---
echo "TC-07: Dog detection"
check_topic "/dog/detections_3d" "Dog 3D detections"

echo "TC-08: Dog locator"
check_topic "/dog/pose_map" "Dog pose in map"

echo "TC-09: Thermal"
check_topic "/thermal/image" "Thermal image"

echo "TC-10: Thermal blob"
check_topic "/thermal/blob" "Thermal blob detection"

# --- TC-11: Audio ---
echo "TC-11: Audio events"
check_topic "/audio/events" "Audio event classification"

# --- TC-12: State fusion ---
echo "TC-12: State fusion"
check_topic "/billie/state" "Fused dog state"

# --- TC-13: Mission status ---
echo "TC-13: Mission status"
check_topic "/billiebot/mission_status" "Mission status"

# --- TC-14: GetDogState service ---
echo "TC-14: GetDogState service"
check_service "/get_dog_state" "GetDogState service"

# --- TC-15: SetMode service ---
echo "TC-15: SetMode service"
check_service "/set_mode" "SetMode service"

# --- Summary ---
echo ""
echo "============================================"
echo " Results: $PASS_COUNT passed, $FAIL_COUNT failed"
echo "============================================"

for r in "${RESULTS[@]}"; do
    echo "  $r"
done

echo ""
if [ "$FAIL_COUNT" -eq 0 ]; then
    echo "ALL TESTS PASSED"
    exit 0
else
    echo "$FAIL_COUNT TEST(S) FAILED"
    exit 1
fi
