#!/bin/bash
# Preflight check for the three Jetson-attached devices, before rung 01/02/07.
#
# Expected serial paths are read out of the configs themselves so this script cannot
# drift from what the launch files actually open:
#   RPLidar A1   -> billiebot_bringup/config/lidar.yaml       (serial_port)
#   Arduino Nano -> billiebot_base/config/base_driver.yaml    (port)
#
# Run via: ros2 run billiebot_bringup check_devices.sh

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FAILURES=0

pass() { echo "[PASS] $1"; }
skip() { echo "[SKIP] $1"; }
info() { echo "[INFO] $1"; }
fail() { echo "[FAIL] $1"; FAILURES=$((FAILURES + 1)); }

# Locate a package file across the source tree, the colcon install space, and a
# sourced workspace, in that order. Echoes the path, or nothing if not found.
find_pkg_file() {
    local pkg="$1" rel="$2" prefix candidate
    for candidate in \
        "$SCRIPT_DIR/../../$pkg/$rel" \
        "$SCRIPT_DIR/../../share/$pkg/$rel" \
        "$SCRIPT_DIR/../../../$pkg/share/$pkg/$rel"; do
        if [ -f "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    if command -v ros2 >/dev/null 2>&1; then
        prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
        if [ -n "$prefix" ] && [ -f "$prefix/share/$pkg/$rel" ]; then
            echo "$prefix/share/$pkg/$rel"
            return 0
        fi
    fi
    return 1
}

# Pull a scalar value out of a ROS parameter YAML: strips the key, any inline
# comment, and surrounding quotes. Takes the first match.
yaml_value() {
    local file="$1" key="$2"
    sed -n "s/^[[:space:]]*${key}:[[:space:]]*//p" "$file" \
        | sed 's/[[:space:]]*#.*$//' \
        | tr -d "\"'" \
        | sed 's/[[:space:]]*$//' \
        | head -1
}

# Check one serial device: symlink present, resolves to a live character device,
# and is readable/writable by the current user.
check_serial() {
    local label="$1" path="$2" target

    if [ -z "$path" ]; then
        fail "$label: could not read the port out of its config file"
        return
    fi

    if [ ! -e "$path" ]; then
        fail "$label not found at $path"
        info "  The device is unplugged, powered off, or enumerated under a different name."
        return
    fi

    target="$(readlink -f "$path")"
    if [ ! -c "$target" ]; then
        fail "$label: $path does not resolve to a character device (got $target)"
        return
    fi

    if [ -r "$target" ] && [ -w "$target" ]; then
        pass "$label at $path -> $target"
    else
        fail "$label: $target is not read/writable by $USER (dialout group? re-login needed?)"
    fi
}

echo "=== BillieBot device preflight ==="
echo

# --- 1. RPLidar A1 ---------------------------------------------------------
LIDAR_YAML="$(find_pkg_file billiebot_bringup config/lidar.yaml)"
if [ -n "$LIDAR_YAML" ]; then
    check_serial "RPLidar A1" "$(yaml_value "$LIDAR_YAML" serial_port)"
else
    fail "RPLidar A1: could not locate billiebot_bringup/config/lidar.yaml"
fi

# --- 2. Arduino Nano -------------------------------------------------------
BASE_YAML="$(find_pkg_file billiebot_base config/base_driver.yaml)"
if [ -n "$BASE_YAML" ]; then
    check_serial "Arduino Nano" "$(yaml_value "$BASE_YAML" port)"
else
    fail "Arduino Nano: could not locate billiebot_base/config/base_driver.yaml"
fi

# --- 3. Serial port permissions --------------------------------------------
if [ "$EUID" -eq 0 ]; then
    skip "dialout group check (running as root)"
elif id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    pass "$USER is in the dialout group"
else
    fail "$USER is not in the dialout group -- run install_udev_rules.sh, then log out and back in"
fi

# --- 4. OAK-D Lite ---------------------------------------------------------
if ! python3 -c "import depthai" >/dev/null 2>&1; then
    skip "OAK-D Lite (depthai not importable here -- expected on the Pi and in the mock container)"
else
    # depthai's own logger writes warnings to stdout, so key off explicit sentinel
    # lines rather than "did anything get printed".
    OAKD_OUT="$(python3 -c "
import depthai as dai
devs = dai.Device.getAllAvailableDevices()
print('BB_OAKD_COUNT=%d' % len(devs))
for d in devs:
    print('BB_OAKD_DEV=%s %s' % (d.getMxId(), d.state.name))
" 2>/dev/null)"
    OAKD_COUNT="$(echo "$OAKD_OUT" | sed -n 's/^BB_OAKD_COUNT=//p' | head -1)"
    if [ -z "$OAKD_COUNT" ]; then
        fail "OAK-D Lite: depthai enumeration failed to run"
    elif [ "$OAKD_COUNT" -gt 0 ]; then
        pass "OAK-D Lite enumerated ($OAKD_COUNT): $(echo "$OAKD_OUT" \
            | sed -n 's/^BB_OAKD_DEV=//p' | tr '\n' ' ')"
    else
        fail "OAK-D Lite not enumerated by depthai"
        info "  Check the udev rule (install_udev_rules.sh), use a USB 3 port and the supplied"
        info "  cable, and replug after 'sudo udevadm trigger'."
        info "  In a container, USB passthrough also needs -v /dev/bus/usb:/dev/bus/usb"
        info "  --device-cgroup-rule='c 189:* rmw'."
    fi
fi

# --- Context ---------------------------------------------------------------
echo
echo "--- /dev/serial/by-id/ ---"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "(no USB serial devices enumerated)"

echo
if [ "$FAILURES" -eq 0 ]; then
    echo "=== All device checks passed ==="
    exit 0
fi
echo "=== $FAILURES device check(s) failed ==="
exit 1
