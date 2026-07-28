#!/bin/bash
# Install BillieBot's udev rules (Jetson).
#
# Installs udev/99-billiebot.rules -- the OAK-D Lite permissions rule that lets the
# depthai SDK claim the camera as a non-root user -- and puts the current user in the
# dialout group for the RPLidar and Arduino serial ports.
#
# The serial devices need no rule of their own: they are addressed by their stock
# /dev/serial/by-id/ paths, which systemd-udev creates automatically. See GAP-20.
#
# Run from the source tree or via: ros2 run billiebot_bringup install_udev_rules.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_NAME="99-billiebot.rules"
RULES_DEST="/etc/udev/rules.d/$RULES_NAME"

echo "=== BillieBot udev rule installation ==="

# The rules file sits at ../udev/ in the source tree and at
# ../../share/billiebot_bringup/udev/ in the colcon install space.
RULES_SRC=""
for candidate in \
    "$SCRIPT_DIR/../udev/$RULES_NAME" \
    "$SCRIPT_DIR/../../share/billiebot_bringup/udev/$RULES_NAME"; do
    if [ -f "$candidate" ]; then
        RULES_SRC="$(cd "$(dirname "$candidate")" && pwd)/$(basename "$candidate")"
        break
    fi
done

if [ -z "$RULES_SRC" ] && command -v ros2 >/dev/null 2>&1; then
    prefix="$(ros2 pkg prefix billiebot_bringup 2>/dev/null || true)"
    if [ -n "$prefix" ] && [ -f "$prefix/share/billiebot_bringup/udev/$RULES_NAME" ]; then
        RULES_SRC="$prefix/share/billiebot_bringup/udev/$RULES_NAME"
    fi
fi

if [ -z "$RULES_SRC" ]; then
    echo "[FAIL] Could not locate $RULES_NAME relative to $SCRIPT_DIR"
    exit 1
fi

echo "[INFO] Source: $RULES_SRC"
sudo install -m 0644 "$RULES_SRC" "$RULES_DEST"
echo "[PASS] Installed $RULES_DEST"

# Superseded by 99-billiebot.rules; older setup guides wrote this by hand.
if [ -f /etc/udev/rules.d/80-movidius.rules ]; then
    sudo rm -f /etc/udev/rules.d/80-movidius.rules
    echo "[INFO] Removed superseded /etc/udev/rules.d/80-movidius.rules"
fi

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "[PASS] Reloaded udev rules"

if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "[PASS] $USER is already in the dialout group"
else
    sudo usermod -aG dialout "$USER"
    echo "[PASS] Added $USER to the dialout group"
    echo "[INFO] Log out and back in for the group change to take effect."
fi

echo
echo "Next: replug the OAK-D, then run  ros2 run billiebot_bringup check_devices.sh"
