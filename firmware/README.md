# BillieBot Firmware Modifications

## Arduino ROSArduinoBridge — Watchdog Timer Change

### Required Change (GAP 3 — SYS-PLT-5)

In `ROSArduinoBridge.ino`, change:

```cpp
// BEFORE (original):
#define AUTO_STOP_INTERVAL 2000

// AFTER (BillieBot):
#define AUTO_STOP_INTERVAL 500
```

This ensures motors stop within 500ms of losing the serial heartbeat from
the Jetson, satisfying SYS-PLT-5.

The Python `base_bridge.py` node sends motor commands at 30Hz (~33ms interval),
well within the 500ms window during normal operation.

### Optional: IMU Extension (when hardware is rewired)

Add an `'i'` command handler to the Arduino firmware to read BNO055 IMU data
over I2C and return it in the serial stream. This requires rewiring the right
encoder from A4/A5 to other pins (e.g., D4/D7) to free the I2C bus.

See `docs/MEASURE_ME.md` for the full rewire procedure.
