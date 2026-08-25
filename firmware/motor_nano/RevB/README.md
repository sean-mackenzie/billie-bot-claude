# BillieBot Motor Nano Firmware — RevB

RevB is the BillieBot motor controller firmware for the Arduino Nano V3 (ATmega328P):
a ROSArduinoBridge derivative that takes single-letter serial commands from the ROS 2
host over USB and drives two JGA25-371 quadrature encoder-motors through an L298N
module in sign-magnitude mode (two PWM channels per motor, one per direction).

**RevB differs from RevA in pin assignment only** — no functional, behavioral, or
protocol change. RevA put the left encoder on D2/D3 and the right on A4/A5, which forced
the encoder harnesses to cross the board, and it split the four motor channels across
Timer0 (D5/D6) and Timer1 (D9/D10) at two different PWM frequencies. RevB routes each
encoder to the header on its own side of the Nano, puts IN1–IN4 in ascending order on
the D2–D12 header, and runs all four motor channels at a uniform 490 Hz. RevA remains
the known-good fallback; if RevB misbehaves on the bench, reflash RevA.

## Pin assignment

| Signal | Nano pin | Port bit | L298N terminal | Notes |
|---|---|---|---|---|
| IN1 — right motor forward | D3 | — | IN1 → OUT1/OUT2 (channel A, ENA) | Timer2 (OC2B), 490 Hz |
| IN2 — right motor backward | D9 | — | IN2 → OUT1/OUT2 (channel A, ENA) | Timer1 (OC1A), 490 Hz |
| IN3 — left motor forward | D10 | — | IN3 → OUT3/OUT4 (channel B, ENB) | Timer1 (OC1B), 490 Hz |
| IN4 — left motor backward | D11 | — | IN4 → OUT3/OUT4 (channel B, ENB) | Timer2 (OC2A), 490 Hz |
| Right encoder A | D6 | PD6 | — | `PCINT2_vect` |
| Right encoder B | D7 | PD7 | — | `PCINT2_vect` |
| Left encoder A | A4 | PC4 | — | `PCINT1_vect` |
| Left encoder B | A5 | PC5 | — | `PCINT1_vect` |

The right encoder-motor is the L298N's channel A (OUT1/OUT2), the left is channel B
(OUT3/OUT4). Note that A4/A5 keep the pins they had in RevA but now serve the **left**
encoder: the two encoders swapped ports, left moving PORTD → PORTC and right PORTC →
PORTD.

Pin defines live in `ROSArduinoBridge/motor_driver.h` and `ROSArduinoBridge/encoder_driver.h`.

## Why these four PWM pins

D3/D9/D10/D11 is the only ascending four-pin set that gives every motor channel the same
frequency while leaving Timer0 alone:

- **Uniform 490 Hz.** D9/D10 are Timer1 and D3/D11 are Timer2; under the Arduino core
  both timers run 8-bit phase-correct PWM with a /64 prescaler, so 16 MHz / (64 × 510) ≈
  490 Hz on all four. D5/D6 would have been Timer0 fast PWM at ~980 Hz.
- **One Timer1 channel + one Timer2 channel per motor.** Forward matches reverse *and*
  left matches right. This matters because `Kp/Kd/Ki/Ko` in `diff_controller.h` are
  globals shared by both wheels — any per-wheel PWM asymmetry would be tracked unequally
  by a single gain set, and low-duty creep speed is exactly where that shows up.
- **Timer0 untouched.** `millis()`, `micros()`, and `delay()` keep their timebase, which
  the PID loop interval and the auto-stop watchdog both depend on.

## Encoder constraints — what is and isn't safe to move

Encoders are read with **pin-change interrupts**, not external interrupts; the firmware
never calls `attachInterrupt()`. Two constraints follow from how the ISRs decode, and
breaking either one silently produces garbage counts:

1. **Each encoder's A and B must be an adjacent bit pair inside one port, A on the lower
   bit.** The ISRs gather both channels in a single masked read —
   `(PINC & (3 << 4)) >> 4` for the left, `(PIND & (3 << 6)) >> 6` for the right — and
   feed the 2-bit result into the `ENC_STATES` quadrature lookup. Scattered pins cannot
   work without restructuring the decode.
2. **The two encoders must live in different ports.** There is one ISR per pin-change
   bank: `PCINT1_vect` covers PORTC and `PCINT2_vect` covers PORTD. Two encoders in one
   bank would land in the same ISR with no way to tell them apart. Do not merge, add, or
   remove ISRs.

Within those rules the only remaining PORTC pair is A0–A3 and the only remaining PORTD
pair is D4/D5 (D0/D1 are the USB serial pins). `PCMSK` bit numbering coincides with port
bit numbering in all three banks, which is why `PCMSK2 |= (1 << PD6)` is correct — these
are not to be "corrected" to `PCINT22`-style names. Internal pull-ups stay enabled on
all four encoder pins in `setup()`; the encoders are open-collector and will not work
without them.

## Vestigial enable pins (D12/D13)

`motor_driver.h` still defines `RIGHT_MOTOR_ENABLE 12` and `LEFT_MOTOR_ENABLE 13`, and
`initMotorController()` writes both HIGH. **Neither does anything in this build.** The
function never calls `pinMode(..., OUTPUT)` first, so writing HIGH to an input pin only
enables the internal pull-up — no drive strength reaches the L298N. It doesn't matter,
because this build assumes the L298N module's **ENA and ENB jumpers are installed**,
holding both channels permanently enabled. D12 and D13 are therefore free for other use.
This is deliberately left as-is rather than "fixed": making the enables real would mean
either removing the jumpers or fighting them.

## Caveat: `servoPins {3, 4}`

`USE_SERVOS` is undefined and must stay that way. If it is ever enabled, note that
`servos.h` declares `byte servoPins[N_SERVOS] = { 3, 4 }` — **pin 3 is now IN1**, the
right motor's forward channel. Attaching a `Servo` object to D3 takes over Timer2 and
would break motor PWM. Change `servoPins` before enabling servos; nothing about it has
been changed here.

## Bench bringup

After rewiring, verify encoder direction **before** ever sending `m`. Open a serial
terminal at 57600 baud with a carriage-return line ending, put the robot on blocks, and:

1. `r` — reset both encoder counts to zero (also resets the PID state).
2. `o 100 100` — drive both motors at raw PWM 100. `o` sets `moving = 0`, which bypasses
   the PID entirely, so an inverted encoder cannot run away here. The 500 ms
   `AUTO_STOP_INTERVAL` means you must re-send `o` to keep the wheels turning.
3. `e` — read the counts (left first, then right). **Both must count up** while driving
   forward, and each wheel must physically turn forward.

Only once both wheels drive forward and count up should you try `m`. The firmware PID
closes on raw encoder counts and requires `sign(PWM output) == sign(encoder delta)`; run
`m` against an inverted encoder and the loop is in positive feedback — the error grows
with every correction and the motor pins at `MAX_PWM` almost immediately.

If a wheel drives forward but counts down, fix it in the **wiring**: swap that encoder's
A and B leads, or swap that motor's two output leads on the L298N. The ROS-layer signs
in `billiebot_ws/src/billiebot_base/config/base_driver.yaml`
(`left_motor_sign`, `right_motor_sign`, `left_encoder_sign`, `right_encoder_sign`)
correct odometry and command direction *as ROS sees them* — they cannot fix this,
because the PID loop closes inside the Nano on counts ROS never touches.

## Serial command reference

Commands are single letters terminated by a carriage return, at **57600 baud**. Full
list in `ROSArduinoBridge/commands.h`; the ones that matter here:

| Command | Effect |
|---|---|
| `e` | Print current encoder counts: left, then right |
| `r` | Reset both encoder counts to 0 and reset the PID |
| `o <pwm_l> <pwm_r>` | Raw PWM, −255…255. Bypasses the PID (`moving = 0`) |
| `m <cpl_l> <cpl_r>` | Closed-loop speed in counts per PID frame (PID runs at 30 Hz) |
| `u <Kp>:<Kd>:<Ki>:<Ko>` | Update PID gains (note the order: P, D, I, O) |

Motors auto-stop **500 ms** after the last motor command — BillieBot's watchdog value,
lowered from the upstream 2000 ms so the base halts promptly if the serial heartbeat
from the host drops. Keep sending commands to keep moving.

## Free pins after RevB

D2, D4, D5, D8, D12, D13, A0–A3, A6, A7.

**A4/A5 are now committed to the left encoder, so the Nano's hardware I2C is spent.**
Any I2C peripheral (an IMU, for instance) needs either a software I2C implementation on
free pins or a different board — see the encoder constraints above before assuming the
encoder can simply be moved off A4/A5.

---

The original upstream ROSArduinoBridge documentation is preserved in
`firmware/motor_nano/RevA/README.md`. It is not repeated here: its pin tables describe
the upstream wiring, not RevB.
