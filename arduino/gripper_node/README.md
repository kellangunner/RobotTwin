# Gripper node

An Arduino Uno driving the SG-90 parallel-jaw gripper, independent of the ESP32
that runs the arm.

## Why it is a separate board

The gripper was always the least-coupled subsystem in the robot — no kinematics,
no trajectory, no homing, no torque governor, no place in the firmware's state
machine. Giving it its own MCU therefore costs almost nothing and buys three
things:

- **Power isolation.** A stalled SG-90 pulls ~650 mA. That is precisely the
  transient that browns out a board generating step pulses, and it was already
  the documented brown-out risk on the shared 5 V rail.
- **Correct logic levels.** An SG-90 expects ~5 V pulses. The ESP32 drives
  3.3 V — it works, but with no margin. The Uno's 5 V output is right.
- **A timing consumer removed** from a board running a 1 kHz step engine.

## Two ways to command it, last one wins

- **The knob on A0.** Needs no computer at all, which is what makes it the
  calibration tool. It only takes control once it moves past a small deadband,
  so ADC jitter cannot silently steal the jaws back from a running script.
- **`G <mm>` / `GRIP <mm>` / `R` on either serial port.** USB from a host, or the
  one-way relay from the ESP32 so a scripted move can grip mid-sequence. A
  command re-anchors the knob and takes control back from it.

The onboard LED is solid while the jaws hold, blinks while they move, and is off
when the servo is slack.

Nothing is driven at power-on. The jaws are only *assumed* to sit at
`boot_opening_mm`, so the first deliberate command is what energizes the servo —
that way a power-on cannot lurch the mechanism.

## Where the numbers come from

None of the calibration lives in this sketch. `scripts/gen_gripper_config.py`
reads the `gripper:` block of `config/firmware.yaml` before every build and
emits `src/gripper_config_gen.hpp`, so a pulse-width change is a config edit plus
a reflash — never a code change.

The slew limiting and the opening → pulse-width mapping are not written here
either. They come from `rt::GripperAxis` in `src/hardware/gripper.cpp`, compiled
straight out of the repo core — the same class the twin, the native tests, and
the ESP32 use. That is deliberate: the ESP32 predicts when the jaws land without
ever hearing back from this board, which only works because both run identical
math on identical numbers.

## Pins

| Pin | Function |
| --- | --- |
| D9  | Servo signal (orange). Must be D9 or D10 — the `Servo` library owns Timer1 |
| A0  | Potentiometer wiper; the pot's ends go to Uno 5 V and GND |
| D2  | `SoftwareSerial` RX ← ESP32 GPIO 13 through 1 kΩ |
| D3  | Reserved. Unwired — a future return path would use it |
| D13 | Onboard status LED |
| USB | Host: power, uploads, and the console at 115200 |

D2 rather than D0 because the Uno's hardware UART is wired to its USB chip;
putting the relay there would fight the console and break sketch uploads.

Servo power is a bench supply at 5.0 V with a 1 A limit, with a 470–1000 µF
electrolytic at the servo connector. It shares only a ground with the Uno and
the ESP32. Full wiring, including what not to do with the Uno's barrel jack, is
in [docs/wiring-and-bringup.md](../../docs/wiring-and-bringup.md).

## Build

```bash
pio run -t upload
```

```bash
pio device monitor
```

Commands at the console: `G <mm>`, `GRIP <mm>`, `R`. Traffic arriving on the
relay link is echoed here too — with a one-way link, that echo is the only
evidence the ESP32 is reaching this board.
