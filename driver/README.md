# Servo configuration tools

Tools for setting up and testing the Waveshare ST3020 bus servos in MonkeArmBot.
They set a servo's ID, its middle position and its min/max position limits.
They can also move a servo to the middle, to either limit or to any position.

## How the PC reaches the servos

```
PC ──USB──> CP2102 (/dev/ttyUSB0) ──> ESP32 on Waveshare "Servo Driver with ESP32" ──1 Mbaud bus──> ST3020 servos
```

The USB port belongs to the ESP32, not to the servo bus. The ESP32 passes PC
traffic to the servos only while the board is in **serial forwarding** mode.
In that mode the PC side runs at **115200 baud**. The board talks to the servos
at 1 Mbaud.

The servos use the Feetech STS protocol: `FF FF ID LEN INSTR PARAMS CHECKSUM`.

## Setup

1. Power the board from the servo supply. The servos report about 12.2 V on this setup.
2. Plug in the USB cable. The board shows up as `/dev/ttyUSB0`. Your user must be in the `dialout` group.
3. Turn on serial forwarding:
   - Join the Wi-Fi network `ESP32_DEV`, password `12345678`.
   - Open `http://192.168.4.1` in a browser.
   - Press **Start Serial Forwarding**.
4. Check that the servos answer:

   ```
   cd driver
   source .venv/bin/activate
   python motors_cfg.py scan
   ```

The forwarding setting is not saved. **Turn it on again after every power-up or reset of the board.**

The Python environment is `driver/.venv` and needs `pyserial`. Version 3.5 is installed.

## Do not use the web page's servo buttons

The board runs Waveshare's demo firmware built for their **SC-series** servos.
Every servo button on its web page sends SC-protocol commands. SC servos use a
different register map and the opposite byte order, so the ST3020 misreads
them. Effects seen on this robot:

| Button | Effect on an ST3020 |
|---|---|
| Set Servo Mode | Limits become 5120 .. 60163, which is garbage |
| Middle, Position+, Position- | Sends targets the servo misreads, so it moves to the wrong place |
| Set Middle Position | Does nothing in the SC build |
| Any button that unlocks or locks EEPROM | Lowers the servo's torque limit from 1000 to 769 |

Use the web page **only** for Start/Stop Serial Forwarding. Do all servo work with `motors_cfg.py`.

The firmware source is at <https://github.com/waveshare/Servo-Driver-with-ESP32>.
`ServoDriver/STSCTRL.h` holds a commented-out block for ST servos. That block
does not compile as shipped, for three reasons:

- `int SERVO_TYPE_SELECT = 1` is missing its semicolon.
- `MAX_MIN_OFFSET` is defined only in the SC block, but other code uses it.
- `CONNECT.h` calls `st.WritePos`, which the ST library (`SMS_STS`) does not have.

## Files

| File | What it is |
|---|---|
| `motors_cfg.py` | Command-line tool for configuring and moving servos. Everything below is about this file. |
| `sts.py` | Minimal STS protocol client: ping, read, 1- and 2-byte writes, EEPROM lock/unlock, read position. |
| `ping.py` | Raw ping of ID 1 at 1 Mbaud. It only works with a direct USB bus adapter. Through the ESP32 board, change the baud to 115200 or use `motors_cfg.py scan`. |

## Positions

- One full turn is 0 .. 4095 steps, about 11.4 steps per degree.
- **2048 is the middle.** Positions are measured relative to the calibrated middle, so `middle` shifts every position.
- Speeds are in steps per second. Waveshare gives about 3073 steps/s as the practical maximum for ST servos.

## Commands

Run every command from the `driver` folder with the virtual environment active:

```
source .venv/bin/activate
python motors_cfg.py <command> ...
```

The examples further down shorten this to `motors_cfg.py`.

Global options go before the command:

| Option | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port |
| `--baud` | `115200` | 115200 through the ESP32 board, 1000000 with a direct bus adapter |

Every command that takes an ID checks first that the servo answers. Every write to permanent
settings unlocks the servo's EEPROM, writes, locks it again and reads the value
back to confirm it.

### Reading

| Command | What it does |
|---|---|
| `scan [--first N] [--last N]` | Lists the IDs that answer. The default range is 0 .. 20, which takes about 2 s. |
| `info ID` | Shows position, limits, middle offset, torque, EEPROM lock, voltage, temperature and status byte. |

### Configuration, saved in the servo

| Command | What it does |
|---|---|
| `set-id OLD NEW` | Changes the ID. It refuses if `NEW` is already on the bus. **Connect only the servo being changed.** |
| `middle ID` | Makes the servo's current position the middle, 2048. It fails if the servo doesn't read close to 2048 afterwards. |
| `set-min ID [POS]` | Sets the min limit. Without `POS` it uses the current position. |
| `set-max ID [POS]` | Sets the max limit. Without `POS` it uses the current position. |
| `limits ID --min POS --max POS` | Sets one or both limits. `here` works as a position. |

The limit commands refuse to save a min that isn't below the max. If a limit
you leave unchanged is invalid, they reset it to the full range: 0 for the min,
4095 for the max. This happens after the web page corrupts the limits, for
example a max of 60163. They print a note when they do this, and when the
current position or the middle ends up outside the new limits.

### Torque and movement

| Command | What it does |
|---|---|
| `torque ID off` | Releases the servo so you can turn the horn by hand. **A loaded joint will drop.** |
| `torque ID on` | Sets the goal to the current position first, then holds it, so the servo doesn't jump. |
| `move ID middle` | Moves to 2048. |
| `move ID min` | Moves to the servo's min limit. |
| `move ID max` | Moves to the servo's max limit. |
| `move ID POS` | Moves to any position from 0 to 4095. |

`move` options:

| Option | Default | Meaning |
|---|---|---|
| `--speed` | 500 | Steps per second, about 44 degrees per second. Range 1 .. 4000. |
| `--acc` | 50 | Acceleration in units of 100 steps/s². 0 means no ramp. Range 0 .. 254. |
| `--no-wait` | off | Return straight away instead of waiting for the move to end. |

`move` refuses when the servo isn't in position mode, when its limits are
invalid, or when the target lies outside the limits. It turns torque on if it is
off. It then waits and prints where the servo stopped. It exits with an error if
the servo stops more than 15 steps from the target, which usually means the
joint is blocked or overloaded.

## Setting up a new servo

1. Connect **only** the new servo to the board, and turn on serial forwarding.
2. Check that it answers as ID 1, the factory default. All examples here use ID 1.

   ```
   motors_cfg.py scan
   ```

3. Release it and turn the horn by hand to the joint's centre:

   ```
   motors_cfg.py torque 1 off
   motors_cfg.py middle 1
   ```

4. Turn it by hand to one end of the joint's travel, then the other:

   ```
   motors_cfg.py set-min 1
   motors_cfg.py set-max 1
   ```

   Or give numbers: `motors_cfg.py limits 1 --min 1024 --max 3072`.

5. Check the result and test it, with the arm clear:

   ```
   motors_cfg.py info 1
   motors_cfg.py move 1 middle
   motors_cfg.py move 1 min
   motors_cfg.py move 1 max
   ```

If a servo ever needs an ID other than 1, connect only that servo and run
`motors_cfg.py set-id 1 <new id>`.

**Set the middle before the limits.** The limits are measured from the
calibrated middle, so calibrating afterwards shifts what the limits mean.

## Registers used

All 2-byte values are little-endian.

| Address | Name | Notes |
|---|---|---|
| 5 | ID | EEPROM |
| 9-10 | Min position limit | EEPROM |
| 11-12 | Max position limit | EEPROM |
| 31-32 | Middle offset | EEPROM, sign in bit 11 |
| 33 | Operating mode | 0 is position mode |
| 40 | Torque | 0 off, 1 on. Writing 128 makes the current position 2048. |
| 41 | Acceleration | Units of 100 steps/s² |
| 42-43 | Goal position | Writing it starts a move |
| 46-47 | Goal speed | Steps per second |
| 48-49 | Torque limit | Set from max torque at power-up |
| 55 | EEPROM lock | 0 unlocked, 1 locked |
| 56-57 | Present position | Sign in bit 15 |
| 62 | Voltage | Units of 0.1 V |
| 63 | Temperature | °C |
| 66 | Moving flag | 1 while moving |

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `servo N does not respond` | Forwarding is off, usually after a board restart. Turn it on again. Also check power, the ID with `scan`, and `--baud`. |
| No reply at 1 Mbaud | Through the ESP32 board the PC side is 115200 baud. |
| Limits read 5120 .. 60163 | The web page's Set Servo Mode corrupted them. Fix with `limits ID --min 0 --max 4095`, or your own limits. |
| `move` says the limits are invalid | Same cause and fix as the corrupted limits. `set-min` and `set-max` repair the other limit on their own. |
| Servo is weaker than usual | Web page lock writes lowered the torque limit to 769. It should return to 1000 at the next power-up. |
| Servo goes about 90 degrees off from the middle you set | A web page button moved it. Check with `info ID`. Your offset is still stored, so `move ID middle` goes back. |
| `move` reports it did not reach the target | The joint is blocked, overloaded, or the move took longer than expected. |

## Not yet verified

- Whether saved settings survive a power cycle of the servo.
- `move` on the real servo. It was tested only against a simulated servo, because the servo's limits were corrupted at the time.
- `torque on` on the real servo. It was not run, to avoid moving the arm unattended.
- Whether the servo's moving flag behaves as `move` expects when it waits.
