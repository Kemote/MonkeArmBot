# MonkeArm servo driver

Python driver for the Waveshare ST3020 bus servos in MonkeArmBot. It finds the
servos, reads their state, moves the joints and changes their settings: ID, middle
position, middle offset, position limits and torque limit. The same driver is
used from Python, from the command line and by the OPC UA server (`../ua_server.py`).

**Units.** Angles are radians measured from the servo's calibrated middle, so
0 rad is the middle. Speeds are rad/s and accelerations rad/s². The raw steps
(0 .. 4095 per turn, 2048 at the middle) stay inside the driver.

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
4. Check that the servos answer. Run this from the repository root:

   ```
   source .venv/bin/activate
   python -m driver scan
   ```

The forwarding setting is not saved. **Turn it on again after every power-up or reset of the board.**

The Python environment is `.venv` in the repository root. The driver needs
`pyserial` (3.5 is installed). The OPC UA server also needs `asyncua` (2.1.0).

Only one program can use the port at a time. The driver opens it exclusively,
so stop the OPC UA server before running commands.

## Do not use the web page's servo buttons

The board runs Waveshare's demo firmware built for their **SC-series** servos.
Every servo button on its web page sends SC-protocol commands. SC servos use a
different register map and the opposite byte order, so the ST3020 misreads
them. Effects seen on this robot:

| Button | Effect on an ST3020 |
|---|---|
| Set Servo Mode | Limits become 5120 .. 60163 steps, which is garbage |
| Middle, Position+, Position- | Sends targets the servo misreads, so it moves to the wrong place |
| Set Middle Position | Does nothing in the SC build |
| Any button that unlocks or locks EEPROM | Lowers the servo's torque limit from 100 % to 76.9 % |

Use the web page **only** for Start/Stop Serial Forwarding. Do all servo work with this driver.

The firmware source is at <https://github.com/waveshare/Servo-Driver-with-ESP32>.
`ServoDriver/STSCTRL.h` holds a commented-out block for ST servos. That block
does not compile as shipped, for three reasons:

- `int SERVO_TYPE_SELECT = 1` is missing its semicolon.
- `MAX_MIN_OFFSET` is defined only in the SC block, but other code uses it.
- `CONNECT.h` calls `st.WritePos`, which the ST library (`SMS_STS`) does not have.

## Files

| File | What it is |
|---|---|
| `arm.py` | `Arm`: the base class of the real and the virtual arm. MAC address, joints, and reading with cached settings. |
| `monkearm_driver.py` | `MonkeArmDriver`: the whole arm. Finds the servos, caches their settings, reads and moves all joints together. |
| `virtual_arm.py` | `VirtualArm`: a simulated arm that moves at random and makes up its values, for testing without hardware. |
| `servo.py` | `Servo`: one motor. Reading, moving, and every setting: limits, middle, offset, ID, torque, torque limit. |
| `fields.py` | `FIELDS`: every value the driver can read, with register, unit, OPC UA data type and description. |
| `cli.py` | `ArmCli`: the command line, run as `python -m driver`. |
| `sts.py` | STS protocol: packets, register addresses, read, write, sync read, sync write. |
| `ping.py` | Raw ping of ID 1 at 1 Mbaud. It only works with a direct USB bus adapter. Through the ESP32 board use `python -m driver scan`. |

## Using the driver from Python

```python
from driver.monkearm_driver import MonkeArmDriver

with MonkeArmDriver("/dev/ttyUSB0") as arm:            # scans for servos, reads their settings
    arm.ids                                              # [1, 2, 3, 4, 5, 6]
    arm.read(["position"])                               # {"Joint1": {"position": 0.12}, ...}
    arm.read(["position", "temperature"], joints=[1, 2])
    arm.move({"Joint1": 0.5, 2: -0.2, 3: "middle"}, speed=1.0, wait=True)

    j1 = arm.joint(1)                                    # a Servo, by ID or by name "Joint1"
    j1.read(["position", "load"])
    j1.set_limits(minimum=-1.2, maximum="here")
    j1.calibrate_middle()
    j1.set_offset(0.0123)
    j1.set_torque(False)
```

- **Startup** pings IDs 0 .. 20 (about 1 s) and then reads every servo's settings.
  `MonkeArmDriver(ids=[1, 2, 3, 4, 5, 6])` skips the scan and takes about 0.2 s.
- **Settings are cached.** `model`, `min_limit`, `max_limit`, `offset`, `mode` and
  the protection limits are read once at startup and kept. Reading them costs no
  bus traffic. Every setter reads its value back and updates the cache.
  `arm.load_config()` reads them again, for example after another program changed them.
- **Ask only for what you need.** `read(fields)` reads only the registers those
  fields use. Registers close together share one request, and all joints are asked at
  once with one sync read. Through the ESP32 board this costs about 1 ms per byte:

  | Request for six joints | Time |
  |---|---|
  | `["position"]` | about 64 ms |
  | `["position", "temperature"]` | about 100 ms |
  | every live field | about 240 ms |
  | settings only | no bus traffic |

- **Moves are checked first.** `move()` checks every target against its joint's
  limits and mode before anything is sent, so one bad target stops the whole move.
  All goals then go out in one sync write, so the joints start together.
  `move()` returns at once. `wait()`, or `wait=True`, polls all moving joints
  together until they stop.
- **Errors.** `ServoError` means the request was refused or a check failed.
  `BusError` (an `IOError`) means a servo did not answer or its reply was damaged.
- **Real and virtual arms.** `MonkeArmDriver` and `VirtualArm` are both an `Arm`
  (`arm.py`). They share `mac`, `name` (`Arm_` and the MAC, for example
  `Arm_2805A5C4EF34`), `virtual`, `ids`, `joints`, `joint()` and `read()`. Only the
  real arm moves and changes settings. The real arm's MAC is `BOARD_MAC` in
  `monkearm_driver.py`, a constant because reading it resets the board.

  ```python
  from driver.virtual_arm import VirtualArm, virtual_mac

  arm = VirtualArm(virtual_mac(1))      # MAC 02-00-00-00-00-01, no hardware needed
  arm.read(["position", "load"])        # as on the real arm
  ```

## Values

`fields.FIELDS` describes each value. `constant` marks the settings that are cached.

| Field | Unit | Setting? | Meaning |
|---|---|---|---|
| `model` | | yes | Model number |
| `min_limit`, `max_limit` | rad | yes | Position limits |
| `offset` | rad | yes | Middle offset |
| `mode` | | yes | Operating mode, 0 = position mode |
| `max_temperature` | °C | yes | Overheating protection |
| `max_voltage`, `min_voltage` | V | yes | Voltage protection |
| `max_torque` | % | yes | Torque limit loaded at power-up |
| `position` | rad | | Present position |
| `velocity` | rad/s | | Present speed, signed |
| `load` | % | | Present load, signed |
| `current` | A | | Motor current |
| `voltage` | V | | Input voltage |
| `temperature` | °C | | Temperature |
| `moving` | | | True while moving |
| `torque_enabled` | | | True while the motor holds its goal |
| `goal_position` | rad | | Goal of the last move |
| `goal_speed` | rad/s | | Speed of the last move |
| `acceleration` | rad/s² | | Acceleration of the last move, 0 = no ramp |
| `torque_limit` | % | | Torque limit now |
| `eeprom_locked` | | | True while settings are write-protected |
| `status` | | | Status byte of the replies, 0 = no alarm |

## OPC UA server

`python ua_server.py` (from the repository root) serves
`opc.tcp://0.0.0.0:4840/monkeArm`. It serves the real arm and, for testing clients
with many arms, virtual arms that move at random and make up their values:

| Command | Arms served |
|---|---|
| `python ua_server.py` | The real arm on `/dev/ttyUSB0` and 9 virtual arms |
| `python ua_server.py --virtual 0` | Only the real arm |
| `python ua_server.py --no-real --virtual 3` | Three virtual arms, no hardware needed |
| `python ua_server.py --port /dev/ttyUSB1` | The real arm on another port, and 9 virtual arms |

- **Address space:** `Objects/<arm>/<JointN>/<field>`. Each arm is named after the MAC
  address of its driver board, for example `Arm_2805A5C4EF34`, and has the properties
  `MacAddress` (`28-05-A5-C4-EF-34`) and `Virtual`. Virtual arms have the MACs
  `02-00-00-00-00-01` and up: `02` marks a locally administered address, which no
  board comes with. Each joint (`Joint1` ..) has a `ServoId` property and one variable
  per field above, with its OPC UA data type and a description. To find the arms,
  browse `Objects` for the objects in namespace `urn:monkearm:arm`.
- **Node IDs** are strings made from the path, so they don't change between runs:
  `ns=<index of urn:monkearm:arm>;s=Arm_2805A5C4EF34.Joint1.position`. Properties add
  their name, for example `Arm_2805A5C4EF34.Joint1.position.EngineeringUnits`.
- **Values with a unit** are `AnalogUnitType` variables (OPC UA Part 8). They have an
  `EngineeringUnits` property with the UNECE unit code. Fields with a known range also
  have `EURange`: `position` and `goal_position` use the joint's limits, `load` is
  -100..100 % and `torque_limit` is 0..100 %. Other values are `BaseDataVariableType`.
- **Timing:** settings are published once. Positions are read every cycle, about 15
  times a second, and the other live values once a second. Each value's
  `SourceTimestamp` is the time it was read. Every arm has its own read loop and
  thread, so a slow or failed arm doesn't hold up the others.
- **Read failures:** if reading an arm fails (power off, cable out, forwarding off),
  that arm's live values get the status `BadCommunicationError` until a read works
  again. The other arms carry on. OPC UA requires a Bad value to be null, so clients
  get no value, and the `SourceTimestamp` still shows the last good read. Settings
  stay Good because they come from the cache. Clients should check the status before
  using a value.
- **Load:** with the real arm and 9 virtual arms the server uses about 16 % of one CPU core.

`monkearm_client.py` is a smoke test. It finds every arm, subscribes to all joint
positions for five seconds, and prints for each arm how many updates arrived and the
last positions.

### Virtual arms

`virtual_arm.py` simulates an arm in real time. It has the real arm's six joints, with
each limit shifted by up to 0.1 rad, and the real servos' settings. It reports every
field with the resolution of the servo's register.

- It rests 0.5 .. 3 s, then moves all joints together to a random pose inside the
  limits, at a random speed (0.3 .. 1.5 rad/s) and acceleration (2 .. 10 rad/s²), as
  `move()` does. Then it rests again.
- `load` is holding the joint against gravity, plus accelerating it, plus friction,
  plus noise. `current` follows the load, `voltage` drops as the arm draws more
  current, and `temperature` drifts slowly towards a level set by the load. A resting
  joint's position flickers by a step, as a real one does.
- `torque_enabled` and `eeprom_locked` are always on, and `status` is always 0.

The server seeds virtual arm *n* with *n*, so its settings are the same in every run.
Its moves differ. To change the data, change the numbers at the top of `virtual_arm.py`.

## Commands

Run from the repository root with the virtual environment active:

```
python -m driver <command> ...
```

Global options go before the command:

| Option | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port |
| `--baud` | `115200` | 115200 through the ESP32 board, 1000000 with a direct bus adapter |

A command that takes an ID works only with that servo and doesn't scan the bus.
Every write to permanent settings unlocks the servo's EEPROM, writes, locks it
again and reads the value back to confirm it.

### Reading

| Command | What it does |
|---|---|
| `scan [--first N] [--last N]` | Lists the IDs that answer. The default range is 0 .. 20, about 1 s. |
| `info ID [--fields F ...]` | Shows the servo's settings and state, or only the fields named. |
| `info-all [--ids ID ...] [--fields F ...]` | The same for every servo, or for the IDs given. |

### Configuration, saved in the servo

| Command | What it does |
|---|---|
| `set-id OLD NEW` | Changes the ID. It refuses if `NEW` is already on the bus. **Connect only the servo being changed.** |
| `set-middle ID` | Makes the servo's present position the middle, 0 rad. It fails if the servo doesn't read close to 0 afterwards. |
| `set-offset ID RAD` | Saves the middle offset directly, for example to restore a calibration. Range about ±3.14 rad. |
| `set-min ID [RAD]` | Sets the min limit. Without a value it uses the present position. |
| `set-max ID [RAD]` | Sets the max limit. Without a value it uses the present position. |
| `limits ID --min RAD --max RAD` | Sets one or both limits. `here` works as a value. |

The servo's range is -3.1416 .. 3.1401 rad (0 .. 4095 steps). The limit commands
refuse to save a min that isn't below the max. If a limit you leave unchanged is
invalid, they reset it to the end of the range. This happens after the web page
corrupts the limits. They print a note when they do this, and when the present
position or the middle ends up outside the new limits.

### Torque and movement

| Command | What it does |
|---|---|
| `torque ID off` | Releases the servo so you can turn the horn by hand. **A loaded joint will drop.** |
| `torque ID on` | Sets the goal to the present position first, then holds it, so the servo doesn't jump. |
| `torque-limit ID PERCENT` | Limits the torque until the next power-up. |
| `move ID TARGET` | Moves to `TARGET`: radians, or `middle` (0 rad), `min` or `max` (the servo's limits). |
| `move-all TARGET [--ids ID ...]` | Moves every servo, or the IDs given, to the same target. All start together. |

`move` and `move-all` options:

| Option | Default | Meaning |
|---|---|---|
| `--speed` | 0.75 | rad/s, about 43 degrees per second. Range 0.0015 .. 6.14. |
| `--acc` | 7.5 | rad/s². 0 means no ramp. Range 0 .. 39. |
| `--no-wait` | off | Return straight away instead of waiting for the move to end. |

A move is refused when a servo isn't in position mode, when its limits are
invalid, or when the target lies outside its limits. A move turns torque on. It
then waits and prints where each servo stopped. It exits with an error if a
servo stops more than 0.023 rad (15 steps) from its target, which usually means
the joint is blocked or overloaded.

## Setting up a new servo

1. Connect **only** the new servo to the board, and turn on serial forwarding.
2. Check that it answers as ID 1, the factory default. All examples here use ID 1.

   ```
   python -m driver scan
   ```

3. Release it, turn the horn by hand to the joint's centre, and save that as the middle:

   ```
   python -m driver torque 1 off
   python -m driver set-middle 1
   ```

4. Turn it by hand to one end of the joint's travel, then the other:

   ```
   python -m driver set-min 1
   python -m driver set-max 1
   ```

   Or give numbers: `python -m driver limits 1 --min -1.5 --max 1.5`.

5. Check the result and test it, with the arm clear:

   ```
   python -m driver info 1
   python -m driver move 1 middle
   python -m driver move 1 min
   python -m driver move 1 max
   ```

If a servo ever needs an ID other than 1, connect only that servo and run
`python -m driver set-id 1 <new id>`.

**Set the middle before the limits.** The limits are measured from the
calibrated middle, so calibrating afterwards shifts what the limits mean.
`set-offset` shifts them the same way.

## Registers used

All 2-byte values are little-endian. Addresses follow Feetech's STS memory table (`SMS_STS.h`).

| Address | Name | Notes |
|---|---|---|
| 3-4 | Model | EEPROM, read only |
| 5 | ID | EEPROM |
| 9-10 | Min position limit | EEPROM |
| 11-12 | Max position limit | EEPROM |
| 13 | Max temperature | EEPROM, °C |
| 14 | Max input voltage | EEPROM, units of 0.1 V |
| 15 | Min input voltage | EEPROM, units of 0.1 V |
| 16-17 | Max torque | EEPROM, units of 0.1 % |
| 31-32 | Middle offset | EEPROM, sign in bit 11 |
| 33 | Operating mode | 0 is position mode |
| 40 | Torque | 0 off, 1 on. Writing 128 makes the present position the middle. |
| 41 | Acceleration | Units of 100 steps/s² |
| 42-43 | Goal position | Sign in bit 15. Writing it starts a move. |
| 44-45 | Goal time | Not used, written as 0 |
| 46-47 | Goal speed | Steps per second |
| 48-49 | Torque limit | Units of 0.1 %, set from max torque at power-up |
| 55 | EEPROM lock | 0 unlocked, 1 locked |
| 56-57 | Present position | Sign in bit 15 |
| 58-59 | Present speed | Steps per second, sign in bit 15 |
| 60-61 | Present load | Units of 0.1 %, sign in bit 10 |
| 62 | Voltage | Units of 0.1 V |
| 63 | Temperature | °C |
| 66 | Moving flag | 1 while moving |
| 69-70 | Present current | Units of 6.5 mA, sign in bit 15 |

A move writes registers 41 .. 47 in one go (acceleration, goal, goal time, speed),
as Feetech's own `WritePosEx` does. Moving several joints sends that as one sync write.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `servo N does not respond` | Forwarding is off, usually after a board restart. Turn it on again. Also check power, the ID with `scan`, and `--baud`. |
| `cannot open /dev/ttyUSB0` | Another program has the port, often the OPC UA server. Stop it. |
| No reply at 1 Mbaud | Through the ESP32 board the PC side is 115200 baud. |
| Limits read 4.71 .. 89.15 rad | The web page's Set Servo Mode corrupted them. Fix with `limits ID --min -3.14 --max 3.14`, or your own limits. |
| A move says the limits are invalid | Same cause and fix as the corrupted limits. `set-min` and `set-max` repair the other limit on their own. |
| Servo is weaker than usual | Web page lock writes lowered the torque limit. Check `torque_limit` with `info`, and fix it with `torque-limit ID 100` or a power-up. |
| Servo goes about 90 degrees off from the middle you set | A web page button moved it. Check with `info ID`. Your offset is still stored, so `move ID middle` goes back. |
| A move reports it did not reach the target | The joint is blocked, overloaded, or the move took longer than expected. |

## Not yet verified

These were tested only against a simulated bus. Check them on the real arm:

- Moves on the real servos: the 7-byte write from register 41, sync write for several joints, and waiting on the moving flag.
- `torque on` on the real servo. It was not run, to avoid moving the arm unattended.
- That the scan's 50 ms wait per ID is long enough on the ESP32 board. A reply took about 20 ms when timed in November 2026.
- The registers first used here: 3-4, 13-17, 58-61 and 69-70. Their addresses and units come from Feetech's STS documentation.
- Whether saved settings survive a power cycle of the servo.
