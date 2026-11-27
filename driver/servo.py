"""One ST3020 servo: its settings, its live state and every single-motor operation.

Units: angles in radians from the calibrated middle (0 rad = 2048 steps), speeds in
rad/s, accelerations in rad/s². See fields.py for every value that can be read.

read_fields() is the one place that reads values off the bus. It reads only the
registers the requested fields need, merges nearby registers into one request, and
asks several servos at once with a sync read.
"""
import time

from . import sts
from .fields import FIELDS, CONFIG_FIELDS, MIDDLE, POS_MAX, RAD_PER_STEP, to_rad, to_steps
from .sts import BusError, encode_signed, le16

ID_MAX = 253                  # 254 (0xFE) is the broadcast ID
OFFSET_MAX = 2047             # steps either way (sign in bit 11)
SPEED_MAX = 4000              # steps/s; Waveshare quotes about 3073 as the practical top
ACC_MAX = 254                 # register units of 100 steps/s²
DEFAULT_SPEED = 0.75          # rad/s, about 43 deg/s
DEFAULT_ACCELERATION = 7.5    # rad/s²
ARRIVED = 15 * RAD_PER_STEP   # a servo this close to its goal has arrived
CALIBRATE_MIDDLE = 128        # written to TORQUE: the present position becomes the middle
REQUEST_COST = 14             # one request on the ESP32 board takes about as long as 14 reply bytes
FRAME_BYTES = 6               # framing bytes around the data of every reply


class ServoError(Exception):
    """A request that cannot be carried out: bad value, target outside the limits, failed check."""


def field_names(fields=None):
    """The field names asked for, in the order of FIELDS. None means all of them."""
    if fields is None:
        return list(FIELDS)
    if isinstance(fields, str):
        fields = [fields]
    unknown = [f for f in fields if f not in FIELDS]
    if unknown:
        raise ServoError(f"unknown field {', '.join(map(str, unknown))}; choose from {', '.join(FIELDS)}")
    return [name for name in FIELDS if name in fields]


def plan_blocks(spans, servos=1):
    """Merge (address, length) spans into as few reads as pays off. Returns (address, length) blocks.

    A separate read costs one request plus the framing of every servo's reply. Reading
    across a gap costs its bytes once per servo. So the more servos one sync read asks,
    the smaller the gap worth reading across.
    """
    gap = REQUEST_COST // servos + FRAME_BYTES
    blocks = []
    for addr, n in sorted(spans):
        if blocks and addr - (blocks[-1][0] + blocks[-1][1]) <= gap:
            start, length = blocks[-1]
            blocks[-1] = (start, max(length, addr + n - start))
        else:
            blocks.append((addr, n))
    return blocks


def read_fields(bus, ids, names):
    """Read fields from the bus for every servo in ids. Returns {id: {field: value}}.

    One servo is asked with one read per block of registers, several servos with one
    sync read per block. status is the status bytes of all replies or'ed together.
    """
    ids = list(ids)
    spans = [(FIELDS[n].address, FIELDS[n].size) for n in names if FIELDS[n].address is not None]
    blocks = plan_blocks(spans, len(ids)) or [(sts.ID, 1)]   # status alone: any reply will do
    status = {sid: 0 for sid in ids}
    regs = {sid: {} for sid in ids}
    for addr, n in blocks:
        if len(ids) == 1:
            replies = {ids[0]: bus.read(ids[0], addr, n)}
        else:
            replies = bus.sync_read(ids, addr, n)
            missing = [str(sid) for sid in ids if sid not in replies]
            if missing:
                raise BusError(f"servo {', '.join(missing)} did not answer (power? serial forwarding on?)")
        for sid in ids:
            error, data = replies[sid]
            status[sid] |= error
            regs[sid].update(zip(range(addr, addr + n), data))
    result = {}
    for sid in ids:
        values = {}
        for name in names:
            f = FIELDS[name]
            if f.address is None:
                values[name] = status[sid]
            else:
                raw = bytes(regs[sid][a] for a in range(f.address, f.address + f.size))
                values[name] = f.decode(int.from_bytes(raw, "little"))
        result[sid] = values
    return result


def speed_steps(speed):
    """rad/s as the servo's speed register (steps/s)."""
    v = round(speed / RAD_PER_STEP)
    if not 1 <= v <= SPEED_MAX:
        raise ServoError(f"speed must be {RAD_PER_STEP:.4f} .. {SPEED_MAX * RAD_PER_STEP:.2f} rad/s")
    return v


def acc_units(acceleration):
    """rad/s² as the servo's acceleration register (units of 100 steps/s², 0 = no ramp)."""
    v = round(acceleration / (100 * RAD_PER_STEP))
    if acceleration > 0:
        v = max(v, 1)   # a small ramp must not round to 0, which means no ramp at all
    if not 0 <= v <= ACC_MAX or acceleration < 0:
        raise ServoError(f"acceleration must be 0 (no ramp) .. {ACC_MAX * 100 * RAD_PER_STEP:.1f} rad/s²")
    return v


def move_data(goal, speed, acceleration):
    """The 7 bytes written from ACC to start a move: acceleration, goal, goal time (unused), speed."""
    return [acc_units(acceleration), *le16(encode_signed(goal, 15)), 0, 0, *le16(speed_steps(speed))]


def wait_for_moves(bus, moves, poll=0.05):
    """Wait until every servo in moves has stopped. Returns the moves, each with
    position, error (final position minus goal, rad) and reached added.

    reached is False when a servo stopped more than ARRIVED away from its goal, which
    means it is blocked or overloaded, or when it was still moving at its deadline.
    All servos still moving are polled together, with one sync read.
    """
    pending = {mv["id"]: mv for mv in moves}
    deadline, done = {}, {}
    while pending:
        time.sleep(poll)
        now = time.time()
        state = read_fields(bus, list(pending), ["position", "moving"])
        for sid, mv in list(pending.items()):
            pos, moving = state[sid]["position"], state[sid]["moving"]
            if sid not in deadline:
                deadline[sid] = now + abs(mv["goal"] - pos) / mv["speed"] + 2.0
            close = abs(pos - mv["goal"]) <= ARRIVED
            settled = not moving and (close or now - mv["started"] > 0.5)
            if settled or now > deadline[sid]:
                done[sid] = {**mv, "position": pos, "error": pos - mv["goal"], "reached": close}
                del pending[sid]
    return [done[mv["id"]] for mv in moves]


class Servo:
    """One servo on the bus.

    Settings (the constant fields) are cached in self.config. They are read when the
    Servo is made, unless config is given, and again after every change made here.
    """

    def __init__(self, bus, sid, config=None):
        self.bus = bus
        self.id = sid
        self.config = config if config is not None else read_fields(bus, [sid], CONFIG_FIELDS)[sid]

    @property
    def name(self):
        return f"Joint{self.id}"

    def __repr__(self):
        return f"Servo({self.id})"

    # ---- reading -----------------------------------------------------------

    def load_config(self):
        """Read the settings from the servo again and cache them."""
        self.config = read_fields(self.bus, [self.id], CONFIG_FIELDS)[self.id]
        return self.config

    def read(self, fields=None):
        """This servo's values as {field: value}. fields names what is needed (None: all).

        Settings come from the cache. Only the live fields asked for are read from the bus.
        """
        names = field_names(fields)
        live = [n for n in names if not FIELDS[n].constant]
        values = read_fields(self.bus, [self.id], live)[self.id] if live else {}
        return {n: values[n] if n in values else self.config[n] for n in names}

    def position(self):
        return self.read("position")["position"]

    # ---- moving ------------------------------------------------------------

    def goal(self, target):
        """Check that the servo can move to target and return the goal in steps.

        target is radians, or 'middle', 'min' or 'max'. Nothing is written.
        """
        if self.config["mode"] != 0:
            raise ServoError(f"{self.name} is in mode {self.config['mode']}; moves need mode 0 (position mode)")
        lo, hi = to_steps(self.config["min_limit"]), to_steps(self.config["max_limit"])
        if not 0 <= lo < hi <= POS_MAX:
            raise ServoError(f"{self.name} limits {to_rad(lo):.4f} .. {to_rad(hi):.4f} rad are invalid; "
                             f"set them first, for example:\n"
                             f"  python -m driver limits {self.id} --min -3.14 --max 3.14")
        if isinstance(target, str):
            goal = {"middle": MIDDLE, "min": lo, "max": hi}.get(target)
            if goal is None:
                raise ServoError(f"unknown target {target!r}: use middle, min, max or radians")
        else:
            goal = to_steps(target)
        if not lo <= goal <= hi:
            raise ServoError(f"{self.name}: target {to_rad(goal):.4f} rad is outside the limits "
                             f"{to_rad(lo):.4f} .. {to_rad(hi):.4f} rad")
        return goal

    def new_move(self, goal, speed):
        """The move dict for a move to goal (steps) that has just been started."""
        return {"joint": self.name, "id": self.id, "goal": to_rad(goal),
                "speed": speed_steps(speed) * RAD_PER_STEP, "started": time.time()}

    def move(self, target, speed=DEFAULT_SPEED, acceleration=DEFAULT_ACCELERATION):
        """Start moving to target and return at once. Turns torque on.

        target is radians, or 'middle', 'min' or 'max'. A target outside the limits is
        refused. Returns a move dict for wait().
        """
        goal = self.goal(target)
        self.bus.write(self.id, sts.ACC, move_data(goal, speed, acceleration))  # the goal starts the move
        self.bus.write(self.id, sts.TORQUE, [1])
        return self.new_move(goal, speed)

    def wait(self, move):
        """Wait for a move started by move(). See wait_for_moves()."""
        return wait_for_moves(self.bus, [move])[0]

    # ---- settings ----------------------------------------------------------

    def _write_eeprom(self, addr, data, settle=0.0):
        """Unlock the settings, write, wait settle seconds, lock them again."""
        self.bus.write(self.id, sts.LOCK, [0])
        try:
            self.bus.write(self.id, addr, data)
            time.sleep(settle)
        finally:
            self.bus.write(self.id, sts.LOCK, [1])

    def set_limits(self, minimum=None, maximum=None):
        """Save new position limits. Each is radians, 'here' for the present position,
        or None to keep that limit.

        A kept limit that is invalid (as the board's web page leaves it) is reset to the
        end of the range instead of being saved again. Returns a dict with min_limit,
        max_limit, position and notes (things worth telling the user).
        """
        fresh = read_fields(self.bus, [self.id], ["min_limit", "max_limit", "position"])[self.id]
        lo, hi, here = (to_steps(fresh[n]) for n in ("min_limit", "max_limit", "position"))
        notes = []
        if "here" in (minimum, maximum) and not 0 <= here <= POS_MAX:
            raise ServoError(f"present position {fresh['position']:.4f} rad is outside the servo's range")
        new = {}
        for name, value in (("min", minimum), ("max", maximum)):
            if value is not None:
                new[name] = here if value == "here" else to_steps(value)
                if not 0 <= new[name] <= POS_MAX:
                    raise ServoError(f"{name} limit must be {to_rad(0):.4f} .. {to_rad(POS_MAX):.4f} rad")
        lo, hi = new.get("min", lo), new.get("max", hi)
        if "min" not in new and not 0 <= lo <= POS_MAX:
            notes.append(f"the min limit {to_rad(lo):.4f} rad was invalid, reset to {to_rad(0):.4f} rad")
            lo = 0
        if "max" not in new and not 0 <= hi <= POS_MAX:
            notes.append(f"the max limit {to_rad(hi):.4f} rad was invalid, reset to {to_rad(POS_MAX):.4f} rad")
            hi = POS_MAX
        if not lo < hi:
            raise ServoError(f"min {to_rad(lo):.4f} rad must be below max {to_rad(hi):.4f} rad")
        self._write_eeprom(sts.MIN_LIMIT, le16(lo) + le16(hi))   # both limits, registers 9..12
        self.load_config()
        got = to_steps(self.config["min_limit"]), to_steps(self.config["max_limit"])
        if got != (lo, hi):
            raise ServoError(f"limits read back as {to_rad(got[0]):.4f} .. {to_rad(got[1]):.4f} rad, "
                             f"expected {to_rad(lo):.4f} .. {to_rad(hi):.4f} rad")
        if not lo <= here <= hi:
            notes.append(f"the present position {to_rad(here):.4f} rad is outside the new limits")
        if not lo <= MIDDLE <= hi:
            notes.append(f"the middle (0 rad) is outside the new limits, so moves to 'middle' will be refused")
        return {"min_limit": to_rad(lo), "max_limit": to_rad(hi), "position": to_rad(here), "notes": notes}

    def calibrate_middle(self):
        """Make the present position the middle (0 rad) and save it.

        Do this before setting the limits: the limits are measured from the middle.
        Returns a dict with before, after, offset, torque_enabled and ok. ok is False
        when the position afterwards is not close to 0 rad, so the calibration failed.
        """
        before = self.position()
        self._write_eeprom(sts.TORQUE, [CALIBRATE_MIDDLE], settle=0.1)
        self.load_config()
        after = self.read(["position", "torque_enabled"])
        return {"before": before, "after": after["position"], "offset": self.config["offset"],
                "torque_enabled": after["torque_enabled"], "ok": abs(after["position"]) <= 10 * RAD_PER_STEP}

    def set_offset(self, offset):
        """Save the middle offset directly (radians), for example to restore a saved calibration.

        Like calibrate_middle(), it moves the frame that positions and limits are
        measured in. Returns the offset read back.
        """
        steps = round(offset / RAD_PER_STEP)
        if not -OFFSET_MAX <= steps <= OFFSET_MAX:
            raise ServoError(f"offset must be within ±{OFFSET_MAX * RAD_PER_STEP:.4f} rad")
        self._write_eeprom(sts.OFFSET, le16(encode_signed(steps, 11)))
        self.load_config()
        got = round(self.config["offset"] / RAD_PER_STEP)
        if got != steps:
            raise ServoError(f"offset read back as {self.config['offset']:.4f} rad, expected {steps * RAD_PER_STEP:.4f} rad")
        return self.config["offset"]

    def set_id(self, new):
        """Change the servo's ID and save it. Connect only this servo while doing so.

        Returns the ID read back from the servo.
        """
        if not 0 <= new <= ID_MAX:
            raise ServoError(f"ID must be 0..{ID_MAX}")
        if new == self.id:
            raise ServoError("old and new ID are the same")
        if self.bus.ping(new):
            raise ServoError(f"ID {new} is already used by another servo on the bus")
        old = self.id
        self.bus.write(old, sts.LOCK, [0])
        try:
            self.bus.write(old, sts.ID, [new])
        except BusError:
            pass  # the reply comes from the new ID or gets lost while the ID switches; checked below
        time.sleep(0.05)
        if not self.bus.ping(new):
            if self.bus.ping(old):
                self.bus.write(old, sts.LOCK, [1])
            raise ServoError(f"ID change failed: servo does not answer as {new}")
        self.id = new
        self.bus.write(new, sts.LOCK, [1])
        got = self.bus.read(new, sts.ID, 1)[1][0]
        if got != new:
            raise ServoError(f"ID register reads {got}, expected {new}")
        return got

    def set_torque(self, on):
        """Turn torque on (holding the present position) or off. Returns the state read back."""
        if on:
            # Hold the present position instead of jumping to an old goal.
            here = to_steps(self.position())
            self.bus.write(self.id, sts.GOAL_POS, le16(encode_signed(here, 15)))
        self.bus.write(self.id, sts.TORQUE, [1 if on else 0])
        return self.read("torque_enabled")["torque_enabled"]

    def set_torque_limit(self, percent):
        """Limit the torque to percent of the maximum until the next power-up. Returns it read back."""
        v = round(percent * 10)
        if not 0 <= v <= 1000:
            raise ServoError("torque limit must be 0 .. 100 %")
        self.bus.write(self.id, sts.TORQUE_LIMIT, le16(v))
        return self.read("torque_limit")["torque_limit"]
