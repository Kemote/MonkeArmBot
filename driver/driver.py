#!/usr/bin/env python3
"""Configure and move ST3020 bus servos: ID, middle position and min/max angle limits.

The servo speaks the Feetech STS protocol. On this robot it is reached through
the Waveshare ESP32 servo driver board in "Serial Forwarding" mode: the USB
side runs at 115200 baud and the board relays to the servo at 1 Mbaud.

Positions are raw steps: 0..4095 is one full turn and 2048 is the middle.

How this file is organised:
    servo functions   talk to the servos and return values (dicts, lists, numbers).
                      They never print. A request that cannot be carried out
                      raises ServoError.
    cmd_* functions   the command line. Each one calls a servo function and
                      prints what it returned.

Typical workflow for the servo (ID 1):
    python driver.py scan
    python driver.py torque 1 off        # then turn the horn by hand
    python driver.py set-middle 1        # current position becomes 2048
    python driver.py set-min 1           # at one end of travel: current position
    python driver.py set-max 1           # at the other end
    python driver.py set-min 1 1024      # or give positions directly
    python driver.py limits 1 --min 1024 --max 3072   # both at once
    python driver.py info 1

Moving the servo (the target must lie inside its limits):
    python driver.py move 1 middle
    python driver.py move 1 min
    python driver.py move 1 max
    python driver.py move 1 3000 --speed 1000 --acc 50

All detected servos, or only the IDs given with --ids:
    python driver.py info-all
    python driver.py move-all middle
    python driver.py move-all max --ids 1 2 --speed 300

Reading only what you need is faster, because only those registers are requested.
Put the servo ID before --fields:
    python driver.py info 1 --fields pos voltage
    python driver.py info-all --fields pos temp

Changing the ID, only if a servo needs one other than 1 (connect only that servo):
    python driver.py set-id 1 <new id>

Set the middle position before the limits: calibrating the middle shifts the
coordinate frame that the limits are measured in.
"""
import argparse
import sys
import time

import serial

from sts import STS, ID, MIN_LIMIT, MAX_LIMIT, TORQUE, LOCK, PRESENT_POS

OFFSET, GOAL_POS, VOLTAGE = 31, 42, 62  # temperature follows voltage at 63
TEMP = 63
READ_GAP = 10           # two registers share one read when at most this many bytes lie between them
                        # (on the ESP32 board a request costs about as much as 14 extra bytes)
CALIBRATE_MIDDLE = 128  # written to TORQUE: the current position becomes MIDDLE
POS_MAX, MIDDLE = 4095, 2048
ID_MAX = 253            # 254 (0xFE) is the broadcast ID
MODE, ACC, GOAL_SPEED, MOVING = 33, 41, 46, 66
SPEED_MAX = 4000        # steps per second
ACC_MAX = 254           # units of 100 steps/s^2, 0 means no ramp
ARRIVED = 15            # steps from the goal that still count as arrived
SCAN_LAST = 20          # scan() looks at IDs 0..SCAN_LAST unless told otherwise


class ServoError(Exception):
    """A request that cannot be carried out: servo missing, bad value, failed check."""


# ============================================================================
# Servo functions: no printing, no sys.exit. They return values or raise.
# ============================================================================

def u16(lo, hi):
    return lo | (hi << 8)


def signed(value, sign_bit):
    """Decode the servo's sign-magnitude numbers (sign bit 15 for position, 11 for offset)."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def deg(pos):
    return pos * 360 / (POS_MAX + 1)


def responds(m, sid):
    try:
        m._txrx(sid, 0x01)  # any status reply counts, even one with error bits set
        return True
    except IOError:
        return False


def require(m, sid):
    if not responds(m, sid):
        raise ServoError(f"servo {sid} does not respond (power? ID? serial forwarding on? baud?)")


def read_limits(m, sid):
    lo0, lo1, hi0, hi1 = m.read(sid, MIN_LIMIT, 4)
    return u16(lo0, lo1), u16(hi0, hi1)


def torque_is_on(m, sid):
    return m.read(sid, TORQUE, 1)[0] != 0


def scan(m, first=0, last=SCAN_LAST):
    """IDs from first to last that answer, in ascending order."""
    return [sid for sid in range(first, last + 1) if responds(m, sid)]


def resolve_ids(m, ids=None):
    """The IDs given, or when none are given every servo that scan() finds."""
    if not ids:
        ids = scan(m)
        if not ids:
            raise ServoError("no servo responds (power? serial forwarding on? baud?)")
    return sorted(set(ids))


# What get_info() can report. Each value is read from registers (address, bytes) and decoded.
INFO_REGISTERS = {
    "pos": (PRESENT_POS, 2, lambda b: signed(u16(*b), 15)),
    "limits": (MIN_LIMIT, 4, lambda b: [u16(b[0], b[1]), u16(b[2], b[3])]),
    "offset": (OFFSET, 2, lambda b: signed(u16(*b), 11)),
    "torque": (TORQUE, 1, lambda b: b[0] != 0),
    "eeprom": (LOCK, 1, lambda b: b[0] != 0),       # True when the EEPROM is locked
    "voltage": (VOLTAGE, 1, lambda b: b[0] / 10),   # the servo reports tenths of a volt
    "temp": (TEMP, 1, lambda b: b[0]),
}
# Computed from a value above, so asking for one reads the register of the value it uses.
INFO_DERIVED = {
    "pos_deg": ("pos", deg),
    "limits_deg": ("limits", lambda lim: [deg(lim[0]), deg(lim[1])]),
}
# status is the error byte that every reply carries, so it costs no extra read.
INFO_FIELDS = ("pos", "pos_deg", "limits", "limits_deg", "offset", "torque", "eeprom",
               "voltage", "temp", "status")


def info_names(fields):
    """The field names to report, in the usual order. None means all of INFO_FIELDS."""
    if fields is None:
        return list(INFO_FIELDS)
    if isinstance(fields, str):
        fields = [fields]
    unknown = [f for f in fields if f not in INFO_FIELDS]
    if unknown:
        raise ServoError(f"unknown field {', '.join(map(str, unknown))}; choose from {', '.join(INFO_FIELDS)}")
    return [f for f in INFO_FIELDS if f in fields]


def plan_blocks(spans, gap=READ_GAP):
    """Merge (address, length) spans into as few read requests as is worthwhile.

    Two spans share one request when at most gap unused bytes lie between them.
    Returns a list of (address, length) blocks.
    """
    blocks = []
    for addr, n in sorted(spans):
        if blocks and addr - (blocks[-1][0] + blocks[-1][1]) <= gap:
            start, length = blocks[-1]
            blocks[-1] = (start, max(length, addr + n - start))
        else:
            blocks.append((addr, n))
    return blocks


def read_registers(m, ids, blocks):
    """Read every (address, length) block from every servo in ids.

    Returns two dicts: {id: status} and {id: {address: byte}}. status is the error
    bytes of all replies or'ed together. One servo is asked with one request per
    block. Several servos are asked with one sync read per block, which asks them all
    at once and saves most of the waiting.
    """
    status = {sid: 0 for sid in ids}
    regs = {sid: {} for sid in ids}
    for addr, n in blocks:
        if len(ids) == 1:
            sid = ids[0]
            try:
                replies = {sid: m._txrx(sid, 0x02, [addr, n])}
            except IOError:
                require(m, sid)  # raises ServoError when the servo is simply not there
                raise
        else:
            replies = m.sync_read(ids, addr, n)
        for sid in ids:
            if sid not in replies:
                raise ServoError(f"servo {sid} does not respond (power? ID? serial forwarding on? baud?)")
            error, data = replies[sid]
            if len(data) != n:
                raise ServoError(f"servo {sid} sent {len(data)} bytes instead of {n}")
            status[sid] |= error
            regs[sid].update(zip(range(addr, addr + n), data))
    return status, regs


def get_info_all(m, ids=None, fields=None):
    """State of each servo in ids, or of every servo scan() finds, as a list of dicts.

    fields names what to read, from INFO_FIELDS, or None for all of it. Only the
    registers those fields need are requested, and registers close together share one
    request. Each dict has the servo's "id" plus the fields asked for.
    """
    ids = resolve_ids(m, ids)
    names = info_names(fields)
    wanted = {INFO_DERIVED[n][0] if n in INFO_DERIVED else n for n in names} - {"status"}
    # With only status asked for, any reply will do, so read the ID register.
    blocks = plan_blocks([INFO_REGISTERS[n][:2] for n in wanted]) or [(ID, 1)]
    status, regs = read_registers(m, ids, blocks)
    result = []
    for sid in ids:
        raw = {}
        for n in wanted:
            addr, size, decode = INFO_REGISTERS[n]
            raw[n] = decode([regs[sid][a] for a in range(addr, addr + size)])
        info = {"id": sid}
        for n in names:
            if n == "status":
                info[n] = status[sid]
            elif n in INFO_DERIVED:
                source, convert = INFO_DERIVED[n]
                info[n] = convert(raw[source])
            else:
                info[n] = raw[n]
        result.append(info)
    return result


def get_info(m, sid, fields=None):
    """State of one servo as a dict: position, limits, offset, torque, voltage, ...

    fields works as in get_info_all(): name what you need and only that is read."""
    return get_info_all(m, [sid], fields)[0]


def set_id(m, old, new):
    """Change a servo's ID and save it. Returns the ID read back from the servo."""
    if old == new:
        raise ServoError("old and new ID are the same")
    require(m, old)
    if responds(m, new):
        raise ServoError(f"ID {new} is already used by another servo on the bus")
    m.unlock(old)
    try:
        m.write1(old, ID, new)
    except IOError:
        pass  # the reply may be lost while the ID switches; checked below
    time.sleep(0.05)
    if not responds(m, new):
        if responds(m, old):
            m.lock(old)
        raise ServoError(f"ID change failed: servo does not answer as {new}")
    m.lock(new)
    got = m.read(new, ID, 1)[0]
    if got != new:
        raise ServoError(f"ID register reads {got}, expected {new}")
    return got


def set_middle(m, sid):
    """Make the servo's current position the middle (2048).

    Returns a dict with before, after, offset, torque and ok. ok is False when the
    position afterwards is not close to 2048, which means the calibration failed.
    """
    require(m, sid)
    before = m.read_pos(sid)
    m.unlock(sid)
    try:
        m.write1(sid, TORQUE, CALIBRATE_MIDDLE)
        time.sleep(0.1)
    finally:
        m.lock(sid)
    after = m.read_pos(sid)
    offset = signed(u16(*m.read(sid, OFFSET, 2)), 11)
    return {"id": sid, "before": before, "after": after, "offset": offset,
            "torque": torque_is_on(m, sid), "ok": abs(after - MIDDLE) <= 10}


def set_limits(m, sid, new_min=None, new_max=None):
    """Write the min and/or max limit and save it.

    new_min and new_max are positions 0..4095, 'here' for the current position, or
    None to keep that limit. A kept limit that is invalid (above 4095, as left by the
    board's web page) is reset to the full range instead of being saved again.
    Returns a dict with id, min, max, pos and notes (things worth telling the user).
    """
    require(m, sid)
    lo, hi = read_limits(m, sid)
    here = m.read_pos(sid)
    notes = []
    if "here" in (new_min, new_max) and not 0 <= here <= POS_MAX:
        raise ServoError(f"current position {here} is outside 0..{POS_MAX}")
    if new_min is not None:
        lo = here if new_min == "here" else new_min
    if new_max is not None:
        hi = here if new_max == "here" else new_max
    if new_min is None and lo > POS_MAX:
        notes.append(f"the servo's min limit {lo} is invalid, resetting it to 0")
        lo = 0
    if new_max is None and hi > POS_MAX:
        notes.append(f"the servo's max limit {hi} is invalid, resetting it to {POS_MAX}")
        hi = POS_MAX
    if not lo < hi:
        raise ServoError(f"min {lo} must be below max {hi}")
    m.unlock(sid)
    try:
        m.write2(sid, MIN_LIMIT, lo)
        m.write2(sid, MAX_LIMIT, hi)
    finally:
        m.lock(sid)
    got = read_limits(m, sid)
    if got != (lo, hi):
        raise ServoError(f"limits read back as {got[0]} .. {got[1]}, expected {lo} .. {hi}")
    if not lo <= here <= hi:
        notes.append(f"current position {here} is outside the new limits")
    if not lo <= MIDDLE <= hi:
        notes.append(f"the middle {MIDDLE} is outside the new limits, so 'move {sid} middle' will be refused")
    return {"id": sid, "min": lo, "max": hi, "pos": here, "notes": notes}


def set_torque(m, sid, on):
    """Turn torque on (holding the current position) or off. Returns the state read back."""
    require(m, sid)
    if on:
        pos = m.read_pos(sid)
        # Hold the current position instead of jumping to a stale goal.
        m.write2(sid, GOAL_POS, pos if pos >= 0 else (-pos) | 0x8000)
        m.write1(sid, TORQUE, 1)
    else:
        m.write1(sid, TORQUE, 0)
    return torque_is_on(m, sid)


def plan_move(m, sid, target):
    """Check that the servo can move to target and return the goal position in steps.

    target is 'middle', 'min', 'max' or a position 0..4095. Nothing is written.
    """
    require(m, sid)
    mode = m.read(sid, MODE, 1)[0]
    if mode != 0:
        raise ServoError(f"servo {sid} is in mode {mode}; position moves need mode 0 (servo mode)")
    lo, hi = read_limits(m, sid)
    if not lo < hi <= POS_MAX:
        raise ServoError(f"servo {sid} limits {lo} .. {hi} are invalid; set them first, for example:\n"
                         f"  python driver.py limits {sid} --min 0 --max {POS_MAX}")
    goal = {"middle": MIDDLE, "min": lo, "max": hi}.get(target, target)
    if isinstance(goal, str):
        raise ServoError(f"unknown target {goal!r}: use middle, min, max or a position")
    if not lo <= goal <= hi:
        raise ServoError(f"servo {sid}: target {goal} is outside the limits {lo} .. {hi}")
    return goal


def start_move(m, sid, target, speed=500, acc=50):
    """Start moving one servo to target and return at once.

    Returns a move dict (id, start, goal, speed, started) to give to wait_for_move().
    """
    if not 1 <= speed <= SPEED_MAX:
        raise ServoError(f"speed must be 1..{SPEED_MAX}")
    if not 0 <= acc <= ACC_MAX:
        raise ServoError(f"acc must be 0..{ACC_MAX}")
    goal = plan_move(m, sid, target)
    start = m.read_pos(sid)
    m.write1(sid, ACC, acc)
    m.write2(sid, GOAL_SPEED, speed)
    m.write2(sid, GOAL_POS, goal)  # this write starts the move
    if not torque_is_on(m, sid):
        m.write1(sid, TORQUE, 1)
    return {"id": sid, "start": start, "goal": goal, "speed": speed, "started": time.time()}


def wait_for_move(m, move):
    """Wait until the servo stops. Returns the move dict plus pos, error and reached.

    error is the final position minus the goal, in steps. reached is False when the
    servo stopped more than ARRIVED steps away, which means it is blocked or overloaded.
    """
    sid, goal = move["id"], move["goal"]
    began = move["started"]
    deadline = began + abs(goal - move["start"]) / move["speed"] + 2.0
    while True:
        time.sleep(0.05)
        pos = m.read_pos(sid)
        moving = m.read(sid, MOVING, 1)[0]
        settled = not moving and (abs(pos - goal) <= ARRIVED or time.time() - began > 0.5)
        if settled or time.time() > deadline:
            break
    return {**move, "pos": pos, "error": pos - goal, "reached": abs(pos - goal) <= ARRIVED}


def move(m, sid, target, speed=500, acc=50, wait=True):
    """Move one servo. Returns the finished move (see wait_for_move), or the started
    move when wait is False."""
    started = start_move(m, sid, target, speed, acc)
    return wait_for_move(m, started) if wait else started


def start_all(m, target, ids=None, speed=500, acc=50):
    """Start every servo moving to target. Returns the list of started moves.

    Every servo is checked before any of them moves, so one servo that cannot go to
    the target stops the whole request and none of them moves. The servos start one
    after another, a few hundredths of a second apart.
    """
    ids = resolve_ids(m, ids)
    for sid in ids:
        plan_move(m, sid, target)
    return [start_move(m, sid, target, speed, acc) for sid in ids]


def move_all(m, target, ids=None, speed=500, acc=50, wait=True):
    """Move the IDs given, or every servo scan() finds, to target.

    target is 'middle', 'min', 'max' or a position. 'min' and 'max' use each servo's
    own limits. Returns one dict per servo, as move() does.
    """
    started = start_all(m, target, ids, speed, acc)
    return [wait_for_move(m, mv) for mv in started] if wait else started


# ============================================================================
# Command line: call a servo function above, print what it returns.
# ============================================================================

INFO_TEXT = {   # how print_info shows the fields that are one value: name -> function
    "offset": lambda v: f"{v:5d}",
    "torque": lambda v: "on" if v else "off",
    "eeprom": lambda v: "locked" if v else "unlocked",
    "voltage": lambda v: f"{v:.1f} V",
    "temp": lambda v: f"{v} C",
    "status": lambda v: f"0x{v:02x}",
}


def print_info(info):
    """Print the fields that are in info. A full info looks as it always did."""
    print(f"servo {info['id']}")
    parts = []
    if "pos" in info:
        parts.append(f"{info['pos']:5d}")
    if "pos_deg" in info:
        parts.append(f"({info['pos_deg']:.1f} deg)")
    if parts:
        print(f"  {'position':<10}{'  '.join(parts)}")
    parts = []
    if "limits" in info:
        lo, hi = info["limits"]
        parts.append(f"{lo:5d} .. {hi}")
    if "limits_deg" in info:
        lo_deg, hi_deg = info["limits_deg"]
        parts.append(f"({lo_deg:.1f} .. {hi_deg:.1f} deg)")
    if parts:
        print(f"  {'limits':<10}{'  '.join(parts)}")
    for name, show in INFO_TEXT.items():
        if name in info:
            print(f"  {name:<10}{show(info[name])}")


def print_limits(result):
    lo, hi = result["min"], result["max"]
    print(f"servo {result['id']}: limits {lo} .. {hi} ({deg(lo):.1f} .. {deg(hi):.1f} deg) (saved)")
    for note in result["notes"]:
        print(f"  note: {note}")


def cmd_scan(m, a):
    if a.first > a.last:
        raise ServoError("--first must not be above --last")
    found = scan(m, a.first, a.last)
    print("found IDs:", " ".join(map(str, found)) if found else "none")


def cmd_info(m, a):
    print_info(get_info(m, a.id, a.fields))


def cmd_info_all(m, a):
    for info in get_info_all(m, a.ids, a.fields):
        print_info(info)


def cmd_set_id(m, a):
    new = set_id(m, a.old, a.new)
    print(f"servo {a.old} is now ID {new} (saved)")


def cmd_set_middle(m, a):
    r = set_middle(m, a.id)
    print(f"servo {a.id}: old position {r['before']} is now the middle")
    print(f"  position now {r['after']} (expected about {MIDDLE}), offset {r['offset']}")
    print(f"  torque {'on' if r['torque'] else 'off'}")
    if not r["ok"]:
        raise ServoError("calibration did not take effect")


def cmd_limits(m, a):
    if a.min is None and a.max is None:
        raise ServoError("give --min, --max or both")
    print_limits(set_limits(m, a.id, a.min, a.max))


def cmd_set_min(m, a):
    print_limits(set_limits(m, a.id, a.position, None))


def cmd_set_max(m, a):
    print_limits(set_limits(m, a.id, None, a.position))


def cmd_torque(m, a):
    on = set_torque(m, a.id, a.state == "on")
    print(f"servo {a.id}: torque {'on' if on else 'off'}")


def run_moves(m, ids, a):
    """Start the moves, then wait for each servo (unless --no-wait) and report."""
    moves = start_all(m, a.target, ids, a.speed, a.acc)
    for mv in moves:
        print(f"servo {mv['id']}: moving from {mv['start']} to {mv['goal']} "
              f"({deg(mv['goal']):.1f} deg) at {mv['speed']} steps/s")
    if a.no_wait:
        return
    several = len(moves) > 1
    missed = []
    for mv in moves:
        r = wait_for_move(m, mv)
        who = f"servo {r['id']}: " if several else ""
        print(f"  {who}stopped at {r['pos']} ({deg(r['pos']):.1f} deg), {r['error']:+d} steps from the target")
        if not r["reached"]:
            missed.append(str(r["id"]))
    if missed and several:
        raise ServoError(f"servo{'s' if len(missed) > 1 else ''} {', '.join(missed)} "
                         "did not reach the target (blocked, overloaded or still moving?)")
    if missed:
        raise ServoError("servo did not reach the target (blocked, overloaded or still moving?)")


def cmd_move(m, a):
    run_moves(m, [a.id], a)


def cmd_move_all(m, a):
    run_moves(m, a.ids, a)


# ---- argument parsing -------------------------------------------------------

def servo_id(text):
    v = int(text)
    if not 0 <= v <= ID_MAX:
        raise argparse.ArgumentTypeError(f"ID must be 0..{ID_MAX}")
    return v


def position(text):
    if text == "here":
        return text
    v = int(text)
    if not 0 <= v <= POS_MAX:
        raise argparse.ArgumentTypeError(f"position must be 0..{POS_MAX} or 'here'")
    return v


def target(text):
    if text in ("middle", "min", "max"):
        return text
    v = int(text)
    if not 0 <= v <= POS_MAX:
        raise argparse.ArgumentTypeError(f"target must be middle, min, max or 0..{POS_MAX}")
    return v


def bounded(name, lo, hi):
    def parse(text):
        v = int(text)
        if not lo <= v <= hi:
            raise argparse.ArgumentTypeError(f"{name} must be {lo}..{hi}")
        return v
    return parse


def add_fields_option(c):
    c.add_argument("--fields", nargs="+", choices=INFO_FIELDS, metavar="FIELD",
                   help="read and show only these (default: all): " + " ".join(INFO_FIELDS))


def add_move_options(c):
    c.add_argument("target", type=target, help=f"middle, min, max or a position 0..{POS_MAX}")
    c.add_argument("--speed", type=bounded("speed", 1, SPEED_MAX), default=500,
                   help="steps per second (default 500, about 44 deg/s)")
    c.add_argument("--acc", type=bounded("acc", 0, ACC_MAX), default=50,
                   help="acceleration in 100 steps/s^2, 0 means no ramp (default 50)")
    c.add_argument("--no-wait", action="store_true", help="return without waiting for the move to finish")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200,
                   help="115200 through the ESP32 board (default), 1000000 for a direct bus adapter")
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("scan", help="list servo IDs on the bus")
    c.add_argument("--first", type=servo_id, default=0)
    c.add_argument("--last", type=servo_id, default=SCAN_LAST)
    c.set_defaults(func=cmd_scan)

    c = sub.add_parser("info", help="show position, limits, offset, torque, voltage")
    c.add_argument("id", type=servo_id)
    add_fields_option(c)
    c.set_defaults(func=cmd_info)

    c = sub.add_parser("info-all", help="show the same for every detected servo")
    c.add_argument("--ids", type=servo_id, nargs="+", help="only these IDs (default: scan the bus)")
    add_fields_option(c)
    c.set_defaults(func=cmd_info_all)

    c = sub.add_parser("set-id", help="change a servo's ID")
    c.add_argument("old", type=servo_id)
    c.add_argument("new", type=servo_id)
    c.set_defaults(func=cmd_set_id)

    c = sub.add_parser("set-middle", help=f"make the current position the middle ({MIDDLE})")
    c.add_argument("id", type=servo_id)
    c.set_defaults(func=cmd_set_middle)

    c = sub.add_parser("limits", help="set min and/or max position limits")
    c.add_argument("id", type=servo_id)
    c.add_argument("--min", type=position, help=f"0..{POS_MAX}, or 'here' for the current position")
    c.add_argument("--max", type=position, help=f"0..{POS_MAX}, or 'here' for the current position")
    c.set_defaults(func=cmd_limits)

    for name, func, which in (("set-min", cmd_set_min, "min"), ("set-max", cmd_set_max, "max")):
        c = sub.add_parser(name, help=f"set the {which} position limit (default: the current position)")
        c.add_argument("id", type=servo_id)
        c.add_argument("position", type=position, nargs="?", default="here",
                       help=f"0..{POS_MAX}, or 'here' (default) for the current position")
        c.set_defaults(func=func)

    c = sub.add_parser("torque", help="turn torque on or off (off lets you turn the horn by hand)")
    c.add_argument("id", type=servo_id)
    c.add_argument("state", choices=["on", "off"])
    c.set_defaults(func=cmd_torque)

    c = sub.add_parser("move", help="move to middle, min, max or a position")
    c.add_argument("id", type=servo_id)
    add_move_options(c)
    c.set_defaults(func=cmd_move)

    c = sub.add_parser("move-all", help="move every detected servo to middle, min, max or a position")
    c.add_argument("--ids", type=servo_id, nargs="+", help="only these IDs (default: scan the bus)")
    add_move_options(c)
    c.set_defaults(func=cmd_move_all)

    a = p.parse_args()
    try:
        m = STS(a.port, a.baud)
    except serial.SerialException as e:
        sys.exit(f"cannot open {a.port}: {e}")
    try:
        a.func(m, a)
    except ServoError as e:
        sys.exit(str(e))
    except IOError as e:
        sys.exit(f"error: {e}")
    finally:
        m.close()


if __name__ == "__main__":
    # main()
    while True:
        m = STS("/dev/ttyUSB0", 115200)
        print(get_info_all(m, [1,2,3,4,5,6], ['pos']))
