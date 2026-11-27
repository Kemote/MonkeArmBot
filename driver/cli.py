"""Command line for MonkeArmDriver: configure, read and move the arm's servos.

Angles are radians from the servo's calibrated middle (0 rad), speeds rad/s,
accelerations rad/s². Run from the repository root:

    python -m driver scan
    python -m driver info 1
    python -m driver info-all --fields position temperature
    python -m driver torque 1 off            # then turn the horn by hand
    python -m driver set-middle 1            # the present position becomes 0 rad
    python -m driver set-min 1               # at one end of travel: the present position
    python -m driver set-max 1               # at the other end
    python -m driver limits 1 --min -1.2 --max 1.5
    python -m driver set-offset 1 0.0123     # restore a saved middle offset
    python -m driver torque-limit 1 50       # until the next power-up
    python -m driver move 1 middle
    python -m driver move 1 0.5 --speed 1.0 --acc 5
    python -m driver move-all min --ids 1 2

Changing the ID, only if a servo needs one other than 1 (connect only that servo):
    python -m driver set-id 1 <new id>

Set the middle before the limits: calibrating the middle shifts the frame that the
limits are measured in.
"""
import argparse
import sys

import serial

from .fields import FIELDS
from .monkearm_driver import MonkeArmDriver, SCAN_LAST, scan
from .servo import DEFAULT_ACCELERATION, DEFAULT_SPEED, ID_MAX, ServoError
from .sts import STS

DIGITS = {"rad": 4, "rad/s": 4, "rad/s²": 3, "A": 4, "V": 1, "%": 1, "°C": 0}


def show(name, value):
    """A value as text for people, with its unit."""
    if name == "status":
        return f"0x{value:02x}"
    if isinstance(value, bool):
        return "yes" if value else "no"
    unit = FIELDS[name].unit
    if unit is None:
        return str(value)
    return f"{value:.{DIGITS[unit]}f} {unit}"


def print_values(servo, values):
    print(f"{servo.name} (ID {servo.id})")
    for name, value in values.items():
        print(f"  {name:<17}{show(name, value)}")


def print_limits(servo, result):
    print(f"{servo.name}: limits {result['min_limit']:.4f} .. {result['max_limit']:.4f} rad (saved)")
    for note in result["notes"]:
        print(f"  note: {note}")


# ---- argument types ---------------------------------------------------------

def servo_id(text):
    v = int(text)
    if not 0 <= v <= ID_MAX:
        raise argparse.ArgumentTypeError(f"ID must be 0..{ID_MAX}")
    return v


def angle_or(*words):
    """An argument type: radians, or one of words."""
    def parse(text):
        if text in words:
            return text
        try:
            return float(text)
        except ValueError:
            raise argparse.ArgumentTypeError(f"give radians or {', '.join(words)}") from None
    return parse


class ArmCli:
    """Parses the command line, opens the arm with only the servos the command needs,
    runs the command and prints what the driver returns."""

    def __init__(self, argv=None):
        self.args = self.parser().parse_args(argv)

    def run(self):
        a = self.args
        try:
            bus = STS(a.port, a.baud)
        except serial.SerialException as e:
            sys.exit(f"cannot open {a.port}: {e}")
        with bus:
            try:
                if a.func == ArmCli.cmd_scan:
                    self.cmd_scan(bus)
                else:
                    a.func(self, MonkeArmDriver(bus=bus, ids=self.ids_needed()))
            except (ServoError, IOError) as e:
                sys.exit(str(e))

    def ids_needed(self):
        """The servos the command works on; None makes the driver scan the bus."""
        a = self.args
        for name in ("id", "old"):
            if hasattr(a, name):
                return [getattr(a, name)]
        return a.ids

    # ---- commands -----------------------------------------------------------

    def cmd_scan(self, bus):
        a = self.args
        if a.first > a.last:
            raise ServoError("--first must not be above --last")
        found = scan(bus, a.first, a.last)
        print("found IDs:", " ".join(map(str, found)) if found else "none")

    def cmd_info(self, arm):
        for name, values in arm.read(self.args.fields).items():
            print_values(arm.joint(name), values)

    def cmd_set_id(self, arm):
        new = arm.set_id(self.args.old, self.args.new)
        print(f"servo {self.args.old} is now ID {new} (saved)")

    def cmd_set_middle(self, arm):
        servo = arm.joint(self.args.id)
        r = servo.calibrate_middle()
        print(f"{servo.name}: old position {r['before']:.4f} rad is now the middle")
        print(f"  position now {r['after']:.4f} rad (expected about 0), offset {r['offset']:.4f} rad")
        print(f"  torque {'on' if r['torque_enabled'] else 'off'}")
        if not r["ok"]:
            raise ServoError("calibration did not take effect")

    def cmd_set_offset(self, arm):
        servo = arm.joint(self.args.id)
        print(f"{servo.name}: offset {servo.set_offset(self.args.offset):.4f} rad (saved)")

    def cmd_limits(self, arm):
        a = self.args
        if a.min is None and a.max is None:
            raise ServoError("give --min, --max or both")
        servo = arm.joint(a.id)
        print_limits(servo, servo.set_limits(a.min, a.max))

    def cmd_set_min(self, arm):
        servo = arm.joint(self.args.id)
        print_limits(servo, servo.set_limits(minimum=self.args.position))

    def cmd_set_max(self, arm):
        servo = arm.joint(self.args.id)
        print_limits(servo, servo.set_limits(maximum=self.args.position))

    def cmd_torque(self, arm):
        servo = arm.joint(self.args.id)
        on = servo.set_torque(self.args.state == "on")
        print(f"{servo.name}: torque {'on' if on else 'off'}")

    def cmd_torque_limit(self, arm):
        servo = arm.joint(self.args.id)
        print(f"{servo.name}: torque limit {servo.set_torque_limit(self.args.percent):.1f} % (until power-up)")

    def cmd_move(self, arm):
        a = self.args
        targets = {sid: a.target for sid in arm.ids}
        moves = arm.move(targets, a.speed, a.acc)
        for mv in moves:
            print(f"{mv['joint']}: moving to {mv['goal']:.4f} rad at {mv['speed']:.3f} rad/s")
        if a.no_wait:
            return
        missed = []
        for r in arm.wait(moves):
            print(f"  {r['joint']}: stopped at {r['position']:.4f} rad, {r['error']:+.4f} rad from the target")
            if not r["reached"]:
                missed.append(r["joint"])
        if missed:
            raise ServoError(f"{', '.join(missed)} did not reach the target (blocked, overloaded or still moving?)")

    # ---- argument parsing ----------------------------------------------------

    @staticmethod
    def parser():
        p = argparse.ArgumentParser(prog="python -m driver", description=__doc__,
                                    formatter_class=argparse.RawDescriptionHelpFormatter)
        p.add_argument("--port", default="/dev/ttyUSB0")
        p.add_argument("--baud", type=int, default=115200,
                       help="115200 through the ESP32 board (default), 1000000 for a direct bus adapter")
        sub = p.add_subparsers(dest="cmd", required=True)

        def command(name, func, help_text, servo=True):
            c = sub.add_parser(name, help=help_text)
            if servo:
                c.add_argument("id", type=servo_id)
            c.set_defaults(func=func)
            return c

        def fields_option(c):
            c.add_argument("--fields", nargs="+", choices=list(FIELDS), metavar="FIELD",
                           help="read and show only these (default: all): " + " ".join(FIELDS))

        def move_options(c):
            c.add_argument("target", type=angle_or("middle", "min", "max"),
                           help="radians, or middle, min, max (each servo's own limits)")
            c.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                           help=f"rad/s (default {DEFAULT_SPEED})")
            c.add_argument("--acc", type=float, default=DEFAULT_ACCELERATION,
                           help=f"rad/s², 0 means no ramp (default {DEFAULT_ACCELERATION})")
            c.add_argument("--no-wait", action="store_true", help="return without waiting for the move to end")

        c = command("scan", ArmCli.cmd_scan, "list servo IDs on the bus", servo=False)
        c.add_argument("--first", type=servo_id, default=0)
        c.add_argument("--last", type=servo_id, default=SCAN_LAST)

        fields_option(command("info", ArmCli.cmd_info, "show one servo's settings and state"))

        c = command("info-all", ArmCli.cmd_info, "show the same for every servo", servo=False)
        c.add_argument("--ids", type=servo_id, nargs="+", help="only these IDs (default: scan the bus)")
        fields_option(c)

        c = command("set-id", ArmCli.cmd_set_id, "change a servo's ID", servo=False)
        c.add_argument("old", type=servo_id)
        c.add_argument("new", type=servo_id)

        command("set-middle", ArmCli.cmd_set_middle, "make the present position the middle (0 rad)")

        c = command("set-offset", ArmCli.cmd_set_offset, "save the middle offset directly")
        c.add_argument("offset", type=float, help="radians")

        c = command("limits", ArmCli.cmd_limits, "set min and/or max position limits")
        c.add_argument("--min", type=angle_or("here"), help="radians, or 'here' for the present position")
        c.add_argument("--max", type=angle_or("here"), help="radians, or 'here' for the present position")

        for name, func, which in (("set-min", ArmCli.cmd_set_min, "min"), ("set-max", ArmCli.cmd_set_max, "max")):
            c = command(name, func, f"set the {which} position limit (default: the present position)")
            c.add_argument("position", type=angle_or("here"), nargs="?", default="here",
                           help="radians, or 'here' (default) for the present position")

        c = command("torque", ArmCli.cmd_torque, "turn torque on or off (off lets you turn the horn by hand)")
        c.add_argument("state", choices=["on", "off"])

        c = command("torque-limit", ArmCli.cmd_torque_limit, "limit the torque until the next power-up")
        c.add_argument("percent", type=float, help="percent of the maximum torque, 0..100")

        move_options(command("move", ArmCli.cmd_move, "move to radians, middle, min or max"))

        c = command("move-all", ArmCli.cmd_move, "move every servo to the same target", servo=False)
        c.add_argument("--ids", type=servo_id, nargs="+", help="only these IDs (default: scan the bus)")
        move_options(c)
        return p


def main(argv=None):
    ArmCli(argv).run()


if __name__ == "__main__":
    main()
