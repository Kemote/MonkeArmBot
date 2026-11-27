"""MonkeArmDriver: every servo of the arm behind one object.

The servos are ST3020s speaking the Feetech STS protocol. On this robot they are
reached through the Waveshare ESP32 servo driver board in "Serial Forwarding" mode:
the USB side runs at 115200 baud and the board relays to the servos at 1 Mbaud.

Units: angles in radians from each servo's calibrated middle (0 rad = 2048 steps),
speeds in rad/s, accelerations in rad/s². fields.py lists every value the driver can
read, with its unit and OPC UA data type.

    from driver.monkearm_driver import MonkeArmDriver

    with MonkeArmDriver("/dev/ttyUSB0") as arm:        # finds the servos, reads their settings
        arm.read(["position"])                           # {"Joint1": {"position": 0.12}, ...}
        arm.move({"Joint1": 0.5, 2: -0.2}, wait=True)    # by joint name or servo ID
        arm.joint(1).set_limits(minimum=-1.2, maximum=1.5)

Reading is as cheap as the request allows: settings come from a cache, and only the
registers of the fields asked for are read, for all joints in one sync read per block
of nearby registers. On the ESP32 board the positions of six servos take about 64 ms.

MonkeArmDriver is an Arm (arm.py), like the simulated VirtualArm, so front ends can
read either the same way.

The command line is in cli.py: python -m driver --help
"""
from . import sts
from .arm import Arm
from .fields import CONFIG_FIELDS
from .servo import (Servo, ServoError, DEFAULT_ACCELERATION, DEFAULT_SPEED,
                    move_data, read_fields, wait_for_moves)
from .sts import STS

SCAN_LAST = 20        # scan() looks at IDs 0..SCAN_LAST unless told otherwise
SCAN_TIMEOUT = 0.05   # seconds a scan waits for each ID; a reply takes about 20 ms on the ESP32 board
# Reading the MAC address resets the Waveshare board, so for testing it is a constant.
BOARD_MAC = "28-05-A5-C4-EF-34"


def scan(bus, first=0, last=SCAN_LAST, timeout=SCAN_TIMEOUT):
    """IDs from first to last that answer a ping, in ascending order."""
    return [sid for sid in range(first, last + 1) if bus.ping(sid, timeout)]


class MonkeArmDriver(Arm):
    """The arm: one Servo per joint, read and moved together.

    ids lists the servo IDs to use. Without it the bus is scanned (IDs 0..scan_last),
    which costs about 1 s; giving the IDs skips that. bus is an open STS to use
    instead of opening port; the caller then closes it. mac is the MAC address of the
    driver board, BOARD_MAC unless given.

    Raises ServoError when no servo answers, BusError when a servo in ids does not.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, ids=None, bus=None, scan_last=SCAN_LAST, mac=None):
        super().__init__(mac or BOARD_MAC)
        self._owns_bus = bus is None
        self.bus = STS(port, baud) if bus is None else bus
        try:
            ids = sorted(set(ids)) if ids else scan(self.bus, 0, scan_last)
            if not ids:
                raise ServoError("no servo responds (power? serial forwarding on? baud?)")
            config = read_fields(self.bus, ids, CONFIG_FIELDS)
            self._servos = {sid: Servo(self.bus, sid, config[sid]) for sid in ids}
        except BaseException:
            self.close()
            raise

    def close(self):
        if self._owns_bus:
            self.bus.close()

    # ---- reading -----------------------------------------------------------

    def _read_live(self, ids, names):
        return read_fields(self.bus, ids, names)

    def load_config(self):
        """Read every servo's settings again, for example after another program changed them."""
        config = read_fields(self.bus, self.ids, CONFIG_FIELDS)
        for sid, servo in self._servos.items():
            servo.config = config[sid]

    # ---- moving ------------------------------------------------------------

    def move(self, targets, speed=DEFAULT_SPEED, acceleration=DEFAULT_ACCELERATION, wait=False):
        """Move several joints at once.

        targets maps a joint name or servo ID to radians, 'middle', 'min' or 'max'.
        Every target is checked against its joint's limits before anything moves, so
        one bad target stops the whole request. The goals go out in one sync write, so
        the joints start together. Returns one move dict per joint, or with wait=True
        the finished moves (see wait()).
        """
        if not targets:
            return []
        planned = [(s, s.goal(t)) for s, t in ((self.joint(j), t) for j, t in targets.items())]
        data = {s.id: move_data(goal, speed, acceleration) for s, goal in planned}
        self.bus.sync_write(sts.ACC, data)                       # the goals start the moves
        self.bus.sync_write(sts.TORQUE, {sid: [1] for sid in data})
        moves = [s.new_move(goal, speed) for s, goal in planned]
        return self.wait(moves) if wait else moves

    def wait(self, moves):
        """Wait until the moved joints stop. Each move gets position, error (rad) and reached.

        reached is False for a joint that stopped away from its goal: blocked or overloaded.
        """
        return wait_for_moves(self.bus, moves)

    # ---- settings that change the joint list -------------------------------

    def set_id(self, joint, new):
        """Change a servo's ID and save it. Connect only that servo while doing so."""
        servo = self.joint(joint)
        got = servo.set_id(new)
        self._servos = {s.id: s for s in sorted(self._servos.values(), key=lambda s: s.id)}
        return got
