"""VirtualArm: a simulated arm that moves at random and makes up its values.

It is for testing front ends, such as the OPC UA server, with many arms and no
hardware. Like the real MonkeArmDriver it is an Arm (arm.py): it has the same joints
and the same fields, in the same units and with the same resolution as the servos'
registers. Its MAC comes from virtual_mac(), a locally administered address that no
real board has.

    from driver.virtual_arm import VirtualArm, virtual_mac

    arm = VirtualArm(virtual_mac(1))     # 02-00-00-00-00-01, with joints like the real arm's
    arm.read(["position", "load"])       # {"Joint1": {"position": 0.43, "load": 3.1}, ...}

How the values are made up:

- Moves: the arm rests for a while. Then it picks a random pose inside the joint
  limits, and a random speed and acceleration, and moves all joints there together,
  as MonkeArmDriver.move() does. Each joint speeds up and slows down at that
  acceleration and stops at its goal. Then the arm rests again.
- Load: holding the joint against gravity, plus accelerating it, plus friction, plus noise.
- Current follows the load. The supply voltage drops as the whole arm draws more
  current. The temperature drifts slowly towards a level set by the load.

The simulation runs in real time. Every read first brings it up to the present, so
the values don't depend on how often the arm is read. The numbers below set the
model. Change them to change the data.
"""
import math
import random
import time

from .arm import Arm
from .fields import RAD_PER_STEP
from .servo import DEFAULT_ACCELERATION, DEFAULT_SPEED, acc_units, speed_steps

# ---- the synthetic data ----------------------------------------------------

# The joints, like the real arm's: limits in rad as read from it on 2026-11-27, and the
# load in % of the maximum torque that holding the joint level against gravity takes
# (made up). Each virtual arm shifts every limit by up to LIMIT_JITTER, so arms differ.
JOINTS = {
    #    min_limit  max_limit  gravity load
    1: (-2.3086,    2.1936,     3.0),
    2: (-2.0816,    1.4266,    30.0),
    3: (-1.5693,    1.3054,    20.0),
    4: (-3.1201,    3.1078,     8.0),
    5: (-2.3224,    1.3177,     5.0),
    6: (-0.5031,    0.8406,     2.0),
}
LIMIT_JITTER = 0.1                 # rad
# The other settings, as the real ST3020s report them. Each joint gets a random offset.
SETTINGS = {"model": 1289, "mode": 0, "max_temperature": 80.0, "max_voltage": 14.0,
            "min_voltage": 6.0, "max_torque": 100.0}
OFFSET_RANGE = 3.0                 # rad either way

MOVE_SPEED = (0.3, 1.5)            # rad/s: each move's speed is drawn from this range
MOVE_ACCELERATION = (2.0, 10.0)    # rad/s²
REST = (0.5, 3.0)                  # s between moves
GOAL_MARGIN = 0.1                  # rad: goals stay this far inside the limits

ACCELERATION_LOAD = 2.0            # % of the maximum torque per rad/s²
FRICTION_LOAD = 4.0                # % per rad/s
LOAD_NOISE = 0.5                   # %, standard deviation
POSITION_NOISE = 0.5 * RAD_PER_STEP  # rad, standard deviation: a resting joint flickers by a step
IDLE_CURRENT = 0.01                # A
CURRENT_PER_LOAD = 0.025           # A per % of load
SUPPLY_VOLTAGE = 12.2              # V when no current is drawn
SUPPLY_RESISTANCE = 0.1            # ohm: the voltage drops by this times the arm's total current
VOLTAGE_NOISE = 0.03               # V, standard deviation
AMBIENT = (28.0, 32.0)             # °C: each joint's temperature with no load is drawn from this range
HEATING = 0.3                      # °C above that per % of load, once settled
THERMAL_TIME = 300.0               # s: how slowly the temperature follows the load

STEP = 0.01                        # s per simulation step
MAX_CATCH_UP = 10.0                # s: after a longer pause between reads, the arm skips the rest

# Resolution of the servo registers
LOAD_UNIT = 0.1                    # %
CURRENT_UNIT = 0.0065              # A
VOLTAGE_UNIT = 0.1                 # V


def virtual_mac(n):
    """The MAC of virtual arm n (1 .. 65535): 02-00-00-00-00-01 and up.

    The 02 marks a locally administered address, which no board comes with.
    """
    if not 1 <= n <= 0xFFFF:
        raise ValueError("virtual arms are numbered 1 .. 65535")
    return f"02-00-00-00-{n >> 8:02X}-{n & 0xFF:02X}"


def quantize(value, unit):
    """value rounded to a whole number of units, as a servo register holds it."""
    return round(value / unit) * unit


def stored_speed(speed):
    """A speed (rad/s) as the servo stores it: whole steps per second."""
    return speed_steps(speed) * RAD_PER_STEP


def stored_acceleration(acceleration):
    """An acceleration (rad/s²) as the servo stores it: whole units of 100 steps/s²."""
    return acc_units(acceleration) * 100 * RAD_PER_STEP


class VirtualJoint:
    """One simulated servo: its settings in config, as a Servo has them, and its motion."""

    def __init__(self, sid, rng):
        low, high, gravity = JOINTS[sid]
        self.id = sid
        self.name = f"Joint{sid}"
        self.config = {
            **SETTINGS,
            "min_limit": quantize(low + rng.uniform(-LIMIT_JITTER, LIMIT_JITTER), RAD_PER_STEP),
            "max_limit": quantize(high + rng.uniform(-LIMIT_JITTER, LIMIT_JITTER), RAD_PER_STEP),
            "offset": quantize(rng.uniform(-OFFSET_RANGE, OFFSET_RANGE), RAD_PER_STEP),
        }
        self.gravity = gravity
        self.goal = self.position = self.random_goal(rng)   # starts at rest
        self.velocity = self.acceleration = 0.0              # now
        self.speed = stored_speed(DEFAULT_SPEED)             # of the moves: goal_speed
        self.ramp = stored_acceleration(DEFAULT_ACCELERATION)  # of the moves: acceleration
        self.ambient = rng.uniform(*AMBIENT)
        self.temperature = self.ambient

    def __repr__(self):
        return f"VirtualJoint({self.id})"

    @property
    def moving(self):
        return self.position != self.goal

    def random_goal(self, rng):
        """A random goal inside the limits, GOAL_MARGIN in from them, on a whole step."""
        low = self.config["min_limit"] + GOAL_MARGIN
        high = self.config["max_limit"] - GOAL_MARGIN
        if low > high:
            low = high = (self.config["min_limit"] + self.config["max_limit"]) / 2
        return quantize(rng.uniform(low, high), RAD_PER_STEP)

    def load(self):
        """The load in % of the maximum torque, without noise."""
        return (self.gravity * math.sin(self.position) + ACCELERATION_LOAD * self.acceleration
                + FRICTION_LOAD * self.velocity)

    def step(self, dt):
        """Move on by dt seconds: towards the goal, no faster than speed, speeding up and
        slowing down at ramp. The temperature follows the load."""
        to_go = self.goal - self.position
        # the fastest the joint may go and still stop at the goal
        allowed = math.copysign(min(self.speed, math.sqrt(2 * self.ramp * abs(to_go))), to_go)
        change = max(-self.ramp * dt, min(self.ramp * dt, allowed - self.velocity))
        self.velocity += change
        self.acceleration = change / dt
        if abs(to_go) <= abs(self.velocity) * dt:   # it arrives during this step
            self.position, self.velocity = self.goal, 0.0
        else:
            self.position += self.velocity * dt
        settled = self.ambient + HEATING * abs(self.load())
        self.temperature += (settled - self.temperature) * dt / THERMAL_TIME


class VirtualArm(Arm):
    """A simulated arm with the joints in JOINTS, moving at random.

    mac is its MAC address, for example virtual_mac(1). seed makes its settings the
    same in every run; without it they differ.
    """

    virtual = True

    def __init__(self, mac, seed=None):
        super().__init__(mac)
        self._rng = random.Random(seed)
        self._servos = {sid: VirtualJoint(sid, self._rng) for sid in sorted(JOINTS)}
        self._time = time.monotonic()                            # the simulation's present
        self._rest_until = self._time + self._rng.uniform(*REST)  # None while moving

    def _read_live(self, ids, names):
        self._advance(time.monotonic())
        rng = self._rng
        joints = self._servos.values()
        supply = SUPPLY_VOLTAGE - SUPPLY_RESISTANCE * sum(
            IDLE_CURRENT + CURRENT_PER_LOAD * abs(j.load()) for j in joints)
        result = {}
        for sid in ids:
            j = self._servos[sid]
            load = max(-100.0, min(100.0, j.load() + rng.gauss(0, LOAD_NOISE)))
            values = {
                "position": quantize(j.position + rng.gauss(0, POSITION_NOISE), RAD_PER_STEP),
                "velocity": quantize(j.velocity, RAD_PER_STEP),
                "load": quantize(load, LOAD_UNIT),
                "current": quantize(IDLE_CURRENT + CURRENT_PER_LOAD * abs(load), CURRENT_UNIT),
                "voltage": quantize(supply + rng.gauss(0, VOLTAGE_NOISE), VOLTAGE_UNIT),
                "temperature": float(round(j.temperature)),
                "moving": j.moving,
                "torque_enabled": True,
                "goal_position": j.goal,
                "goal_speed": j.speed,
                "acceleration": j.ramp,
                "torque_limit": j.config["max_torque"],
                "eeprom_locked": True,
                "status": 0,
            }
            result[sid] = {n: values[n] for n in names}
        return result

    def _advance(self, now):
        """Run the simulation up to now, in steps of STEP: rest, start a move, move."""
        self._time = max(self._time, now - MAX_CATCH_UP)
        joints = self._servos.values()
        while self._time < now:
            dt = min(STEP, now - self._time)
            if self._rest_until is None:
                if not any(j.moving for j in joints):   # all arrived: rest
                    self._rest_until = self._time + self._rng.uniform(*REST)
            elif self._time >= self._rest_until:
                self._start_move()
                self._rest_until = None
            for j in joints:
                j.step(dt)
            self._time += dt

    def _start_move(self):
        """Send every joint to a random goal, all at one random speed and acceleration."""
        speed = stored_speed(self._rng.uniform(*MOVE_SPEED))
        ramp = stored_acceleration(self._rng.uniform(*MOVE_ACCELERATION))
        for j in self._servos.values():
            j.goal, j.speed, j.ramp = j.random_goal(self._rng), speed, ramp
