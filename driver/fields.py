"""Every value the driver can read from a servo, described so any front end can use it.

Each Field says where the value lives in the servo's registers, how the raw number
becomes a value in SI units, and how to publish it over OPC UA: the built-in data type
and the UNECE code for the EngineeringUnits property.

Angles are radians measured from the middle of the servo's range (2048 steps = 0 rad),
so a joint's zero is the middle set with Servo.calibrate_middle().

Fields with constant=True are settings stored in the servo. The driver reads them once
at start and serves them from a cache afterwards. The others are read from the bus
every time they are asked for.
"""
import math
from dataclasses import dataclass
from typing import Callable

from . import sts
from .sts import decode_signed

STEPS_PER_TURN = 4096
MIDDLE = 2048           # steps, 0 rad
POS_MAX = STEPS_PER_TURN - 1
RAD_PER_STEP = 2 * math.pi / STEPS_PER_TURN


def to_rad(steps):
    """A position in steps (0..4095) as radians from the middle."""
    return (steps - MIDDLE) * RAD_PER_STEP


def to_steps(rad):
    """A position in radians from the middle as the nearest step."""
    return round(rad / RAD_PER_STEP) + MIDDLE


@dataclass(frozen=True)
class Field:
    name: str               # key in the driver's result dicts and the OPC UA browse name
    address: int | None     # first register; None for status, which every reply carries
    size: int               # number of registers (bytes)
    decode: Callable        # raw register value (unsigned) -> value in the unit below
    ua_type: str            # OPC UA built-in data type: Double, Boolean, Byte or UInt16
    unit: str | None        # unit symbol; None for counts, codes and flags
    unece: str | None       # UNECE code of the unit, for OPC UA EngineeringUnits
    constant: bool          # a setting stored in the servo: read once, then cached
    description: str


RAD = ("rad", "C81")
RAD_S = ("rad/s", "2A")
RAD_S2 = ("rad/s²", "2B")
VOLT = ("V", "VLT")
CELSIUS = ("°C", "CEL")
PERCENT = ("%", "P1")
AMPERE = ("A", "AMP")
NO_UNIT = (None, None)

FIELDS = {f.name: f for f in (
    # ---- settings (EEPROM) -------------------------------------------------
    Field("model", sts.MODEL, 2, int, "UInt16", *NO_UNIT, True,
          "Model number the servo reports."),
    Field("min_limit", sts.MIN_LIMIT, 2, to_rad, "Double", *RAD, True,
          "Lowest goal position the servo accepts."),
    Field("max_limit", sts.MAX_LIMIT, 2, to_rad, "Double", *RAD, True,
          "Highest goal position the servo accepts."),
    Field("offset", sts.OFFSET, 2, lambda r: decode_signed(r, 11) * RAD_PER_STEP, "Double", *RAD, True,
          "Middle offset: shift between the encoder and the reported position. "
          "calibrate_middle() sets it so that the position at that moment reads 0 rad."),
    Field("mode", sts.MODE, 1, int, "Byte", *NO_UNIT, True,
          "Operating mode. 0 is position mode, the only one the driver moves in."),
    Field("max_temperature", sts.MAX_TEMP, 1, float, "Double", *CELSIUS, True,
          "Temperature at which the overheating protection trips."),
    Field("max_voltage", sts.MAX_VOLTAGE, 1, lambda r: r / 10, "Double", *VOLT, True,
          "Highest input voltage before the voltage protection trips."),
    Field("min_voltage", sts.MIN_VOLTAGE, 1, lambda r: r / 10, "Double", *VOLT, True,
          "Lowest input voltage before the voltage protection trips."),
    Field("max_torque", sts.MAX_TORQUE, 2, lambda r: r / 10, "Double", *PERCENT, True,
          "Torque limit loaded at power-up, in percent of the servo's maximum torque."),
    # ---- live state (RAM) --------------------------------------------------
    Field("position", sts.PRESENT_POS, 2, lambda r: to_rad(decode_signed(r, 15)), "Double", *RAD, False,
          "Present position."),
    Field("velocity", sts.PRESENT_SPEED, 2, lambda r: decode_signed(r, 15) * RAD_PER_STEP, "Double", *RAD_S, False,
          "Present speed. The sign gives the direction."),
    Field("load", sts.PRESENT_LOAD, 2, lambda r: decode_signed(r, 10) / 10, "Double", *PERCENT, False,
          "Present load in percent of the maximum torque. The sign gives the direction."),
    Field("current", sts.CURRENT, 2, lambda r: decode_signed(r, 15) * 0.0065, "Double", *AMPERE, False,
          "Present motor current."),
    Field("voltage", sts.VOLTAGE, 1, lambda r: r / 10, "Double", *VOLT, False,
          "Present input voltage."),
    Field("temperature", sts.TEMP, 1, float, "Double", *CELSIUS, False,
          "Present temperature."),
    Field("moving", sts.MOVING, 1, bool, "Boolean", *NO_UNIT, False,
          "True while the servo moves towards its goal."),
    Field("torque_enabled", sts.TORQUE, 1, bool, "Boolean", *NO_UNIT, False,
          "True while the motor holds its goal. False lets the joint turn freely."),
    Field("goal_position", sts.GOAL_POS, 2, lambda r: to_rad(decode_signed(r, 15)), "Double", *RAD, False,
          "Position the servo moves to or holds."),
    Field("goal_speed", sts.GOAL_SPEED, 2, lambda r: decode_signed(r, 15) * RAD_PER_STEP, "Double", *RAD_S, False,
          "Top speed of moves."),
    Field("acceleration", sts.ACC, 1, lambda r: r * 100 * RAD_PER_STEP, "Double", *RAD_S2, False,
          "Acceleration of moves. 0 means no ramp."),
    Field("torque_limit", sts.TORQUE_LIMIT, 2, lambda r: r / 10, "Double", *PERCENT, False,
          "Present torque limit in percent of the maximum torque. Resets to max_torque at power-up."),
    Field("eeprom_locked", sts.LOCK, 1, bool, "Boolean", *NO_UNIT, False,
          "True while the settings are protected from writes."),
    Field("status", None, 0, int, "Byte", *NO_UNIT, False,
          "Status byte of the replies. 0 is fine; each set bit is an alarm "
          "(voltage, sensor, temperature, current, angle, overload)."),
)}

CONFIG_FIELDS = [name for name, f in FIELDS.items() if f.constant]
LIVE_FIELDS = [name for name, f in FIELDS.items() if not f.constant]
