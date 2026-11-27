"""Arm: what the real arm (MonkeArmDriver) and a simulated one (VirtualArm) have in common.

An arm is known by the MAC address of its driver board and has one joint per servo.
Each joint has an id, a name ('Joint1') and its settings cached in config; the real
arm's joints are Servos. read() returns the values of fields.FIELDS in their units,
so a front end such as the OPC UA server works the same with either kind of arm:

    arm.name, arm.mac, arm.virtual    # "Arm_2805A5C4EF34", "28-05-A5-C4-EF-34", False
    arm.read(["position"])            # {"Joint1": {"position": 0.12}, ...}

A subclass fills self._servos and implements _read_live(); settings come from the
cache, so it is only asked for the live values.
"""
import re
from abc import ABC, abstractmethod

from .fields import FIELDS
from .servo import ServoError, field_names

MAC_DIGITS = re.compile(r"[0-9A-F]{12}")


def normalize_mac(mac):
    """A MAC address written as 28-05-A5-C4-EF-34. Takes ':' or '-' between the bytes, or nothing."""
    digits = mac.replace(":", "").replace("-", "").upper()
    if not MAC_DIGITS.fullmatch(digits):
        raise ValueError(f"{mac!r} is not a MAC address")
    return "-".join(digits[i:i + 2] for i in range(0, 12, 2))


class Arm(ABC):
    """An arm: its joints, read together."""

    virtual = False   # True for a simulated arm

    def __init__(self, mac):
        self.mac = normalize_mac(mac)
        self._servos = {}   # {servo ID: joint}, in ID order

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        """Release what the arm holds, such as its serial port."""

    def __repr__(self):
        return f"{type(self).__name__}({self.mac}, ids={self.ids})"

    @property
    def name(self):
        """'Arm_' and the MAC without separators: the arm's name in OPC UA."""
        return "Arm_" + self.mac.replace("-", "")

    # ---- joints ------------------------------------------------------------

    @property
    def ids(self):
        return list(self._servos)

    @property
    def joints(self):
        """{joint name: joint}, in ID order."""
        return {s.name: s for s in self._servos.values()}

    def joint(self, key):
        """A joint, by name ('Joint1') or servo ID (1)."""
        for s in self._servos.values():
            if key == s.id or key == s.name:
                return s
        raise ServoError(f"no joint {key!r}; the arm has {', '.join(self.joints)}")

    def _select(self, joints):
        return list(self._servos.values()) if joints is None else [self.joint(j) for j in joints]

    # ---- reading -----------------------------------------------------------

    def read(self, fields=None, joints=None):
        """Values of every joint, or of the joints given, as {joint name: {field: value}}.

        fields names what is needed now, from fields.FIELDS; None means all of it.
        Settings come from the cache. Live fields come from _read_live(), for all the
        joints at once.
        """
        servos = self._select(joints)
        names = field_names(fields)
        live = [n for n in names if not FIELDS[n].constant]
        values = self._read_live([s.id for s in servos], live) if live else {}
        result = {}
        for s in servos:
            got = values.get(s.id, {})
            result[s.name] = {n: got[n] if n in got else s.config[n] for n in names}
        return result

    @abstractmethod
    def _read_live(self, ids, names):
        """The live fields names of the servos ids, as {servo ID: {field: value}}."""
