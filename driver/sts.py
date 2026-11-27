"""Feetech STS serial bus protocol, as spoken by the ST3020 servos.

Every packet is FF FF ID LEN INSTR PARAMS CHECKSUM. This module builds packets, sends
them and checks the replies. It knows the register addresses but not what the values
mean: fields.py and servo.py do that.
"""
import serial

# Instructions
PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE    # every servo obeys a packet sent to this ID, and none of them replies

# Register addresses. Two-byte values are little-endian.
# EEPROM: kept through power-off. LOCK must be 0 while writing here.
MODEL = 3           # 2 bytes, read only
ID = 5
MIN_LIMIT = 9       # 2 bytes
MAX_LIMIT = 11      # 2 bytes
MAX_TEMP = 13
MAX_VOLTAGE = 14
MIN_VOLTAGE = 15
MAX_TORQUE = 16     # 2 bytes
OFFSET = 31         # 2 bytes, sign in bit 11
MODE = 33
# RAM: lost at power-off.
TORQUE = 40         # 0 off, 1 on, 128 makes the present position the middle
ACC = 41
GOAL_POS = 42       # 2 bytes, sign in bit 15
GOAL_TIME = 44      # 2 bytes
GOAL_SPEED = 46     # 2 bytes, sign in bit 15
TORQUE_LIMIT = 48   # 2 bytes
LOCK = 55           # 0 unlocks the EEPROM, 1 locks it
PRESENT_POS = 56    # 2 bytes, sign in bit 15
PRESENT_SPEED = 58  # 2 bytes, sign in bit 15
PRESENT_LOAD = 60   # 2 bytes, sign in bit 10
VOLTAGE = 62
TEMP = 63
MOVING = 66
CURRENT = 69        # 2 bytes, sign in bit 15

BYTE_TIME = 0.002   # seconds allowed per reply byte: the ESP32 board relays about one byte per ms


class BusError(IOError):
    """A servo did not answer, or its reply was cut short or damaged."""


def decode_signed(value, sign_bit):
    """Decode the servo's sign-magnitude numbers: bit sign_bit set means negative."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def encode_signed(value, sign_bit):
    return (-value) | (1 << sign_bit) if value < 0 else value


def le16(value):
    """A 2-byte register value as [low byte, high byte]."""
    return [value & 0xFF, (value >> 8) & 0xFF]


def checksum(body):
    return (~sum(body)) & 0xFF


class STS:
    """One serial connection to the servo bus.

    115200 baud through the Waveshare ESP32 board (serial forwarding on), 1000000 with
    a direct bus adapter. The port is opened exclusively, so a second program cannot
    mix its packets with ours.
    """

    def __init__(self, port, baud=115200, timeout=0.1):
        self.serial = serial.Serial(port, baud, timeout=timeout, exclusive=True)
        self.timeout = timeout

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.serial.close()

    def _send(self, sid, instr, params):
        body = [sid, len(params) + 2, instr, *params]
        if body[1] > 0xFF:
            raise ValueError(f"packet too long: {len(params)} parameter bytes")
        self.serial.reset_input_buffer()
        self.serial.write(bytes([0xFF, 0xFF, *body, checksum(body)]))

    def _receive(self, n, timeout):
        """Up to n bytes, waiting at most timeout seconds for all of them."""
        if self.serial.timeout != timeout:
            self.serial.timeout = timeout
        return self.serial.read(n)

    def request(self, sid, instr, params=(), reply_len=0):
        """Send one packet to servo sid and return its reply as (status byte, data bytes).

        reply_len is the number of data bytes the reply carries. Raises BusError when the
        servo does not answer or the reply is damaged.
        """
        self._send(sid, instr, list(params))
        expect = reply_len + 6
        reply = self._receive(expect, self.timeout + expect * BYTE_TIME)
        if not reply:
            raise BusError(f"servo {sid} does not respond (power? ID? serial forwarding on? baud?)")
        if (len(reply) != expect or reply[:2] != b"\xff\xff" or reply[2] != sid
                or reply[3] != reply_len + 2 or checksum(reply[2:-1]) != reply[-1]):
            raise BusError(f"servo {sid} sent a damaged reply: {reply.hex(' ')}")
        return reply[4], list(reply[5:-1])

    def ping(self, sid, timeout=None):
        """True when anything answers as servo sid, whatever its status byte says."""
        self._send(sid, PING, [])
        return bool(self._receive(6, (self.timeout if timeout is None else timeout) + 6 * BYTE_TIME))

    def read(self, sid, addr, n):
        """Read n registers from addr. Returns (status byte, data bytes)."""
        return self.request(sid, READ, [addr, n], n)

    def write(self, sid, addr, data):
        """Write the bytes in data from addr on. Returns the status byte of the reply."""
        return self.request(sid, WRITE, [addr, *data])[0]

    def sync_read(self, ids, addr, n):
        """Ask several servos for n registers from addr with one packet (instruction 0x82).

        Returns {id: (status byte, data bytes)} for each servo whose reply arrived intact.
        A servo that does not answer is missing from the result.
        """
        self._send(BROADCAST, SYNC_READ, [addr, n, *ids])
        expect = len(ids) * (n + 6)
        buf = self._receive(expect, self.timeout + expect * BYTE_TIME)
        replies, i = {}, 0
        while i + 6 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            sid, length = buf[i + 2], buf[i + 3]
            pkt = buf[i:i + 4 + length]
            if length == n + 2 and len(pkt) == 4 + length and checksum(pkt[2:-1]) == pkt[-1]:
                replies[sid] = (pkt[4], list(pkt[5:-1]))
                i += len(pkt)
            else:
                i += 1
        return replies

    def sync_write(self, addr, data):
        """Write to several servos with one broadcast packet (instruction 0x83).

        data maps each servo ID to its bytes, the same number for every servo. No servo
        replies, so nothing confirms that the write arrived.
        """
        sizes = {len(values) for values in data.values()}
        if len(sizes) != 1:
            raise ValueError("sync_write needs the same number of bytes for every servo")
        params = [addr, sizes.pop()]
        for sid, values in data.items():
            params += [sid, *values]
        self._send(BROADCAST, SYNC_WRITE, params)
        self.serial.flush()
