import serial

ID, MIN_LIMIT, MAX_LIMIT, TORQUE, LOCK, PRESENT_POS = 5, 9, 11, 40, 55, 56

class STS:
    def __init__(self, port, baud=1_000_000, timeout=0.1):
        self.serial = serial.Serial(port, baud, timeout=timeout)

    def _txrx(self, sid, instr, params=()):
        body = [sid, len(params) + 2, instr, *params]
        self.serial.reset_input_buffer()
        self.serial.write(bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF]))
        hdr = self.serial.read(4)
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF:
            raise IOError(f"brak odpowiedzi od serwa {sid} (zasilanie? ID? baud?)")
        rest = self.serial.read(hdr[3])
        if len(rest) < hdr[3]:
            raise IOError("ucięta odpowiedź")
        return rest[0], list(rest[1:-1])   # (kod błędu, dane)

    def ping(self, sid):
        return self._txrx(sid, 0x01)[0] == 0

    def read(self, sid, addr, n):
        return self._txrx(sid, 0x02, [addr, n])[1]

    def sync_read(self, ids, addr, n):
        """Ask several servos for n registers from addr with one request (instruction 0x82).

        Returns {id: (error byte, data)} for each servo whose reply arrived intact.
        A servo that does not answer is simply missing from the result.
        """
        body = [0xFE, len(ids) + 4, 0x82, addr, n, *ids]
        expect = len(ids) * (n + 6)               # every reply is n data bytes plus 6 framing bytes
        old = self.serial.timeout
        self.serial.timeout = old + expect * 0.002   # the board relays about one byte per millisecond
        try:
            self.serial.reset_input_buffer()
            self.serial.write(bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF]))
            buf = self.serial.read(expect)
        finally:
            self.serial.timeout = old
        replies, i = {}, 0
        while i + 6 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            sid, length = buf[i + 2], buf[i + 3]
            pkt = buf[i:i + 4 + length]
            if len(pkt) == 4 + length and (~sum(pkt[2:-1])) & 0xFF == pkt[-1]:
                replies[sid] = (pkt[4], list(pkt[5:-1]))
                i += len(pkt)
            else:
                i += 1
        return replies

    def write1(self, sid, addr, val):
        self._txrx(sid, 0x03, [addr, val & 0xFF])

    def write2(self, sid, addr, val):
        self._txrx(sid, 0x03, [addr, val & 0xFF, (val >> 8) & 0xFF])

    def unlock(self, sid): self.write1(sid, LOCK, 0)
    def lock(self, sid):   self.write1(sid, LOCK, 1)

    def read_pos(self, sid):
        lo, hi = self.read(sid, PRESENT_POS, 2)
        v = lo | (hi << 8)
        return -(v & 0x7FFF) if v & 0x8000 else v

    def close(self):
        self.serial.close()