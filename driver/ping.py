import serial
s = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.5)
s.write(bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]))  # ping ID 1
print(s.read(6).hex())   # expect ffff010200fc