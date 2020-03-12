#!/usr/bin/python3
import serial
import struct

ser = serial.Serial('/dev/ttyUSB1', 115200)
print(ser.name)
while True:
    data = ser.read(1)
    if data[0] == 0x0:
        break
    print(data[0], data)

print ("found null byte, receiving data")

f_count = 0
s_count = 0
frames_count = 1000
data_width = 3
samples_per_frame = 256
samples = []

while f_count < frames_count:
    while s_count <= samples_per_frame:
        data_1 = struct.unpack(">H",ser.read(2))[0]
        samples.append((data_1&0xff00)>>8)
        samples.append((data_1&0x00ff))
        data_1 = struct.unpack(">B",ser.read(1))[0]
        samples.append(data_1)

        s_count += data_width

    f_count+=1
    s_count = 0
    print(f"Already received {f_count}")

with open("data.raw", "wb") as f:
    f.write(bytearray(samples))

