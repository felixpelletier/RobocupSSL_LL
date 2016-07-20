#Under MIT License, see LICENSE.txt
#!/usr/bin/python
import serial_protocol as protocol
import os
import time
import serial

serial_ports = [port for port in os.listdir('/dev')
                if port.startswith("ttyACM")]
port = serial_ports[0]
print(port)

with serial.Serial('/dev/' + port, 115200, timeout=0) as serial:

    x = 0
    y = 0
    while True:
        time.sleep(1.0/120.0)
        x = ((x + 1) % 10)
        y = ((y + 2) % 9)

        print(x,y)
        sercommand = bytearray(protocol.create_speed_command(x, y, 4, 3))
        print("Sending %s" % str(sercommand))
        for i in range(100):
            serial.write(sercommand)
        print("Sent")
        while serial.in_waiting > 0:
            print(serial.readline())
