#!/usr/bin/env python

import sys
import serial
import re

if len(sys.argv) < 2:
    print("Please input comx and baudrate !")
    exit(-1)
 
portx   = sys.argv[1]
bps     = sys.argv[2]

print('open ' + portx + ' with baudrate %s ...' % bps)

ser = serial.Serial(portx, int(bps), timeout=1, parity=serial.PARITY_NONE, stopbits=1)  
 
if ser.isOpen():
    print("open " + portx + " success !")

while True:
    line = ser.readline()
    if line:
        line.decode('ascii', 'ignore')
        try:
            line = str(line, 'utf-8')
            print(line, end="")
        except UnicodeDecodeError:
            pass; # ignore unicode decode error

ser.close()
