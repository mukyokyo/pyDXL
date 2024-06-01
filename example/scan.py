#!/usr/bin/python3

# Tool to change ID using pyDXL

import sys, kbhit
from pyDXL import DXLProtocolV2

kb = kbhit.KBHit()
try:
  dx = DXLProtocolV2(sys.argv[1] if len(sys.argv) == 2 else '/dev/ttyAMA0', 9600, 0.01)
  for b in (9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000):
    dx.baudrate = b
    for id in range(253):
      if kb.kbhit():
        k = kb.getch()
        break
      r = dx.Ping(id)
      print(f' baud:{b:7} id:{id:3} ', end='find\n' if r else '\r')
  sys.stdout.write('\033[2K\033[1G')
except:
  print(' ERR:There is some problem.')
