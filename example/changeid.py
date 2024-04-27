#!/usr/bin/python3

# Tool to change ID using pyDXL

import sys, time
from pyDXL import DXLProtocolV2

if len(sys.argv) == 4:
  arg = sys.argv[1:]
  dev = '\\\\.\\COM10'
elif len(sys.argv) == 5:
  arg = sys.argv[2:]
  dev = sys.argv[1]
else:
  arg = []
  dev = ''

if dev != '':
  try:
    baud = int(arg[0])
    previd = int(arg[1])
    newid = int(arg[2])
    dx = DXLProtocolV2(dev, baud)
  except:
    print('ERR:There is some problem.')
  else:
    if previd >= 0 and previd <= 252 and newid >= 0 and newid <= 252:
      if not dx.Ping(newid):
        if dx.Reboot(previd):
          time.sleep(0.5)
          if dx.Write8(previd, 7, newid):
            print('OK')
          else:
            print('NG')
        else:
          print(f' ERR: Device with ID:{previd} not found with BAUDRATE:{baud}')
      else:
        print('ERR:Competing.')
    else:
      print('ERR:There is some problem.')
else:
  print('usage: changeid <baudrate> <prev id> <new id>')
