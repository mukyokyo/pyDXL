#!/usr/bin/python3

# Tool to change baudrate using pyDXL

import sys, time
from pyDXL import DXLProtocolV2

baudlist = { 9600 : 0, 57600 : 1, 115200 : 2, 1000000 : 3, 2000000 : 4, 3000000 : 5, 4000000 : 6 }

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
    prevbaud = int(arg[0])
    prevind = baudlist[prevbaud]
    id = int(arg[1])
    newbaud = int(arg[2])
    newind = baudlist[newbaud]
    dx = DXLProtocolV2(dev, newbaud, 0.05)
  except:
   print('ERR:There is some problem.')
  else:
    if not dx.Ping(id):
      dx.baudrate = prevbaud
      if dx.Reboot(id):
        time.sleep(0.5)
        if dx.Write8(id, 8, newind):
          print('OK')
        else:
          print('NG')
      else:
        print(f' ERR: Device with ID:{id} not found with BAUDRATE:{prevbaud}')
    else:
      print('ERR:Competing.')
else:
  print('usage: changebaud <baudrate> <id> <new baudrate>')
