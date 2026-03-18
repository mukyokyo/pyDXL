## Overview
A class that communicates with ROBOTIS DYNAMIXEL using only generic libraries.
## Requirement
- Python
- pyserial
## Usage
This section assumes the use of the DYNAMIXEL Protocol 2.

You must instantiate the class with an arbitrary port or socket as an argument. You should specify the `timeoutoffset` as appropriate when latency from the interface or operating system is a factor.
``` python
from pyDXL import DXLProtocolV2

with DXLProtocolV2('/dev/ttyAMA0', 57600, timeoutoffset=0.2) as dx2:
```
You can also specify a socket when instantiating the object. This is intended for use with wireless modules that utilize Wi-Fi.
``` python
from pyDXL import DXLProtocolV2
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
sock.connect((socket.gethostbyname('10.0.0.1'), 5050))

with DXLProtocolV2(sock, 57600, timeoutoffset=0.4, protocoltype=2) as dx2:
```
DYNAMIXEL is designed to access the control table by specifying an ID, address, and byte size.<br>
To read 4 bytes of data from address 0 of the DXL with ID=1, do the following. If successful, the data read is returned as a `bytes` type.
``` python
  r = dx2.Read(1, 0, 4)
  print(r)
```
To write data to 4 bytes starting at address 634 of the DXL with ID=1, do the following. The return value is a boolean indicating success or failure.
``` python
  r = dx2.Write(1, 634, bytes((1,2,3,4)))
  print(r)
```
When reading from or writing to 8/16/32 bit data, it is convenient to use functions with 8/16/32 appended to their names. If you specify `length`, you can handle contiguous data with the same bit size; if you need signed values, simply set `signed=True`.
``` python
  led = dx2.Read8(1, 65)
  if led is not None:
    dx2.Write8(1, 65, led ^ 1)

  pos = dx2.Read32(1, 132, signed=True)
  if pos is not None:
    dx2.Write32(1, 116, pos)
```
The functions for instructions that require special structures, such as `BULK` and `SYNC`, are a bit complex, so please refer to the sample code.
## Licence
[MIT](https://github.com/mukyokyo/pyDXL/blob/main/LICENSE)
