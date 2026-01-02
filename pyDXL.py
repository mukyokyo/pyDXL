#!/usr/bin/python3
#
# pyDXL.py
# Class that handles the DYNAMIXEL communication protocol.
# Supports protocols V1 and V2.
# Additionally supports serial to ethernet bridge I/F.
#
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: (C) 2024-2025 mukyokyo

import serial, socket, errno, select, threading, array, time
from typing import Union
from collections import namedtuple
from struct import pack, unpack, iter_unpack


##########################################################
# Functionalized the part of converting int to bytes.
# If specified as a tuple, it is converted to bytes at once.
##########################################################
def B2Bs(d) -> bytes:
  if isinstance(d, list) or isinstance(d, tuple):
    return bytes(((d & 0x7f) | 0x80) if d < 0 else d for d in d)
  else:
    return bytes(pack('<B', ((d & 0x7f) | 0x80) if d < 0 else d))


def W2Bs(d) -> bytes:
  if isinstance(d, list) or isinstance(d, tuple):
    return b''.join([pack('<H', d) for d in [((d & 0x7fff) | 0x8000) if d < 0 else d for d in d]])
  else:
    return bytes(pack('<H', ((d & 0x7fff) | 0x8000) if d < 0 else d))


def L2Bs(d) -> bytes:
  if isinstance(d, list) or isinstance(d, tuple):
    return b''.join([pack('<I', d) for d in [((d & 0x7fffffff) | 0x80000000) if d < 0 else d for d in d]])
  else:
    return bytes(pack('<I', ((d & 0x7fffffff) | 0x80000000) if d < 0 else d))


##########################################################
# API for Dynamixel protocol V1
##########################################################
class DXLProtocolV1:
  BROADCASTING_ID = 0xfe
  INST_PING = 0x01
  INST_READ = 0x02
  INST_WRITE = 0x03
  INST_REG_WRITE = 0x04
  INST_ACTION = 0x05
  INST_FACTORY_RESET = 0x06
  INST_REBOOT = 0x08
  INST_SYNC_WRITE = 0x83
  INST_SYNG_REG_WRITE = 0x85

  TSyncW = namedtuple("TSyncW", ("id", ("data")))

  def __init__(self, port: Union[serial.Serial, socket.socket, str], baudrate=57600, timeout=0.05, lock=None):
    """
    Initalize

    parameters
    -------------
    port : str
      Device name
    baudrate : int
      Serial baudrate[bps]
    timeout : float
      Read timeout[s]
    """
    if isinstance(port, serial.Serial):
      self.__serial = port
      self.__sock = None
      self.__baudrate = port.baudrate
      self.__timeout = port.timeout
    elif isinstance(port, socket.socket):
      self.__serial = None
      self.__sock = port
      self.__baudrate = baudrate
      self.__timeout = timeout
      self.__sock.settimeout(timeout)
      self.__send_uart_conf(baudrate)
    else:
      self.__serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
      self.__sock = None
      self.__baudrate = self.__serial.baudrate
      self.__timeout = self.__serial.timeout
    if lock is None:
      self.__lock = threading.Lock()
    else:
      self.__lock = lock
    self.__Error = 0

  def __send_uart_conf(self, baud):
    pconf_packet = bytearray([0x55, 0xaa, 0x55, 0, 0, 0, 0x83, 0])
    pconf_packet[3] = (baud >> 16) & 0xff
    pconf_packet[4] = (baud >> 8) & 0xff
    pconf_packet[5] = (baud) & 0xff
    pconf_packet[7] = sum(pconf_packet[3:7]) & 0xff
    try:
      self.__sock.sendall(pconf_packet)
    except socket.timeout:
      return None, False

  @property
  def lock(self):
    return self.__lock

  @property
  def baudrate(self):
    if self.__sock:
      return self.__baudrate
    else:
      return self.__serial.baudrate

  @baudrate.setter
  def baudrate(self, baudrate):
    self.__baudrate = baudrate
    if self.__sock:
      self.__send_uart_conf(baudrate)
    else:
      self.__serial.baudrate = baudrate

  @property
  def timeout(self):
    return self.__timeout

  @timeout.setter
  def timeout(self, timeout):
    self.__timeout = timeout
    if self.sock:
      self.__sock.settimeout(self.__timeout)
    else:
      self.__serial.timeout = timeout

  def __reconfig(self):
    if self.__sock:
      self.__sock.settimeout(self.__timeout)
    else:
      self.__serial.baudrate = self.__baudrate
      self.__serial.timeout = self.__timeout

  @property
  def Error(self):
    return self.__Error

  def __clear_sock_rx_buf(self):
    try:
      self.__sock.setblocking(False)
      while True:
        data = self.__sock.recv(65536)
        if not data:
          break
    except socket.error as e:
      if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
        pass # buffer is empty
      else:
        raise # Other errors can be reproduced
    finally:
      self.__sock.setblocking(True)
      self.__sock.settimeout(self.__timeout)

  def TxPacket(self, id: int, inst: int, param: bytes, echo=False) -> (bytes, bool):
    """
    Sending packets

    parameters
    -------------
    id : int
      Target ID
    inst : int
      Instruction command
    param : bytes
      Packet parameters

    Returns
    -------
    bytes
      Packets sent
    bool
      Success or failure
    """
    self.__reconfig()
    if ((id == self.BROADCASTING_ID) or (id >= 0 and id <= 253)) and len(param) <= (256 - 6):
      instp = bytearray([0xff, 0xff, id, 0, inst]) + bytes(param)
      instp[3] = len(instp) - 3
      instp += B2Bs(~sum(instp[2:]) & 0xff)
      if echo:
        print('TX:', instp.hex(':'))
      if self.__sock:
        self.__clear_sock_rx_buf()
        try:
          self.__sock.sendall(instp)
        except socket.timeout:
          return None, False
      else:
        self.__serial.reset_input_buffer()
        self.__serial.write(instp)
      return bytes(instp), True
    return None, False

  def __rx(self, length) -> bytes:
    if self.__sock:
      tout = time.time() + self.__sock.gettimeout()
      try:
        s = self.__sock.recv(length)
      except TimeoutError:
        return b''
      else:
        rxl = len(s)
        if rxl == length:
          return s
        else:
          r = s
          length -= rxl
          if length > 0:
            while length > 0 and time.time() < tout:
              try:
                s = self.__sock.recv(length)
              except TimeoutError:
                s = b''
              r += s
              length -= len(s)
          return r
    else:
      s = self.__serial.read(length)
      rxl = len(s)
      if rxl == length:
        return s
      else:
        r = s
        length -= rxl
        if length > 0:
          while self.__serial.in_waiting > 0:
            s = self.__serial.read(length)
            r += s
            length -= len(s)
            if length == 0:
              break
        return r

  def RxPacket(self, echo=False, timeout=0.0) -> (bytes, bool):
    """
    Receiving packets

    Returns
    -------
    bytes
      Packets received
    bool
      Success or failure
    """
    if timeout > 0:
      if self.__sock:
        prev_timeout = self.__sock.gettimeout()
        self.__sock.settimeout(timeout)
      else:
        prev_timeout = self.__serial.timeout
        self.__serial.timeout = timeout
        self.__serial.flushOutput()
    statp = self.__rx(5)
    if statp:
      if len(statp) == 5:
        if statp[0] == 0xff and statp[1] == 0xff:
          pl = statp[3] - 1
          self.__Error = statp[4]
          statp += self.__rx(pl)
          if len(statp) == pl + 5:
            if statp[-1:][0] == ((~sum(statp[2:-1])) & 0xff):
              if echo:
                print('RX:', statp.hex(':'))
              if timeout > 0:
                if self.__sock:
                  self.__sock.settimeout(prev_timeout)
                else:
                  self.__serial.timeout = prev_timeout
              return bytes(statp), (statp[4] & 0x40) == 0
    if echo:
      print('RX:', statp.hex(';'))
    if timeout > 0:
      if self.__sock:
        self.__sock.settimeout(prev_timeout)
      else:
        self.__serial.timeout = prev_timeout
    return None, False

  def Write(self, id: int, addr: int, data: bytes, echo=False) -> bool:
    """
    Write instruction

    parameters
    -------------
    id : int
      Target ID
    addr : int
      Target item address
    data : bytes
      Data to be written

    Returns
    -------
    result : bool
      Success or failure
    """
    with self.__lock:
      if id >= 0 and id <= self.BROADCASTING_ID and addr >= 0 and addr <= 254:
        if self.TxPacket(id, self.INST_WRITE, B2Bs(addr) + data, echo)[1]:
          if id != self.BROADCASTING_ID:
            dat, r = self.RxPacket(echo)
            if r:
              return dat[2] == id and (dat[4] & 0x18) == 0
          else:
            return True
      return False

  def Write8(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, B2Bs(data), echo)

  def Write16(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, W2Bs(data), echo)

  def Write32(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, L2Bs(data), echo)

  def Read(self, id: int, addr: int, length: int, echo=False) -> bytes:
    """
    Read instruction

    parameters
    -------------
    id : int
      Target ID
    addr : int
      Target item address
    length : int
      Number of bytes to read

    Returns
    -------
    bytes
      Data read
    """
    with self.__lock:
      if id >= 0 and id <= 253 and addr >= 0 and addr <= 254 and length > 0 and length <= (256 - 6):
        if self.TxPacket(id, self.INST_READ, B2Bs((addr, length)), echo)[1]:
          dat, r = self.RxPacket(echo)
          if r:
            if dat[2] == id and (dat[4] & 0x8) == 0:
              return bytes(dat[5:-1])
      return None

  def Read8(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, length, echo)
    if r is not None:
      n = sum(iter_unpack('b' if signed else 'B', r), ())
      return n if length > 1 else n[0]
    return None

  def Read16(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, 2 << (length - 1), echo)
    if r is not None:
      n = sum(iter_unpack('h' if signed else 'H', r), ())
      return n if length > 1 else n[0]
    return None

  def Read32(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, 4 << (length - 1), echo)
    if r is not None:
      n = sum(iter_unpack('i' if signed else 'I', r), ())
      return n if length > 1 else n[0]
    return None

  def SyncWrite(self, addr: int, length: int, id_datas: (TSyncW), echo=False) -> bool:
    """
    Sync Write instruction

    parameters
    -------------
    addr : int
      Target item address
    length : int
      Number of bytes to write
    id_datas : (TSyncW)
      Target ID and data

    Returns
    -------
    bool
      Success or failure
    """
    with self.__lock:
      if addr >= 0 and addr <= 254 and length > 0 and length < (256 - 6):
        param = B2Bs((addr, length))
        for d in id_datas:
          param += B2Bs(d.id) + d.data
          if len(d.data) != length or d.id < 0 or d.id > 253:
            del param
            return False
        return self.TxPacket(self.BROADCASTING_ID, self.INST_SYNC_WRITE, param, echo)[1]
      return False

  def Ping(self, id: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_PING, bytes(), echo)[1]:
        dat, r = self.RxPacket(echo)
        if r:
          return id == dat[2] and dat[3] == 2
      return False

  def FactoryReset(self, id: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_FACTORY_RESET, bytes(), echo)[1]:
        dat, r = self.RxPacket(echo)
        if r:
          return id == dat[2] and dat[3] == 2
      return False

  def Reboot(self, id: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_REBOOT, bytes(), echo)[1]:
        dat, r = self.RxPacket(echo)
        if r:
          return id == dat[2] and dat[3] == 2
      return False


##########################################################
# API for Dynamixel protocol V2
##########################################################
class DXLProtocolV2:
  BROADCASTING_ID = 0xfe
  INST_PING = 0x01
  INST_READ = 0x02
  INST_WRITE = 0x03
  INST_REG_WRITE = 0x04
  INST_ACTION = 0x05
  INST_FACTORY_RESET = 0x06
  INST_REBOOT = 0x08
  INST_SYS_WRITE = 0x0d
  INST_CLEAR = 0x10
  INST_CONTROL_TABLE_BACKUP = 0x20
  INST_STATUS = 0x55
  INST_SYNC_READ = 0x82
  INST_SYNC_WRITE = 0x83
  INST_SYNG_REG_WRITE = 0x85
  INST_FAST_SYNC_READ = 0x8a
  INST_BULK_READ = 0x92
  INST_BULK_WRITE = 0x93
  INST_FAST_BULK_READ = 0x9a

  TSyncW = namedtuple("TSyncW", ("id", ("data")))
  TBulkW = namedtuple("TBulkW", ("id", "addr", ("data")))
  TBulkR = namedtuple("TBulkR", ("id", "addr", "length"))

  __crc16_lutable = array.array('H')

  def __init__(self, port: Union[serial.Serial, socket.socket, str], baudrate=57600, timeout=0.05, lock=None):
    """
    Initalize

    parameters
    -------------
    port : str
      Device name
    baudrate : int
      Serial baudrate[bps]
    timeout : float
      Read timeout[s]
    """
    if isinstance(port, serial.Serial):
      self.__serial = port
      self.__sock = None
      self.__baudrate = port.baudrate
      self.__timeout = port.timeout
    elif isinstance(port, socket.socket):
      self.__serial = None
      self.__sock = port
      self.__baudrate = baudrate
      self.__timeout = timeout
      self.__sock.settimeout(timeout)
      self.__send_uart_conf(baudrate)
    else:
      self.__serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
      self.__sock = None
      self.__baudrate = self.__serial.baudrate
      self.__timeout = self.__serial.timeout
    if lock is None:
      self.__lock = threading.Lock()
    else:
      self.__lock = lock
    self.__Error = 0
    poly = 0x8005
    for i in range(256):
      nData = i << 8
      nAccum = 0
      for j in range(8):
        nAccum = ((nAccum << 1) ^ poly if (nData ^ nAccum) & 0x8000 else nAccum << 1) & 0xffff
        nData <<= 1
      self.__crc16_lutable.append(nAccum)

  def __send_uart_conf(self, baud):
    pconf_packet = bytearray([0x55, 0xaa, 0x55, 0, 0, 0, 0x83, 0])
    pconf_packet[3] = (baud >> 16) & 0xff
    pconf_packet[4] = (baud >> 8) & 0xff
    pconf_packet[5] = (baud) & 0xff
    pconf_packet[7] = sum(pconf_packet[3:7]) & 0xff
    try:
      self.__sock.sendall(pconf_packet)
    except socket.timeout:
      return None, False

  @property
  def lock(self):
    return self.__lock

  @property
  def baudrate(self):
    if self.__sock:
      return self.__baudrate
    else:
      return self.__serial.baudrate

  @baudrate.setter
  def baudrate(self, baudrate):
    self.__baudrate = baudrate
    if self.__sock:
      self.__send_uart_conf(baudrate)
    else:
      self.__serial.baudrate = baudrate

  @property
  def timeout(self):
    return self.__timeout

  @timeout.setter
  def timeout(self, timeout):
    self.__timeout = timeout
    if self.sock:
      self.__sock.settimeout(self.__timeout)
    else:
      self.__serial.timeout = timeout

  def __reconfig(self):
    if self.__sock:
      self.__sock.settimeout(self.__timeout)
    else:
      self.__serial.baudrate = self.__baudrate
      self.__serial.timeout = self.__timeout

  @property
  def Error(self):
    return self.__Error

  def __crc16(self, data: bytes) -> int:
    crc = 0
    for d in data:
      crc = (crc << 8) ^ self.__crc16_lutable[(((crc >> 8) ^ d) & 0xff)]
    return crc & 0xffff

  def __clear_sock_rx_buf(self):
    try:
      self.__sock.setblocking(False)
      while True:
        data = self.__sock.recv(65536)
        if not data:
          break
    except socket.error as e:
      if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
        pass # buffer is empty
      else:
        raise # Other errors can be reproduced
    finally:
      self.__sock.setblocking(True)
      self.__sock.settimeout(self.__timeout)

  def TxPacket(self, id: int, inst: int, param: bytes, echo=False) -> (bytes, bool):
    """
    Sending packets

    parameters
    -------------
    id : int
      Target ID
    inst : int
      Instruction command
    param : bytes
      Packet parameters

    Returns
    -------
    bytes
      Packets sent
    bool
      Success or failure
    """
    self.__reconfig()
    if ((id == self.BROADCASTING_ID) or (id >= 0 and id <= 252)) and len(param) < (65536 - 10):
      instp = bytearray([0xff, 0xff, 0xfd, 0x00, id, 0, 0, inst]) + bytearray(param).replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')
      instp[5:7] = W2Bs(len(instp) - 5)
      instp += W2Bs(self.__crc16(instp))
      if echo:
        print('TX:', instp.hex(':'))
      if self.__sock:
        self.__clear_sock_rx_buf()
        try:
          self.__sock.sendall(instp)
        except socket.timeout:
          return None, False
      else:
        self.__serial.reset_input_buffer()
        self.__serial.write(instp)
      return bytes(instp), True
    return None, False

  def __rx(self, length) -> bytes:
    if self.__sock:
      tout = time.time() + self.__sock.gettimeout()
      try:
        s = self.__sock.recv(length)
      except TimeoutError:
        return b''
      else:
        rxl = len(s)
        if rxl == length:
          return s
        else:
          r = s
          length -= rxl
          if length > 0:
            while length > 0 and time.time() < tout:
              try:
                s = self.__sock.recv(length)
              except TimeoutError:
                s = b''
              r += s
              length -= len(s)
          return r
    else:
      s = self.__serial.read(length)
      rxl = len(s)
      if rxl == length:
        return s
      else:
        r = s
        length -= rxl
        if length > 0:
          while self.__serial.in_waiting > 0:
            s = self.__serial.read(length)
            r += s
            length -= len(s)
            if length == 0:
              break
        return r

  def RxPacket(self, echo=False, timeout=0.0) -> (bytes, bool):
    """
    Receiving packets

    Returns
    -------
    bytes
      Packets received
    bool
      Success or failure
    """
    if timeout > 0:
      if self.__sock:
        prev_timeout = self.__sock.gettimeout()
        self.__sock.settimeout(timeout)
      else:
        prev_timeout = self.__serial.timeout
        self.__serial.timeout = timeout
        self.__serial.flushOutput()
    statp = self.__rx(9)
    if statp:
      if len(statp) == 9:
        if statp[0] == 0xff and statp[1] == 0xff and statp[2] == 0xfd and statp[3] == 0 and statp[7] == 0x55:
          pl = unpack('<H', statp[5:7])[0] - 2
          self.__Error = statp[8]
          statp += self.__rx(pl)
          if len(statp) == pl + 9:
            if unpack('<H', statp[-2:])[0] == self.__crc16(statp[:-2]):
              statp = statp[0:9] + statp[9:].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
              if echo:
                print('RX:', statp.hex(':'))
              if timeout > 0:
                if self.__sock:
                  self.__sock.settimeout(prev_timeout)
                else:
                  self.__serial.timeout = prev_timeout
              return bytes(statp), (statp[8] & 0x7f) == 0
    if echo:
      print('RX:', statp.hex(';'))
    if timeout > 0:
      if self.__sock:
        self.__sock.settimeout(prev_timeout)
      else:
        self.__serial.timeout = prev_timeout
    return None, False

  def Write(self, id: int, addr: int, data: bytes, echo=False) -> bool:
    """
    Write instruction

    parameters
    -------------
    id : int
      Target ID
    addr : int
      Target item address
    data : bytes
      Data to be written

    Returns
    -------
    result : bool
      Success or failure
    """
    with self.__lock:
      if ((id >= 0 and id <= 252) or (id == self.BROADCASTING_ID)) and addr >= 0 and addr <= 65535:
        if self.TxPacket(id, self.INST_WRITE, W2Bs(addr) + data, echo)[1]:
          if id != self.BROADCASTING_ID:
            dat, r = self.RxPacket(echo)
            if r:
              return dat[4] == id and (dat[8] & 0x7f) == 0
          else:
            return True
      return False

  def Write8(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, B2Bs(data), echo)

  def Write16(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, W2Bs(data), echo)

  def Write32(self, id: int, addr: int, data: Union[int, tuple, list], echo=False) -> bool:
    return self.Write(id, addr, L2Bs(data), echo)

  def Read(self, id: int, addr: int, length: int, echo=False) -> bytes:
    """
    Read instruction

    parameters
    -------------
    id : int
      Target ID
    addr : int
      Target item address
    length : int
      Number of bytes to read

    Returns
    -------
    bytes
      Data read
    bool
      Success or failure
    """
    with self.__lock:
      if id >= 0 and id <= 252 and addr >= 0 and addr <= 65535 and length > 0 and length < (65536 - 10):
        if self.TxPacket(id, self.INST_READ, W2Bs(addr) + W2Bs(length), echo)[1]:
          dat, r = self.RxPacket(echo)
          if r:
            if len(dat[9:-2]) == length:
              return bytes(dat[9:-2])
      return None

  def Read8(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, length, echo)
    if r is not None:
      n = sum(iter_unpack('b' if signed else 'B', r), ())
      return n if length > 1 else n[0]
    return None

  def Read16(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, 2 << (length - 1), echo)
    if r is not None:
      n = sum(iter_unpack('h' if signed else 'H', r), ())
      return n if length > 1 else n[0]
    return None

  def Read32(self, id: int, addr: int, length=1, signed=False, echo=False) -> int:
    r = self.Read(id, addr, 4 << (length - 1), echo)
    if r is not None:
      n = sum(iter_unpack('i' if signed else 'I', r), ())
      return n if length > 1 else n[0]
    return None

  def SyncWrite(self, addr: int, length: int, id_datas: (TSyncW), echo=False) -> bool:
    """
    return None, False
    Sync Write instruction

    parameters
    -------------
    addr : int
      Target item address
    length : int
      Number of bytes to write
    id_datas : (TSyncW)
      Target ID and data

    Returns
    -------
    bool
      Success or failure
    """
    with self.__lock:
      if addr >= 0 and addr <= 65535 and length > 0 and length < (65536 - 10):
        param = W2Bs(addr) + W2Bs(length)
        for d in id_datas:
          param += bytes((d.id,)) + d.data
          if len(d.data) != length or d.id < 0 or d.id > 252:
            del param
            return False
        return self.TxPacket(self.BROADCASTING_ID, self.INST_SYNC_WRITE, param, echo)[1]
      return False

  def SyncRead(self, addr: int, length: int, ids: (int), echo=False) -> tuple:
    """
    Sync Read instruction

    parameters
    -------------
    addr : int
      Target item address
    length : int
      Number of bytes to read
    ids : (int)
      Target IDs

    Returns
    -------
    tuple
      Read ID and data
    """
    with self.__lock:
      result = ()
      if addr >= 0 and addr < 65535 and length > 0 and length < (65536 - 10):
        if self.TxPacket(self.BROADCASTING_ID, self.INST_SYNC_READ, W2Bs(addr) + W2Bs(length) + B2Bs(ids), echo)[1]:
          for id in ids:
            dat, r = self.RxPacket(echo)
            if r:
              if dat[4] == id:
                result += (id, bytes(dat[9:9 + length])),
              else:
                result += (id, bytes([])),
            else:
              result += (id, bytes([])),
      return result

  def BulkWrite(self, data: (TBulkW), echo=False) -> bool:
    """
    Bulk Write instruction

    parameters
    -------------
    data : (TBulkW)
      Target ID and address, data
    ids : (TSyncW)
      Target ID and data

    Returns
    -------
    bool
      Success or failure
    """
    with self.__lock:
      param = bytes()
      for d in data:
        if d.id >= 0 and d.id <= 252 and d.addr >= 0 and d.addr <= 65535:
          param += B2Bs(d.id) + W2Bs(d.addr) + W2Bs(len(d.data)) + d.data
        else:
          del param
          return False
      return self.TxPacket(self.BROADCASTING_ID, self.INST_BULK_WRITE, param, echo)[1]

  def BulkRead(self, data: (TBulkR), echo=False) -> tuple:
    """
    Bulk Read instruction

    parameters
    -------------
    data : (TBulkR)
      ID, address, and number of bytes to be read

    Returns
    -------
    tuple
      Read ID and data
    """
    with self.__lock:
      result = ()
      param = bytes()
      for d in data:
        if d.id < 0 or d.addr < 0 or d.length < 0:
          return result
        param += B2Bs(d.id) + W2Bs(d.addr) + W2Bs(d.length)
      if self.TxPacket(self.BROADCASTING_ID, self.INST_BULK_READ, param, echo)[1]:
        for d in data:
          rxd, r = self.RxPacket(echo)
          if r:
            if d.id == rxd[4]:
              result += (d.id, bytes(rxd[9:-2])),
            else:
              result += (d.id, bytes([])),
          else:
            result += (d.id, bytes([])),
      del param
      return result

  def Ping(self, id: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_PING, bytes(), echo)[1]:
        rxd, r = self.RxPacket(echo)
        if r:
          return id == rxd[4] and rxd[5] == 7 and rxd[6] == 0
      return False

  def FactoryReset(self, id: int, p1: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_FACTORY_RESET, bytes((p1,)), echo)[1]:
        rxd, r = self.RxPacket(echo)
        if r:
          return id == rxd[4] and rxd[5] == 4 and rxd[6] == 0
      return False

  def Reboot(self, id: int, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_REBOOT, bytes(), echo)[1]:
        rxd, r = self.RxPacket(echo)
        if r:
          return id == rxd[4] and rxd[5] == 4 and rxd[6] == 0
      return False

  def Clear(self, id: int, val, echo=False) -> bool:
    with self.__lock:
      if self.TxPacket(id, self.INST_CLEAR, B2Bs(1) + L2Bs(val), echo)[1]:
        rxd, r = self.RxPacket(echo)
        if r:
          return id == rxd[4] and rxd[5] == 4 and rxd[6] == 0
      return False


##########################################################
# test code
##########################################################
if __name__ == "__main__":
  from contextlib import contextmanager
  from threading import Thread
  import sys
  import time
  import traceback

  ID = 1
  ec = False
  fin = False

  @contextmanager
  def stopwatch():
    # start_time = time.time()
    yield
    # print('..proc time={:.1f}ms {}'.format((time.time() - start_time) * 1000, psutil.Process(os.getpid()).memory_info().rss))

  ###
  ### The XM/XH/XD/XW series is assumed, so care should be taken if trying with other models.
  ###
  def func1(dx):
    global ID, fin, ec

    try:
      '''
      """ reset dxl """
      with stopwatch():
        r = dx.FactoryReset(ID, 0xff, echo=ec)
      print(f'FactoryReset({ID}): {r}')
      time.sleep(0.5)
      '''

      """ ping dxl """
      print(f'Ping({ID})=', dx.Ping(ID, echo=ec))
      print('Ping({ID + 1})=', dx.Ping(ID + 1, echo=ec))
      print('Ping({ID + 2})=', dx.Ping(ID + 2, echo=ec))
      print('Ping({ID + 3})=', dx.Ping(ID + 3, echo=ec))
      print('Ping({ID + 4})=', dx.Ping(ID + 4, echo=ec))

      """ reboot dxl """
      with stopwatch():
        r = dx.Reboot(ID, echo=ec)
      print(f'Reboot({ID})={r}')
      time.sleep(0.5)

      """ basic packet proc """
      with stopwatch():
        with dx.lock:
          r0 = dx.TxPacket(ID, dx.INST_WRITE, (65, 0, 1))
          r1 = dx.RxPacket()
      print(f'TxPacket({ID})={r0[1]}', r0[0].hex(':'))
      if r1[0]:
        print(f'RxPacket({ID})={r1[1]}', r1[0].hex(':'))

      """ dump memory """
      dl = 50
      for addr in range(0, 700, dl):
        r = dx.Read(ID, addr, dl, echo=ec)
        print(f'Read({addr};{dl})=', r.hex(':') if r else '!!!!!!!!!!!!!!!!!!!!!!!!')

      """ read byte item """
      with stopwatch():
        r = dx.Read8(ID, 65, echo=ec)
      print(f'Read8({ID})={r}')

      """ sync read inst. """
      print('SyncRead=')
      with stopwatch():
        d = dx.SyncRead(0, 4, (ID, ID + 1, ID + 2, ID + 3, ID + 4), echo=ec)
      for d in d:
        print(f' ({d[0]}) 0,4 hex=', d[1].hex(':') if len(d[1]) > 0 else ())
      with stopwatch():
        d = dx.SyncRead(132, 4, (ID, ID + 1, ID + 2, ID + 3, ID + 4), echo=ec)
      for d in d:
        print(f' ({d[0]}) 132,4 int32=', unpack('<i', pack('<I', unpack('<I', d[1])[0]))[0] if len(d[1]) > 0 else ())

      """ sync write inst. """
      for i in range(30):
        with stopwatch():
          dx.SyncWrite(65, 1, (dx.TSyncW(ID, B2Bs(1)), dx.TSyncW(ID + 1, B2Bs(1)), dx.TSyncW(ID + 2, B2Bs(1)), dx.TSyncW(ID + 3, B2Bs(1)), dx.TSyncW(ID + 4, B2Bs(1))), echo=ec)
        time.sleep(0.05)
        with stopwatch():
          dx.SyncWrite(65, 1, (dx.TSyncW(ID, B2Bs(0)), dx.TSyncW(ID + 1, B2Bs(0)), dx.TSyncW(ID + 2, B2Bs(0)), dx.TSyncW(ID + 3, B2Bs(0)), dx.TSyncW(ID + 4, B2Bs(0))), echo=ec)
        time.sleep(0.05)

      """ set goal position """
      # torque off
      if dx.Write8(ID, 64, 0, echo=ec):
        # multi turn
        if dx.Write8(ID, 11, 4, echo=ec):
          # torque on
          if dx.Write8(ID, 64, 1, echo=ec):
            print(f'Write32/Read32({ID})')
            for gp in tuple(range(2047, 2047 + 4096, 64)) + tuple(range(2047 + 4096, 2047 - 4096, -64)) + tuple(range(2047 - 4096, 2047, 64)):
              # set goal position
              with stopwatch():
                dx.Write32(ID, 116, gp, echo=ec)
                # get present position
                s = time.time() + 100 / 1000.0
                while s > time.time():
                  with stopwatch():
                    pp = dx.Read32(ID, 132, length=4, signed=True, echo=ec)
                    vv = dx.Read(ID, 0, 20, echo=ec)
                    if vv is None:
                      print('\n!!!\n')
                  if pp is not None:
                    print(f' w={gp:6} r={pp[0]:6} diff={gp - pp[0]:5}', end='\r')
                  else:
                    print('None                            ', end='\r')
                    print()
                    break
                  time.sleep(0.001)
            print('')
            # torque off
            with stopwatch():
              dx.Write8(ID, 64, 0, echo=ec)
            # normal turn
            with stopwatch():
              dx.Write8(ID, 11, 3, echo=ec)

      """ block read/write (add/remove suffixes) """
      with stopwatch():
        r = dx.Read(ID, 120, 27, echo=ec)
      if r:
        print(f'Read({ID};128)=', tuple(iter_unpack('<HBBhhiiIIHB', r))[0])

      fffffd = bytes((
        0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd,
        0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd,
        0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfd,
      ))
      with stopwatch():
        r = dx.Write(ID, 634, fffffd, echo=ec)
      print(f'Write({ID})={r}')
      with stopwatch():
        r = dx.Read(ID, 634, len(fffffd), echo=ec)
      if r:
        print(f'Read({ID})=', r.hex(':'))

      """ bulk read inst. """
      with stopwatch():
        d = dx.BulkRead((dx.TBulkR(ID, 0, 10), dx.TBulkR(ID + 1, 0, 20), dx.TBulkR(ID + 2, 0, 30), dx.TBulkR(ID + 3, 0, 40), dx.TBulkR(ID + 4, 0, 40)), echo=ec)
      print('BulkRead=')
      for d in d:
        print(f' ({d[0]},0)', d[1].hex(':'))

      dx.Write8(ID, 64, 1, echo=ec)
      """ bulk write inst. """
      print('BulkWrite=')
      with stopwatch():
        print(' 1', dx.BulkWrite((dx.TBulkW(ID, 104, L2Bs((0, 0, 0, 1024))), dx.TBulkW(ID + 1, 65, B2Bs(0)), dx.TBulkW(ID + 2, 65, B2Bs(0)), dx.TBulkW(ID + 3, 65, B2Bs(0))), echo=ec))
      time.sleep(0.5)
      with stopwatch():
        print(' 2', dx.BulkWrite((dx.TBulkW(ID, 104, L2Bs(0) + L2Bs(0) + L2Bs(0) + L2Bs(2048)), dx.TBulkW(ID + 1, 65, B2Bs(1)), dx.TBulkW(ID + 2, 65, B2Bs(1)), dx.TBulkW(ID + 3, 65, B2Bs(1))), echo=ec))
      time.sleep(0.5)
      with stopwatch():
        print(' 3', dx.BulkWrite((dx.TBulkW(ID, 104, L2Bs(0) + L2Bs(0) + L2Bs(0) + L2Bs(1024)), dx.TBulkW(ID + 1, 65, B2Bs(0)), dx.TBulkW(ID + 2, 65, B2Bs(0))), echo=ec))
      time.sleep(0.5)
      with stopwatch():
        print(' 4', dx.BulkWrite((dx.TBulkW(ID, 116, L2Bs(2048)), dx.TBulkW(ID + 1, 65, B2Bs(1)), dx.TBulkW(ID + 2, 65, B2Bs(1))), echo=ec))
      time.sleep(0.5)
      with stopwatch():
        print(' 5', dx.BulkWrite((dx.TBulkW(ID, 65, bytes(0)), dx.TBulkW(ID + 1, 65, bytes(0)), dx.TBulkW(ID + 2, 65, bytes(0))), echo=ec))

      dx.Write8(ID, 64, 0, echo=ec)

    except:
      print('--- Caught Exception ---')
      traceback.print_exc()
      print('------------------------')

    fin = True

  def func2(dx):
    global ID, fin, ec

    while not fin:
      try:
        led = dx.Read8(ID, 65, echo=ec)
        if led is not None:
          led ^= 1
          dx.Write8(ID, 65, led, echo=ec)
        time.sleep(0.05)
      except:
        print('--- Caught Exception ---')
        traceback.print_exc()
        print('------------------------')

  ###
  ### The DX/AX/MX series is assumed, so care should be taken if trying with other models.
  ###
  def func3(dx):
    global ID, fin, ec

    try:
      """ dump memory """
      rxl = 20
      for addr in range(0, 250, rxl):
        r = dx.Read(ID, addr, rxl, echo=ec)
        print(f'Read({addr};{rxl})=', r.hex(':') if r else '!!!!!!!!!!!!!!!!!!!!!!!!')

      '''
      """ reset dxl """
      with stopwatch():
        r = dx.FactoryReset(ID, echo=ec)
      print(f'FactoryReset({ID}): {r}')
      time.sleep(0.5)
      '''

      """ ping dxl """
      print(f'Ping({ID})=', dx.Ping(ID, echo=ec))

      """ reboot dxl """
      with stopwatch():
        r = dx.Reboot(ID, echo=ec)
      print(f'Reboot({ID})={r}')
      time.sleep(1)

      """ basic packet proc """
      with stopwatch():
        with dx.lock:
          r0 = dx.TxPacket(ID, dx.INST_WRITE, (25, 1))
          r1 = dx.RxPacket()
      print(f'TxPacket({ID})={r0[1]}', r0[0].hex(':'))
      if r1[0]:
        print(f'RxPacket({ID})={r1[1]}', r1[0].hex(':'))

      """ read byte item """
      with stopwatch():
        r = dx.Read8(ID, 25, echo=ec)
      print(f'Read8({ID})={r}')

      """ sync write inst. """
      for i in range(30):
        with stopwatch():
          dx.SyncWrite(25, 1, (dx.TSyncW(ID, B2Bs(1)), dx.TSyncW(ID + 1, B2Bs(1)), dx.TSyncW(ID + 2, B2Bs(1))), echo=ec)
        time.sleep(0.05)
        with stopwatch():
          dx.SyncWrite(25, 1, (dx.TSyncW(ID, B2Bs(0)), dx.TSyncW(ID + 1, B2Bs(0)), dx.TSyncW(ID + 2, B2Bs(0))), echo=ec)
        time.sleep(0.05)

      """ set goal position """
      # torque off
      if dx.Write8(ID, 24, 0, echo=ec):
        # multi turn
        if dx.Write16(ID, 6, (4095, 4095), echo=ec):
          # torque on
          if dx.Write8(ID, 24, 1, echo=ec):
            print(f'Write16/Read16({ID})')
            for gp in tuple(range(2047, 2047 + 4096, 64)) + tuple(range(2047 + 4096, 2047 - 4096, -64)) + tuple(range(2047 - 4096, 2047, 64)):
              for i in range(10):
                # set goal position
                with stopwatch():
                  dx.Write16(ID, 30, gp, echo=ec)
                # get present position
                with stopwatch():
                  pp = dx.Read16(ID, 36, signed=True, echo=ec)
                if pp is not None:
                  print(f' w={gp:6} r={pp:6} diff={gp - pp:5}', end='\r')
                else:
                  pass
                  print('None                            ', end='\r')
                  break
                time.sleep(0.005)
              else:
                continue
              pass
              break
            print('')
            # torque off
            dx.Write8(ID, 24, 0, echo=ec)
            # normal turn
            with stopwatch():
              dx.Write16(ID, 6, (0, 4095), echo=ec)
    except:
      print('--- Caught Exception ---')
      traceback.print_exc()
      print('------------------------')

  print(sys.version)

  try:
    dx = DXLProtocolV2('\\\\.\\COM12', 57600, timeout=0.05)
  except:
    pass
  else:
    th1 = Thread(target=func1, args=(dx,))
    th2 = Thread(target=func2, args=(dx,))
    th1.start()
    th2.start()
    th1.join()
    th2.join()
    del th1, th2, dx

  """
  try:
    serv_address = ('10.0.0.1', 23)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) 
    sock.connect(serv_address)

    dx = DXLProtocolV2(sock, 57600, timeout=0.1)
    time.sleep(1)
  except:
    pass
  else:
    th1 = Thread(target=func1, args=(dx,))
    th2 = Thread(target=func2, args=(dx,))
    th1.start()
    th2.start()
    th1.join()
    th2.join()
    del th1, th2, dx
    sock.close()
  """

  """
  try:
    serv_address = ('10.0.0.1', 23)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) 
    sock.connect(serv_address)

    dx = DXLProtocolV1(sock, 57600, timeout=0.05)
    time.sleep(1)

  except:
    pass
  else:
    th3 = Thread(target=func3, args=(dx,))
    th3.start()
    th3.join()
    del th3, dx
    sock.close()
  """

  print('fin')
