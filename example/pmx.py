#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pyPMX import PMXProtocol
import warnings, struct, contextlib, json, os


class pmx:

  BaudrateList = {0: 57600, 1: 115200, 2: 625000, 3: 1000000, 4: 1250000, 5: 1500000, 6: 2000000, 7: 3000000}

  def __del__(self):
    pass

  class WriteError(Exception):
    pass

  class ReadError(Exception):
    pass

  def _build_index(self, config_dir):
    master_config = {}
    for filename in os.listdir(config_dir):
      if filename.endswith('.json'):
        with open(os.path.join(config_dir, filename), 'r', encoding='utf-8', errors='ignore') as f:
          data = json.load(f)
          for key, value in data.items():
            if key == 'models' and key in master_config:
              master_config[key].update(value)
            else:
              master_config[key] = value
    return self._parse_expressions(master_config)

  def _parse_expressions(self, data):
    if isinstance(data, dict):
      return {k: self._parse_expressions(v) for k, v in data.items()}
    elif isinstance(data, list):
      return [self._parse_expressions(i) for i in data]
    elif isinstance(data, str):
      if any(op in data for op in '+*/-') and not any(b in data for b in '[]{}'):
        try:
          return eval(data)
        except:
          return data
    return data

  def __init__(self, pmx_instance, pmx_id):
    self._cache = {}
    self._model_index = self._build_index('model_data/')
    self._pmx = pmx_instance
    self._id = pmx_id
    sysinfo = self._pmx.SystemREAD(pmx_id)
    if sysinfo is not None:
      json_path = self._model_index['models'].get(f'0x{sysinfo[1]:08X}')
      if not json_path:
        self._modelname = None
        self._items = {}
        self._firmware_version = None
        raise Exception(f'Model(0x{sysinfo[1]:08X}) is not supported.')
      else:
        model_info = self._model_index.get('models').get(f'0x{sysinfo[1]:08X}')
        self._modelname = model_info.get('modelname')
        self._firmware_version = sysinfo[2]
        self._items = self._model_index.get(model_info.get('controltable')).copy()
        self._items.update(model_info.get('overrides', {}))
    else:
      self._items = {}
      self._modelname = None
      self._firmware_version = None

  def __enter__(self):
    return self

  def __exit__(self, ex_type, ex_value, trace):
    pass

  @property
  def id(self):
    return self._id

  @property
  def modelname(self):
    return self._modelname

  @property
  def firmwareversion(self):
    return self._firmware_version

  @property
  def items(self):
    return self._items

  def updateitems(self, itm):
    self._items.update(itm)

  def _genstr(self, val, coef, unit):
    ret = ''
    if isinstance(val, list | tuple):
      for i, _val in enumerate(val):
        if coef is not None and isinstance(coef, list | tuple):
          if coef[i] is not None:
            ret += f'{float(_val) * coef[i]:.3f}'
          else:
            ret += f'{float(_val)}'
        else:
          ret += f'{_val}'
        if unit is not None and unit != '' and isinstance(unit, list | tuple):
          if unit[i] != '':
            ret += f'[{unit[i]}]'
        ret += ', '
      ret = ret.rstrip(', ')
    else:
      if coef is not None:
        ret += f'{float(val) * coef:.3f}'
      else:
        ret += f'{val}'
      if unit is not None and unit != '':
        ret += f'[{unit}]'
    return ret

  def _conv_format_value(self, name, fmt, unit, coef, val):
    p = tuple(struct.iter_unpack('<' + fmt, val))[0]
    if len(p) == 1:
      class conv_physvalue(int):

        @property
        def info(self_val):
          return self._items[name]

        @property
        def phys(self_val):
          return float(self_val) * coef

        @property
        def str(self_val):
          return self._genstr(self_val, coef, unit)

        @phys.setter
        def phys(self_val, val):
          setattr(self, name, int(val / coef))

      return conv_physvalue(p[0])
    else:
      class conv_physvalue(tuple):

        @property
        def info(self_val):
          return self._items[name]

        @property
        def phys(self_val):
          if isinstance(self_val, list | tuple):
            return list(float(x) * (y if y is not None else 1.0) for (x, y) in zip(self_val, coef))
          else:
            return float(self_val) * coef

        @property
        def str(self_val):
          return self._genstr(self_val, coef, unit)

        @phys.setter
        def phys(self_val, val):
          if isinstance(val, list | tuple):
            setattr(self, name, list(int(x / (y if y is not None else 1.0)) for (x, y) in zip(val, coef)))
          else:
            setattr(self, name, int(val / coef))

      return conv_physvalue(p[0] if len(p) == 1 else p)

  def __getattr__(self, name):
    if name in self._items:
      addr, fmt, _, _, unit, coef = self._items[name]
      size = struct.calcsize(fmt)

      if hasattr(self, f"_{name}"):
        return getattr(self, f"_{name}")

      r = self._pmx.MemREAD(self._id, addr, size)
      if r is not None:
        return self._conv_format_value(name, fmt, unit, coef, r)
      else:
        if self._pmx.status == 0:
          warnings.warn('Read operation failed. It appears to be a receve timeout.', UserWarning)
          return None
        else:
          raise self.ReadError(f'Read operation failed. Error code:${self._pmx.status:02X}({self.StatusError.get(self._dx.status & 0x7F, "")})')
    raise AttributeError(f'No such item: {name}')

  def __setattr__(self, name, value):
    if name.startswith('_'):
      self.__dict__[name] = value
      return

    if name in self._items:
      addr, fmt, access, val_range, _, _ = self._items[name]
      if 'w' not in access:
        raise AttributeError(f'{name} is Read-Only')
      v_min, v_max = val_range
      if v_min is not None and v_max is not None:
        if not (v_min <= value <= v_max):
          newvalue = min(max(value, v_min), v_max)
          warnings.warn(f'Out of Range: {name} ({value}) must be {v_min}~{v_max}. Clipped at {newvalue}', UserWarning)
          value = newvalue
      if isinstance(value, list | tuple):
        wvalue = struct.pack('<' + fmt, *value)
      else:
        wvalue = struct.pack('<' + fmt, value)

      if self._pmx.MemWRITE(self.id, addr, wvalue):
        if name == 'ID':
          self._id = value
        if name == 'Baudrate':
          baud = self.BaudrateList.get(value)
          if baud is not None:
            self._pmx.baudrate = baud
      else:
        if self._pmx.status == 0:
          warnings.warn('Write operation failed. It appears to be a receve timeout.', UserWarning)
        else:
          raise self.WriteError(f'Write operation failed. Error code:${self._dx.status:02X}({self.StatusError.get(self._dx.status & 0x7F, "")})')
    else:
      raise AttributeError(f'No such item: {name}')


if __name__ == '__main__':
  from time import sleep, time
  from contextlib import contextmanager

  def wait(t):
    end_time = time() + t
    while end_time > time():
      yield

  with PMXProtocol('\\\\.\\COM20', 115200, timeoutoffset=3.0) as pmx_if:
    # Instance the PMX with ID 0
    p = pmx(pmx_if, 0)
    print(p.id, p.modelname, p.firmwareversion)

    # Add properties based on the angle and velocity cascade
    p.updateitems({
      'PresentValue': (300, "hhh", "r", (None, None), ('º','º/s','mA'), (1/100,1/10,1.0)),
      'GoalPos': (700, 'h', 'rw', (None, None), 'º', 1 / 100),
      'GoalPosSpd': (700, 'hh', 'rw', (None, None), ('º', 'º/s'), (1 / 100, 1/10)),
      'GoalPosSpdCur': (700, 'hhh', 'rw', (None, None), ('º', 'º/s', 'mA'), (1 / 100, 1/10, 1.0)),
      'GoalSpdCur': (700, 'hh', 'rw', (None, None), ('º/s', 'mA'), (1/10, 1.0)),
    })

    # Set the control mode (position)
    p.TorqueSwitch = 2
    p.ControlMode = 1
    print(f'Control_Mode={bin(p.ControlMode)}')
    p.TorqueSwitch = 1
    for i in tuple(range(0, 180, 60)) + tuple(range(180, -180, -60)) + tuple(range(-180, 0, 60)):
      p.GoalPos.phys = i
      for _ in wait(0.5):
        print(p.GoalPos.phys, p.PresentValue.str, p.MotorTemp.str, end='     \r')
    else:
      print()

    # Set the control mode (position & speed)
    p.TorqueSwitch = 2
    p.ControlMode = 1 | 2
    print(f'Control_Mode={bin(p.ControlMode)}')
    p.TorqueSwitch = 1
    for i in tuple(range(0, 180, 60)) + tuple(range(180, -180, -60)) + tuple(range(-180, 0, 60)):
      p.GoalPosSpd.phys = i, 100
      for _ in wait(1.0):
        print(p.GoalPosSpd.phys, p.PresentValue.str, p.MotorTemp.str, end='     \r')
    else:
      print()

    # Set the control mode (position & speed & current)
    p.TorqueSwitch = 2
    p.ControlMode = 1 | 2 | 4
    print(f'Control_Mode={bin(p.ControlMode)}')
    p.TorqueSwitch = 1
    for i in tuple(range(0, 180, 60)) + tuple(range(180, -180, -60)) + tuple(range(-180, 0, 60)):
      p.GoalPosSpdCur.phys = i, 100, 100
      for _ in wait(1.0):
        print(p.GoalPosSpdCur.phys, p.PresentValue.str, p.MotorTemp.str, end='     \r')
    else:
      print()

    # Set the control mode (speed & current)
    p.TorqueSwitch = 2
    p.ControlMode = 2 | 4
    print(f'Control_Mode={bin(p.ControlMode)}')
    p.TorqueSwitch = 1
    for i in tuple(range(0, 200, 20)) + tuple(range(200, -200, -20)) + tuple(range(-200, 0, 20)):
      p.GoalSpdCur.phys = i, 500
      for _ in wait(1.0):
        print(p.GoalSpdCur.phys, p.PresentValue.str, p.MotorTemp.str, end='     \r')
    else:
      print()

    # free
    p.TorqueSwitch = 2
