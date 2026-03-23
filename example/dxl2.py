#!/usr/bin/env python3

# A class that allows control tables to be treated like properties.
# It can handle numerical clips and physical values, and you can add items with custom structures.
# Since the control table is of type `dict`, one option would be to save it as a separate file for
# each model and load it from there.
# Oh, right, it also supports sync and bulk writes.

from pyDXL import DXLProtocolV2
import warnings, struct, contextlib, json, os


class DXL_master(DXLProtocolV2):
  def __init__(self, *args, **kwargs):
    self._dx = super().__init__(*args, **kwargs)
    self._syncw_context = None
    self._bulkw_context = None

  @contextlib.contextmanager
  def sync_write(self, info):
    self._syncw_context = {'name': info, 'data': []}
    try:
      yield
      if self._syncw_context['data']:
        self.SyncWrite(info[0], struct.calcsize(info[1]), (self.TSyncW(d[0], d[1]) for d in self._syncw_context['data']), wait=0)
    finally:
      self._syncw_context = None

  @contextlib.contextmanager
  def bulk_write(self):
    self._bulkw_context = {'data': []}
    try:
      yield
      if self._bulkw_context['data']:
        self.BulkWrite((self.TBulkW(d[0], d[1], d[2]) for d in self._bulkw_context['data']), wait=0)
    finally:
      self._bulkw_context = None


class dxl:

  BaudrateList = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000, 8: 6000000, 9: 10500000}
  StatusError = {0x01: 'Result Fail', 0x02: 'Instruction Error', 0x03: 'CRC Error', 0x04: 'Data Range Error', 0x05: 'Data Length Error', 0x06: 'Data Limit Error', 0x07: 'Access Error'}

  # name : (address, format, direction, (range), unit/(unit), coefficient/(coefficient))

  def __del__(self):
    pass

  class WriteError(Exception):
    pass

  class ReadError(Exception):
    pass

  def _parse_expressions(self, data):
    if isinstance(data, dict):
      return {k: self._parse_expressions(v) for k, v in data.items()}
    elif isinstance(data, list):
      return [self._parse_expressions(i) for i in data]
    elif isinstance(data, str):
      has_op = any(op in data for op in "+*/-")
      has_bracket = any(b in data for b in "[]{}")

      if has_op and not has_bracket:
        try:
          return eval(data)
        except:
          return data
    return data

  def _build_index(self, config_dir):
    master_config = {}

    for filename in os.listdir(config_dir):
      if filename.endswith('.json'):
        with open(os.path.join(config_dir, filename), 'r') as f:
          data = json.load(f)
          for key, value in data.items():
            if key == 'models' and key in master_config:
              master_config[key].update(value)
            else:
              master_config[key] = value
    return self._parse_expressions(master_config)

  def __init__(self, dx_instance, dxl_id):
    self._model_index = self._build_index('model_data/')
    self._dx = dx_instance
    self._id = dxl_id
    model_num = self._dx.Read16(dxl_id, 0)
    if model_num is not None:
      json_path = self._model_index['models'].get(f'0x{model_num:04X}')
      if not json_path:
        self._modelname = None
        self._items = {}
        raise Exception(f'Model(0x{model_num:04X}) is not supported.')
      else:
        model_info = self._model_index.get('models').get(f'0x{model_num:04X}')
        self._modelname = model_info.get('modelname')
        self._items = self._model_index.get(model_info.get('controltable')).copy()
        self._items.update(model_info.get('overrides', {}))
    else:
      self._items = {}
      self._modelname = None

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

  def __getattr__(self, name):
    if name in self._items:
      addr, fmt, _, _, unit, coef = self._items[name]
      size = struct.calcsize(fmt)
      r = self._dx.Read(self._id, addr, size)
      if r is not None:
        p = tuple(struct.iter_unpack('<' + fmt, r))[0]
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
      else:
        if self._dx.Error == 0:
          warnings.warn('Read operation failed. It appears to be a receve timeout.', UserWarning)
          return None
        else:
          raise self.ReadError(f'Read operation failed. Error code:${self._dx.Error:02X}({self.StatusError.get(self._dx.Error & 0x7F, "")})')
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
      if self._dx._syncw_context is not None:
        if self._dx._syncw_context['name'][0] == addr:
          self._dx._syncw_context['data'].append((self._id, wvalue))
        else:
          if self._dx.Write(self.id, addr, wvalue):
            if name == 'ID':
              self._id = value
            if name == 'Baudrate':
              baud = self.BaudrateList.get(value)
              if baud is not None:
                self._dx.baudrate = baud
          else:
            if self._dx.Error == 0:
              warnings.warn('Write operation failed. It appears to be a receve timeout.', UserWarning)
            else:
              raise self.WriteError(f'Write operation failed. Error code:${self._dx.Error:02X}({self.StatusError.get(self._dx.Error & 0x7F, "")})')
      elif self._dx._bulkw_context is not None:
        self._dx._bulkw_context['data'].append((self._id, addr, wvalue))
      else:
        if self._dx.Write(self.id, addr, wvalue):
          if name == 'ID':
            self._id = value
          if name == 'Baudrate':
            baud = self.BaudrateList.get(value)
            if baud is not None:
              self._dx.baudrate = baud
        else:
          if self._dx.Error == 0:
            warnings.warn('Write operation failed. It appears to be a receve timeout.', UserWarning)
          else:
            raise self.WriteError(f'Write operation failed. Error code:${self._dx.Error:02X}({self.StatusError.get(self._dx.Error & 0x7F, "")})')
    else:
      raise AttributeError(f'No such item: {name}')


if __name__ == '__main__':
  from time import sleep
  import random

  with DXL_master('\\\\.\\COM20', 9600, timeoutoffset=0.5, protocoltype=2) as dx2:

    '''
    rets = dx2.Ping2()
    rets.sort()
    d = []
    for ret in rets:
      d += dxl(dx2, ret[0]),
      if d[-1].modelname is not None:
        print(ret[0], d[-1].modelname)
      else:
        d.pop(-1)

    for _d in d:
      _d.Torque_Enable = 1
      _d.Profile_Velocity.phys = 29.0

    d[0].Goal_Position.phys = 0
    sleep(1)
    d[0].Goal_Position.phys = -90.0
    sleep(1)
    d[0].Goal_Position.phys = 90.0
    sleep(1)

    for i in range(36):
      d[0].Goal_Position.phys = -180 + 10.0 * i
      for j in range(10):
        print(f'{d[0].Goal_Position.phys:7.1f} {d[0].Present_Position.phys:7.1f}', end='\r')
        sleep(0.01)
    else:
      print()

    for _d in d:
      _d.Profile_Velocity.phys = 0

    with dx2.sync_write(d[0].Goal_Position.info):
      for _d in d:
        _d.Goal_Position = 0
    sleep(0.5)

    for _d in d:
      _d.Torque_Enable = 0
    '''

    rets = dx2.Ping2()
    rets.sort()
    d = []
    for ret in rets:
      d += dxl(dx2, ret[0]),
      if d[-1].modelname is not None:
        d[-1].updateitems({
          'GoalVeloPos': (112, 'ii', 'rw', (None, None), ('rpm', 'deg'), (0.229, 360 / 4096)),
          'PresentValues': (124, 'hhiiiiHB', 'r', (None, None), ('%', '', 'rpm', 'deg', 'rpm', 'deg', 'V', 'degC'), (100 / 885, None, 0.229, 360 / 4096, 0.229, 360 / 4096, 0.1, 1.0)),
        })
        print(ret[0], d[-1].modelname)
      else:
        d.pop(-1)

    for _d in d:
      _d.Torque_Enable = 1
      _d.Profile_Velocity = 0
      print('Torque_Enable=', _d.Torque_Enable.str)
      print('GoalVeloPos.phys=', _d.GoalVeloPos.str)
      print('PresentValues.phys=', _d.PresentValues.str)

    d[0].GoalVeloPos.phys = 20.0, 0.5
    sleep(1)
    d[0].GoalVeloPos.phys = 10.0, 280.0
    sleep(1)
    d[0].GoalVeloPos.phys = 60.0, 180.0
    sleep(1)

    for i in range(20):
      print(f'bulk {i + 1}/20', end='\r')
      with dx2.bulk_write():
        if len(d) >= 1: d[0].GoalVeloPos = random.randint(50, 300), random.randint(2047 - 512, 2047 + 512)
        if len(d) >= 2: d[1].Goal_Position = random.randint(2047 - 512, 2047 + 512)
        if len(d) >= 3: d[2].Goal_Position = random.randint(2047 - 512, 2047 + 512)
        if len(d) >= 4: d[3].LED = random.randint(0, 1)
        if len(d) >= 5: d[4].LED = random.randint(0, 1)
        if len(d) >= 6: d[5].LED = random.randint(0, 1)
      sleep(0.3)
    else:
      print()

    d[0].Profile_Velocity = 0
    d[0].Goal_Position = 2047
    sleep(0.3)
    d[0].Goal_Position = 0
    sleep(0.3)
    d[0].Goal_Position = 4095
    sleep(0.3)
    d[0].Goal_Position = 0
    sleep(0.3)

    for i in range(36):
      d[0].Goal_Position.phys = 10.0 * i
      for j in range(10):
        print(f'{d[0].Goal_Position.phys:7.1f} {d[0].Present_Position.phys:7.1f}', end='\r')
        sleep(0.01)
    else:
      print()

    with dx2.sync_write(d[0].Goal_Position.info):
      for _d in d:
        _d.Goal_Position = 0
    sleep(1)

    with dx2.sync_write(d[0].Goal_Position.info):
      for _d in d:
        _d.Goal_Position = 2047
    sleep(1)

    with dx2.sync_write(d[0].Goal_Position.info):
      for _d in d:
        _d.Goal_Position = 4095
    sleep(1)

    for i in range(50):
      with dx2.sync_write(d[0].LED.info):
        for _d in d:
          _d.LED ^= 1
      sleep(0.01)

    for i, p in enumerate(range(0, 4095, 100)):
      print(f'sync {i + 1}/41', end='\r')
      with dx2.sync_write(d[0].GoalVeloPos.info):
        for _d in d:
          _d.GoalVeloPos = 45, p
      sleep(0.5)
    else:
      print()

    with dx2.sync_write(d[0].GoalVeloPos.info):
      for _d in d:
        _d.GoalVeloPos = 0, 2047
    sleep(0.5)

    for _d in d:
      _d.Torque_Enable = 0
