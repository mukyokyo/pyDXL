#!/usr/bin/env python3

# A class that allows control tables to be treated like properties.
# It can handle numerical clips and physical values, and you can add items with custom structures.
# Since the control table is of type `dict`, one option would be to save it as a separate file for
# each model and load it from there.
# Oh, right, it also supports sync write, sync read and bulk write.

from pyDXL import DXLProtocolV2
import warnings, struct, contextlib, json, os


class DXL_master(DXLProtocolV2):
  def __init__(self, *args, **kwargs):
    self._dx = super().__init__(*args, **kwargs)

    self._syncw_context = None
    self._bulkw_context = None
    self._syncr_group = None
    self._syncr_names = set()

  @contextlib.contextmanager
  def sync_write(self, info):
    self._syncw_context = {'name': info, 'data': []}
    yield
    if self._syncw_context['data']:
      self.SyncWrite(info[0], struct.calcsize(info[1]), (self.TSyncW(d[0], d[1]) for d in self._syncw_context['data']), wait=0)
    self._syncw_context = None

  @contextlib.contextmanager
  def bulk_write(self):
    self._bulkw_context = {'data': []}
    yield
    if self._bulkw_context:
      self.BulkWrite((self.TBulkW(d[0], d[1], d[2]) for d in self._bulkw_context['data']), wait=0)
    self._bulkw_context = None

  @contextlib.contextmanager
  def sync_read(self, *instances):
    self._syncr_group = instances
    self._syncr_names.clear()
    yield self
    for name in list(self._syncr_names):
      cache_name = f"_{name}"
      for obj in instances:
        if cache_name in obj.__dict__:
          del obj.__dict__[cache_name]
    self._syncr_group = None
    self._syncr_names.clear()


class dxl:

  BaudrateList = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000, 8: 6000000, 9: 10500000}
  StatusError = {0x01: 'Result Fail', 0x02: 'Instruction Error', 0x03: 'CRC Error', 0x04: 'Data Range Error', 0x05: 'Data Length Error', 0x06: 'Data Limit Error', 0x07: 'Access Error'}

  # items format
  #   item name : (address, format, 'direction', (range), unit/(unit), coefficient/(coefficient))

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
      if any(op in data for op in '+*/-') and not any(b in data for b in '[]{}'):
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

  def _custom_formatwarning(self, message, category, filename, lineno, line=None):
    return f'{category.__name__}: {message}\n'

  def __init__(self, dx_instance, dxl_id):
    warnings.formatwarning = self._custom_formatwarning
    self._cache = {}
    self._model_index = self._build_index('model_data/')
    self._dx = dx_instance
    self._id = dxl_id
    self._modelno = self._dx.Read16(dxl_id, 0)
    if self._modelno is not None:
      json_path = self._model_index['models'].get(f'0x{self._modelno:04X}')
      if not json_path:
        self._modelname = None
        self._items = {}
        warnings.warn(f'Model(0x{self._modelno:04X}) is not supported.', UserWarning)
      else:
        model_info = self._model_index.get('models').get(f'0x{self._modelno:04X}')
        self._modelname = model_info.get('modelname')
        self._items = self._model_index.get(model_info.get('controltable')).copy()
        self._items.update(model_info.get('overrides', {}))
        [self._items.pop(key, None) for key in model_info.get('remove', {})]
        self._items = dict(sorted(self._items.items(), key=lambda x: x[1][0]))
    else:
      self._items = {}
      self._modelname = None

  def __enter__(self):
    return self

  def __exit__(self, ex_type, ex_value, trace):
    pass

  def __dir__(self):
    return dir(super()) + list(self._items)

  @property
  def id(self):
    return self._id

  @property
  def modelname(self):
    return self._modelname

  @property
  def modelno(self):
    return self._modelno

  @property
  def items(self):
    return self._items

  def updateitems(self, itm):
    self._items.update(itm)
    self._items = dict(sorted(self._items.items(), key=lambda x: x[1][0]))

  def dump(self):
    for i in self._items:
      print(f'{self._items[i][0]}:{i}={self.__getattr__(i)}')

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

      group = self._dx._syncr_group
      if group is not None:
        if self in group:
          if name not in self._dx._syncr_names:
            results = dict(self._dx.SyncRead(addr, size, [obj.id for obj in self._dx._syncr_group]))
            for obj in group:
              if results and obj.id in results:
                wrapped_val = self._conv_format_value(name, fmt, unit, coef, results[obj.id])
                setattr(obj, f"_{name}", wrapped_val)
              else:
                setattr(obj, f"_{name}", None)
              self._dx._syncr_names.add(name)
            return getattr(self, f"_{name}", 0)

      r = self._dx.Read(self._id, addr, size)
      if r is not None:
        return self._conv_format_value(name, fmt, unit, coef, r)
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
        if isinstance(v_min, list | tuple) and isinstance(v_max, list | tuple) and isinstance(value, list | tuple):
          newvalue = list(value)
          for i, (_vmin, _vmax, _v) in enumerate(zip(v_min, v_max, newvalue)):
            if not (_vmin <= _v <= _vmax):
              _nv = min(max(_v, _vmin), _vmax)
              warnings.warn(f'Out of Range: {name} ({_v}) must be {_vmin}~{_vmax}. Clipped at {_nv}', UserWarning)
              newvalue[i] = _nv
          value = list(newvalue)

        elif not (v_min <= value <= v_max):
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
  from time import sleep, time
  import random, traceback, sys

  def wait(t):
    end_time = time() + t
    while end_time > time():
      yield end_time - time()

  with DXL_master('\\\\.\\COM20', 2000000, timeoutoffset=0.5) as dx2:
    try:
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

      if d == []:
        sys.exit()

      for _d in d:
        _d.Torque_Enable = 1
        _d.Profile_Velocity.phys = 29.0

      for _d in d:
        _d.Goal_Position.phys = 0
      sleep(1)
      for _d in d:
        _d.Goal_Position.phys = -90.0
      sleep(1)
      for _d in d:
        _d.Goal_Position.phys = 90.0
      sleep(1)

      for i in range(36):
        d[0].Goal_Position.phys = -179 + 10.0 * i
        for j in range(10):
          print(f'{d[0].Goal_Position.phys:7.1f} {d[0].Present_Position.phys:7.1f}', end='\r')
          sleep(0.01)
      else:
        print()

      with dx2.sync_write(d[0].Goal_Position.info):
        for _d in d:
          _d.Goal_Position = 0
      sleep(2.5)

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

      if d == []:
        sys.exit()

      d[0].dump()
      input('Waiting for the Enter key to be pressed...')

      print('>all axis off')
      for _d in d:
        _d.Torque_Enable = 0
        _d.Profile_Velocity = 0
        print(f' {_d.id} Torque_Enable={_d.Torque_Enable}')

      print('>all axis pos monitor with SI value (sync read)')
      for t in wait(5):
        p = []
        with dx2.sync_read([_d for _d in d]):
          p = [_d.Present_Position.phys for _d in d]
        print(f' {t:4.1f} present position=', *[f'{_p:6.1f}' for _p in p], end='\r')
      else:
        print()

      print('>all axis on')
      for _d in d:
        _d.Operating_Mode = 3
        _d.Torque_Enable = 1
        _d.Profile_Velocity = 0
        print(f' {_d.id} Torque_Enable={_d.Torque_Enable} Operating_Mode={_d.Operating_Mode}')

      print('>1 axis pos set with SI value')
      for i in range(36):
        d[0].Goal_Position.phys = 10.0 * i
        for _ in wait(0.5):
          print(f'{d[0].Goal_Position.phys:6.1f} {d[0].Present_Position.phys:7.1f}', end='\r')
      else:
        print()

      print('>1 axis velo+pos with SI value')
      d[0].GoalVeloPos.phys = 20.0, 0.5
      sleep(1)
      d[0].GoalVeloPos.phys = 10.0, 280.0
      sleep(1)
      d[0].GoalVeloPos.phys = 60.0, 180.0
      sleep(1)

      print('>all axis pos set (sync write)')
      with dx2.sync_write(d[0].Goal_Position.info):
        for _d in d:
          _d.Goal_Position = 0
      sleep(1)

      with dx2.sync_write(d[0].Goal_Position.info):
        for _d in d:
          _d.Goal_Position = 2048
      sleep(1)

      with dx2.sync_write(d[0].Goal_Position.info):
        for _d in d:
          _d.Goal_Position = 4095
      sleep(1)

      print('>all axis LED flash (sync write)')
      for i in range(50):
        with dx2.sync_write(d[0].LED.info):
          for _d in d:
            _d.LED ^= 1
        sleep(0.01)

      print('>all axis velo+pos set (sync write)')
      for i, p in enumerate(range(0, 4095, 100)):
        print(f'sync {i + 1}/41', end='\r')
        with dx2.sync_write(d[0].GoalVeloPos.info):
          for _d in d:
            _d.GoalVeloPos = 45, p
        for _ in wait(0.5):
          with dx2.sync_read([_d for _d in d]):
            print(*[f'{_d.Present_Position:5}' for _d in d], end='\r')
      else:
        print()

      print('>all axis ramdom test(bulk write)')
      for i in range(20):
        print(f' {i + 1}/20', end='\r')
        with dx2.bulk_write():
          if len(d) >= 1: d[0].GoalVeloPos = random.randint(50, 300), random.randint(2048 - 512, 2048 + 512)
          if len(d) >= 2: d[1].Goal_Position = random.randint(2048 - 512, 2048 + 512)
          if len(d) >= 3: d[2].Goal_Position = random.randint(2048 - 512, 2048 + 512)
          if len(d) >= 4: d[3].LED = random.randint(0, 1)
          if len(d) >= 5: d[4].LED = random.randint(0, 1)
          if len(d) >= 6: d[5].LED = random.randint(0, 1)
        sleep(0.3)
      else:
        print()

      with dx2.sync_write(d[0].GoalVeloPos.info):
        for _d in d:
          _d.GoalVeloPos = 0, 2047
      sleep(0.5)

    except KeyboardInterrupt:
      pass
    except:
      print('--- Caught Exception ---')
      traceback.print_exc()
      print('------------------------')
    finally:
      if d != []:
        sleep(0.3)
        for _d in d:
          _d.Torque_Enable = 0
      print('fin.')
