## Usage
When using this, make sure to place `pyDXL.py` in the same directory.

Since “pyDXL.py” also contains code for unit testing, please use that to handle simple tests.<br>
This directory contains scripts for changing ID and baud rates, as well as for searching for them if you've forgotten them.

Furthermore, I have prepared a script (`dxl2.py`) that operates the DYNAMIXEL using the names of items assigned in the control table, without relying on serial communication or dedicated commands. This script defines the dxl class; by instantiating it and associating a single DYNAMIXEL with it, you can utilize its functions. The control table is generated based on a JSON file located in the `model_data` directory, but you can add parameters via code as needed.<br>
I’ll briefly touch on `dxl2.py` below.

Use `pyDXL` to initialize the interface, then pass the instance and the DYNAMIXEL's ID to `dxl` to instantiate it.
``` python
from pyDXL import DXLProtocolV2
from dxl2 import dxl
with DXLProtocolV2('\\\\.\\COM10', 115200, timeoutoffset=0.1) as dx_if:
  dx1 = dxl(dx_if, 1)
```
After communicating with the ID specified at this point, the information in the internal model is updated. If there is a problem, `dx1.modelname` will be None, so you can use that to determine whether the operation succeeded or failed.

If successful, the memory map will be assigned as a property of dx1, so you can simply read and write to it using those names, just as you would with variables.
``` python
  dx1.Torque_Enable = 0
  dx1.Operating_Mode = 3
  print(f'OPMode={bin(dx1.Operating_Mode)}')
  dx1.Torque_Enable = 1
  dx1.Profile_Velocity = 0

  dx1.Goal_Position = 1024
```
Note that some parameters have coefficient to SI units, which can be accessed by adding the `.phys` suffix as a subproperty. As mentioned earlier, you can also redefine the parameters yourself.
``` python
  dx1.updateitems({
    'GoalVeloPos': (112, 'ii', 'rw', (None, None), ('rpm', 'deg'), (0.229, 360 / 4096)),
    'PresentValues': (124, 'hhiiiiHB', 'r', (None, None), ('%', '', 'rpm', 'deg', 'rpm', 'deg', 'V', 'degC'), (100 / 885, None, 0.229, 360 / 4096, 0.229, 360 / 4096, 0.1, 1.0)),
  })

  print(dx1.PresentValues.phys)
  dx1.GoalVeloPos.phys = 120.0, 30.0
```
You can also easily write synchronization logic using the `with` statement.
``` python
  with dx1.sync_read(dx1, dx2, dx3, dx4):
    p1 = dx1.Present_Position
    p2 = dx2.Present_Position
    p3 = dx3.Present_Position
    p4 = dx4.Present_Position
  print(pos1, pos2, pos3, pos4)
```

Please do give it a try.
