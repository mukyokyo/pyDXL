#!/usr/bin/env python3

# A class that allows control tables to be treated like properties.
# It can handle numerical clips and physical values, and you can add items with custom structures.
# Since the control table is of type `dict`, one option would be to save it as a separate file for
# each model and load it from there.
# Oh, right, it also supports sync and bulk writes.

from pyDXL import DXLProtocolV2
import warnings, struct, contextlib


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
  Yseries = {
    'Model_Number': (0, 'H', 'r', (None, None), '', None),
    'Model_Information': (2, 'I', 'r', (None, None), '', None),
    'Firmware_Version': (6, 'B', 'r', (None, None), '', None),
    'ID': (7, 'B', 'rw', (None, None), '', None),
    'Bus_Watchdog': (8, 'B', 'rw', (0, 127), '', None),
    'Secondary_ID': (10, 'B', 'rw', (None, None), '', None),
    'Protocol_Type': (11, 'B', 'rw', (None, None), '', None),
    'Baudrate': (12, 'B', 'rw', (None, None), '', None),
    'Return_Delay_Time': (13, 'B', 'rw', (0, 254), '', None),
    'Status_Return_Level': (15, 'B', 'rw', (None, None), '', None),
    'Registered_Instruction': (16, 'B', 'r', (None, None), '', None),
    'Drive_Mode': (32, 'B', 'rw', (None, None), '', None),
    'Operating_Mode': (33, 'B', 'rw', (None, None), '', None),
    'Startup_Configuration': (34, 'B', 'rw', (None, None), '', None),
    'Position_Limit_Threshold': (38, 'H', 'rw', (0, 32767), '', None),
    'InPosition_Threshold': (40, 'I', 'rw', (0, 2147483647), '', None),
    'Following_Error_Threshold': (44, 'I', 'rw', (0, 2147483647), '', None),
    'Moving_Threshold': (48, 'I', 'rw', (0, 2147483647), '', None),
    'Homing_Offset': (52, 'i', 'rw', (-2147483648, 2147483647), '', None),
    'Inverter_Temperature_Limit': (56, 'B', 'rw', (0, 100), 'degC', 1.0),
    'Motor_Temperature_Limit': (57, 'B', 'rw', (0, 100), 'degC', 1.0),
    'Max_Voltage_Limit': (60, 'H', 'rw', (240, 400), 'V', 0.1),
    'Min_Voltage_Limit': (62, 'H', 'rw', (160, 400), 'V', 0.1),
    'PWM_Limit': (64, 'H', 'rw', (0, 1000), '%', 0.1),
    'Current_Limit': (66, 'H', 'rw', (0, 2080), 'mA', 10.0),
    'Acceleration_Limit': (68, 'I', 'rw', (0, 38520000), 'rpm^2', 10.0),
    'Velocity_Limit': (72, 'I', 'rw', (0, 642200), 'rpm^2', 0.01),
    'Max_Position_Limit': (76, 'i', 'rw', (-2147483648, 2147483647), '', None),
    'Min_Position_Limit': (84, 'i', 'rw', (-2147483648, 2147483647), '', None),
    'Electronic_GearRatio_Numerator': (96, 'I', 'rw', (1, 1045576), '', None),
    'Electronic_GearRatio_Denominator': (100, 'I', 'rw', (1, 1045576), '', None),
    'Safe_Stop_Time': (104, 'H', 'rw', (0, 32767), 'ms', 1.0),
    'Brake_Delay': (106, 'H', 'rw', (0, 32767), 'ms', 1.0),
    'Goal_Update_Delay': (108, 'H', 'rw', (0, 32767), 'ms', 1.0),
    'Overexcitation_Voltage': (110, 'B', 'rw', (0, 100), '%', 1.0),
    'Normal_Excitation_Voltage': (111, 'B', 'rw', (0, 100), '%', 1.0),
    'Overexcitation_Time': (112, 'H', 'rw', (0, 32767), 'ms', 1.0),
    'Present_Velocity_LPF_Frequency': (132, 'H', 'rw', (0, 32767), 'Hz', 0.1),
    'Goal_Current_LPF_Frequency': (134, 'H', 'rw', (0, 32767), 'Hz', 0.1),
    'Position_FF_LPF_Time': (136, 'H', 'rw', (0, 512), 'ms', 0.2),
    'Velocity_FF_LPF_Time': (138, 'H', 'rw', (0, 512), 'ms', 0.2),
    'Controller_State': (152, 'B', 'r', (None, None), '', None),
    'Error_Code': (153, 'B', 'r', (None, None), '', None),
    'Error_Code_History': (154, 'B' * 16, 'r', (None, None), '', None),
    'Hybrid_Save': (170, 'B', 'rw', (0, 2), '', None),
    'Velocity_I_Gain': (212, 'i', 'rw', (0, 2147483647), '', None),
    'Velocity_P_Gain': (216, 'i', 'rw', (0, 2147483647), '', None),
    'Velocity_FF_Gain': (220, 'i', 'rw', (0, 2147483647), '', None),
    'Position_D_Gain': (224, 'i', 'rw', (0, 2147483647), '', None),
    'Position_I_Gain': (228, 'i', 'rw', (0, 2147483647), '', None),
    'Position_P_Gain': (232, 'i', 'rw', (0, 2147483647), '', None),
    'Position_FF_Gain': (236, 'i', 'rw', (0, 2147483647), '', None),
    'Profile_Acceleration': (240, 'i', 'rw', (None, None), 'rpm^2', 10.0),
    'Profile_Velocity': (244, 'i', 'rw', (None, None), 'rpm', 0.01),
    'Profile_Acceleration_Time': (248, 'i', 'rw', (0, 262144), 'ms', 0.2),
    'Profile_Time': (252, 'i', 'rw', (0, 524288), 'ms', 0.2),
    'Indirect_Address_1_128': (256, 'H' * 128, 'rw', (None, None), '', None),
    'Torque_Enable': (512, 'B', 'rw', (0, 1), '', None),
    'LED': (513, 'B', 'rw', (0, 1), '', None),
    'PWM_Offset': (516, 'h', 'rw', (None, None), '%', 0.1),
    'Current_Offset': (518, 'h', 'rw', (-2080, 2080), 'mA', 10.0),
    'Velocity_Offset': (520, 'i', 'rw', (None, None), 'rpm', 0.01),
    'Goal_PWM': (524, 'h', 'rw', (None, None), '%', 0.1),
    'Goal_Current': (526, 'h', 'rw', (-2080, 2080), 'mA', 10.0),
    'Goal_Velocity': (528, 'i', 'rw', (None, None), 'rpm', 0.01),
    'Goal_Position': (532, 'i', 'rw', (None, None), 'deg', 180 / 262144),
    'Moving_Status': (541, 'B', 'r', (None, None), '', None),
    'Realtime_Tick': (542, 'H', 'r', (None, None), 'ms', 1.0),
    'Present_PWM': (544, 'h', 'r', (None, None), '%', 0.1),
    'Present_Current': (546, 'h', 'r', (None, None), 'mA', 10.0),
    'Present_Velocity': (548, 'i', 'r', (None, None), 'rpm', 0.01),
    'Present_Position': (552, 'i', 'r', (None, None), 'deg', 180 / 262144),
    'Position_Trajectory': (560, 'I', 'r', (None, None), '', None),
    'Velocity_Trajectory': (564, 'I', 'r', (None, None), '', None),
    'Present_Input_Voltage': (568, 'H', 'r', (None, None), 'V', 0.1),
    'Present_Inverter_Temperature': (570, 'B', 'r', (None, None), 'degC', 1.0),
    'Present_Motor_Temperature': (571, 'B', 'r', (None, None), 'degC', 1.0),
    'Indirect_Data_1_128': (634, 'B' * 128, 'rw', (None, None), '', None),
    'Backup_Ready': (919, 'B', 'rw', (None, None), '', None),
  }

  Pseries = {
    'Model_Number': (0, 'H', 'r', (None, None), '', None),
    'Model_Information': (2, 'I', 'r', (None, None), '', None),
    'Firmware_Version': (6, 'B', 'r', (None, None), '', None),
    'ID': (7, 'B', 'rw', (None, None), '', None),
    'Baudrate': (8, 'B', 'rw', (None, None), '', None),
    'Return_Delay_Time': (9, 'B', 'rw', (0, 254), '', None),
    'Drive_Mode': (10, 'B', 'rw', (None, None), '', None),
    'Operating_Mode': (11, 'B', 'rw', (None, None), '', None),
    'Secondary_ID': (12, 'B', 'rw', (None, None), '', None),
    'Protocol_Type': (13, 'B', 'rw', (None, None), '', None),
    'Homing_Offset': (20, 'i', 'rw', (-2147483648, 2147483647), '', None),
    'Moving_Threshold': (24, 'I', 'rw', (None, None), 'rpm', 0.01),
    'Temperature_Limit': (31, 'B', 'rw', (0, 100), 'degC', 1.0),
    'Max_Voltage_Limit': (32, 'H', 'rw', (None, None), 'V', 0.1),
    'Min_Voltage_Limit': (34, 'H', 'rw', (None, None), 'V', 0.1),
    'PWM_Limit': (36, 'H', 'rw', (None, None), '%', 100.0 / 2009),
    'Current_Limit': (38, 'H', 'rw', (None, None), 'mA', 1.0),
    'Acceleration_Limit': (40, 'i', 'rw', (None, None), 'rpm^2', 1.0),
    'Velocity_Limit': (44, 'i', 'rw', (None, None), 'rpm', 0.01),
    'Max_Position_Limit': (48, 'I', 'rw', (None, None), '', None),
    'Min_Position_Limit': (52, 'I', 'rw', (None, None), '', None),
    'External_Port_Mode_1': (56, 'B', 'rw', (0, 3), '', None),
    'External_Port_Mode_2': (57, 'B', 'rw', (0, 3), '', None),
    'External_Port_Mode_3': (58, 'B', 'rw', (0, 3), '', None),
    'Startup_Configuration': (60, 'B', 'rw', (None, None), '', None),
    'Shutdown': (63, 'B', 'rw', (None, None), '', None),
    'Indirect_Address_1_128': (168, 'H' * 128, 'rw', (None, None), '', None),
    'Torque_Enable': (512, 'B', 'rw', (0, 1), '', None),
    'LED_Red': (513, 'B', 'rw', (0, 255), '', None),
    'LED_Green': (513, 'B', 'rw', (0, 255), '', None),
    'LED_Blue': (513, 'B', 'rw', (0, 255), '', None),
    'Status_Return_Level': (516, 'B', 'rw', (0, 2), '', None),
    'Registered_Instruction': (517, 'B', 'r', (0, 1), '', None),
    'Hardware_Error_Status': (518, 'B', 'r', (None, None), '', None),
    'Velocity_I_Gain': (524, 'H', 'rw', (0, 32767), '', None),
    'Velocity_P_Gain': (526, 'H', 'rw', (0, 32767), '', None),
    'Position_D_Gain': (528, 'H', 'rw', (0, 32767), '', None),
    'Position_I_Gain': (530, 'H', 'rw', (0, 32767), '', None),
    'Position_P_Gain': (532, 'H', 'rw', (0, 32767), '', None),
    'Feedforward_2nd_Gain': (536, 'H', 'rw', (0, 32767), '', None),
    'Feedforward_1st_Gain': (538, 'H', 'rw', (0, 32767), '', None),
    'Bus_Watchdog': (546, 'B', 'rw', (1, 127), '', None),
    'Goal_PWM': (548, 'h', 'rw', (None, None), '%', 100.0 / 2009),
    'Goal_Current': (550, 'h', 'rw', (None, None), 'mA', 1.0),
    'Goal_Velocity': (552, 'i', 'rw', (None, None), 'rpm', 0.01),
    'Profile_Acceleration': (556, 'i', 'rw', (0, None), 'rpm^2', 1.0),
    'Profile_Velocity': (560, 'i', 'rw', (0, None), 'rpm', 0.01),
    'Goal_Position': (564, 'i', 'rw', (None, None), '', None),
    'Realtime_Tick': (568, 'H', 'r', (0, 32767), 'ms', 1),
    'Moving': (570, 'B', 'r', (0, 1), '', None),
    'Moving_Status': (571, 'B', 'r', (None, None), '', None),
    'Present_PWM': (572, 'h', 'r', (None, None), '%', 100.0 / 2009),
    'Present_Current': (574, 'h', 'r', (None, None), 'mA', 1.0),
    'Present_Velocity': (576, 'i', 'r', (None, None), 'rpm', 0.01),
    'Present_Position': (580, 'i', 'r', (None, None), '', None),
    'Velocity_Trajectory': (584, 'i', 'r', (None, None), 'rpm', 0.01),
    'Position_Trajectory': (588, 'i', 'r', (None, None), '', None),
    'Present_Input_Voltage': (592, 'H', 'r', (None, None), 'V', 0.1),
    'Present_Temperature': (594, 'B', 'r', (None, None), 'degC', 1.0),
    'External_Port_Data_1': (600, 'H', 'rw', (None, None), '', None),
    'External_Port_Data_2': (602, 'H', 'rw', (None, None), '', None),
    'External_Port_Data_3': (604, 'H', 'rw', (None, None), '', None),
    'External_Port_Data_4': (606, 'H', 'rw', (None, None), '', None),
    'Indirect_Data_1_128': (634, 'B' * 128, 'rw', (None, None), '', None),
    'Backup_Ready': (878, 'B', 'r', (0, 1), '', None),
  }

  PROseries = {
    'Model_Number': (0, 'H' * 1, 'r', (None, None), '', None),
    'Model_Information': (2, 'I' * 1, 'r', (None, None), '', None),
    'Firmware_Version': (6, 'B' * 1, 'r', (None, None), '', None),
    'ID': (7, 'B' * 1, 'rw', (None, None), '', None),
    'Baudrate': (8, 'B' * 1, 'rw', (None, None), '', None),
    'Return_Delay_Time': (9, 'B' * 1, 'rw', (None, None), '', None),
    'Operating_Mode': (11, 'B' * 1, 'rw', (None, None), '', None),
    'Homing_Offset': (13, 'i' * 1, 'rw', (None, None), '', None),
    'Moving_Threshold': (17, 'I' * 1, 'rw', (None, None), '', None),
    'Temperature_Limit': (21, 'B' * 1, 'rw', (None, None), 'degC', 1.0),
    'Max_Voltage_Limit': (22, 'H' * 1, 'rw', (None, None), 'V', 0.1),
    'Min_Voltage_Limit': (24, 'H' * 1, 'rw', (None, None), 'V', 0.1),
    'Acceleration_Limit': (26, 'I' * 1, 'rw', (None, None), '', None),
    'Torque_Limit': (30, 'H' * 1, 'rw', (None, None), '', None),
    'Velocity_Limit': (32, 'I' * 1, 'rw', (None, None), '', None),
    'Max_Position_Limit': (36, 'i' * 1, 'rw', (None, None), '', None),
    'Min_Position_Limit': (40, 'i' * 1, 'rw', (None, None), '', None),
    'External_Port_Mode_1': (44, 'B' * 1, 'rw', (None, None), '', None),
    'External_Port_Mode_2': (45, 'B' * 1, 'rw', (None, None), '', None),
    'External_Port_Mode_3': (46, 'B' * 1, 'rw', (None, None), '', None),
    'External_Port_Mode_4': (47, 'B' * 1, 'rw', (None, None), '', None),
    'Shutdown': (48, 'B' * 1, 'rw', (None, None), '', None),
    'Indirect_Address_1_256': (49, 'H' * 256, 'rw', (None, None), '', None),
    'Torque_Enable': (562, 'B' * 1, 'rw', (None, None), '', None),
    'LED_Red': (563, 'B' * 1, 'rw', (None, None), '', None),
    'LED_Green': (564, 'B' * 1, 'rw', (None, None), '', None),
    'LED_Blue': (565, 'B' * 1, 'rw', (None, None), '', None),
    'Velocity_I_Gain': (586, 'H' * 1, 'rw', (None, None), '', None),
    'Velocity_P_Gain': (588, 'H' * 1, 'rw', (None, None), '', None),
    'Position_P_Gain': (594, 'H' * 1, 'rw', (None, None), '', None),
    'Goal_Position': (596, 'i' * 1, 'rw', (None, None), '', None),
    'Goal_Velocity': (600, 'i' * 1, 'rw', (None, None), '', None),
    'Goal_Torque': (604, 'h' * 1, 'rw', (None, None), '', None),
    'Goal_Acceleration': (606, 'i' * 1, 'rw', (None, None), '', None),
    'Moving': (610, 'B' * 1, 'r', (None, None), '', None),
    'Present_Position': (611, 'i' * 1, 'r', (None, None), '', None),
    'Present_Velocity': (615, 'i' * 1, 'r', (None, None), '', None),
    'Present_Current': (621, 'h' * 1, 'r', (None, None), '', None),
    'Present_Input_Voltage': (623, 'H' * 1, 'r', (None, None), 'V', 0.1),
    'Present_Temperature': (625, 'B' * 1, 'r', (None, None), 'degC', 1.0),
    'External_Port_Data_1': (626, 'H' * 1, 'rw', (None, None), '', None),
    'External_Port_Data_2': (628, 'H' * 1, 'rw', (None, None), '', None),
    'External_Port_Data_3': (630, 'H' * 1, 'rw', (None, None), '', None),
    'External_Port_Data_4': (632, 'H' * 1, 'rw', (None, None), '', None),
    'Indirect_Data_1_256': (634, 'B' * 256, 'rw', (None, None), '', None),
    'Registered_Instruction': (890, 'B' * 1, 'r', (None, None), '', None),
    'Status_Return_Level': (891, 'B' * 1, 'rw', (None, None), '', None),
    'Hardware_Error_Status': (892, 'B' * 1, 'r', (None, None), '', None),
  }

  Xseries = {
    'Model_Number': (0, 'H', 'r', (None, None), '', None),
    'Model_Information': (2, 'I', 'r', (None, None), '', None),
    'Firmware_Version': (6, 'B', 'r', (None, None), '', None),
    'ID': (7, 'B', 'rw', (None, None), '', None),
    'Baudrate': (8, 'B', 'rw', (None, None), '', None),
    'Return_Delay_Time': (9, 'B', 'rw', (None, None), '', None),
    'Drive_Mode': (10, 'B', 'rw', (None, None), '', None),
    'Operating_Mode': (11, 'B', 'rw', (None, None), '', None),
    'Secondary_ID': (12, 'B', 'rw', (None, None), '', None),
    'Protocol_Type': (13, 'B', 'rw', (None, None), '', None),
    'Homing_Offset': (20, 'i', 'rw', (-1044479, 1044479), 'deg', 360 / 4096),
    'Moving_Threshold': (24, 'I', 'rw', (0, 1023), 'rpm', 0.229),
    'Temperature_Limit': (31, 'B', 'rw', (0, 100), 'degC', 1.0),
    'Max_Voltage_Limit': (32, 'H', 'rw', (None, None), 'V', 0.1),
    'Min_Voltage_Limit': (34, 'H', 'rw', (None, None), 'V', 0.1),
    'PWM_Limit': (36, 'H', 'rw', (0, 885), '%', 100 / 885),
    'Current_Limit': (38, 'H', 'rw', (None, None), 'mA', 2.69),
    'Acceleration_Limit': (40, 'i', 'rw', (None, None), '', None),
    'Velocity_Limit': (44, 'i', 'rw', (None, None), 'rpm', 0.229),
    'Max_Position_Limit': (48, 'I', 'rw', (0, 4095), 'deg', 360 / 4096),
    'Min_Position_Limit': (52, 'I', 'rw', (0, 4095), 'deg', 360 / 4096),
    'External_Port_Mode_1': (56, 'B', 'rw', (0, 3), '', None),
    'External_Port_Mode_2': (57, 'B', 'rw', (0, 3), '', None),
    'External_Port_Mode_3': (58, 'B', 'rw', (0, 3), '', None),
    'Startup_Configuration': (60, 'B', 'rw', (0, 3), '', None),
    'Shutdown': (63, 'B', 'rw', (None, None), '', None),
    'Torque_Enable': (64, 'B', 'rw', (0, 1), '', None),
    'LED': (65, 'B', 'rw', (0, 1), '', None),
    'Status_Return_Level': (68, 'B', 'rw', (0, 2), '', None),
    'Registered_Instruction': (69, 'B', 'r', (0, 1), '', None),
    'Hardware_Error_Status': (70, 'B', 'r', (None, None), '', None),
    'Velocity_I_Gain': (76, 'H', 'rw', (0, 16383), '', None),
    'Velocity_P_Gain': (78, 'H', 'rw', (0, 16383), '', None),
    'Position_D_Gain': (80, 'H', 'rw', (0, 16383), '', None),
    'Position_I_Gain': (82, 'H', 'rw', (0, 16383), '', None),
    'Position_P_Gain': (84, 'H', 'rw', (0, 16383), '', None),
    'Feedforward_2nd_Gain': (88, 'H', 'rw', (0, 16383), '', None),
    'Feedforward_1st_Gain': (90, 'H', 'rw', (0, 16383), '', None),
    'Bus_Watchdog': (98, 'B', 'rw', (1, 127), '', None),
    'Goal_PWM': (100, 'h', 'rw', (0, 885), '%', 100 / 885),
    'Goal_Current': (102, 'h', 'rw', (None, None), 'mA', 2.69),
    'Goal_Velocity': (104, 'i', 'rw', (None, None), 'rpm', 0.229),
    'Profile_Acceleration': (108, 'i', 'rw', (0, 32767), 'rpm^2', 214.577),
    'Profile_Time': (108, 'i', 'rw', (0, 32767), 'ms', 1.0),
    'Profile_Velocity': (112, 'i', 'rw', (0, 32767), 'rpm', 0.229),
    'Goal_Position': (116, 'i', 'rw', (None, None), 'deg', 360 / 4096),
    'Realtime_Tick': (120, 'H', 'r', (0, 32767), 'ms', 1),
    'Moving': (122, 'B', 'r', (0, 1), '', None),
    'Moving_Status': (123, 'B', 'r', (None, None), '', None),
    'Present_PWM': (124, 'h', 'r', (None, None), '%', 100 / 885),
    'Present_Current': (126, 'h', 'r', (None, None), 'mA', 2.69),
    'Present_Velocity': (128, 'i', 'r', (None, None), 'rpm', 0.229),
    'Present_Position': (132, 'i', 'r', (None, None), 'deg', 360 / 4096),
    'Velocity_Trajectory': (136, 'i', 'r', (None, None), 'rpm', 0.229),
    'Position_Trajectory': (140, 'i', 'r', (None, None), 'deg', 360 / 4096),
    'Present_Input_Voltage': (144, 'H', 'r', (None, None), 'V', 0.1),
    'Present_Temperature': (146, 'B', 'r', (None, None), 'degC', 1.0),
    'Backup_Ready': (147, 'B', 'r', (0, 1), '', None),
    'External_Port_Data_1': (152, 'B', 'rw', (None, None), '', None),
    'External_Port_Data_2': (154, 'B', 'rw', (None, None), '', None),
    'External_Port_Data_3': (156, 'B', 'rw', (None, None), '', None),
    'Indirect_Address_1_28': (168, 'H' * 28, 'rw', (None, None), '', None),
    'Indirect_Data_1_28': (224, 'B' * 28, 'rw', (None, None), '', None),
    'Indirect_Address_29_56': (578, 'H' * 28, 'rw', (None, None), '', None),
    'Indirect_Data_29_56': (634, 'B' * 28, 'rw', (None, None), '', None),
  }

  models = {
    0x0FA0: {'modelname': 'YM070-210-M001-RH', 'controltable': Yseries},
    0x0FAA: {'modelname': 'YM070-210-B001-RH', 'controltable': Yseries},
    0x0FB4: {'modelname': 'YM070-210-R051-RH', 'controltable': Yseries,
             'overrides': {
               'Acceleration_Limit': (68, 'I', 'rw', (0, 755294), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 12592), 'rpm^2', 0.01),
             }},
    0x0FC8: {'modelname': 'YM070-210-A051-RH', 'controltable': Yseries,
             'overrides': {
               'Acceleration_Limit': (68, 'I', 'rw', (0, 755294), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 12592), 'rpm^2', 0.01),
             }},
    0x0FBE: {'modelname': 'YM070-210-R099-RH', 'controltable': Yseries,
             'overrides': {
               'Acceleration_Limit': (68, 'I', 'rw', (0, 389090), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 6486), 'rpm^2', 0.01), }},
    0x0FD2: {'modelname': 'YM070-210-A099-RH', 'controltable': Yseries,
             'overrides': {
               'Acceleration_Limit': (68, 'I', 'rw', (0, 389090), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 6486), 'rpm^2', 0.01), }},

    0x1018: {'modelname': 'YM080-230-M001-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 21336000), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 355600), 'rpm^2', 0.01), }},
    0x1022: {'modelname': 'YM080-230-B001-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 21336000), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 355600), 'rpm^2', 0.01), }},
    0x102C: {'modelname': 'YM080-230-R051-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 418352), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 6972), 'rpm^2', 0.01), }},
    0x1040: {'modelname': 'YM080-230-A051-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 418352), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 6972), 'rpm^2', 0.01),
             }},
    0x1036: {'modelname': 'YM080-230-R099-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 215515), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 3591), 'rpm^2', 0.01),
             }},
    0x104A: {'modelname': 'YM080-230-A099-RH', 'controltable': Yseries,
             'overrides': {
               'Current_Limit': (66, 'H', 'rw', (0, 2240), 'mA', 10.0),
               'Acceleration_Limit': (68, 'I', 'rw', (0, 215515), 'rpm^2', 10.0),
               'Velocity_Limit': (72, 'I', 'rw', (0, 3591), 'rpm^2', 0.01),
             }},

    0x8900: {'modelname': 'L42-10-S300-R', 'controltable': PROseries},
    0x9428: {'modelname': 'L54-30-S400-R', 'controltable': PROseries},
    0x9408: {'modelname': 'L54-30-S500-R', 'controltable': PROseries},
    0x9520: {'modelname': 'L54-50-S290-R', 'controltable': PROseries},
    0x9508: {'modelname': 'L54-50-S500-R', 'controltable': PROseries},
    0xA918: {'modelname': 'M42-10-S260-R', 'controltable': PROseries},
    0xB410: {'modelname': 'M54-40-S250-R', 'controltable': PROseries},
    0xB510: {'modelname': 'M54-60-S250-R', 'controltable': PROseries},
    0xC800: {'modelname': 'H42-20-S300-R', 'controltable': PROseries},
    0xD208: {'modelname': 'H54-100-S500-R', 'controltable': PROseries},
    0xD308: {'modelname': 'H54-200-S500-R', 'controltable': PROseries},

    0xA919: {'modelname': 'M42-10-S260-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 526375),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 526375),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 526375),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 526375), }},
    0xB411: {'modelname': 'M54-40-S250-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 502834),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 502834), }},
    0xB511: {'modelname': 'M54-60-S250-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 502834),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 502834), }},
    0xC801: {'modelname': 'H42-20-S300-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 607500),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 607500),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 607500),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 607500), }},
    0xD209: {'modelname': 'H54-100-S500-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 1003846),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 1003846), }},
    0xD309: {'modelname': 'H54-200-S500-RA', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 1003846),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 1003846), }},

    0x0834: {'modelname': 'PM42-010-S260-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 526374),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 526374),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 526374),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 526374), }},
    0x083E: {'modelname': 'PM54-040-S250-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 502834),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 502834), }},
    0x0848: {'modelname': 'PM54-060-S250-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 502834),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 502834),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 502834), }},
    0x07D0: {'modelname': 'PH42-020-S300-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 607500),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 607500),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 607500),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 607500), }},
    0x07DA: {'modelname': 'PH54-100-S500-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 1003846),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 1003846), }},
    0x07E4: {'modelname': 'PH54-200-S500-R', 'controltable': Pseries,
             'overrides': {
               'Max_Position_Limit': (48, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Min_Position_Limit': (52, 'I', 'rw', (None, None), 'deg', 360 / 1003846),
               'Goal_Position': (564, 'i', 'rw', (None, None), 'deg', 360 / 1003846),
               'Present_Position': (580, 'i', 'r', (None, None), 'deg', 360 / 1003846), }},

    0x001E: {'modelname': 'MX-28(2.0)', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x0137: {'modelname': 'MX-64(2.0)', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 3.36),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 3.36),
             }},
    0x0141: {'modelname': 'MX-106(2.0)', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 3.36),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 3.36),
             }},
    0x04A6: {'modelname': 'XL330-M077', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1750), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-1750, 1750), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x04B0: {'modelname': 'XL330-M288', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1750), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-1750, 1750), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x04CE: {'modelname': 'XC330-M181', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2352), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-2352, 2352), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x04D8: {'modelname': 'XC330-M288', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 910), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-910, 910), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x04BA: {'modelname': 'XC330-T181', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2352), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-2352, 2352), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x04C4: {'modelname': 'XC330-T288', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 910), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-910, 910), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x0424: {'modelname': 'XL430-W250', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x0442: {'modelname': '2XL430-W250', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x0488: {'modelname': '2XC430-W250', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x042E: {'modelname': 'XC430-W150', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x0438: {'modelname': 'XC430-W240', 'controltable': Xseries,
             'overrides': {
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Present_Load': (126, 'h', 'r', (None, None), '%', 0.1),
             }},
    0x06A5: {'modelname': 'XM335-T323', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 910), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 2047), 'rpm', 0.229),
               'PWM Slope': (62, 'B', 'rw', (1, 255), 'mV/ms', 3.955),
               'Goal_Current': (102, 'h', 'rw', (-910, 910), 'mA', 1.0),
               'Present_Current': (126, 'h', 'r', (None, None), 'mA', 1.0),
             }},
    0x0406: {'modelname': 'XM430-W210', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1193), 'mA', 1.0),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-1193, 1193), 'mA', 1.0),
             }},
    0x03F2: {'modelname': 'XH430-W210', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x03F3: {'modelname': 'XD430-T210', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x041A: {'modelname': 'XH430-V210', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 689), 'mA', 1.34),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-689, 689), 'mA', 1.34),
             }},
    0x03FC: {'modelname': 'XM430-W350', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1193), 'mA', 1.34),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-1193, 1193), 'mA', 1.34),
             }},
    0x03E8: {'modelname': 'XH430-W350', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x03E9: {'modelname': 'XD430-T350', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x0410: {'modelname': 'XH430-V350', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 689), 'mA', 1.34),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-689, 689), 'mA', 1.34),
             }},
    0x0500: {'modelname': 'XW430-T200', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x04F6: {'modelname': 'XW430-T333', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 648), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-648, 648), 'mA', 2.69),
             }},
    0x046A: {'modelname': 'XM540-W150', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x0456: {'modelname': 'XH540-W150', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x0457: {'modelname': 'XD540-T150', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x047E: {'modelname': 'XH540-V150', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1188), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-1188, 1188), 'mA', 2.69),
             }},
    0x0460: {'modelname': 'XM540-W270', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x044C: {'modelname': 'XH540-W270', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x044D: {'modelname': 'XD540-T270', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x0474: {'modelname': 'XH540-V270', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 1188), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-1188, 1188), 'mA', 2.69),
             }},
    0x049C: {'modelname': 'XW540-T140', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
    0x0492: {'modelname': 'XW540-T260', 'controltable': Xseries,
             'overrides': {
               'Current_Limit': (38, 'H', 'rw', (0, 2047), 'mA', 2.69),
               'Velocity_Limit': (44, 'i', 'rw', (0, 1023), 'rpm', 0.229),
               'Goal_Current': (102, 'h', 'rw', (-2047, 2047), 'mA', 2.69),
             }},
  }

  def __del__(self):
    pass

  class WriteError(Exception):
    pass

  class ReadError(Exception):
    pass

  def __init__(self, dx_instance, dxl_id):
    self._dx = dx_instance
    self._id = dxl_id
    model_num = self._dx.Read16(dxl_id, 0)
    if model_num is not None:
      if self.models.get(model_num, {}) != {}:
        self._modelname = self.models.get(model_num, {}).get('modelname', None)
        self._items = self.models.get(model_num, {}).get('controltable').copy()
        self._items.update(self.models.get(model_num, {}).get('overrides', {}))
      else:
        self._modelname = None
        self._items = {}
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

  with DXL_master('\\\\.\\COM10', 57600, timeoutoffset=0.5, protocoltype=2) as dx2:

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
