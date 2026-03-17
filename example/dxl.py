#!/usr/bin/env python3
# ---------------------------------------------
# Access by item name in control table
# ---------------------------------------------
from pyDXL import DXLProtocolV2
from ctypes import c_int8, c_uint8, c_int16, c_uint16, c_int32, c_uint32, c_float, Union, Structure, sizeof, byref, POINTER, memmove
from collections import namedtuple
from enum import Enum


# Minimum unit for each item
class _i8(Union):
  _pack_ = 1
  _fields_ = ('d', c_int8), ('u', c_uint8 * 1)


class _u8(Union):
  _pack_ = 1
  _fields_ = ('d', c_uint8), ('u', c_uint8 * 1)


class _i16(Union):
  _pack_ = 1
  _fields_ = ('d', c_int16), ('u', c_uint8 * 2)


class _u16(Union):
  _pack_ = 1
  _fields_ = ('d', c_uint16), ('u', c_uint8 * 2)


class _i32(Union):
  _pack_ = 1
  _fields_ = ('d', c_int32), ('u', c_uint8 * 4)


class _u32(Union):
  _pack_ = 1
  _fields_ = ('d', c_uint32), ('u', c_uint8 * 4)


class _flt32(Union):
  _pack_ = 1
  _fields_ = ('d', c_float), ('u', c_uint8 * 4)


# Supported Dynamixel
class _TdevYTable(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1),
    ('BusWatchdog', _u8 * 1), ('SecondaryID', _u8 * 1), ('ProtocolType', _u8 * 1),
    ('Baudrate', _u16 * 1), ('ReturnDelayTime', _u8 * 1),
    ('reserve', _u8 * 1),
    ('StatusReturnLevel', _u8 * 1), ('RegisteredInstruction', _u8 * 1),
    ('reserve', _u8 * 15),
    ('DriveMode', _u8 * 1), ('OperatingMode', _u8 * 1), ('StartupConfiguration', _u8 * 1),
    ('reserve', _u8 * 3),
    ('PositionLimitThreshold', _u16 * 1), ('In-PositionThreshold', _u32 * 1), ('FollowingErrorThreshold', _u32 * 1), ('MovingThreshold', _u32 * 1),
    ('HomingOffset', _i32 * 1),
    ('InverterTemperatureLimit', _u8 * 1), ('MotorTemperatureLimit', _u8 * 1),
    ('reserve', _u8 * 2),
    ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1),
    ('PWMLimit', _u16 * 1), ('CurrentLimit', _u16 * 1),
    ('AccelerationLimit', _u32 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _i32 * 1),
    ('reserve', _u8 * 4),
    ('MinPositionLimit', _i32 * 1),
    ('reserve', _u8 * 8),
    ('Electronic GearRatioNumerator', _u32 * 1),
    ('Electronic GearRatioDenominator', _u32 * 1),
    ('SafeStopTime', _u16 * 1), ('BrakeDelay', _u16 * 1), ('GoalUpdateDelay', _u16 * 1),
    ('OverexcitationVoltage', _u8 * 1), ('NormalExcitationVoltage', _u8 * 1), ('OverexcitationTime', _u16 * 1),
    ('reserve', _u8 * 18),
    ('PresentVelocityLPF Frequency', _u16 * 1), ('GoalCurrentLPFFrequency', _u16 * 1),
    ('PositionFFLPFTime', _u16 * 1), ('VelocityFFLPFTime', _u16 * 1),
    ('reserve', _u8 * 12),
    ('Controller State', _u8 * 1),
    ('Error Code', _u8 * 1), ('Error Code History', _u8 * 16),
    ('Hybrid Save', _u8 * 1),
    ('reserve', _u8 * 41),
    ('VelocityIGain', _i32 * 1), ('VelocityPGain', _i32 * 1), ('VelocityFFGain', _i32 * 1),
    ('PositionDGain', _i32 * 1), ('PositionIGain', _i32 * 1), ('PositionPGain', _i32 * 1), ('PositionFFGain', _i32 * 1),
    ('ProfileAcceleration', _i32 * 1), ('ProfileVelocity', _i32 * 1), ('ProfileAcceleration Time', _i32 * 1), ('ProfileTime', _i32 * 1),
    ('IndirectAddress1_128', _u16 * 128),
    ('TorqueEnable', _u8 * 1), ('LED', _u8 * 1),
    ('reserve', _u8 * 2),
    ('PWMOffset', _i16 * 1), ('CurrentOffset', _i16 * 1), ('VelocityOffset', _i32 * 1),
    ('GoalPWM', _i16 * 1), ('GoalCurrent', _i16 * 1), ('GoalVelocity', _i32 * 1), ('GoalPosition', _i32 * 1),
    ('reserve', _u8 * 5),
    ('MovingStatus', _u8 * 1),
    ('RealtimeTick', _u16 * 1),
    ('PresentPWM', _i16 * 1), ('PresentCurrent', _i16 * 1), ('PresentVelocity', _i32 * 1), ('PresentPosition', _i32 * 1),
    ('reserve', _u8 * 4),
    ('PositionTrajectory', _u32 * 1), ('VelocityTrajectory', _u32 * 1),
    ('PresentInputVoltage', _u16 * 1), ('PresentInverterTemperature', _u8 * 1), ('PresentMotorTemperature', _u8 * 1),
    ('reserve', _u8 * 62),
    ('IndirectData1_128', _u8 * 128),
    ('reserve', _u8 * 157),
    ('BackupReady', _u8 * 1),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevPROTable(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('BaudRate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('reserve', _u8 * 1),
    ('OperatingMode', _u8 * 1),
    ('reserve', _u8 * 1),
    ('HomingOffset', _i32 * 1), ('MovingThreshold', _u32 * 1),
    ('TemperatureLimit', _u8 * 1), ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1), ('AccelerationLimit', _u32 * 1), ('TorqueLimit', _u16 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _i32 * 1), ('MinPositionLimit', _i32 * 1),
    ('ExternalPortMode1', _u8 * 1), ('ExternalPortMode2', _u8 * 1), ('ExternalPortMode3', _u8 * 1), ('ExternalPortMode4', _u8 * 1),
    ('Shutdown', _u8 * 1),
    ('IndirectAddress1_256', _u16 * 256),
    ('reserve', _u8 * 1),
    ('TorqueEnable', _u8 * 1),
    ('LEDRed', _u8 * 1), ('LEDGreen', _u8 * 1), ('LEDBlue', _u8 * 1),
    ('reserve', _u8 * 20),
    ('VelocityIGain', _u16 * 1), ('VelocityPGain', _u16 * 1),
    ('reserve', _u8 * 4),
    ('PositionPGain', _u16 * 1),
    ('GoalPosition', _i32 * 1), ('GoalVelocity', _i32 * 1), ('GoalTorque', _i16 * 1), ('GoalAcceleration', _i32 * 1),
    ('Moving', _u8 * 1),
    ('PresentPosition', _i32 * 1), ('PresentVelocity', _i32 * 1),
    ('reserve', _u8 * 2),
    ('PresentCurrent', _i16 * 1), ('PresentInputVoltage', _u16 * 1), ('PresentTemperature', _u8 * 1),
    ('ExternalPortData1', _u16 * 1), ('ExternalPortData2', _u16 * 1), ('ExternalPortData3', _u16 * 1), ('ExternalPortData4', _u16 * 1),
    ('IndirectData1_256', _u8 * 256),
    ('RegisteredInstruction', _u8 * 1), ('StatusReturnLevel', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevPROPTable(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('BaudRate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('DriveMode', _u8 * 1), ('OperatingMode', _u8 * 1),
    ('SecondaryID', _u8 * 1),
    ('ProtocolType', _u8 * 1),
    ('reserve', _u8 * 6),
    ('HomingOffset', _i32 * 1), ('MovingThreshold', _u32 * 1),
    ('reserve', _u8 * 3),
    ('TemperatureLimit', _u8 * 1), ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1), ('PWMLimit', _u16 * 1), ('CurrentLimit', _u16 * 1), ('AccelerationLimit', _u32 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _i32 * 1), ('MinPositionLimit', _i32 * 1),
    ('ExternalPortMode1', _u8 * 1), ('ExternalPortMode2', _u8 * 1), ('ExternalPortMode3', _u8 * 1), ('ExternalPortMode4', _u8 * 1),
    ('StartupConfiguration', _u8 * 1),
    ('reserve', _u8 * 2),
    ('Shutdown', _u8 * 1),
    ('reserve', _u8 * 104),
    ('IndirectAddress1_128', _u16 * 128),
    ('reserve', _u8 * 88),
    ('TorqueEnable', _u8 * 1), ('LEDRed', _u8 * 1), ('LEDGreen', _u8 * 1), ('LEDBlue', _u8 * 1),
    ('StatusReturnLevel', _u8 * 1), ('RegisteredInstruction', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
    ('reserve', _u8 * 5),
    ('VelocityIGain', _u16 * 1), ('VelocityPGain', _u16 * 1), ('PositionDGain', _u16 * 1), ('PositionIGain', _u16 * 1), ('PositionPGain', _u16 * 1),
    ('reserve', _u8 * 2),
    ('Feedforward2ndGain', _u16 * 1), ('Feedforward1stGain', _u16 * 1),
    ('reserve', _u8 * 6),
    ('BusWatchdog', _u8 * 1),
    ('reserve', _u8 * 1),
    ('GoalPWM', _i16 * 1), ('GoalCurrent', _i16 * 1), ('GoalVelocity', _i32 * 1), ('ProfileAcceleration', _u32 * 1), ('ProfileVelocity', _u32 * 1), ('GoalPosition', _i32 * 1),
    ('RealtimeTick', _u16 * 1),
    ('Moving', _u8 * 1), ('MovingStatus', _u8 * 1),
    ('PresentPWM', _i16 * 1), ('PresentCurrent', _i16 * 1), ('PresentVelocity', _i32 * 1), ('PresentPosition', _i32 * 1),
    ('VelocityTrajectory', _u32 * 1), ('PositionTrajectory', _u32 * 1),
    ('PresentInputVoltage', _u16 * 1), ('PresentTemperature', _u8 * 1),
    ('reserve', _u8 * 5),
    ('ExternalPortData1', _u16 * 1), ('ExternalPortData2', _u16 * 1), ('ExternalPortData3', _u16 * 1), ('ExternalPortData4', _u16 * 1),
    ('reserve', _u8 * 26),
    ('IndirectData1_128', _u8 * 128),
    ('reserve', _u8 * 116),
    ('BackupReady', _u8 * 1),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevXTable(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('Baudrate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('DriveMode', _u8 * 1), ('OperatingMode', _u8 * 1),
    ('SecondaryID', _u8 * 1),
    ('ProtocolType', _u8 * 1),
    ('reserve', _u8 * 6),
    ('HomingOffset', _i32 * 1), ('MovingThreshold', _u32 * 1),
    ('reserve', _u8 * 3),
    ('TemperatureLimit', _u8 * 1), ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1), ('PWMLimit', _u16 * 1), ('CurrentLimit', _u16 * 1), ('AccelerationLimit', _u32 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _u32 * 1), ('MinPositionLimit', _u32 * 1),
    ('ExternalPortMode1', _u8 * 1), ('ExternalPortMode2', _u8 * 1), ('ExternalPortMode3', _u8 * 1),
    ('reserve', _u8 * 1),
    ('StartupConfiguration', _u8 * 1),
    ('reserve', _u8 * 2),
    ('Shutdown', _u8 * 1), ('TorqueEnable', _u8 * 1), ('LED', _u8 * 1),
    ('reserve', _u8 * 2),
    ('StatusReturnLevel', _u8 * 1), ('RegisteredInstruction', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
    ('reserve', _u8 * 5),
    ('VelocityIGain', _u16 * 1), ('VelocityPGain', _u16 * 1), ('PositionDGain', _u16 * 1), ('PositionIGain', _u16 * 1), ('PositionPGain', _u16 * 1),
    ('reserve', _u8 * 2),
    ('Feedforward2ndGain', _u16 * 1), ('Feedforward1stGain', _u16 * 1),
    ('reserve', _u8 * 6),
    ('BusWatchdog', _i8 * 1),
    ('reserve', _u8 * 1),
    ('GoalPWM', _i16 * 1), ('GoalCurrent', _i16 * 1), ('GoalVelocity', _i32 * 1),
    ('ProfileAcceleration', _u32 * 1), ('ProfileVelocity', _u32 * 1),
    ('GoalPosition', _i32 * 1),
    ('RealtimeTick', _u16 * 1),
    ('Moving', _u8 * 1), ('MovingStatus', _u8 * 1),
    ('PresentPWM', _i16 * 1), ('PresentCurrent', _i16 * 1), ('PresentVelocity', _i32 * 1), ('PresentPosition', _i32 * 1),
    ('VelocityTrajectory', _u32 * 1), ('PositionTrajectory', _u32 * 1),
    ('PresentInputVoltage', _u16 * 1), ('PresentTemperature', _u8 * 1), ('BackupReady', _u8 * 1),
    ('reserve', _u8 * 4),
    ('ExternalPortData1', _u16 * 1), ('ExternalPortData2', _u16 * 1), ('ExternalPortData3', _u16 * 1),
    ('reserve', _u8 * 10),
    ('IndirectAddress1_28', _u16 * 28), ('IndirectData1_28', _u8 * 28),
    ('reserve', _u8 * 326),
    ('IndirectAddress29_56', _u16 * 28), ('IndirectData29_56', _u8 * 28),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevX330Table(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('Baudrate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('DriveMode', _u8 * 1), ('OperatingMode', _u8 * 1),
    ('SecondaryID', _u8 * 1),
    ('ProtocolType', _u8 * 1),
    ('reserve', _u8 * 6),
    ('HomingOffset', _i32 * 1), ('MovingThreshold', _u32 * 1),
    ('reserve', _u8 * 3),
    ('TemperatureLimit', _u8 * 1), ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1), ('PWMLimit', _u16 * 1), ('CurrentLimit', _u16 * 1), ('AccelerationLimit', _u32 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _u32 * 1), ('MinPositionLimit', _u32 * 1),
    ('ExternalPortMode1', _u8 * 1), ('ExternalPortMode2', _u8 * 1), ('ExternalPortMode3', _u8 * 1),
    ('reserve', _u8 * 1),
    ('StartupConfiguration', _u8 * 1),
    ('reserve', _u8 * 2),
    ('Shutdown', _u8 * 1), ('TorqueEnable', _u8 * 1), ('LED', _u8 * 1),
    ('reserve', _u8 * 2),
    ('StatusReturnLevel', _u8 * 1), ('RegisteredInstruction', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
    ('reserve', _u8 * 5),
    ('VelocityIGain', _u16 * 1), ('VelocityPGain', _u16 * 1), ('PositionDGain', _u16 * 1), ('PositionIGain', _u16 * 1), ('PositionPGain', _u16 * 1),
    ('reserve', _u8 * 2),
    ('Feedforward2ndGain', _u16 * 1), ('Feedforward1stGain', _u16 * 1),
    ('reserve', _u8 * 6),
    ('BusWatchdog', _i8 * 1),
    ('reserve', _u8 * 1),
    ('GoalPWM', _i16 * 1), ('GoalCurrent', _i16 * 1), ('GoalVelocity', _i32 * 1),
    ('ProfileAcceleration', _u32 * 1), ('ProfileVelocity', _u32 * 1),
    ('GoalPosition', _i32 * 1),
    ('RealtimeTick', _u16 * 1),
    ('Moving', _u8 * 1), ('MovingStatus', _u8 * 1),
    ('PresentPWM', _i16 * 1), ('PresentCurrent', _i16 * 1), ('PresentVelocity', _i32 * 1), ('PresentPosition', _i32 * 1),
    ('VelocityTrajectory', _u32 * 1), ('PositionTrajectory', _u32 * 1),
    ('PresentInputVoltage', _u16 * 1), ('PresentTemperature', _u8 * 1), ('BackupReady', _u8 * 1),
    ('reserve', _u8 * 4),
    ('ExternalPortData1', _u16 * 1), ('ExternalPortData2', _u16 * 1), ('ExternalPortData3', _u16 * 1),
    ('reserve', _u8 * 10),
    ('IndirectAddress1_28', _u16 * 28), ('IndirectData1_28', _u8 * 28),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevX335Table(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('Baudrate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('DriveMode', _u8 * 1), ('OperatingMode', _u8 * 1),
    ('SecondaryID', _u8 * 1),
    ('ProtocolType', _u8 * 1),
    ('reserve', _u8 * 6),
    ('HomingOffset', _i32 * 1), ('MovingThreshold', _u32 * 1),
    ('reserve', _u8 * 3),
    ('TemperatureLimit', _u8 * 1), ('MaxVoltageLimit', _u16 * 1), ('MinVoltageLimit', _u16 * 1), ('PWMLimit', _u16 * 1), ('CurrentLimit', _u16 * 1), ('AccelerationLimit', _u32 * 1), ('VelocityLimit', _u32 * 1),
    ('MaxPositionLimit', _u32 * 1), ('MinPositionLimit', _u32 * 1),
    ('ExternalPortMode1', _u8 * 1), ('ExternalPortMode2', _u8 * 1), ('ExternalPortMode3', _u8 * 1),
    ('reserve', _u8 * 1),
    ('StartupConfiguration', _u8 * 1),
    ('reserve', _u8 * 1),
    ('PWMSlope', _u8 * 1),
    ('Shutdown', _u8 * 1), ('TorqueEnable', _u8 * 1), ('LED', _u8 * 1),
    ('reserve', _u8 * 2),
    ('StatusReturnLevel', _u8 * 1), ('RegisteredInstruction', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
    ('reserve', _u8 * 5),
    ('VelocityIGain', _u16 * 1), ('VelocityPGain', _u16 * 1), ('PositionDGain', _u16 * 1), ('PositionIGain', _u16 * 1), ('PositionPGain', _u16 * 1),
    ('reserve', _u8 * 2),
    ('Feedforward2ndGain', _u16 * 1), ('Feedforward1stGain', _u16 * 1),
    ('reserve', _u8 * 6),
    ('BusWatchdog', _i8 * 1),
    ('reserve', _u8 * 1),
    ('GoalPWM', _i16 * 1), ('GoalCurrent', _i16 * 1), ('GoalVelocity', _i32 * 1),
    ('ProfileAcceleration', _u32 * 1), ('ProfileVelocity', _u32 * 1),
    ('GoalPosition', _i32 * 1),
    ('RealtimeTick', _u16 * 1),
    ('Moving', _u8 * 1), ('MovingStatus', _u8 * 1),
    ('PresentPWM', _i16 * 1), ('PresentCurrent', _i16 * 1), ('PresentVelocity', _i32 * 1), ('PresentPosition', _i32 * 1),
    ('VelocityTrajectory', _u32 * 1), ('PositionTrajectory', _u32 * 1),
    ('PresentInputVoltage', _u16 * 1), ('PresentTemperature', _u8 * 1), ('BackupReady', _u8 * 1),
    ('reserve', _u8 * 4),
    ('ExternalPortData1', _u16 * 1), ('ExternalPortData2', _u16 * 1), ('ExternalPortData3', _u16 * 1),
    ('reserve', _u8 * 10),
    ('IndirectAddress1_28', _u16 * 28), ('IndirectData1_28', _u8 * 28),
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class _TdevXL320Table(Structure):
  _pack_ = 1
  _fields_ = (
    ('ModelNumber', _u16 * 1), ('FirmwareVersion', _u8 * 1),
    ('ID', _u8 * 1), ('Baudrate', _u8 * 1), ('ReturnDelayTime', _u8 * 1),
    ('CWAngleLimit', _u16 * 1), ('CCWAngleLimit', _u16 * 1),
    ('reserve', _u8 * 1),
    ('ControlMode', _u8 * 1),
    ('TemperatureLimit', _u8 * 1),
    ('MinVoltageLimit', _u8 * 1), ('MaxVoltageLimit', _u8 * 1), ('MaxTorque', _u16 * 1),
    ('StatusReturnLevel', _u8 * 1),
    ('Shutdown', _u8 * 1),
    ('reserve', _u8 * 5),
    ('TorqueEnable', _u8 * 1), ('LED', _u8 * 1),
    ('reserve', _u8 * 1),
    ('DGain', _u8 * 1), ('IGain', _u8 * 1), ('PGain', _u8 * 1),
    ('GoalPosition', _i16 * 1), ('MovingSpeed', _u16 * 1),
    ('reserve', _u8 * 1),
    ('TorqueLimit', _u16 * 1),
    ('PresentPosition', _i16 * 1), ('PresentSpeed', _i16 * 1), ('PresentLoad', _i16 * 1),
    ('reserve', _u8 * 2),
    ('PresentVoltage', _i8 * 1), ('PresentTemperature', _u8 * 1),
    ('RegisteredInstruction', _u8 * 1),
    ('reserve', _u8 * 1),
    ('Moving', _u8 * 1), ('HardwareErrorStatus', _u8 * 1),
    ('Punch', _u16 * 1)
  )

  def __str__(self):
    return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))


class DevType(Enum):
  NONE = 0
  DX = 1
  AX = 2
  RX = 3
  EX = 4
  MX = 5
  XL320 = 6
  PRO = 7
  PROP = 8
  X = 9
  Y = 10


ModelInfo = namedtuple('ModelInfo', ('modelno', 'name', 'devtype'))

ModelInfoList = (
  ModelInfo(0x015E, 'XL-320', DevType.XL320),

  ModelInfo(0x001E, 'MX-28(2.0)', DevType.X),
  ModelInfo(0x0137, 'MX-64(2.0)', DevType.X),
  ModelInfo(0x0141, 'MX-106(2.0)', DevType.X),
  ModelInfo(0x04A6, 'XL330-M077', DevType.X),
  ModelInfo(0x04B0, 'XL330-M288', DevType.X),
  ModelInfo(0x04CE, 'XC330-M181', DevType.X),
  ModelInfo(0x04D8, 'XC330-M288', DevType.X),
  ModelInfo(0x04BA, 'XC330-T181', DevType.X),
  ModelInfo(0x04C4, 'XC330-T288', DevType.X),
  ModelInfo(0x0424, 'XL430-W250', DevType.X),
  ModelInfo(0x0442, '2XL430-W250', DevType.X),
  ModelInfo(0x0488, '2XC430-W250', DevType.X),
  ModelInfo(0x042E, 'XC430-W150', DevType.X),
  ModelInfo(0x0438, 'XC430-W240', DevType.X),
  ModelInfo(0x06A5, 'XM335-T323', DevType.X),
  ModelInfo(0x0406, 'XM430-W210', DevType.X),
  ModelInfo(0x03F2, 'XH430-W210', DevType.X),
  ModelInfo(0x03F3, 'XD430-T210', DevType.X),
  ModelInfo(0x041A, 'XH430-V210', DevType.X),
  ModelInfo(0x03FC, 'XM430-W350', DevType.X),
  ModelInfo(0x03E8, 'XH430-W350', DevType.X),
  ModelInfo(0x03E9, 'XD430-T350', DevType.X),
  ModelInfo(0x0410, 'XH430-V350', DevType.X),
  ModelInfo(0x0500, 'XW430-T200', DevType.X),
  ModelInfo(0x04F6, 'XW430-T333', DevType.X),
  ModelInfo(0x046A, 'XM540-W150', DevType.X),
  ModelInfo(0x0456, 'XH540-W150', DevType.X),
  ModelInfo(0x0457, 'XD540-T150', DevType.X),
  ModelInfo(0x047E, 'XH540-V150', DevType.X),
  ModelInfo(0x0460, 'XM540-W270', DevType.X),
  ModelInfo(0x044C, 'XH540-W270', DevType.X),
  ModelInfo(0x044D, 'XD540-T270', DevType.X),
  ModelInfo(0x0474, 'XH540-V270', DevType.X),
  ModelInfo(0x049C, 'XW540-T140', DevType.X),
  ModelInfo(0x0492, 'XW540-T260', DevType.X),

  ModelInfo(0x8900, 'L42-10-S300-R', DevType.PRO),
  ModelInfo(0x9428, 'L54-30-S400-R', DevType.PRO),
  ModelInfo(0x9408, 'L54-30-S500-R', DevType.PRO),
  ModelInfo(0x9520, 'L54-50-S290-R', DevType.PRO),
  ModelInfo(0x9508, 'L54-50-S500-R', DevType.PRO),
  ModelInfo(0xA918, 'M42-10-S260-R', DevType.PRO),
  ModelInfo(0xB410, 'M54-40-S250-R', DevType.PRO),
  ModelInfo(0xB510, 'M54-60-S250-R', DevType.PRO),
  ModelInfo(0xC800, 'H42-20-S300-R', DevType.PRO),
  ModelInfo(0xD208, 'H54-100-S500-R', DevType.PRO),
  ModelInfo(0xD308, 'H54-200-S500-R', DevType.PRO),
  ModelInfo(0xA919, 'M42-10-S260-RA', DevType.PROP),
  ModelInfo(0xB411, 'M54-40-S250-RA', DevType.PROP),
  ModelInfo(0xB511, 'M54-60-S250-RA', DevType.PROP),
  ModelInfo(0xC801, 'H42-20-S300-RA', DevType.PROP),
  ModelInfo(0xD209, 'H54-100-S500-RA', DevType.PROP),
  ModelInfo(0xD309, 'H54-200-S500-RA', DevType.PROP),

  ModelInfo(0x0834, 'PM42-010-S260-R', DevType.PROP),
  ModelInfo(0x083E, 'PM54-040-S250-R', DevType.PROP),
  ModelInfo(0x0848, 'PM54-060-S250-R', DevType.PROP),
  ModelInfo(0x07D0, 'PH42-020-S300-R', DevType.PROP),
  ModelInfo(0x07DA, 'PH54-100-S500-R', DevType.PROP),
  ModelInfo(0x07E4, 'PH54-200-S500-R', DevType.PROP),

  ModelInfo(0x0FA0, 'YM070-210-M001-RH', DevType.Y),
  ModelInfo(0x0FAA, 'YM070-210-B001-RH', DevType.Y),
  ModelInfo(0x0FB4, 'YM070-210-R051-RH', DevType.Y),
  ModelInfo(0x0FC8, 'YM070-210-A051-RH', DevType.Y),
  ModelInfo(0x0FBE, 'YM070-210-R099-RH', DevType.Y),
  ModelInfo(0x0FD2, 'YM070-210-A099-RH', DevType.Y),

  ModelInfo(0x1018, 'YM080-230-M001-RH', DevType.Y),
  ModelInfo(0x1022, 'YM080-230-B001-RH', DevType.Y),
  ModelInfo(0x102C, 'YM080-230-R051-RH', DevType.Y),
  ModelInfo(0x1040, 'YM080-230-A051-RH', DevType.Y),
  ModelInfo(0x1036, 'YM080-230-R099-RH', DevType.Y),
  ModelInfo(0x104A, 'YM080-230-A099-RH', DevType.Y),
)


class dxl:
  def __del__(self):
    pass

  def __init__(self, dx, id, orgtype=None, orgname=''):
    """
    Init dxl object.

    Parameters
    ----------
    - dx (pyDXL.DXLProtocolV2): Class
    - id (int): Dynamixel ID to be linked
    """
    self._dx = dx
    self._id = id
    self._ctbl = None
    self._devtype = DevType.NONE
    self._modelname = ''

    modelno = self._dx.Read16(self._id, 0)
    if modelno is not None:
      _p = [u for u in ModelInfoList if u.modelno == modelno]
      if _p != []:
        p = _p[0]
        self._modelname = p.name
        self._devtype = p.devtype
        match p.devtype:
          case DevType.PRO:
            self._ctbl = _TdevPROTable()
          case DevType.PROP:
            self._ctbl = _TdevPROPTable()
          case DevType.X:
            if ('XC330' or 'XL330') in self._modelname:
              self._ctbl = _TdevX330Table()
            elif 'XM335' in self._modelname:
              self._ctbl = _TdevX330Table()
            else:
              self._ctbl = _TdevXTable()
          case DevType.Y:
            self._ctbl = _TdevYTable()
        if self._ctbl:
          self.update()
        else:
          self._modelname = ''
          self._ctrl = None
      else:
        if orgtype is not None:
          self._modelname = orgname
          self._ctbl = orgtype()
          if self._ctbl:
            self.update()
          else:
            self._modelname = ''
            self._ctrl = None

  def __enter__(self):
    return self

  def __exit__(self, ex_type, ex_value, trace):
    pass

  @property
  def __str__(self):
    return self._ctbl.__str__()

  @property
  def id(self):
    return self._id

  @property
  def modelname(self):
    return self._modelname

  @property
  def devtype(self):
    return self._devtype

  def __iteminfo(self, name):
    try:
      m = self._ctbl.__class__.__dict__[name]
      if m:
        num = 0
        t_sz = 0
        typ = type(m.__get__(self._ctbl))._type_()
        match typ:
          case _i8():
            t_sz = 1
            num = m.size
          case _u8():
            t_sz = 1
            num = m.size
          case _i16():
            t_sz = 2
            num = m.size >> 1
          case _u16():
            t_sz = 2
            num = m.size >> 1
          case _i32():
            t_sz = 4
            num = m.size >> 2
          case _u32():
            t_sz = 4
            num = m.size >> 2
          case _flt32():
            t_sz = 4
            num = m.size >> 2
          case _:
            num = m.size
        return m, m.offset, typ, t_sz, num
    except:
      return None, None, None, None, None

  def itemaddress(self, name):
    """
    Get its address from the item name.

    Parameters
    ----------
    - name (str): item name
    """
    try:
      return self._ctbl.__class__.__dict__[name].offset
    except:
      return -1

  def itemname(self, addr):
    """
    Get its item name from the address.

    Parameters
    ----------
    - addr (int): item address
    """
    if self._ctbl:
      for m in self._ctbl._fields_:
        if (addr == self.itemaddress(m[0])) and (m[0] != 'reserve'):
          return m[0]

  def get_bulk(self, dats: ()):
    """
    Get data from multiple devices using ReadBulk instruction.
    Returns value from the item name.

    Parameters
    ----------
    - dats (list|tuple): (dxl, "ItemName"), ...
    """
    bulkdat = []
    for dat in dats:
      n, addr, typ, t_sz, num = dat[0].__iteminfo(dat[1])
      if n is not None:
        bulkdat += self._dx.TBulkR(dat[0].id, addr, t_sz * num),
    r = self._dx.BulkRead(bulkdat)
    result = ()
    for dat in dats:
      for _r in r:
        if _r[0] == dat[0].id:
          n, ofs, typ, t_sz, num = dat[0].__iteminfo(dat[1])
          uu = (c_uint8 * (t_sz * num))(*_r[1])
          match typ:
            case _i8(): d = (_i8 * num)()
            case _u8(): d = (_u8 * num)()
            case _i16(): d = (_i16 * num)()
            case _u16(): d = (_u16 * num)()
            case _i32(): d = (_i32 * num)()
            case _u32(): d = (_u32 * num)()
            case _flt32(): d = (_flt32 * num)()
          memmove(byref(d, 0), byref(uu, 0), t_sz * num)

          if num > 2:
            ret = ()
            for v in d:
              ret += v.d,
          else:
            ret = d[0].d
          result += (ret),
          break
      else:
        result += None,
    return result

  def set_bulk(self, dats: ()):
    """
    Set data to multiple devices using WriteBulk instruction.
    Returns API's result.
    However, the actual success or failure of writing to the device is not determined.

    Parameters
    ----------
    - dats (list|tuple): (dxl, "ItemName", Value), ...
    """
    wbulkdat = []
    if dats:
      ids = ()
      for dat in dats:
        ids += dat[0].id,
        n, ofs, typ, t_sz, num = dat[0].__iteminfo(dat[1])
        if n is not None:
          td = ()
          if isinstance(dat[2], (tuple, list)):
            td = dat[2]
          else:
            td = dat[2],
          sz = t_sz * num
          bsz = 0
          wdat = (c_uint8 * (sz + 5))(dat[0].id, ofs & 0xff, (ofs >> 8) & 0xff)
          match typ:
            case _i8():
              dat = (_i8 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_int8(v[1])
                for j in range(t_sz):
                  wdat[i + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _u8():
              dat = (_u8 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_uint8(v[1])
                for j in range(t_sz):
                  wdat[i + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _i16():
              dat = (_i16 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_int16(v[1])
                for j in range(t_sz):
                  wdat[i * 2 + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _u16():
              dat = (_u16 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_uint16(v[1])
                for j in range(t_sz):
                  wdat[i * 2 + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _i32():
              dat = (_i32 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_int32(v[1])
                for j in range(t_sz):
                  wdat[i * 4 + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _u32():
              dat = (_u32 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_uint32(v[1])
                for j in range(t_sz):
                  wdat[i * 4 + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _flt32():
              dat = (_flt32 * num)()
              for i, v in enumerate(zip(dat, td)):
                dat[i].d = c_float(v[1])
                for j in range(t_sz):
                  wdat[i * 4 + 5 + j] = dat[i].u[j]
                  bsz += 1
            case _:
              return None
          wdat[3] = bsz & 0xff
          wdat[4] = (bsz >> 8) & 0xff
          wbulkdat += wdat[0:bsz + 5]

      if len(ids) != len(set(ids)):
        return None

      if len(wbulkdat) > 0:
        return self._dx.TxPacket(self._dx.BROADCASTING_ID, self._dx.INST_BULK_WRITE, wbulkdat, wait=0.005)[1]

  def update(self):
    """
    Receive information on all items on the control table.

    Parameters
    ----------
    """
    sz = sizeof(self._ctbl)
    d = self._dx.Read(self._id, 0, sz)
    if d is not None:
      if len(d) == sz:
        memmove(POINTER(c_uint8)(self._ctbl), d, sz)

  def _get(self, name, upd):
    if self.itemaddress(name) >= 0:
      n, ofs, typ, t_sz, num = self.__iteminfo(name)
      if n is not None:
        sz = t_sz * num
        if upd:
          rb = (c_uint8 * sz)(0)
          rb = self._dx.Read(self._id, ofs, sz)
          if rb is not None:
            memmove(byref(self._ctbl, ofs), rb, sz)
        if num == 1:
          return (n.__get__(self._ctbl))[0].d
        else:
          r = ()
          for i in range(num):
            r += n.__get__(self._ctbl)[i].d,
          return r

  def get(self, name, update=False):
    """
    Get its value from the item name.
    Returns value from the item name.

    Parameters
    ----------
    - name (str): item name
    - update (bool): If true, it communicates with dynamiel to get data
    """
    if name:
      if isinstance(name, (tuple, list)):
        result = ()
        for m in name:
          result += self._get(m, update),
        return result
      else:
        return self._get(name, update)

  def _set(self, name, d):
    if self.itemaddress(name) >= 0:
      n, ofs, typ, t_sz, num = self.__iteminfo(name)
      if n is not None:
        td = ()
        if isinstance(d, (tuple, list)):
          td = d
        else:
          td = d,
        sz = t_sz * num
        match typ:
          case _i8():
            dat = (_i8 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_int8(v[1])
          case _u8():
            dat = (_u8 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_uint8(v[1])
          case _i16():
            dat = (_i16 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_int16(v[1])
          case _u16():
            dat = (_u16 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_uint16(v[1])
          case _i32():
            dat = (_i32 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_int32(v[1])
          case _u32():
            dat = (_u32 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_uint32(v[1])
          case _flt32():
            dat = (_flt32 * num)()
            for i, v in enumerate(zip(dat, td)):
              dat[i].d = c_float(v[1])
          case _:
            return False

        if self._dx.Write(self._id, ofs, dat):
          memmove(byref(self._ctbl, ofs), dat, sz)
          n.__set__(self._ctbl, dat)
          return True
      return False
    return False

  def set(self, *args):
    """
    Write data by specifying items and data.
    Returns decision.

    Parameters
    ----------
    - *args (list/tuple): (item name, data),
    """
    if len(args) == 2:
      return self._set(args[0], args[1])
    elif isinstance(args[0], (tuple, list)):
      result = ()
      for i in args[0]:
        m = i[0]
        d = i[1]
        result += (self._set(m, d),)
      return result
    return False


if __name__ == '__main__':
  from time import sleep
  import socket

  class TDXMIO2(Structure):
    _pack_ = 1
    _fields_ = (
      ('ModelNumber', _u16 * 1), ('ModelInformation', _u32 * 1), ('VersionOfFirmware', _u8 * 1),
      ('ID', _u8 * 1), ('Baudrate', _u8 * 1), ('WriteNVM', _u8 * 1), ('LED', _u8 * 1), ('Terminator', _u8 * 1),
      ('PinConfig', _u8 * 12), ('PWMFrequencty', _u16 * 1), ('PWMDuty', _i16 * 11), ('Capture', _u16 * 4),
      ('OUT', _u16 * 1), ('IN', _u16 * 1), ('ADV', _u16 * 12), ('reserve', _u8 * 18), ('DAV', _u16 * 1), ('reserve', _u8 * 4),
      ('Accelerometer', _flt32 * 3), ('GyroscopeCalibrated', _flt32 * 3), ('MagneticFieldCalibrated', _flt32 * 3), ('LinearAcceleration', _flt32 * 3),
      ('RotationVector_i', _flt32 * 1), ('RotationVector_j', _flt32 * 1), ('RotationVector_k', _flt32 * 1), ('RotationVector_real', _flt32 * 1), ('RotationVector_accuracy', _flt32 * 1),
      ('Gravity', _flt32 * 3), ('Temperature', _u16 * 1),
      ('USER', _u8 * 5)
    )

    def __str__(self):
      return '{}: {{{}}}'.format(self.__class__.__name__, ', '.join(['{}: {}'.format(field[0], getattr(self, field[0])) for field in self._fields_]))

  # Instantiate pyDXL
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
  sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 10000, 3000))
  '''
  sock.connect((socket.gethostbyname('wifiserial.local'), 8000))
  '''
#  with DXLProtocolV2(sock, 57600, timeoutoffset=0.2, protocoltype=2) as dx2:
  with DXLProtocolV2('\\\\.\\COM20', 57600, timeoutoffset=0.2) as dx2:
    # List the responding DXLs
    dxs = []
    print('<Start scan>')
    rets = dx2.Ping2()
    dxs.append(dxl(dx2, 200, orgtype=TDXMIO2, orgname='dxmio2'))
    print(rets)
    for ret in rets:
      dxs += dxl(dx2, ret[0]),
      if dxs[-1].modelname == '':
        dxs.pop(-1)

    # Dump the control table based on address
    for dx in dxs:
      print(f'----------- ID={dx.id}  {dx.modelname}')
      for addr in range(1000):
        n = dx.itemname(addr)
        if n:
          print(f'{n}({addr})={dx.get(n)}, ', end='')
      else:
        print()

    if len(dxs) > 0:
      try:
        input('\n<Please push enter key>')

        # Extract only the servo motors
        svs = []
        for dx in dxs:
          if dx.get('MaxPositionLimit') is not None:
            svs += dx,

        # Set the operating mode to 3 (Position ctrl) and torque enable
        print('>Initalization')
        for dx in svs:
          print(f' id={dx.id} init=', all(dx.set((('TorqueEnable', 0), ('OperatingMode', 3), ('TorqueEnable', 1)))))

        # Get different items from each axis (The same ID can only be used once.)
        print('>Get different items with bulk')
        VariousItems = ('TorqueEnable', 'LED', 'ID', 'ModelNumber', 'PresentCurrent', 'MaxVoltageLimit', 'RealtimeTick', 'uho')
        bulkparam = []
        for i, dx in enumerate(dxs):
          bulkparam += (dxs[i], VariousItems[i % len(VariousItems)]),
        ret = dxs[0].get_bulk(bulkparam)
        for i, dx in enumerate(dxs):
          print(f' id={dx.id} {VariousItems[i]}={ret[i]}')

        # Blink the LED
        print('>Blink the LED')
        for dx in dxs:
          ledname = 'LED' if dx.get('LED') is not None else 'LEDRed'
          if dx.get(ledname) is not None:
            for i in range(20):
              led = dx.get(ledname) ^ (1 if ledname == 'LED' else 255)
              print(f'\r id={dx.id} led={led:3}', end='', flush=True)
              dx.set(ledname, led)
              sleep(0.1)
            else:
              print()

        # Get the position information
        print('>Get information about position')
        maxpos = [x for x in dxs[0].get_bulk([(d, 'MaxPositionLimit') for d in svs]) if x is not None]
        print(' maxposlimit=', maxpos)
        minpos = [x for x in dxs[0].get_bulk([(d, 'MinPositionLimit') for d in svs]) if x is not None]
        print(' minposlimit=', minpos)
        goalpos = [x for x in dxs[0].get_bulk([(d, 'PresentPosition') for d in svs]) if x is not None]
        print(' presentpos=', goalpos)

        increment = [abs((_maxpos - _minpos) // 16) for (_maxpos, _minpos) in zip(maxpos, minpos)]
        print(' increment=', increment)

        # Set the goal position on 1 axis
        print('>Set goal position one axis')
        for i, dx in enumerate(svs):
          for pos in tuple(range(goalpos[i], maxpos[i], increment[i])) + tuple(range(maxpos[i], minpos[i], -increment[i])) + tuple(range(minpos[i], goalpos[i], increment[i])):
            print(f'\r id={dx.id} gpos={pos:7}', end='', flush=True)
            dx.set('GoalPosition', pos)
            sleep(0.3)
          else:
            print()

        # Set the goal position on all axis
        print('>Set goal position all axis')
        for i in range(160):
          for j in range(len(goalpos)):
            goalpos[j] += increment[j]
            if goalpos[j] > maxpos[j]:
              goalpos[j] = maxpos[j]
              increment[j] *= -1
            elif goalpos[j] < minpos[j]:
              goalpos[j] = minpos[j]
              increment[j] *= -1
          svs[0].set_bulk([(_dx, 'GoalPosition', _pos) for (_dx, _pos) in zip(svs, goalpos)])
          print(f"\r gpos={('{:7d} ' * len(goalpos)).format(*goalpos)}", end='', flush=True)
          sleep(0.3)
        else:
          print()

      except KeyboardInterrupt:
        pass

      # Free
      for dx in dxs:
        dx.set('TorqueEnable', 0)
    print()
    del dxs
