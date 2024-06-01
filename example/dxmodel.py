#!/usr/bin/python3

# DXL Series Model Information

from enum import Enum
from dataclasses import dataclass

class DXL_DevType(Enum):
  devtNONE  = 0
  devtDX    = 1
  devtAX    = 2
  devtRX    = 3
  devtEX    = 4
  devtMX    = 5
  devtXL320 = 6
  devtPRO   = 7
  devtPROP  = 8
  devtX     = 9
  devtY     = 10

@dataclass
class DXL_ModelInfo:
  modelno: int
  name: str
  devtype: DXL_DevType

ModelInfoList = (
  DXL_ModelInfo( 0x0071, 'DX-113',            DXL_DevType.devtDX ),
  DXL_ModelInfo( 0x0074, 'DX-116',            DXL_DevType.devtDX ),
  DXL_ModelInfo( 0x0075, 'DX-117',            DXL_DevType.devtDX ),

  DXL_ModelInfo( 0x002C, 'AX-12W',            DXL_DevType.devtAX ),
  DXL_ModelInfo( 0x000C, 'AX-12',             DXL_DevType.devtAX ),
  DXL_ModelInfo( 0x0012, 'AX-18',             DXL_DevType.devtAX ),

  DXL_ModelInfo( 0x000A, 'RX-10',             DXL_DevType.devtRX ),
  DXL_ModelInfo( 0x0018, 'RX-24F',            DXL_DevType.devtRX ),
  DXL_ModelInfo( 0x001C, 'RX-28',             DXL_DevType.devtRX ),
  DXL_ModelInfo( 0x0040, 'RX-64',             DXL_DevType.devtRX ),
  DXL_ModelInfo( 0x006B, 'EX-106+',           DXL_DevType.devtEX ),

  DXL_ModelInfo( 0x0068, 'MX-12W',            DXL_DevType.devtMX ),
  DXL_ModelInfo( 0x001D, 'MX-28',             DXL_DevType.devtMX ),
  DXL_ModelInfo( 0x0136, 'MX-64',             DXL_DevType.devtMX ),
  DXL_ModelInfo( 0x0140, 'MX-106',            DXL_DevType.devtMX ),

  DXL_ModelInfo( 0x015E, 'XL-320',            DXL_DevType.devtXL320 ),
  DXL_ModelInfo( 0x001E, 'MX-28(2.0)',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0137, 'MX-64(2.0)',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0141, 'MX-106(2.0)',       DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04A6, 'XL330-M077',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04B0, 'XL330-M288',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04CE, 'XC330-M181',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04D8, 'XC330-M288',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04BA, 'XC330-T181',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04C4, 'XC330-T288',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0424, 'XL430-W250',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0442, '2XL430-W250',       DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0488, '2XC430-W250',       DXL_DevType.devtX ),
  DXL_ModelInfo( 0x042E, 'XC430-W150',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0438, 'XC430-W240',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0406, 'XM430-W210',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x03F2, 'XH430-W210',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x03F3, 'XD430-T210',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x041A, 'XH430-V210',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x03FC, 'XM430-W350',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x03E8, 'XH430-W350',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x03E9, 'XD430-T350',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0410, 'XH430-V350',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0500, 'XW430-T200',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x04F6, 'XW430-T333',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x046A, 'XM540-W150',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0456, 'XH540-W150',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0457, 'XD540-T150',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x047E, 'XH540-V150',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0460, 'XM540-W270',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x044C, 'XH540-W270',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x044D, 'XD540-T270',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0474, 'XH540-V270',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x049C, 'XW540-T140',        DXL_DevType.devtX ),
  DXL_ModelInfo( 0x0492, 'XW540-T260',        DXL_DevType.devtX ),

  DXL_ModelInfo( 0x8900, 'L42-10-S300-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0x9428, 'L54-30-S400-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0x9408, 'L54-30-S500-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0x9520, 'L54-50-S290-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0x9508, 'L54-50-S500-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xA918, 'M42-10-S260-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xB410, 'M54-40-S250-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xB510, 'M54-60-S250-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xC800, 'H42-20-S300-R',     DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xD208, 'H54-100-S500-R',    DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xD308, 'H54-200-S500-R',    DXL_DevType.devtPRO ),
  DXL_ModelInfo( 0xA919, 'M42-10-S260-RA',    DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0xB411, 'M54-40-S250-RA',    DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0xB511, 'M54-60-S250-RA',    DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0xC801, 'H42-20-S300-RA',    DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0xD209, 'H54-100-S500-RA',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0xD309, 'H54-200-S500-RA',   DXL_DevType.devtPROP ),

  DXL_ModelInfo( 0x0834, 'PM42-010-S260-R',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0x083E, 'PM54-040-S250-R',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0x0848, 'PM54-060-S250-R',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0x07D0, 'PH42-020-S300-R',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0x07DA, 'PH54-100-S500-R',   DXL_DevType.devtPROP ),
  DXL_ModelInfo( 0x07E4, 'PH54-200-S500-R',   DXL_DevType.devtPROP ),

  DXL_ModelInfo( 0x0FA0, 'YM070-210-M001-RH', DXL_DevType.devtY ),
  DXL_ModelInfo( 0x0FB4, 'YM070-210-R051-RH', DXL_DevType.devtY ),
  DXL_ModelInfo( 0x0FBE, 'YM070-210-R099-RH', DXL_DevType.devtY ),
  DXL_ModelInfo( 0x0FA0, 'YM080-230-M001-RH', DXL_DevType.devtY ),
  DXL_ModelInfo( 0x0FA0, 'YM080-230-R051-RH', DXL_DevType.devtY ),
  DXL_ModelInfo( 0x0FA0, 'YM080-230-R099-RH', DXL_DevType.devtY ),
)

def get_modelinfo_by_modelno(modelno):
  try:
    return ModelInfoList[[m.modelno for m in ModelInfoList].index(modelno)]
  except:
    pass

@dataclass
class DXL_ItemAddress:
  baudrate: int
  return_delay_time: int
  status_return_level: int
  drive_mode: int
  operating_mode: int
  max_position_limit: int
  min_position_limit: int
  torque_enable: int
  led: int
  goal_pwm: int
  goal_current: int
  goal_velocity: int
  goal_position: int

def get_address_by_modelno(modelno):
  d = get_modelinfo_by_modelno(modelno)
  if d != None:
    match d.devtype:
      case DXL_DevType.devtDX | DXL_DevType.devtAX | DXL_DevType.devtRX:
        return DXL_ItemAddress(4,5,16,-1,-1,8,6,24,25,-1,-1,32,30)
      case DXL_DevType.devtEX:
        return DXL_ItemAddress(4,5,16,10,-1,8,6,24,25,-1,-1,32,30)
      case DXL_DevType.devtMX:
        return DXL_ItemAddress(4,5,16,10,-1,8,6,24,25,-1,71,32,30)
      case DXL_DevType.devtXL320:
        return DXL_ItemAddress(4,5,17,-1,11,8,6,24,25,-1,-1,32,30)
      case DXL_DevType.devtPRO:
        return DXL_ItemAddress(8,9,891,-1,11,36,40,562,563,-1,604,600,596)
      case DXL_DevType.devtPROP:
        return DXL_ItemAddress(8,9,516,10,11,48,52,512,513,548,550,552,564)
      case DXL_DevType.devtX:
        return DXL_ItemAddress(8,9,68,10,11,48,52,64,65,100,102,104,116)
      case DXL_DevType.devtY:
        return DXL_ItemAddress(12,13,15,32,33,76,84,512,513,524,526,528,532)
  return DXL_ItemAddress(-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1)
