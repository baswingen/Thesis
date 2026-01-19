'''
(c) 2023 Twente Medical Systems International B.V., Oldenzaal The Netherlands

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

#######  #     #   #####   #
   #     ##   ##  #        
   #     # # # #  #        #
   #     #  #  #   #####   #
   #     #     #        #  #
   #     #     #        #  #
   #     #     #  #####    #

/**
 * @file legacy_structures.py 
 * @brief 
 * C structure definitions for TMSiSDK.dll interface.
 * Based on TMSiHeader64.m from MATLAB interface.
 */


'''

import ctypes


class SYSTEMTIME(ctypes.Structure):
    """Windows SYSTEMTIME structure"""
    _fields_ = [
        ('wYear', ctypes.c_uint16),
        ('wMonth', ctypes.c_uint16),
        ('wDayOfWeek', ctypes.c_uint16),
        ('wDay', ctypes.c_uint16),
        ('wHour', ctypes.c_uint16),
        ('wMinute', ctypes.c_uint16),
        ('wSecond', ctypes.c_uint16),
        ('wMilliseconds', ctypes.c_uint16)
    ]


class SIGNAL_FORMAT(ctypes.Structure):
    """Channel signal format structure from TMSi SDK"""
    _fields_ = [
        ('Size', ctypes.c_ulong),
        ('Elements', ctypes.c_ulong),
        ('Type', ctypes.c_ulong),
        ('SubType', ctypes.c_ulong),
        ('Format', ctypes.c_ulong),
        ('Bytes', ctypes.c_ulong),
        ('UnitGain', ctypes.c_float),
        ('UnitOffSet', ctypes.c_float),
        ('UnitId', ctypes.c_ulong),
        ('UnitExponent', ctypes.c_long),
        ('Name', ctypes.c_uint16 * 40),
        ('Port', ctypes.c_ulong),
        ('PortName', ctypes.c_uint16 * 40),
        ('SerialNumber', ctypes.c_ulong)
    ]


class FRONTENDINFO(ctypes.Structure):
    """Frontend device information structure from TMSi SDK"""
    _fields_ = [
        ('NrOfChannels', ctypes.c_uint16),
        ('SampleRateSetting', ctypes.c_uint16),
        ('Mode', ctypes.c_uint16),
        ('maxRS232', ctypes.c_uint16),
        ('Serial', ctypes.c_ulong),
        ('NrExg', ctypes.c_uint16),
        ('NrAux', ctypes.c_uint16),
        ('HwVersion', ctypes.c_uint16),
        ('SwVersion', ctypes.c_uint16),
        ('RecBufSize', ctypes.c_uint16),
        ('SendBufSize', ctypes.c_uint16),
        ('NrOfSwChannels', ctypes.c_uint16),
        ('BaseSf', ctypes.c_uint16),
        ('Power', ctypes.c_uint16),
        ('Check', ctypes.c_uint16)
    ]


class FeatureData(ctypes.Structure):
    """Device feature data structure"""
    _fields_ = [
        ('Id', ctypes.c_ulong),
        ('Info', ctypes.c_ulong)
    ]


class TMSiFileInfo(ctypes.Structure):
    """File information structure for card files"""
    _fields_ = [
        ('FileID', ctypes.c_uint16),
        ('StartRecTime', SYSTEMTIME),
        ('StopRecTime', SYSTEMTIME)
    ]


class TMSiTDFHeader(ctypes.Structure):
    """TDF file header structure"""
    _fields_ = [
        ('NumberOfSamp', ctypes.c_uint32),
        ('StartRecTime', SYSTEMTIME),
        ('EndRecTime', SYSTEMTIME),
        ('FrontEndSN', ctypes.c_uint32),
        ('FrontEndAdpSN', ctypes.c_uint32),
        ('FrontEndHWVer', ctypes.c_uint16),
        ('FrontEndSWVer', ctypes.c_uint16),
        ('FrontEndAdpHWVer', ctypes.c_uint16),
        ('FrontEndAdpSWVer', ctypes.c_uint16),
        ('RecordingSampleRate', ctypes.c_uint16),
        ('PatientID', ctypes.c_int8 * 128),
        ('UserString1', ctypes.c_int8 * 64)
    ]


class TMSiRecordingConfig(ctypes.Structure):
    """Recording configuration structure"""
    _fields_ = [
        ('StorageType', ctypes.c_uint16),
        ('RecordingSampleRate', ctypes.c_uint16),
        ('NumberOfChan', ctypes.c_uint16),
        ('StartControl', ctypes.c_uint32),
        ('EndControl', ctypes.c_uint32),
        ('CardStatus', ctypes.c_uint32),
        ('MeasureFileName', ctypes.c_int8 * 32),
        ('AlarmTimeStart', SYSTEMTIME),
        ('AlarmTimeStop', SYSTEMTIME),
        ('AlarmTimeInterval', SYSTEMTIME),
        ('AlarmTimeCount', ctypes.c_uint32),
        ('FrontEndSN', ctypes.c_uint32),
        ('FrontEndAdpSN', ctypes.c_uint32),
        ('RecordCondition', ctypes.c_uint32),
        ('RFInterfStartTime', SYSTEMTIME),
        ('RFInterfStopTime', SYSTEMTIME),
        ('RFInterfInterval', SYSTEMTIME),
        ('RFInterfCount', ctypes.c_uint32),
        ('PatientID', ctypes.c_int8 * 128),
        ('UserString1', ctypes.c_int8 * 64)
    ]


class TMSiBatReport(ctypes.Structure):
    """Battery report structure"""
    _fields_ = [
        ('Temp', ctypes.c_int16),
        ('Voltage', ctypes.c_int16),
        ('Current', ctypes.c_int16),
        ('AccumCurrent', ctypes.c_int16),
        ('AvailableCapacityInPercent', ctypes.c_int16),
        ('DoNotUse1', ctypes.c_uint16),
        ('DoNotUse2', ctypes.c_uint16),
        ('DoNotUse3', ctypes.c_uint16),
        ('DoNotUse4', ctypes.c_uint16)
    ]


class TMSiStorageReport(ctypes.Structure):
    """Storage report structure"""
    _fields_ = [
        ('StructSize', ctypes.c_uint32),
        ('TotalSize', ctypes.c_uint32),
        ('UsedSpace', ctypes.c_uint32),
        ('SDCardCID', ctypes.c_uint32 * 4),
        ('DoNotUse1', ctypes.c_uint16),
        ('DoNotUse2', ctypes.c_uint16),
        ('DoNotUse3', ctypes.c_uint16),
        ('DoNotUse4', ctypes.c_uint16)
    ]


class TMSiDeviceReport(ctypes.Structure):
    """Device report structure"""
    _fields_ = [
        ('AdapterSN', ctypes.c_uint32),
        ('AdapterStatus', ctypes.c_uint32),
        ('AdapterCycles', ctypes.c_uint32),
        ('MobitaSN', ctypes.c_uint32),
        ('MobitaStatus', ctypes.c_uint32),
        ('MobitaCycles', ctypes.c_uint32),
        ('DoNotUse1', ctypes.c_uint16),
        ('DoNotUse2', ctypes.c_uint16),
        ('DoNotUse3', ctypes.c_uint16),
        ('DoNotUse4', ctypes.c_uint16)
    ]


class TMSiExtFrontendInfo(ctypes.Structure):
    """Extended frontend information structure"""
    _fields_ = [
        ('CurrentSamplerate', ctypes.c_uint16),
        ('CurrentInterface', ctypes.c_uint16),
        ('CurrentBlockType', ctypes.c_uint16),
        ('DoNotUse1', ctypes.c_uint16),
        ('DoNotUse2', ctypes.c_uint16),
        ('DoNotUse3', ctypes.c_uint16),
        ('DoNotUse4', ctypes.c_uint16)
    ]
