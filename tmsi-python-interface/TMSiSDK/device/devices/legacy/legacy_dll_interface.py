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
 * @file legacy_dll_interface.py 
 * @brief 
 * ctypes wrapper for TMSiSDK.dll interface.
 * Based on MATLAB Library.m, Device.m, and Sampler.m
 */


'''

import ctypes
import os
import sys
from pathlib import Path

from .legacy_structures import (
    FRONTENDINFO,
    SIGNAL_FORMAT,
    SYSTEMTIME,
    TMSiExtFrontendInfo,
    TMSiBatReport,
    TMSiStorageReport,
    TMSiDeviceReport,
    TMSiRecordingConfig,
    TMSiTDFHeader,
    TMSiFileInfo
)
from .legacy_enums import TMSiConnectionType


class TMSiSDKInterface:
    """
    Low-level interface to TMSiSDK.dll using ctypes.
    Provides Python bindings for all DLL functions.
    """
    
    def __init__(self, dll_path=None):
        """
        Initialize the TMSi SDK DLL interface.
        
        Args:
            dll_path: Optional path to TMSiSDK.dll. If None, searches in:
                     1. Same directory as this file
                     2. System PATH
        """
        self.dll = None
        self._load_dll(dll_path)
        self._configure_functions()
    
    def _load_dll(self, dll_path=None):
        """Load the TMSiSDK.dll library"""
        if dll_path is None:
            # Try to find DLL in same directory as this file
            dll_dir = Path(__file__).parent
            dll_path = dll_dir / 'TMSiSDK.dll'
            
            if not dll_path.exists():
                # Try system PATH
                dll_path = 'TMSiSDK.dll'
        
        try:
            self.dll = ctypes.CDLL(str(dll_path))
        except OSError as e:
            raise RuntimeError(
                f"Failed to load TMSiSDK.dll. "
                f"Make sure the DLL is in the same directory as this file, "
                f"or in the system PATH. Error: {e}"
            )
    
    def _configure_functions(self):
        """Configure all DLL function signatures using ctypes"""
        
        # LibraryInit - Initialize the library
        self.dll.LibraryInit.argtypes = [
            ctypes.c_int,  # TMSiConnectionType
            ctypes.POINTER(ctypes.c_int32)  # ErrorCode
        ]
        self.dll.LibraryInit.restype = ctypes.c_void_p  # HANDLE
        
        # LibraryExit - Clean up the library
        self.dll.LibraryExit.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.LibraryExit.restype = ctypes.c_int32
        
        # GetRevision - Get SDK revision string
        self.dll.GetRevision.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.GetRevision.restype = ctypes.c_char_p
        
        # GetDeviceList - Get list of connected devices
        self.dll.GetDeviceList.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.POINTER(ctypes.c_int32)  # NrOfFrontEnds
        ]
        self.dll.GetDeviceList.restype = ctypes.POINTER(ctypes.c_char_p)
        
        # FreeDeviceList - Free device list memory
        self.dll.FreeDeviceList.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_int32,  # NrOfFrontEnds
            ctypes.POINTER(ctypes.c_char_p)  # DeviceList
        ]
        self.dll.FreeDeviceList.restype = None
        
        # Open - Open connection to device
        self.dll.Open.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_char_p  # DeviceLocator
        ]
        self.dll.Open.restype = ctypes.c_uint8  # BOOLEAN
        
        # Close - Close connection to device
        self.dll.Close.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.Close.restype = ctypes.c_uint8  # BOOLEAN
        
        # GetFrontEndInfo - Get device information
        self.dll.GetFrontEndInfo.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.POINTER(FRONTENDINFO)  # FrontEndInfo
        ]
        self.dll.GetFrontEndInfo.restype = ctypes.c_uint8  # BOOLEAN
        
        # GetSignalFormat - Get channel information
        self.dll.GetSignalFormat.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_char_p  # FrontEndName
        ]
        self.dll.GetSignalFormat.restype = ctypes.POINTER(SIGNAL_FORMAT)
        
        # Free - Free memory allocated by DLL
        self.dll.Free.argtypes = [ctypes.c_void_p]  # Memory pointer
        self.dll.Free.restype = ctypes.c_uint8  # BOOLEAN
        
        # SetSignalBuffer - Configure sample rate and buffer size
        self.dll.SetSignalBuffer.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.POINTER(ctypes.c_ulong),  # SampleRate
            ctypes.POINTER(ctypes.c_ulong)  # BufferSize
        ]
        self.dll.SetSignalBuffer.restype = ctypes.c_uint8  # BOOLEAN
        
        # SetRefCalculation - Enable/disable reference calculation
        self.dll.SetRefCalculation.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_int32  # OnOrOff
        ]
        self.dll.SetRefCalculation.restype = ctypes.c_uint8  # BOOLEAN
        
        # Start - Start sampling
        self.dll.Start.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.Start.restype = ctypes.c_uint8  # BOOLEAN
        
        # Stop - Stop sampling
        self.dll.Stop.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.Stop.restype = ctypes.c_uint8  # BOOLEAN
        
        # GetSamples - Read samples from device
        self.dll.GetSamples.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.POINTER(ctypes.c_uint32),  # SampleBuffer
            ctypes.c_ulong  # Size in bytes
        ]
        self.dll.GetSamples.restype = ctypes.c_long  # Number of bytes read
        
        # SetMeasuringMode - Set measurement mode (normal/impedance)
        self.dll.SetMeasuringMode.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_ulong,  # Mode
            ctypes.c_int32  # Value
        ]
        self.dll.SetMeasuringMode.restype = ctypes.c_uint8  # BOOLEAN
        
        # GetErrorCode - Get last error code
        self.dll.GetErrorCode.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.GetErrorCode.restype = ctypes.c_int32
        
        # GetErrorCodeMessage - Get error message for error code
        self.dll.GetErrorCodeMessage.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.c_int32  # ErrorCode
        ]
        self.dll.GetErrorCodeMessage.restype = ctypes.c_char_p
        
        # GetBufferInfo - Get buffer status
        self.dll.GetBufferInfo.argtypes = [
            ctypes.c_void_p,  # HANDLE
            ctypes.POINTER(ctypes.c_ulong),  # Overflow
            ctypes.POINTER(ctypes.c_ulong)  # PercentFull
        ]
        self.dll.GetBufferInfo.restype = ctypes.c_uint8  # BOOLEAN
        
        # ResetDevice - Reset the device
        self.dll.ResetDevice.argtypes = [ctypes.c_void_p]  # HANDLE
        self.dll.ResetDevice.restype = ctypes.c_uint8  # BOOLEAN
    
    # High-level wrapper methods
    
    def library_init(self, connection_type):
        """
        Initialize the TMSi library.
        
        Args:
            connection_type: TMSiConnectionType enum value
        
        Returns:
            tuple: (handle, error_code)
        """
        error_code = ctypes.c_int32(0)
        handle = self.dll.LibraryInit(
            connection_type,
            ctypes.byref(error_code)
        )
        return handle, error_code.value
    
    def library_exit(self, handle):
        """Clean up the library"""
        return self.dll.LibraryExit(handle)
    
    def get_revision(self, handle):
        """Get SDK revision string"""
        result = self.dll.GetRevision(handle)
        return result.decode('utf-8') if result else ""
    
    def get_device_list(self, handle):
        """
        Get list of connected devices.
        
        Returns:
            list of device name strings
        """
        num_devices = ctypes.c_int32(0)
        device_list_ptr = self.dll.GetDeviceList(
            handle,
            ctypes.byref(num_devices)
        )
        
        if num_devices.value == 0 or not device_list_ptr:
            return []
        
        # Extract device names
        devices = []
        for i in range(num_devices.value):
            device_name = device_list_ptr[i].decode('utf-8')
            devices.append(device_name)
        
        # Free the device list
        self.dll.FreeDeviceList(handle, num_devices.value, device_list_ptr)
        
        return devices
    
    def open_device(self, handle, device_name):
        """Open connection to device"""
        device_bytes = device_name.encode('utf-8')
        return bool(self.dll.Open(handle, device_bytes))
    
    def close_device(self, handle):
        """Close connection to device"""
        return bool(self.dll.Close(handle))
    
    def get_frontend_info(self, handle):
        """Get frontend device information"""
        info = FRONTENDINFO()
        success = self.dll.GetFrontEndInfo(handle, ctypes.byref(info))
        if not success:
            return None
        return info
    
    def get_signal_format(self, handle):
        """Get channel signal format information"""
        signal_format_ptr = self.dll.GetSignalFormat(handle, None)
        if not signal_format_ptr:
            return None
        return signal_format_ptr
    
    def free_memory(self, ptr):
        """Free memory allocated by DLL"""
        return bool(self.dll.Free(ptr))
    
    def set_signal_buffer(self, handle, sample_rate, buffer_size):
        """
        Configure sample rate and buffer size.
        
        Args:
            handle: Device handle
            sample_rate: Desired sample rate in Hz (will be adjusted by device)
            buffer_size: Buffer size in samples
        
        Returns:
            tuple: (actual_sample_rate, actual_buffer_size, success)
        """
        sr = ctypes.c_ulong(int(sample_rate * 1000))  # Convert to mHz
        bs = ctypes.c_ulong(buffer_size)
        
        success = self.dll.SetSignalBuffer(
            handle,
            ctypes.byref(sr),
            ctypes.byref(bs)
        )
        
        actual_sr = sr.value / 1000.0  # Convert back to Hz
        actual_bs = bs.value
        
        return actual_sr, actual_bs, bool(success)
    
    def set_ref_calculation(self, handle, enabled):
        """Enable or disable reference calculation"""
        return bool(self.dll.SetRefCalculation(handle, 1 if enabled else 0))
    
    def start_sampling(self, handle):
        """Start data acquisition"""
        return bool(self.dll.Start(handle))
    
    def stop_sampling(self, handle):
        """Stop data acquisition"""
        return bool(self.dll.Stop(handle))
    
    def get_samples(self, handle, buffer, buffer_size_bytes):
        """
        Read samples from device.
        
        Args:
            handle: Device handle
            buffer: ctypes array to store samples
            buffer_size_bytes: Size of buffer in bytes
        
        Returns:
            Number of bytes read (negative on error, 0 if no data)
        """
        return self.dll.GetSamples(handle, buffer, buffer_size_bytes)
    
    def set_measuring_mode(self, handle, mode, value):
        """
        Set measuring mode (e.g., impedance mode).
        
        Args:
            handle: Device handle
            mode: Mode (0=normal, 3=impedance)
            value: Mode-specific value (e.g., impedance limit)
        """
        return bool(self.dll.SetMeasuringMode(handle, mode, value))
    
    def get_error_code(self, handle):
        """Get last error code"""
        return self.dll.GetErrorCode(handle)
    
    def get_error_message(self, handle, error_code):
        """Get error message for error code"""
        result = self.dll.GetErrorCodeMessage(handle, error_code)
        return result.decode('utf-8') if result else ""
    
    def get_buffer_info(self, handle):
        """
        Get buffer status.
        
        Returns:
            tuple: (overflow_count, percent_full)
        """
        overflow = ctypes.c_ulong(0)
        percent_full = ctypes.c_ulong(0)
        
        success = self.dll.GetBufferInfo(
            handle,
            ctypes.byref(overflow),
            ctypes.byref(percent_full)
        )
        
        if success:
            return overflow.value, percent_full.value
        return None, None
    
    def reset_device(self, handle):
        """Reset the device"""
        return bool(self.dll.ResetDevice(handle))
