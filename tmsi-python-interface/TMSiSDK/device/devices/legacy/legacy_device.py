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
 * @file legacy_device.py 
 * @brief 
 * Legacy TMSi device object for Porti7/REFA devices.
 * Based on MATLAB Device.m
 */


'''

import numpy as np
from ...tmsi_device import TMSiDevice
from ...tmsi_channel import TMSiChannel
from .legacy_dll_interface import TMSiSDKInterface
from .legacy_enums import TMSiConnectionType, get_unit_name

# Import enums locally to avoid circular import
def _get_device_state_enum():
    from ...tmsi_device_enums import DeviceState
    return DeviceState

def _get_channel_type_enum():
    from ...tmsi_device_enums import ChannelType
    return ChannelType

# Global SDK interface instance
_sdk_interface = None
_sdk_handle = None
_connection_type = None


class LegacyDevice(TMSiDevice):
    """
    Legacy TMSi device class for Porti7, REFA, and REFA Extended devices.
    Uses TMSiSDK.dll for communication.
    """
    
    def __init__(self, device_name, connection_type='usb'):
        """
        Initialize a legacy TMSi device.
        
        Args:
            device_name: Device identifier from discovery
            connection_type: Connection type ('usb', 'bluetooth', etc.)
        """
        self._device_name = device_name
        self._connection_type = connection_type
        DeviceState = _get_device_state_enum()
        self._state = DeviceState.disconnected
        self._channels = []
        self._sample_rate = 0
        self._num_channels = 0
        self._serial_number = 0
        self._hardware_version = 0
        self._software_version = 0
        self._base_sample_rate = 0
        
        # Reference to global SDK handle
        global _sdk_handle
        self._sdk_handle = _sdk_handle
    
    @staticmethod
    def discover(connection_type='usb', num_retries=3):
        """
        Discover available legacy TMSi devices.
        
        Args:
            connection_type: Connection type string or DeviceInterfaceType enum
            num_retries: Number of discovery attempts (not used, for API compatibility)
        
        Returns:
            List of LegacyDevice objects
        """
        global _sdk_interface, _sdk_handle, _connection_type
        
        # Convert DeviceInterfaceType enum to string if needed
        if hasattr(connection_type, 'name'):
            # It's an enum, get the name
            connection_type = connection_type.name
        
        # Initialize SDK if not already done
        if _sdk_interface is None:
            _sdk_interface = TMSiSDKInterface()
            
            # Map connection type string to enum
            conn_type_map = {
                'usb': TMSiConnectionType.usb,
                'bluetooth': TMSiConnectionType.bluetooth,
                'fiber': TMSiConnectionType.fiber,
                'optical': TMSiConnectionType.fiber,  # optical maps to fiber
                'wifi': TMSiConnectionType.wifi,
                'network': TMSiConnectionType.network
            }
            
            conn_enum = conn_type_map.get(connection_type.lower(), TMSiConnectionType.usb)
            _connection_type = connection_type
            
            # Initialize library
            handle, error_code = _sdk_interface.library_init(conn_enum)
            if error_code != 0:
                raise RuntimeError(f"Failed to initialize TMSi SDK: error code {error_code}")
            
            _sdk_handle = handle
        
        # Get device list
        device_names = _sdk_interface.get_device_list(_sdk_handle)
        
        # Create device objects
        devices = []
        for device_name in device_names:
            device = LegacyDevice(device_name, connection_type)
            devices.append(device)
        
        return devices
    
    @staticmethod
    def cleanup():
        """Clean up the SDK when done"""
        global _sdk_interface, _sdk_handle, _connection_type
        
        if _sdk_interface is not None and _sdk_handle is not None:
            _sdk_interface.library_exit(_sdk_handle)
            _sdk_interface = None
            _sdk_handle = None
            _connection_type = None
    
    def open(self):
        """Open connection to the device"""
        global _sdk_interface
        DeviceState = _get_device_state_enum()
        
        if self._state != DeviceState.disconnected:
            return
        
        # Open device connection
        success = _sdk_interface.open_device(self._sdk_handle, self._device_name)
        if not success:
            raise RuntimeError(f"Failed to open device {self._device_name}")
        
        # Get device information
        self._retrieve_device_info()
        
        self._state = DeviceState.connected
    
    def close(self):
        """Close connection to the device"""
        global _sdk_interface
        DeviceState = _get_device_state_enum()
        
        if self._state == DeviceState.disconnected:
            return
        
        # Close device
        success = _sdk_interface.close_device(self._sdk_handle)
        if not success:
            raise RuntimeError("Failed to close device")
        
        self._state = DeviceState.disconnected
    
    def _retrieve_device_info(self):
        """Retrieve device information from frontend"""
        global _sdk_interface
        
        # Get frontend info
        frontend_info = _sdk_interface.get_frontend_info(self._sdk_handle)
        if frontend_info is None:
            raise RuntimeError("Failed to get frontend info")
        
        self._num_channels = frontend_info.NrOfChannels
        self._serial_number = frontend_info.Serial
        self._hardware_version = frontend_info.HwVersion
        self._software_version = frontend_info.SwVersion
        self._base_sample_rate = frontend_info.BaseSf
        self._sample_rate = frontend_info.BaseSf
        
        # Get channel information
        self._retrieve_channel_info()
    
    def _retrieve_channel_info(self):
        """Retrieve channel information"""
        global _sdk_interface

        def _decode_utf16le_u16(u16_arr):
            """
            Decode a UTF-16LE string stored as an array of uint16 code units.

            The TMSi SDK exposes fields like SIGNAL_FORMAT.Name / PortName as
            uint16 arrays (UTF-16LE). We must convert each uint16 into 2 bytes
            (little-endian) before decoding. Treating the uint16 values as bytes
            corrupts the string (e.g. 'BIP17' becomes non-ASCII glyphs).
            """
            try:
                # Build little-endian byte stream from uint16 code units
                b = b"".join(int(v).to_bytes(2, byteorder="little", signed=False) for v in u16_arr)
                # Trim at first UTF-16 NUL terminator (0x0000) if present
                nul = b.find(b"\x00\x00")
                if nul != -1:
                    b = b[:nul]
                return b.decode("utf-16le", errors="ignore").strip("\x00").strip()
            except Exception:
                return ""
        
        # Get signal format
        signal_format_ptr = _sdk_interface.get_signal_format(self._sdk_handle)
        if signal_format_ptr is None:
            raise RuntimeError("Failed to get signal format")
        
        # Parse channel information
        self._channels = []
        num_channels = signal_format_ptr.contents.Elements
        
        for i in range(num_channels):
            channel_ptr = signal_format_ptr[i]
            
            # Extract channel + port names (UTF-16LE stored as uint16 array)
            channel_name = _decode_utf16le_u16(channel_ptr.Name)
            port_name = _decode_utf16le_u16(channel_ptr.PortName)
            
            # Get unit name
            unit_name = get_unit_name(channel_ptr.UnitId, channel_ptr.UnitExponent)
            
            # Determine channel type based on channel index and info
            ChannelType = _get_channel_type_enum()
            if channel_ptr.Type in [1, 2, 3]:
                channel_type = ChannelType.UNI if channel_ptr.Type == 1 else ChannelType.BIP
            else:
                channel_type = ChannelType.AUX
            
            # Create TMSiChannel object
            channel = TMSiChannel()
            
            # Set channel properties using private attributes
            channel._alt_name = channel_name
            channel._def_name = channel_name
            channel._index = i
            channel._type = channel_type
            channel._unit_name = unit_name
            channel._format = channel_ptr.Format
            channel._enabled = True
            
            # Store unit gain and offset for sample conversion
            channel._unit_gain = float(channel_ptr.UnitGain)
            channel._unit_offset = float(channel_ptr.UnitOffSet)
            
            # Add getter methods for unit gain and offset
            # IMPORTANT: bind defaults to avoid late-binding closure bugs.
            channel.get_channel_unit_gain = (lambda ch=channel: ch._unit_gain)
            channel.get_channel_unit_offset = (lambda ch=channel: ch._unit_offset)

            # Store port mapping info (useful for matching physical input ports)
            channel.port = int(channel_ptr.Port)
            channel.port_name = port_name
            channel.get_channel_port = (lambda ch=channel: getattr(ch, "port", -1))
            channel.get_channel_port_name = (lambda ch=channel: getattr(ch, "port_name", ""))
            
            # Store additional info for reference
            channel.unit_id = channel_ptr.UnitId
            channel.unit_exponent = channel_ptr.UnitExponent
            channel.legacy_type_id = int(channel_ptr.Type)
            channel.can_overflow = bool(channel.legacy_type_id in [1, 2, 3])
            
            self._channels.append(channel)
        
        # Free memory allocated by DLL
        _sdk_interface.free_memory(signal_format_ptr)
    
    # Device property getters
    
    def get_device_name(self):
        """Get device name"""
        return self._device_name
    
    def get_device_serial_number(self):
        """Get device serial number"""
        return self._serial_number
    
    def get_num_channels(self):
        """Get number of channels"""
        return self._num_channels
    
    def get_device_channels(self):
        """Get list of channels"""
        return self._channels
    
    def get_device_sample_rate(self):
        """Get current sample rate"""
        return self._sample_rate
    
    def get_device_base_sample_rate(self):
        """Get base/maximum sample rate"""
        return self._base_sample_rate
    
    def get_device_state(self):
        """Get device state"""
        return self._state
    
    def get_device_hardware_version(self):
        """Get hardware version"""
        return self._hardware_version
    
    def get_device_software_version(self):
        """Get software version"""
        return self._software_version
    
    # Stub implementations for API compatibility
    
    def set_device_sample_rate(self, sample_rate):
        """Set sample rate (handled by measurement)"""
        self._sample_rate = sample_rate
    
    def get_device_active_channels(self):
        """Get active channels"""
        return self._channels
    
    def get_device_active_impedance_channels(self):
        """Get active impedance channels (not applicable for legacy)"""
        return []
    
    def set_device_sampling_config(self, *args, **kwargs):
        """Set sampling configuration (not applicable for legacy)"""
        raise NotImplementedError('method not available for legacy devices')
    
    def apply_mask(self, *args, **kwargs):
        """Apply mask (not applicable for legacy)"""
        raise NotImplementedError('method not available for legacy devices')
    
    def download_file_from_device(self, *args, **kwargs):
        """Download file (not applicable for legacy)"""
        raise NotImplementedError('method not available for legacy devices')
    
    def export_configuration(self, *args, **kwargs):
        """Export configuration (not applicable for legacy)"""
        raise NotImplementedError('method not available for legacy devices')
    
    def import_configuration(self, *args, **kwargs):
        """Import configuration (not applicable for legacy)"""
        raise NotImplementedError('method not available for legacy devices')
