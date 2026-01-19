"""
EMG Signal Acquisition Module
==============================
Native interface for acquiring EMG signals from TMSi Porti7/REFA devices.

This module provides a clean, Pythonic interface to the TMSi legacy devices,
wrapping the complexity of device management and data acquisition.
"""

import sys
import os
from pathlib import Path
import numpy as np
from typing import Optional, List, Tuple, Callable
import time

# Add TMSi Python Interface to path
_project_root = Path(__file__).parent.parent
_tmsi_interface_path = _project_root / 'tmsi-python-interface'
if _tmsi_interface_path.exists():
    sys.path.insert(0, str(_tmsi_interface_path))

try:
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
except ImportError as e:
    raise ImportError(
        "TMSi Python Interface not found. "
        "Make sure tmsi-python-interface is in the project root."
    ) from e


class EMGDevice:
    """
    Main interface for EMG signal acquisition from TMSi devices.
    
    This class provides a high-level interface for:
    - Device discovery and connection
    - Signal acquisition configuration
    - Real-time data streaming
    - Data export and processing
    
    Example:
        >>> emg = EMGDevice()
        >>> emg.connect()
        >>> emg.start_acquisition(duration=10)
        >>> data = emg.get_data()
        >>> emg.disconnect()
    """
    
    def __init__(self, connection_type: str = 'usb'):
        """
        Initialize EMG device interface.
        
        Args:
            connection_type: Connection type ('usb', 'bluetooth', 'network', 'wifi')
        """
        self.connection_type = connection_type
        self._sdk = None
        self._device = None
        self._measurement = None
        self._is_connected = False
        self._is_acquiring = False
        self._sample_buffer = []
        
        # Device info
        self.device_name = None
        self.serial_number = None
        self.num_channels = 0
        self.sample_rate = 0
        self.channels = []
    
    def discover_devices(self) -> List[str]:
        """
        Discover available TMSi devices.
        
        Returns:
            List of device names
        
        Raises:
            RuntimeError: If discovery fails
        """
        try:
            if self._sdk is None:
                self._sdk = TMSiSDK()
            
            # Map connection type to enum
            interface_map = {
                'usb': DeviceInterfaceType.usb,
                'bluetooth': DeviceInterfaceType.bluetooth,
                'network': DeviceInterfaceType.network,
                'wifi': DeviceInterfaceType.wifi,
                'optical': DeviceInterfaceType.optical
            }
            
            interface = interface_map.get(self.connection_type.lower(), DeviceInterfaceType.usb)
            
            devices, _ = self._sdk.discover(DeviceType.legacy, dr_interface=interface)
            
            device_names = []
            for dev in devices:
                try:
                    device_names.append(dev.get_device_name())
                except:
                    device_names.append("Unknown Device")
            
            return device_names
            
        except Exception as e:
            raise RuntimeError(f"Failed to discover devices: {e}") from e
    
    def connect(self, device_index: int = 0) -> dict:
        """
        Connect to a TMSi device.
        
        Args:
            device_index: Index of device to connect to (default: 0 = first device)
        
        Returns:
            Dictionary with device information
        
        Raises:
            RuntimeError: If connection fails
        """
        if self._is_connected:
            print("Already connected to a device")
            return self.get_device_info()
        
        try:
            # Initialize SDK if needed
            if self._sdk is None:
                self._sdk = TMSiSDK()
            
            # Discover devices
            interface_map = {
                'usb': DeviceInterfaceType.usb,
                'bluetooth': DeviceInterfaceType.bluetooth,
                'network': DeviceInterfaceType.network,
                'wifi': DeviceInterfaceType.wifi,
                'optical': DeviceInterfaceType.optical
            }
            
            interface = interface_map.get(self.connection_type.lower(), DeviceInterfaceType.usb)
            devices, _ = self._sdk.discover(DeviceType.legacy, dr_interface=interface)
            
            if not devices or len(devices) == 0:
                raise RuntimeError("No devices found")
            
            if device_index >= len(devices):
                raise RuntimeError(f"Device index {device_index} out of range (found {len(devices)} devices)")
            
            # Connect to device
            self._device = devices[device_index]
            self._device.open()
            
            # Get device info
            self.device_name = self._device.get_device_name()
            self.serial_number = self._device.get_device_serial_number()
            self.num_channels = self._device.get_num_channels()
            self.sample_rate = self._device.get_device_base_sample_rate()
            self.channels = self._device.get_device_channels()
            
            self._is_connected = True
            
            return self.get_device_info()
            
        except Exception as e:
            raise RuntimeError(f"Failed to connect to device: {e}") from e
    
    def disconnect(self):
        """
        Disconnect from device and cleanup.
        
        Raises:
            RuntimeError: If disconnection fails
        """
        if self._is_acquiring:
            self.stop_acquisition()
        
        if self._is_connected and self._device is not None:
            try:
                self._device.close()
                self._is_connected = False
                print("Device disconnected")
            except Exception as e:
                raise RuntimeError(f"Failed to disconnect: {e}") from e
        
        # Cleanup SDK
        try:
            LegacyDevice.cleanup()
        except:
            pass
    
    def configure_acquisition(self, 
                            sample_rate: Optional[int] = None,
                            reference_calculation: bool = True):
        """
        Configure signal acquisition parameters.
        
        Args:
            sample_rate: Desired sample rate in Hz (None = use device default)
            reference_calculation: Enable common average reference calculation
        
        Raises:
            RuntimeError: If not connected or configuration fails
        """
        if not self._is_connected:
            raise RuntimeError("Not connected to a device")
        
        if self._is_acquiring:
            raise RuntimeError("Cannot configure while acquiring")
        
        try:
            # Create measurement if not exists
            if self._measurement is None:
                self._measurement = MeasurementType.LEGACY_SIGNAL(self._device)
            
            # Configure
            if sample_rate is not None:
                self._measurement.set_sample_rate(sample_rate)
            
            self._measurement.set_reference_calculation(reference_calculation)
            
            # Update actual sample rate
            self.sample_rate = self._measurement.get_device_sample_rate()
            
        except Exception as e:
            raise RuntimeError(f"Failed to configure acquisition: {e}") from e
    
    def start_acquisition(self, duration: Optional[float] = None):
        """
        Start signal acquisition.
        
        Args:
            duration: Acquisition duration in seconds (None = continuous)
        
        Raises:
            RuntimeError: If not connected or acquisition fails
        """
        if not self._is_connected:
            raise RuntimeError("Not connected to a device")
        
        if self._is_acquiring:
            print("Acquisition already running")
            return
        
        try:
            # Create and configure measurement if needed
            if self._measurement is None:
                self.configure_acquisition()
            
            # Clear sample buffer
            self._sample_buffer = []
            
            # Start measurement
            self._measurement.start()
            self._is_acquiring = True
            
            print(f"Acquisition started (Sample rate: {self.sample_rate} Hz)")
            
            # If duration specified, acquire for that time
            if duration is not None:
                self._acquire_for_duration(duration)
            
        except Exception as e:
            self._is_acquiring = False
            raise RuntimeError(f"Failed to start acquisition: {e}") from e
    
    def stop_acquisition(self):
        """
        Stop signal acquisition.
        
        Raises:
            RuntimeError: If stop fails
        """
        if not self._is_acquiring:
            return
        
        try:
            if self._measurement is not None:
                self._measurement.stop()
            self._is_acquiring = False
            print("Acquisition stopped")
        except Exception as e:
            raise RuntimeError(f"Failed to stop acquisition: {e}") from e
    
    def get_samples(self, blocking: bool = False) -> Optional[np.ndarray]:
        """
        Get latest samples from device.
        
        Args:
            blocking: Wait for samples if none available
        
        Returns:
            NumPy array of shape (num_samples, num_channels) or None if no data
        
        Raises:
            RuntimeError: If not acquiring
        """
        if not self._is_acquiring:
            raise RuntimeError("Not currently acquiring data")
        
        try:
            samples = self._measurement.get_samples(blocking=blocking)
            
            # Store in buffer
            if samples is not None and len(samples) > 0:
                self._sample_buffer.append(samples)
            
            return samples
            
        except Exception as e:
            raise RuntimeError(f"Failed to get samples: {e}") from e
    
    def _acquire_for_duration(self, duration: float):
        """
        Internal method to acquire data for specified duration.
        
        Args:
            duration: Duration in seconds
        """
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            try:
                samples = self.get_samples(blocking=False)
                time.sleep(0.01)  # Small delay to avoid busy-waiting
            except KeyboardInterrupt:
                print("\nAcquisition interrupted by user")
                break
        
        self.stop_acquisition()
    
    def get_data(self) -> Optional[np.ndarray]:
        """
        Get all acquired data as a single array.
        
        Returns:
            NumPy array of shape (total_samples, num_channels) or None if no data
        """
        if not self._sample_buffer:
            return None
        
        return np.vstack(self._sample_buffer)
    
    def clear_buffer(self):
        """Clear the internal sample buffer."""
        self._sample_buffer = []
    
    def get_device_info(self) -> dict:
        """
        Get device information.
        
        Returns:
            Dictionary with device information
        """
        if not self._is_connected:
            return {}
        
        return {
            'name': self.device_name,
            'serial_number': self.serial_number,
            'num_channels': self.num_channels,
            'sample_rate': self.sample_rate,
            'connection_type': self.connection_type,
            'is_acquiring': self._is_acquiring
        }
    
    def get_channel_names(self) -> List[str]:
        """
        Get list of channel names.
        
        Returns:
            List of channel names
        """
        if not self.channels:
            return []
        
        return [ch.get_channel_name() for ch in self.channels]
    
    def get_channel_info(self) -> List[dict]:
        """
        Get detailed channel information.
        
        Returns:
            List of dictionaries with channel information
        """
        if not self.channels:
            return []
        
        channel_info = []
        for i, ch in enumerate(self.channels):
            channel_info.append({
                'index': i,
                'name': ch.get_channel_name(),
                'unit': ch.get_channel_unit_name(),
                'type': str(ch.get_channel_type()),
                'enabled': True
            })
        
        return channel_info
    
    def export_data(self, filename: str, data: Optional[np.ndarray] = None):
        """
        Export acquired data to file.
        
        Args:
            filename: Output filename (.npy, .csv, or .txt)
            data: Data to export (None = use internal buffer)
        
        Raises:
            ValueError: If no data available or invalid format
        """
        if data is None:
            data = self.get_data()
        
        if data is None:
            raise ValueError("No data to export")
        
        file_ext = Path(filename).suffix.lower()
        
        if file_ext == '.npy':
            np.save(filename, data)
            print(f"Data exported to {filename} (NumPy format)")
            
        elif file_ext == '.csv':
            np.savetxt(filename, data, delimiter=',', 
                      header=','.join(self.get_channel_names()),
                      comments='')
            print(f"Data exported to {filename} (CSV format)")
            
        elif file_ext == '.txt':
            np.savetxt(filename, data, delimiter='\t',
                      header='\t'.join(self.get_channel_names()),
                      comments='')
            print(f"Data exported to {filename} (TXT format)")
            
        else:
            raise ValueError(f"Unsupported file format: {file_ext}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit with automatic cleanup."""
        self.disconnect()
    
    def __repr__(self):
        """String representation."""
        if self._is_connected:
            return f"EMGDevice(name={self.device_name}, channels={self.num_channels}, rate={self.sample_rate}Hz)"
        return "EMGDevice(disconnected)"


# Convenience function for quick acquisition
def acquire_emg_data(duration: float = 10.0, 
                    connection_type: str = 'usb',
                    sample_rate: Optional[int] = None,
                    export_to: Optional[str] = None) -> Tuple[np.ndarray, dict]:
    """
    Quick convenience function to acquire EMG data.
    
    Args:
        duration: Acquisition duration in seconds
        connection_type: Connection type ('usb', 'bluetooth', etc.)
        sample_rate: Sample rate in Hz (None = device default)
        export_to: Optional filename to export data
    
    Returns:
        Tuple of (data array, device_info dict)
    
    Example:
        >>> data, info = acquire_emg_data(duration=5.0)
        >>> print(f"Acquired {data.shape[0]} samples from {info['name']}")
    """
    with EMGDevice(connection_type=connection_type) as emg:
        device_info = emg.connect()
        print(f"Connected to {device_info['name']}")
        
        if sample_rate is not None:
            emg.configure_acquisition(sample_rate=sample_rate)
        
        emg.start_acquisition(duration=duration)
        
        data = emg.get_data()
        
        if export_to is not None and data is not None:
            emg.export_data(export_to, data)
        
        return data, device_info
