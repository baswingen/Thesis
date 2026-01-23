"""
EMG Signal Acquisition Module
==============================
Native interface for acquiring EMG signals from TMSi Porti7/REFA devices.

This module provides a clean, Pythonic interface to the TMSi legacy devices,
wrapping the complexity of device management and data acquisition.

Features:
- Single channel and differential pair acquisition
- Real-time signal filtering
- Channel name fixing for encoding issues
- Auto-detection of EMG channels
- Data export and visualization support
"""

import sys
import os
from pathlib import Path
import numpy as np
from typing import Optional, List, Tuple, Callable, Union, Dict
import time
import warnings

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


def fix_channel_name(corrupted_name: str, channel_index: int) -> str:
    """
    Fix corrupted UTF-16LE channel names from TMSi legacy device.
    
    The TMSi library treats uint16 values as bytes, corrupting UTF-16LE decoding.
    
    For Porti7 (38 channels):
    - Channels 0-15: UNI1-UNI16 (unipolar)
    - Channels 16-31: BIP1-BIP16 (bipolar inputs 1-16)
    - Channels 32-35: BIP17-BIP20 (bipolar inputs 17-20)
    - Channel 36: STATUS
    - Channel 37: COUNTER
    
    Args:
        corrupted_name: The corrupted channel name from device
        channel_index: 0-based channel index
        
    Returns:
        Fixed channel name
    """
    try:
        if channel_index < 16:
            # Unipolar channels
            return f"UNI{channel_index + 1}"
        elif channel_index < 36:
            # Bipolar channels (BIP1-BIP20)
            return f"BIP{channel_index - 15}"
        elif channel_index == 36:
            return "STATUS"
        elif channel_index == 37:
            return "COUNTER"
        else:
            return f"CH{channel_index}"
    except:
        return f"CH{channel_index}"


class EMGChannelSelector:
    """
    Helper class for selecting and configuring EMG channels.
    
    Supports multiple selection methods:
    - Single channel by index, name, or port
    - Differential pairs (recommended for EMG)
    - Auto-detection based on signal variance
    """
    
    def __init__(self, channels: List):
        """
        Initialize channel selector.
        
        Args:
            channels: List of TMSi channel objects
        """
        self.channels = channels
        self.channel_names = [ch.get_channel_name() for ch in channels]
        self._name_to_index = {}
        
        # Build name lookup with fixed names
        for i, ch in enumerate(channels):
            try:
                name = ch.get_channel_name()
                # Check if name has encoding issues
                if any(ord(c) > 127 for c in name):
                    name = fix_channel_name(name, i)
                self._name_to_index[name] = i
                self._name_to_index[name.upper()] = i
            except:
                pass
    
    def select_by_index(self, index: int) -> Tuple[int, str]:
        """
        Select channel by index.
        
        Args:
            index: 0-based channel index
            
        Returns:
            Tuple of (index, channel_name)
            
        Raises:
            ValueError: If index out of range
        """
        if not (0 <= index < len(self.channels)):
            raise ValueError(f"Channel index {index} out of range (0-{len(self.channels)-1})")
        
        return index, self.channel_names[index]
    
    def select_by_name(self, name: str) -> Tuple[int, str]:
        """
        Select channel by name.
        
        Args:
            name: Channel name (case insensitive)
            
        Returns:
            Tuple of (index, channel_name)
            
        Raises:
            ValueError: If name not found
        """
        if name in self._name_to_index:
            idx = self._name_to_index[name]
            return idx, self.channel_names[idx]
        elif name.upper() in self._name_to_index:
            idx = self._name_to_index[name.upper()]
            return idx, self.channel_names[idx]
        else:
            raise ValueError(f"Channel name '{name}' not found")
    
    def select_by_port(self, port: int) -> Tuple[int, str]:
        """
        Select channel by physical port number.
        
        Args:
            port: Physical port number (device-specific)
            
        Returns:
            Tuple of (index, channel_name)
            
        Raises:
            ValueError: If port not found
        """
        # Try matching on port field
        for i, ch in enumerate(self.channels):
            try:
                if hasattr(ch, "get_channel_port") and int(ch.get_channel_port()) == int(port):
                    return i, self.channel_names[i]
            except:
                continue
        
        # Try matching on name pattern (e.g., BIP17 for port 17)
        port_str = str(port)
        target_name = f"BIP{port_str}"
        
        if target_name in self._name_to_index:
            idx = self._name_to_index[target_name]
            return idx, self.channel_names[idx]
        
        # Try pattern matching
        for i, ch in enumerate(self.channels):
            try:
                ch_name = ch.get_channel_name().upper()
                if f"BIP{port_str}" in ch_name or f"IN{port_str}" in ch_name:
                    return i, self.channel_names[i]
            except:
                continue
        
        raise ValueError(f"Port {port} not found")
    
    def select_differential_pair(self, 
                                 pos_channel: Union[int, str], 
                                 neg_channel: Union[int, str],
                                 channels_are_1_based: bool = True) -> Tuple[int, int, str]:
        """
        Select a differential channel pair.
        
        Args:
            pos_channel: Positive channel (index if int, name if str)
            neg_channel: Negative channel (index if int, name if str)
            channels_are_1_based: If True, treat integer values as 1-based channel numbers
            
        Returns:
            Tuple of (pos_index, neg_index, description)
            
        Raises:
            ValueError: If channels invalid
        """
        # Convert to indices
        if isinstance(pos_channel, int):
            if channels_are_1_based:
                pos_idx = pos_channel - 1
            else:
                pos_idx = pos_channel
            if not (0 <= pos_idx < len(self.channels)):
                raise ValueError(f"Positive channel index {pos_idx} out of range")
        else:
            pos_idx, _ = self.select_by_name(pos_channel)
        
        if isinstance(neg_channel, int):
            if channels_are_1_based:
                neg_idx = neg_channel - 1
            else:
                neg_idx = neg_channel
            if not (0 <= neg_idx < len(self.channels)):
                raise ValueError(f"Negative channel index {neg_idx} out of range")
        else:
            neg_idx, _ = self.select_by_name(neg_channel)
        
        pos_name = self.channel_names[pos_idx]
        neg_name = self.channel_names[neg_idx]
        
        if channels_are_1_based:
            desc = f"Diff {pos_name} - {neg_name} (ch{pos_idx+1} - ch{neg_idx+1})"
        else:
            desc = f"Diff {pos_name} - {neg_name} (idx{pos_idx} - idx{neg_idx})"
        
        return pos_idx, neg_idx, desc
    
    def auto_detect_emg_channel(self, samples: np.ndarray, top_n: int = 8) -> Tuple[int, str, float]:
        """
        Auto-detect most likely EMG channel based on signal variance.
        
        Args:
            samples: Sample array of shape (num_samples, num_channels)
            top_n: Number of top candidates to display
            
        Returns:
            Tuple of (index, channel_name, std_value)
            
        Raises:
            ValueError: If no valid channels found
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            per_ch_std = np.nanstd(samples, axis=0)
        
        candidates = []
        for ci, stdv in enumerate(per_ch_std):
            try:
                unit = self.channels[ci].get_channel_unit_name()
            except Exception:
                unit = ""
            
            stdv = float(stdv)
            if not np.isfinite(stdv):
                continue
            
            # Only consider voltage channels (uV/mV/V)
            if unit.lower().endswith("v"):
                candidates.append((stdv, ci))
        
        if not candidates:
            raise ValueError("No valid voltage channels found for auto-detection")
        
        candidates.sort(key=lambda x: x[0], reverse=True)
        
        # Display top candidates
        print(f"\n[INFO] Top {min(top_n, len(candidates))} EMG candidates (by variance):")
        for rank, (stdv, ci) in enumerate(candidates[:top_n], start=1):
            try:
                nm = self.channel_names[ci]
                un = self.channels[ci].get_channel_unit_name()
            except:
                nm, un = "?", "?"
            print(f"  #{rank}: idx={ci:2d}  name='{nm:15s}'  unit='{un:6s}'  std={stdv:.3f}")
        
        # Return best candidate
        best_std, best_idx = candidates[0]
        return best_idx, self.channel_names[best_idx], best_std


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
        self.channel_selector = None
        
        # EMG channel configuration
        self._emg_mode = None  # 'single', 'differential', or None
        self._emg_channel_index = None
        self._emg_pos_index = None
        self._emg_neg_index = None
        self._emg_description = None
        
        # Data callback (for real-time processing)
        self._data_callback = None
    
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
            
            # Fix channel names for encoding issues
            self._fix_channel_names()
            
            # Initialize channel selector
            self.channel_selector = EMGChannelSelector(self.channels)
            
            self._is_connected = True
            
            return self.get_device_info()
            
        except Exception as e:
            raise RuntimeError(f"Failed to connect to device: {e}") from e
    
    def _fix_channel_names(self):
        """Fix corrupted channel names due to encoding issues."""
        if not self.channels:
            return
        
        channels_fixed = 0
        for i, ch in enumerate(self.channels):
            try:
                ch_name = ch.get_channel_name()
                # Check if name has encoding issues (non-ASCII characters)
                if any(ord(c) > 127 for c in ch_name):
                    ch_name_fixed = fix_channel_name(ch_name, i)
                    # Override the channel's internal name
                    ch._alt_name = ch_name_fixed
                    ch._def_name = ch_name_fixed
                    channels_fixed += 1
            except:
                pass
        
        if channels_fixed > 0:
            print(f"[INFO] Fixed {channels_fixed} channel names with encoding issues")
    
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
    
    def configure_emg_single_channel(self, 
                                    channel: Union[int, str] = None,
                                    auto_detect: bool = False) -> Tuple[int, str]:
        """
        Configure single EMG channel.
        
        Args:
            channel: Channel to use (index or name). If None, must enable auto_detect
            auto_detect: Auto-detect EMG channel based on signal variance
        
        Returns:
            Tuple of (channel_index, channel_name)
        
        Raises:
            RuntimeError: If not connected
            ValueError: If channel not found or auto_detect fails
        """
        if not self._is_connected:
            raise RuntimeError("Not connected to a device")
        
        if channel is None and not auto_detect:
            raise ValueError("Must specify channel or enable auto_detect")
        
        if auto_detect:
            print("[INFO] Auto-detection will occur on first data samples")
            self._emg_mode = 'single'
            self._emg_channel_index = None
            self._emg_description = "Auto-detect (pending)"
            return None, "Auto-detect (pending)"
        
        # Select channel
        if isinstance(channel, int):
            idx, name = self.channel_selector.select_by_index(channel)
        else:
            idx, name = self.channel_selector.select_by_name(channel)
        
        self._emg_mode = 'single'
        self._emg_channel_index = idx
        self._emg_description = f"{name} (idx{idx})"
        
        print(f"[OK] EMG single channel configured: {self._emg_description}")
        return idx, name
    
    def configure_emg_differential_pair(self, 
                                       pos_channel: Union[int, str],
                                       neg_channel: Union[int, str],
                                       channels_are_1_based: bool = True) -> Tuple[int, int, str]:
        """
        Configure differential EMG channel pair (recommended for EMG).
        
        Args:
            pos_channel: Positive channel (index or name)
            neg_channel: Negative channel (index or name)
            channels_are_1_based: If True, treat integers as 1-based channel numbers
        
        Returns:
            Tuple of (pos_index, neg_index, description)
        
        Raises:
            RuntimeError: If not connected
            ValueError: If channels invalid
        """
        if not self._is_connected:
            raise RuntimeError("Not connected to a device")
        
        pos_idx, neg_idx, desc = self.channel_selector.select_differential_pair(
            pos_channel, neg_channel, channels_are_1_based
        )
        
        self._emg_mode = 'differential'
        self._emg_pos_index = pos_idx
        self._emg_neg_index = neg_idx
        self._emg_description = desc
        
        print(f"[OK] EMG differential pair configured: {desc}")
        return pos_idx, neg_idx, desc
    
    def configure_emg_by_port(self, port: int) -> Tuple[int, str]:
        """
        Configure EMG channel by physical port number.
        
        Args:
            port: Physical port number on device
        
        Returns:
            Tuple of (channel_index, channel_name)
        
        Raises:
            RuntimeError: If not connected
            ValueError: If port not found
        """
        if not self._is_connected:
            raise RuntimeError("Not connected to a device")
        
        idx, name = self.channel_selector.select_by_port(port)
        
        self._emg_mode = 'single'
        self._emg_channel_index = idx
        self._emg_description = f"{name} (port{port}, idx{idx})"
        
        print(f"[OK] EMG channel configured: {self._emg_description}")
        return idx, name
    
    def configure_acquisition(self, 
                            sample_rate: Optional[int] = None,
                            reference_calculation: bool = False):
        """
        Configure signal acquisition parameters.
        
        Args:
            sample_rate: Desired sample rate in Hz (None = use device default)
            reference_calculation: Enable common average reference calculation
                                 (For EMG differential pairs, keep this False)
        
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
    
    def set_data_callback(self, callback: Optional[Callable[[np.ndarray, np.ndarray], None]]):
        """
        Set callback function for real-time data processing.
        
        The callback will be called with (raw_samples, emg_signal) for each data chunk.
        
        Args:
            callback: Function with signature callback(raw_samples, emg_signal)
                     raw_samples: shape (n_samples, n_channels)
                     emg_signal: shape (n_samples,) - extracted EMG signal
                     Set to None to disable callback
        """
        self._data_callback = callback
    
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
                
                # Extract EMG and call callback if configured
                if self._data_callback is not None:
                    emg_signal = self.extract_emg_signal(samples)
                    if emg_signal is not None:
                        self._data_callback(samples, emg_signal)
            
            return samples
            
        except Exception as e:
            raise RuntimeError(f"Failed to get samples: {e}") from e
    
    def extract_emg_signal(self, samples: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Extract EMG signal from samples based on configured mode.
        
        Args:
            samples: Sample array (num_samples, num_channels). If None, uses last acquired samples
        
        Returns:
            1D array of EMG values, or None if no EMG configuration or data
        """
        if samples is None:
            data = self.get_data()
            if data is None:
                return None
            samples = data
        
        if samples is None or len(samples) == 0:
            return None
        
        # Auto-detect mode - select channel on first call
        if self._emg_mode == 'single' and self._emg_channel_index is None:
            try:
                idx, name, std = self.channel_selector.auto_detect_emg_channel(samples)
                self._emg_channel_index = idx
                self._emg_description = f"{name} (idx{idx}, auto-detected, std={std:.3f})"
                print(f"\n[OK] Auto-detected EMG channel: {self._emg_description}\n")
            except Exception as e:
                print(f"[!] Auto-detection failed: {e}")
                return None
        
        # Extract based on mode
        if self._emg_mode == 'single' and self._emg_channel_index is not None:
            if self._emg_channel_index < samples.shape[1]:
                return samples[:, self._emg_channel_index]
        
        elif self._emg_mode == 'differential':
            if (self._emg_pos_index < samples.shape[1] and 
                self._emg_neg_index < samples.shape[1]):
                return samples[:, self._emg_pos_index] - samples[:, self._emg_neg_index]
        
        return None
    
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
        
        info = {
            'name': self.device_name,
            'serial_number': self.serial_number,
            'num_channels': self.num_channels,
            'sample_rate': self.sample_rate,
            'connection_type': self.connection_type,
            'is_acquiring': self._is_acquiring,
            'emg_mode': self._emg_mode,
            'emg_description': self._emg_description
        }
        
        if self._emg_mode == 'single':
            info['emg_channel_index'] = self._emg_channel_index
        elif self._emg_mode == 'differential':
            info['emg_pos_index'] = self._emg_pos_index
            info['emg_neg_index'] = self._emg_neg_index
        
        return info
    
    def get_emg_info(self) -> dict:
        """
        Get EMG configuration information.
        
        Returns:
            Dictionary with EMG configuration
        """
        return {
            'mode': self._emg_mode,
            'description': self._emg_description,
            'channel_index': self._emg_channel_index,
            'pos_index': self._emg_pos_index,
            'neg_index': self._emg_neg_index
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
            emg_str = f", emg={self._emg_description}" if self._emg_mode else ""
            return f"EMGDevice(name={self.device_name}, channels={self.num_channels}, rate={self.sample_rate}Hz{emg_str})"
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
