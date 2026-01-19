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
 * @file signal_measurement.py 
 * @brief 
 * Signal measurement class for legacy TMSi devices.
 * Based on MATLAB Sampler.m
 */


'''

import ctypes
import numpy as np
import time

from ..legacy_dll_interface import TMSiSDKInterface
from ..legacy_enums import OVERFLOW_VALUE

# Access to global SDK interface
_sdk_interface = None
_sdk_handle = None


def _get_sdk():
    """Get global SDK interface"""
    from ..legacy_device import _sdk_interface as sdk_if, _sdk_handle as sdk_h
    return sdk_if, sdk_h


class SignalMeasurement:
    """
    Signal measurement class for legacy TMSi devices (Porti7, REFA).
    Handles data acquisition and conversion.
    """
    
    def __init__(self, device, name="Legacy Signal Measurement"):
        """
        Initialize signal measurement.
        
        Args:
            device: LegacyDevice instance
            name: Measurement name
        """
        self._device = device
        self._name = name
        self._is_sampling = False
        
        # Get SDK interface
        self._sdk_interface, self._sdk_handle = _get_sdk()
        
        # Sampling configuration
        self._sample_rate = device.get_device_base_sample_rate()
        self._buffer_size = self._sample_rate * device.get_num_channels() * 30
        self._reference_calculation = True
        
        # Channel information
        self._channels = device.get_device_channels()
        self._num_channels = device.get_num_channels()
        
        # Determine which channels can overflow (EXG, BIP, AUX types)
        self._channels_overflow_types = []
        for channel in self._channels:
            # Channel types 1, 2, 3 can overflow
            can_overflow = hasattr(channel, 'unit_id') and channel.unit_id in [1, 2, 3]
            self._channels_overflow_types.append(can_overflow)
        
        # Sample buffer
        self._sample_buffer = None
        self._sample_buffer_size = 0
        self._sample_buffer_size_bytes = 0
    
    def set_sample_rate(self, sample_rate):
        """
        Set sampling rate.
        
        Args:
            sample_rate: Desired sample rate in Hz
        """
        if self._is_sampling:
            raise RuntimeError("Cannot set sample rate while sampling")
        
        self._sample_rate = sample_rate
    
    def set_reference_calculation(self, enabled):
        """
        Enable/disable common average reference calculation.
        
        Args:
            enabled: True to enable, False to disable
        """
        if self._is_sampling:
            raise RuntimeError("Cannot set reference calculation while sampling")
        
        self._reference_calculation = enabled
    
    def start(self):
        """Start data acquisition"""
        if self._is_sampling:
            return
        
        # Configure signal buffer
        actual_sr, actual_bs, success = self._sdk_interface.set_signal_buffer(
            self._sdk_handle,
            self._sample_rate,
            self._buffer_size
        )
        
        if not success:
            raise RuntimeError("Failed to set signal buffer")
        
        self._sample_rate = actual_sr
        self._buffer_size = actual_bs
        
        # Allocate sample buffer
        self._sample_buffer_size = self._buffer_size
        self._sample_buffer_size_bytes = self._sample_buffer_size * 4  # 4 bytes per uint32
        self._sample_buffer = (ctypes.c_uint32 * self._sample_buffer_size)()
        
        # Set reference calculation
        success = self._sdk_interface.set_ref_calculation(
            self._sdk_handle,
            self._reference_calculation
        )
        if not success:
            raise RuntimeError("Failed to set reference calculation")
        
        # Start sampling
        success = self._sdk_interface.start_sampling(self._sdk_handle)
        if not success:
            raise RuntimeError("Failed to start sampling")
        
        self._is_sampling = True
    
    def stop(self):
        """Stop data acquisition"""
        if not self._is_sampling:
            return
        
        # Stop sampling
        success = self._sdk_interface.stop_sampling(self._sdk_handle)
        if not success:
            raise RuntimeError("Failed to stop sampling")
        
        self._is_sampling = False
    
    def get_samples(self, blocking=False):
        """
        Get samples from device.
        
        Args:
            blocking: If True, wait for samples. If False, return immediately.
        
        Returns:
            numpy array of shape (num_samples, num_channels) or None if no data
        """
        if not self._is_sampling:
            return None
        
        # Read samples from device
        start_time = time.time()
        num_bytes = self._sdk_interface.get_samples(
            self._sdk_handle,
            self._sample_buffer,
            self._sample_buffer_size_bytes
        )
        
        # If blocking, keep trying until we get data or timeout
        while blocking and num_bytes == 0:
            if time.time() - start_time > 30:
                raise TimeoutError("Sample timeout after 30 seconds")
            
            num_bytes = self._sdk_interface.get_samples(
                self._sdk_handle,
                self._sample_buffer,
                self._sample_buffer_size_bytes
            )
        
        # Check for errors
        if num_bytes < 0:
            raise RuntimeError("Error reading samples from device")
        
        if num_bytes == 0:
            return None
        
        # Convert samples
        num_samples = num_bytes // (self._num_channels * 4)
        if num_samples == 0:
            return None
        
        # Extract samples from buffer
        raw_samples = np.array(self._sample_buffer[:self._num_channels * num_samples], dtype=np.uint32)
        raw_samples = raw_samples.reshape(num_samples, self._num_channels)
        
        # Convert to physical units
        samples = np.zeros((num_samples, self._num_channels), dtype=np.float64)
        
        for i, channel in enumerate(self._channels):
            channel_data = raw_samples[:, i]
            
            # Check for overflow
            overflow_mask = (channel_data == OVERFLOW_VALUE) & self._channels_overflow_types[i]
            
            # Convert based on channel format
            if channel.get_channel_format() == 0:
                # Unsigned format
                converted = channel_data.astype(np.float64)
            else:
                # Signed format
                converted = channel_data.astype(np.int32).astype(np.float64)
            
            # Apply unit gain
            converted = converted * channel.get_channel_unit_gain()
            
            # Set overflow values to NaN
            converted[overflow_mask] = np.nan
            
            samples[:, i] = converted
        
        return samples
    
    def get_device_sample_rate(self):
        """Get actual sample rate"""
        return self._sample_rate
    
    def get_name(self):
        """Get measurement name"""
        return self._name
    
    def is_sampling(self):
        """Check if currently sampling"""
        return self._is_sampling
