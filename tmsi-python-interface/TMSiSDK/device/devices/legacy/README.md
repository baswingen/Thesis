# Legacy TMSi Device Support (Porti7, REFA)

This module provides Python support for legacy TMSi devices (Porti7, REFA, REFA Extended) using the TMSiSDK.dll interface.

## Overview

The legacy device module wraps the TMSiSDK.dll using Python ctypes, providing a similar interface to the modern SAGA and APEX device support.

## Architecture

```
legacy/
├── __init__.py                    # Module initialization
├── legacy_dll_interface.py        # Low-level DLL wrapper (ctypes)
├── legacy_structures.py           # C structure definitions
├── legacy_enums.py                # Enumerations and constants
├── legacy_device.py               # LegacyDevice class
├── measurements/
│   ├── __init__.py
│   └── signal_measurement.py      # Signal acquisition
└── README.md                      # This file
```

## Requirements

- **TMSiSDK.dll** must be available:
  - In the system PATH, OR
  - In this directory (`tmsi-python-interface/TMSiSDK/device/devices/legacy/`)
  
- **Windows OS** (DLL is Windows-only)

- **TMSi drivers** installed

## Usage

### Basic Example

```python
from TMSiSDK.tmsi_sdk import TMSiSDK
from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType

# Initialize SDK
sdk = TMSiSDK()

# Discover legacy devices (Porti7/REFA)
devices, _ = sdk.discover(DeviceType.legacy, dr_interface=DeviceInterfaceType.usb)

# Connect to first device
device = devices[0]
device.open()

# Get device info
print(f"Device: {device.get_device_name()}")
print(f"Serial: {device.get_device_serial_number()}")
print(f"Channels: {device.get_num_channels()}")
print(f"Sample rate: {device.get_device_base_sample_rate()} Hz")

# Create and start measurement
measurement = MeasurementType.LEGACY_SIGNAL(device)
measurement.set_sample_rate(2048)  # Optional: set specific rate
measurement.set_reference_calculation(True)  # Enable common average reference
measurement.start()

# Sample data
for i in range(100):
    samples = measurement.get_samples(blocking=False)
    if samples is not None:
        print(f"Got {samples.shape[0]} samples with {samples.shape[1]} channels")
        # samples is numpy array: (num_samples, num_channels)

# Cleanup
measurement.stop()
device.close()
```

### Connection Types

Supported connection types:
- `DeviceInterfaceType.usb` - USB connection (default)
- `DeviceInterfaceType.bluetooth` - Bluetooth connection
- `DeviceInterfaceType.network` - Network connection
- `DeviceInterfaceType.wifi` - WiFi connection
- `DeviceInterfaceType.optical` - Optical/Fiber connection

## Features

### Implemented
- Device discovery
- Device connection/disconnection
- Channel information retrieval
- Signal acquisition
- Sample rate configuration
- Reference calculation (common average)
- Unit conversion (physical units)
- Overflow detection

### Not Implemented
- Impedance measurement (depends on device)
- Card file recording/download
- Configuration import/export

## Data Format

Samples are returned as NumPy arrays with shape `(num_samples, num_channels)`:
- Values are in physical units (e.g., µV for EEG/EMG channels)
- Overflow/disconnected channels are marked as `NaN`
- Sample rate dependent on device and configuration

## Differences from MATLAB Interface

The Python implementation follows the same basic workflow as the MATLAB interface but with Pythonic improvements:

1. **Object-oriented API** instead of function-based
2. **NumPy arrays** for sample data
3. **Automatic unit conversion** applied to samples
4. **Integrated with TMSi Python Interface** (same API as SAGA/APEX)

## Troubleshooting

### "Failed to load TMSiSDK.dll"
- Ensure TMSiSDK.dll is in the system PATH or in this directory
- Check that you're running on Windows (64-bit)
- Verify TMSi drivers are installed

### "No TMSi legacy devices found"
- Check device is powered on and connected
- Verify USB/Bluetooth connection
- Try different connection type (USB vs Bluetooth)
- Check Windows Device Manager for device

### Sample rate issues
- The device may adjust the requested sample rate to a supported value
- Check `measurement.get_device_sample_rate()` for actual rate
- Supported rates depend on the specific device model

## References

Based on TMSi MATLAB Interface rev-1:
- `Library.m` → `legacy_dll_interface.py` + `legacy_device.py`
- `Device.m` → `legacy_device.py`
- `Sampler.m` → `signal_measurement.py`
- `TMSiHeader64.m` → `legacy_structures.py`

## License

Copyright (c) 2023 Twente Medical Systems International B.V.

Licensed under the Apache License, Version 2.0
