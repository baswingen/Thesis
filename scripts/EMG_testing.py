"""
TMSi Porti7/REFA Signal Acquisition
====================================
Signal acquisition script for legacy TMSi devices (Porti7, REFA, REFA Extended).
Uses the TMSi Python Interface with legacy device support.

Prerequisites:
- TMSi device connected via USB (or Bluetooth paired)
- TMSi drivers installed (Windows 10/11 64-bit)
- TMSiSDK.dll in system PATH or in tmsi-python-interface/TMSiSDK/device/devices/legacy/
"""

import sys
import os
import time
import numpy as np

# Add TMSi Python Interface to path
tmsi_interface_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tmsi-python-interface')
if os.path.exists(tmsi_interface_path):
    sys.path.insert(0, tmsi_interface_path)

# Import TMSi SDK
try:
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
except ImportError as e:
    print("="*60)
    print("ERROR: TMSi Python Interface not found!")
    print("="*60)
    print("\nThe TMSi Python Interface needs to be available.")
    print(f"\nImport error: {e}")
    print("\nMake sure tmsi-python-interface is in the project root.")
    print("="*60)
    exit(1)

# -----------------------
# User Settings
# -----------------------
TEST_DURATION = 10  # seconds
INTERFACE_TYPE = DeviceInterfaceType.usb  # or DeviceInterfaceType.bluetooth
SAMPLE_RATE = None  # Use device default (None) or set specific rate (e.g., 2048)
REFERENCE_CALCULATION = True  # Enable common average reference

# -----------------------
# Main Script
# -----------------------
def main():
    print("="*60)
    print("TMSi Porti7/REFA Signal Acquisition")
    print("="*60)
    
    # Initialize SDK
    print("\n1. Initializing TMSi SDK...")
    try:
        sdk = TMSiSDK()
    except Exception as e:
        print(f"ERROR: Failed to initialize SDK: {e}")
        return
    
    # Discover legacy devices
    print(f"\n2. Scanning for TMSi legacy devices (Porti7/REFA)...")
    try:
        devices, dongles = sdk.discover(DeviceType.legacy, dr_interface=INTERFACE_TYPE)
        
        if not devices or len(devices) == 0:
            print("ERROR: No TMSi legacy devices found!")
            print("\nCheck:")
            print("  - Device is powered on")
            print("  - USB cable is connected (or Bluetooth paired)")
            print("  - TMSi drivers are installed")
            print("  - TMSiSDK.dll is available in system PATH or legacy module directory")
            return
        
        print(f"[OK] Found {len(devices)} device(s)")
        for i, dev in enumerate(devices):
            try:
                dev_name = dev.get_device_name()
                print(f"  [{i}] {dev_name}")
            except Exception as e:
                print(f"  [{i}] Device (error getting name: {e})")
    
    except Exception as e:
        print(f"ERROR during device discovery: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Connect to first device
    print("\n3. Connecting to first device...")
    try:
        device = devices[0]
        device.open()
        print(f"[OK] Connected to device")
        
        # Get device info
        try:
            print(f"   Device name: {device.get_device_name()}")
            print(f"   Serial number: {device.get_device_serial_number()}")
            print(f"   Number of channels: {device.get_num_channels()}")
            print(f"   Base sample rate: {device.get_device_base_sample_rate()} Hz")
            print(f"   Hardware version: {device.get_device_hardware_version()}")
            print(f"   Software version: {device.get_device_software_version()}")
            
            # Display channel information
            channels = device.get_device_channels()
            print(f"\n   Channels ({len(channels)}):")
            for i, ch in enumerate(channels[:10]):  # Show first 10 channels
                print(f"     [{i}] {ch.get_channel_name():20s} - {ch.get_channel_unit_name():10s}")
            if len(channels) > 10:
                print(f"     ... and {len(channels) - 10} more channels")
            
        except Exception as e:
            print(f"   (Device info error: {e})")
    
    except Exception as e:
        print(f"ERROR: Failed to open device: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Create measurement
    print(f"\n4. Creating signal measurement...")
    try:
        measurement = MeasurementType.LEGACY_SIGNAL(device)
        
        # Configure measurement
        if SAMPLE_RATE is not None:
            measurement.set_sample_rate(SAMPLE_RATE)
        measurement.set_reference_calculation(REFERENCE_CALCULATION)
        
        print(f"[OK] Measurement created")
        print(f"   Sample rate: {measurement.get_device_sample_rate()} Hz")
        print(f"   Reference calculation: {'Enabled' if REFERENCE_CALCULATION else 'Disabled'}")
    
    except Exception as e:
        print(f"ERROR: Failed to create measurement: {e}")
        import traceback
        traceback.print_exc()
        device.close()
        return
    
    # Start acquisition
    print(f"\n5. Starting data acquisition for {TEST_DURATION} seconds...")
    print("   (Press Ctrl+C to stop early)\n")
    
    try:
        # Start sampling
        measurement.start()
        print("[OK] Streaming started\n")
        
        start_time = time.time()
        sample_count = 0
        read_count = 0
        
        while (time.time() - start_time) < TEST_DURATION:
            try:
                # Read samples from measurement
                samples = measurement.get_samples(blocking=False)
                
                if samples is not None and len(samples) > 0:
                    read_count += 1
                    n_samples = samples.shape[0]
                    sample_count += n_samples
                    
                    # Print sample info
                    elapsed = time.time() - start_time
                    print(f"[{elapsed:6.2f}s] Read #{read_count}: Got {n_samples} samples")
                    print(f"            Shape: {samples.shape}")
                    
                    # Print first sample values (first 5 channels)
                    num_channels_to_show = min(5, samples.shape[1])
                    first_sample = samples[0, :num_channels_to_show]
                    print(f"            First sample (first {num_channels_to_show} channels): {first_sample}")
                    
                    # Calculate basic stats
                    sample_mean = np.nanmean(samples)
                    sample_std = np.nanstd(samples)
                    sample_min = np.nanmin(samples)
                    sample_max = np.nanmax(samples)
                    print(f"            Mean: {sample_mean:.6f}, Std: {sample_std:.6f}")
                    print(f"            Range: [{sample_min:.6f}, {sample_max:.6f}]")
                    print()
                
                time.sleep(0.1)  # Small delay to avoid busy-waiting
                
            except KeyboardInterrupt:
                print("\n[!] Interrupted by user")
                break
            except Exception as e:
                print(f"\n[!] Error reading samples: {e}")
                import traceback
                traceback.print_exc()
                break
        
        elapsed = time.time() - start_time
        print(f"\n[OK] Acquisition complete!")
        print(f"   Duration: {elapsed:.2f} seconds")
        print(f"   Total reads: {read_count}")
        print(f"   Total samples: {sample_count}")
        if sample_count > 0:
            avg_rate = sample_count / elapsed
            print(f"   Average rate: {avg_rate:.1f} samples/sec")
            if measurement.get_device_sample_rate() > 0:
                expected_rate = measurement.get_device_sample_rate()
                print(f"   Expected rate: {expected_rate:.1f} Hz")
                efficiency = (avg_rate / expected_rate) * 100
                print(f"   Efficiency: {efficiency:.1f}%")
    
    except Exception as e:
        print(f"\nERROR during acquisition: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        print("\n6. Shutting down...")
        try:
            if 'measurement' in locals():
                measurement.stop()
                print("[OK] Stopped measurement")
        except Exception as e:
            print(f"[!] Error stopping measurement: {e}")
        
        try:
            device.close()
            print("[OK] Device closed")
        except Exception as e:
            print(f"[!] Error closing device: {e}")
        
        try:
            # Clean up legacy device SDK
            LegacyDevice.cleanup()
            print("[OK] SDK cleaned up")
        except Exception as e:
            print(f"[!] Error cleaning up SDK: {e}")
    
    print("\n" + "="*60)
    print("Test complete!")
    print("="*60)

if __name__ == "__main__":
    main()
