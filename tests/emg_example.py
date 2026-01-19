"""
EMG Acquisition Examples
========================
Demonstration of using the native EMG acquisition module.
"""

from emg_acquisition import EMGDevice, acquire_emg_data
import numpy as np
import time


def example_basic_acquisition():
    """Basic example: Connect, acquire, and save data."""
    print("="*60)
    print("Example 1: Basic EMG Acquisition")
    print("="*60)
    
    # Create EMG device instance
    emg = EMGDevice(connection_type='usb')
    
    try:
        # Connect to device
        device_info = emg.connect()
        print(f"\nConnected to: {device_info['name']}")
        print(f"Channels: {device_info['num_channels']}")
        print(f"Sample rate: {device_info['sample_rate']} Hz")
        
        # Start acquisition for 5 seconds
        print("\nAcquiring for 5 seconds...")
        emg.start_acquisition(duration=5.0)
        
        # Get acquired data
        data = emg.get_data()
        print(f"\nAcquired data shape: {data.shape}")
        print(f"Data range: [{np.nanmin(data):.2f}, {np.nanmax(data):.2f}]")
        
        # Export data
        emg.export_data('emg_data.npy')
        emg.export_data('emg_data.csv')
        
    finally:
        emg.disconnect()


def example_continuous_acquisition():
    """Example: Continuous acquisition with manual control."""
    print("\n" + "="*60)
    print("Example 2: Continuous Acquisition with Manual Control")
    print("="*60)
    
    emg = EMGDevice()
    
    try:
        # Connect
        emg.connect()
        
        # Configure
        emg.configure_acquisition(
            sample_rate=2000,
            reference_calculation=True
        )
        
        # Start continuous acquisition
        emg.start_acquisition()
        
        # Collect data for 10 seconds
        print("\nCollecting data for 10 seconds...")
        start_time = time.time()
        sample_count = 0
        
        while (time.time() - start_time) < 10:
            samples = emg.get_samples(blocking=False)
            
            if samples is not None:
                sample_count += samples.shape[0]
                
                # Print progress every second
                if sample_count % 2000 < 200:
                    elapsed = time.time() - start_time
                    print(f"  {elapsed:.1f}s: {sample_count} samples acquired")
            
            time.sleep(0.05)
        
        # Stop acquisition
        emg.stop_acquisition()
        
        # Get all data
        data = emg.get_data()
        print(f"\nTotal samples collected: {data.shape[0]}")
        
    finally:
        emg.disconnect()


def example_channel_selection():
    """Example: Get channel information and select specific channels."""
    print("\n" + "="*60)
    print("Example 3: Channel Information")
    print("="*60)
    
    emg = EMGDevice()
    
    try:
        emg.connect()
        
        # Get channel names
        channel_names = emg.get_channel_names()
        print(f"\nTotal channels: {len(channel_names)}")
        print("\nFirst 10 channels:")
        for i, name in enumerate(channel_names[:10]):
            print(f"  [{i}] {name}")
        
        # Get detailed channel info
        channel_info = emg.get_channel_info()
        print("\nDetailed info for first 5 channels:")
        for info in channel_info[:5]:
            print(f"  {info}")
        
        # Acquire data
        emg.start_acquisition(duration=3.0)
        data = emg.get_data()
        
        # Extract specific channels (e.g., first 8 EMG channels)
        emg_channels = data[:, :8]
        print(f"\nEMG channels data shape: {emg_channels.shape}")
        
    finally:
        emg.disconnect()


def example_context_manager():
    """Example: Using context manager for automatic cleanup."""
    print("\n" + "="*60)
    print("Example 4: Context Manager (Automatic Cleanup)")
    print("="*60)
    
    # Context manager automatically handles connection and cleanup
    with EMGDevice() as emg:
        emg.connect()
        print(f"Device: {emg}")
        
        emg.start_acquisition(duration=2.0)
        data = emg.get_data()
        
        print(f"Acquired {data.shape[0]} samples")
    
    print("Device automatically disconnected!")


def example_quick_acquisition():
    """Example: Quick acquisition using convenience function."""
    print("\n" + "="*60)
    print("Example 5: Quick Acquisition Function")
    print("="*60)
    
    # One-liner to acquire data
    data, info = acquire_emg_data(
        duration=3.0,
        connection_type='usb',
        export_to='quick_emg_data.npy'
    )
    
    print(f"\nQuick acquisition complete!")
    print(f"Device: {info['name']}")
    print(f"Data shape: {data.shape}")
    print(f"Saved to: quick_emg_data.npy")


if __name__ == "__main__":
    # Run examples
    try:
        example_basic_acquisition()
        time.sleep(1)
        
        example_continuous_acquisition()
        time.sleep(1)
        
        example_channel_selection()
        time.sleep(1)
        
        example_context_manager()
        time.sleep(1)
        
        example_quick_acquisition()
        
        print("\n" + "="*60)
        print("All examples completed successfully!")
        print("="*60)
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
