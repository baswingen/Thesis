"""
EMG Acquisition - Quick Start Examples
======================================
Examples of using the native EMG acquisition module from your project.

Run these examples from the project root directory.
"""

# Import from your src module
from src import EMGDevice, acquire_emg_data
import numpy as np


def example_1_simple_acquisition():
    """
    Example 1: Simple 10-second EMG acquisition
    """
    print("="*60)
    print("Example 1: Simple EMG Acquisition")
    print("="*60)
    
    # Create device interface
    emg = EMGDevice(connection_type='usb')
    
    try:
        # Connect to device
        info = emg.connect()
        print(f"\nConnected to: {info['name']}")
        print(f"Recording from {info['num_channels']} channels at {info['sample_rate']} Hz")
        
        # Acquire 10 seconds of data
        print("\nRecording 10 seconds of EMG data...")
        emg.start_acquisition(duration=10.0)
        
        # Get the data
        data = emg.get_data()
        print(f"\nRecorded {data.shape[0]} samples")
        
        # Save to file
        emg.export_data('my_emg_recording.npy')
        print("Saved to: my_emg_recording.npy")
        
    finally:
        emg.disconnect()


def example_2_one_liner():
    """
    Example 2: One-line acquisition (quick and easy!)
    """
    print("\n" + "="*60)
    print("Example 2: One-Line Quick Acquisition")
    print("="*60)
    
    # Acquire and save in one line!
    data, info = acquire_emg_data(
        duration=5.0,
        export_to='quick_recording.csv'
    )
    
    print(f"\nRecorded {data.shape[0]} samples from {info['name']}")
    print(f"Saved to: quick_recording.csv")


def example_3_process_data():
    """
    Example 3: Acquire and process EMG data
    """
    print("\n" + "="*60)
    print("Example 3: Acquire and Process Data")
    print("="*60)
    
    with EMGDevice() as emg:
        # Connect and configure
        emg.connect()
        emg.configure_acquisition(
            sample_rate=2000,  # 2 kHz sampling
            reference_calculation=True  # Remove common noise
        )
        
        # Acquire data
        print("\nRecording EMG...")
        emg.start_acquisition(duration=5.0)
        data = emg.get_data()
        
        # Process the data
        print(f"\nData shape: {data.shape}")
        
        # Calculate RMS for each channel
        rms_values = np.sqrt(np.nanmean(data**2, axis=0))
        print(f"\nRMS values (first 5 channels): {rms_values[:5]}")
        
        # Find active channels (RMS > threshold)
        threshold = 10.0  # Adjust based on your data
        active_channels = np.where(rms_values > threshold)[0]
        print(f"\nActive channels: {active_channels}")
        
        # Extract only active channel data
        active_data = data[:, active_channels]
        print(f"Active data shape: {active_data.shape}")


def example_4_multiple_trials():
    """
    Example 4: Record multiple trials
    """
    print("\n" + "="*60)
    print("Example 4: Multiple Trial Recording")
    print("="*60)
    
    emg = EMGDevice()
    emg.connect()
    
    num_trials = 3
    trial_duration = 5.0
    
    all_trials = []
    
    for trial in range(num_trials):
        print(f"\nTrial {trial + 1}/{num_trials}")
        print(f"Recording for {trial_duration} seconds...")
        
        # Clear previous data
        emg.clear_buffer()
        
        # Acquire trial
        emg.start_acquisition(duration=trial_duration)
        data = emg.get_data()
        
        # Save trial
        emg.export_data(f'trial_{trial + 1}.npy', data)
        all_trials.append(data)
        
        print(f"  Saved trial_{trial + 1}.npy ({data.shape[0]} samples)")
        
        input("  Press Enter for next trial...")
    
    emg.disconnect()
    
    print(f"\nCompleted {num_trials} trials!")


def example_5_realtime_monitoring():
    """
    Example 5: Real-time EMG monitoring
    """
    print("\n" + "="*60)
    print("Example 5: Real-Time EMG Monitoring")
    print("="*60)
    
    emg = EMGDevice()
    emg.connect()
    
    print("\nStarting real-time monitoring...")
    print("Press Ctrl+C to stop\n")
    
    emg.start_acquisition()  # Continuous mode
    
    try:
        sample_count = 0
        while True:
            samples = emg.get_samples(blocking=False)
            
            if samples is not None:
                sample_count += samples.shape[0]
                
                # Calculate RMS for monitoring
                rms = np.sqrt(np.nanmean(samples**2))
                
                # Print update every ~1000 samples
                if sample_count % 1000 < 100:
                    print(f"Samples: {sample_count:6d} | RMS: {rms:6.2f}")
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    emg.stop_acquisition()
    
    # Get all recorded data
    data = emg.get_data()
    print(f"\nTotal samples recorded: {data.shape[0]}")
    
    emg.disconnect()


def example_6_specific_channels():
    """
    Example 6: Work with specific EMG channels
    """
    print("\n" + "="*60)
    print("Example 6: Specific Channel Selection")
    print("="*60)
    
    emg = EMGDevice()
    emg.connect()
    
    # Get channel information
    channel_names = emg.get_channel_names()
    print(f"\nAvailable channels ({len(channel_names)}):")
    for i, name in enumerate(channel_names[:10]):
        print(f"  [{i}] {name}")
    
    # Acquire data
    emg.start_acquisition(duration=5.0)
    data = emg.get_data()
    
    # Select specific channels (e.g., channels 0-7 for EMG)
    emg_channel_indices = list(range(8))
    emg_data = data[:, emg_channel_indices]
    
    print(f"\nAll data shape: {data.shape}")
    print(f"EMG data shape: {emg_data.shape}")
    
    # Process only EMG channels
    for i, idx in enumerate(emg_channel_indices):
        channel_rms = np.sqrt(np.nanmean(emg_data[:, i]**2))
        print(f"  {channel_names[idx]}: RMS = {channel_rms:.2f}")
    
    emg.disconnect()


if __name__ == "__main__":
    print("\n" + "="*60)
    print(" EMG ACQUISITION - USAGE EXAMPLES")
    print("="*60)
    print("\nChoose an example to run:")
    print("  1. Simple acquisition (10 seconds)")
    print("  2. One-line quick acquisition")
    print("  3. Acquire and process data")
    print("  4. Multiple trial recording")
    print("  5. Real-time monitoring")
    print("  6. Specific channel selection")
    print("  0. Run all examples")
    
    try:
        choice = input("\nEnter choice (0-6): ").strip()
        
        examples = {
            '1': example_1_simple_acquisition,
            '2': example_2_one_liner,
            '3': example_3_process_data,
            '4': example_4_multiple_trials,
            '5': example_5_realtime_monitoring,
            '6': example_6_specific_channels
        }
        
        if choice == '0':
            # Run all examples (except real-time monitoring)
            for key in ['1', '2', '3', '6']:
                examples[key]()
                print("\n")
        elif choice in examples:
            examples[choice]()
        else:
            print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
