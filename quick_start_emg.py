"""
Quick Start EMG Recording
=========================
Simple script to quickly record EMG data with visualization.

Just run this script and it will:
1. Connect to your TMSi device
2. Set up differential EMG recording
3. Apply filtering
4. Show real-time plot
5. Save data and plot
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src import EMGDevice, EMGProcessor, EMGPlotter

# ==================== CONFIGURATION ====================
# Adjust these settings for your setup

# Connection
CONNECTION_TYPE = 'usb'  # 'usb', 'bluetooth', 'network', 'wifi'

# EMG Channel Configuration
USE_DIFFERENTIAL_PAIR = True  # Recommended for EMG
POS_CHANNEL = 13  # Positive electrode channel number
NEG_CHANNEL = 14  # Negative electrode channel number
CHANNELS_ARE_1_BASED = True  # True = channel numbers, False = array indices

# Alternatively, use single channel or auto-detect
# USE_DIFFERENTIAL_PAIR = False
# SINGLE_CHANNEL = 16  # Channel index or name
# AUTO_DETECT = True   # Auto-detect based on signal variance

# Recording
DURATION = 10.0  # seconds

# Signal Processing
APPLY_BANDPASS = True
BANDPASS_LOW = 20.0   # Hz
BANDPASS_HIGH = 450.0  # Hz
REMOVE_POWERLINE = True
POWERLINE_FREQ = 50.0  # 50 Hz (Europe) or 60 Hz (USA)

# Visualization
ENABLE_PLOT = True
WINDOW_SECONDS = 5.0  # Time window to display

# Export
SAVE_DATA = True
SAVE_PLOT = True
OUTPUT_PREFIX = "emg_recording"  # Files will be named: emg_recording.csv, emg_recording.png

# ==================== SCRIPT ====================

def main():
    print("="*60)
    print("Quick Start EMG Recording")
    print("="*60)
    
    # Create device
    print(f"\nConnecting to TMSi device ({CONNECTION_TYPE})...")
    emg = EMGDevice(connection_type=CONNECTION_TYPE)
    
    try:
        # Connect
        device_info = emg.connect()
        print(f"✓ Connected to: {device_info['name']}")
        print(f"  Serial: {device_info['serial_number']}")
        print(f"  Channels: {device_info['num_channels']}")
        print(f"  Sample rate: {device_info['sample_rate']} Hz")
        
        # Configure acquisition
        emg.configure_acquisition(reference_calculation=False)
        
        # Configure EMG channels
        print("\nConfiguring EMG channels...")
        if USE_DIFFERENTIAL_PAIR:
            pos_idx, neg_idx, desc = emg.configure_emg_differential_pair(
                pos_channel=POS_CHANNEL,
                neg_channel=NEG_CHANNEL,
                channels_are_1_based=CHANNELS_ARE_1_BASED
            )
            print(f"✓ Differential pair: {desc}")
        else:
            # Single channel mode
            if 'AUTO_DETECT' in globals() and AUTO_DETECT:
                emg.configure_emg_single_channel(auto_detect=True)
                print("✓ Auto-detect mode (will select on first samples)")
            else:
                idx, name = emg.configure_emg_single_channel(channel=SINGLE_CHANNEL)
                print(f"✓ Single channel: {name} (index {idx})")
        
        # Setup processing
        processor = None
        if APPLY_BANDPASS:
            print(f"\nSetting up signal processing...")
            processor = EMGProcessor(
                fs=emg.sample_rate,
                bandpass_low=BANDPASS_LOW,
                bandpass_high=BANDPASS_HIGH,
                bandpass_order=4,
                notch_freq=POWERLINE_FREQ if REMOVE_POWERLINE else None
            )
            print(f"✓ Bandpass: {BANDPASS_LOW}-{BANDPASS_HIGH} Hz")
            if REMOVE_POWERLINE:
                print(f"✓ Notch filter: {POWERLINE_FREQ} Hz")
        
        # Setup visualization
        plotter = None
        if ENABLE_PLOT:
            try:
                print("\nSetting up real-time plot...")
                plotter = EMGPlotter(
                    sample_rate=emg.sample_rate,
                    window_seconds=WINDOW_SECONDS,
                    show_filtered=APPLY_BANDPASS,
                    title=f"EMG Recording (ch{POS_CHANNEL}-ch{NEG_CHANNEL})" if USE_DIFFERENTIAL_PAIR else "EMG Recording"
                )
                print("✓ Real-time plot enabled")
            except Exception as e:
                print(f"! Visualization disabled: {e}")
                plotter = None
        
        # Setup callback for real-time processing and visualization
        if processor or plotter:
            def callback(raw_samples, emg_signal):
                filtered = processor.process(emg_signal) if processor else emg_signal
                if plotter:
                    plotter.update(emg_signal, filtered if processor else None)
            
            emg.set_data_callback(callback)
        
        # Start recording
        print(f"\n{'='*60}")
        print(f"RECORDING FOR {DURATION} SECONDS...")
        print(f"{'='*60}")
        if plotter:
            print("(Real-time plot will update during recording)")
        print()
        
        emg.start_acquisition(duration=DURATION)
        
        # Get results
        print(f"\n{'='*60}")
        print("Recording complete!")
        print(f"{'='*60}")
        
        raw_data = emg.get_data()
        emg_signal = emg.extract_emg_signal()
        
        print(f"\nAcquired:")
        print(f"  Raw data: {raw_data.shape[0]} samples × {raw_data.shape[1]} channels")
        print(f"  EMG signal: {len(emg_signal)} samples")
        
        # Calculate signal quality
        from src import calculate_signal_quality
        quality = calculate_signal_quality(emg_signal, emg.sample_rate)
        
        print(f"\nSignal Quality:")
        print(f"  Mean: {quality['mean']:.3f} µV")
        print(f"  RMS: {quality['rms']:.3f} µV")
        print(f"  Std: {quality['std']:.3f} µV")
        print(f"  Range: {quality['range']:.3f} µV")
        if 'snr_db' in quality:
            print(f"  SNR: {quality['snr_db']:.1f} dB")
        
        # Save data
        if SAVE_DATA:
            csv_file = f"{OUTPUT_PREFIX}.csv"
            npy_file = f"{OUTPUT_PREFIX}.npy"
            print(f"\nSaving data...")
            emg.export_data(csv_file)
            
            # Also save EMG signal separately
            import numpy as np
            np.save(npy_file, emg_signal)
            print(f"✓ EMG signal saved to: {npy_file}")
        
        # Save plot
        if SAVE_PLOT and plotter:
            png_file = f"{OUTPUT_PREFIX}.png"
            print(f"\nSaving plot...")
            plotter.save(png_file)
        
        # Show plot
        if plotter:
            print("\n" + "="*60)
            print("Plot window open. Close it to exit.")
            print("="*60)
            plotter.show(block=True)
            plotter.close()
        
        print("\n" + "="*60)
        print("SUCCESS!")
        print("="*60)
        print("\nFiles created:")
        if SAVE_DATA:
            print(f"  - {OUTPUT_PREFIX}.csv (all channels)")
            print(f"  - {OUTPUT_PREFIX}.npy (EMG signal)")
        if SAVE_PLOT:
            print(f"  - {OUTPUT_PREFIX}.png (plot)")
        print()
        
    except KeyboardInterrupt:
        print("\n\n[!] Recording interrupted by user")
    except Exception as e:
        print(f"\n[!] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always disconnect
        print("\nDisconnecting...")
        emg.disconnect()
        print("✓ Device disconnected")


if __name__ == "__main__":
    main()
