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
import warnings
from collections import deque
from datetime import datetime

# Optional visualization imports
try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available - visualization disabled")

# Optional filtering imports
try:
    from scipy import signal
    SCIPY_AVAILABLE = True
except Exception as e:
    SCIPY_AVAILABLE = False
    error_type = type(e).__name__
    print("Warning: scipy not available - filtering disabled")
    print(f"         Error type: {error_type}")
    print(f"         Error message: {e}")
    if "DLL" in str(e) or "dll" in str(e) or "beleid" in str(e) or "policy" in str(e).lower():
        print("         Note: This appears to be a Windows security policy issue blocking DLL loading.")
        print("         You may need to adjust Windows Application Control policies or run as administrator.")
    elif isinstance(e, ImportError):
        print("         Install with: pip install scipy>=1.11.0")

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
TEST_DURATION = 5  # seconds
INTERFACE_TYPE = DeviceInterfaceType.usb  # or DeviceInterfaceType.bluetooth
SAMPLE_RATE = None  # Use device default (None) or set specific rate (e.g., 2048)
REFERENCE_CALCULATION = False  # For EMG differential pairs, this is usually best kept OFF

# EMG channel configuration (make this consistent!)
# -------------------------------------------------
# You said: "channel 1 and 2 being connected to my bicep, and ground to ground".
# In the TMSi sample matrix, indices are 0-based, so:
#   - "Channel 1" -> index 0
#   - "Channel 2" -> index 1
#
# Recommended: treat those two channels as a *differential* pair:
#   EMG(t) = channel_1(t) - channel_2(t)
#
# This is robust even if the SDK reports duplicated names (e.g., multiple "ExG")
# and even if the device does not expose physical port numbers.

USE_EMG_DIFFERENTIAL_PAIR = True
# IMPORTANT: Channel numbering can be confusing:
# - In the numpy `samples` array, channels are 0-based (first channel is index 0).
# - Humans often refer to channels as 1-based ("channel 1", "channel 2", ...).
#
# If you say "electrodes on channel 3 and 4", that usually means:
#   channel numbers 3 & 4  -> numpy indices 2 & 3
#
# Set EMG_PAIR_CHANNELS_ARE_1_BASED=True and then use *channel numbers* below.
EMG_PAIR_CHANNELS_ARE_1_BASED = True
EMG_PAIR_POS_CHANNEL = 13   # channel number (if 1-based) OR index (if 0-based)
EMG_PAIR_NEG_CHANNEL = 14   # channel number (if 1-based) OR index (if 0-based)

# Fallback selection options (used only if USE_EMG_DIFFERENTIAL_PAIR=False)
EMG_INPUT_PORT = None            # e.g. 17 for "Input 17" (set to None to disable)
EMG_CHANNEL_NAME = None          # e.g. "Bip1", "ExG1" (overrides EMG_INPUT_PORT if set)
EMG_CHANNEL_INDEX = None         # e.g. 16 (overrides EMG_CHANNEL_NAME/EMG_INPUT_PORT if set)

# Visualization settings
# ----------------------
ENABLE_REALTIME_PLOT = True      # Enable live matplotlib plot
PLOT_WINDOW_SECONDS = 5.0        # Time window to display (seconds)
APPLY_BANDPASS_FILTER = True     # Apply EMG bandpass filter (20-450 Hz)
BANDPASS_LOW_CUTOFF = 20.0       # High-pass cutoff (Hz) - removes DC offset/motion artifacts
BANDPASS_HIGH_CUTOFF = 450.0     # Low-pass cutoff (Hz) - removes high-frequency noise
FILTER_ORDER = 4                 # Butterworth filter order
AUTO_SAVE_PLOT = False           # Automatically save plot at end
PLOT_SAVE_PATH = "emg_plot.png"  # Path to save plot

# -----------------------
# Helper Functions
# -----------------------
class BandpassFilter:
    """
    Real-time bandpass filter using scipy's Butterworth filter.
    Maintains state across calls to avoid edge artifacts.
    """
    def __init__(self, lowcut, highcut, fs, order=4):
        """
        Initialize bandpass filter.
        
        Args:
            lowcut: Low cutoff frequency (Hz)
            highcut: High cutoff frequency (Hz)
            fs: Sample rate (Hz)
            order: Filter order
        """
        self.lowcut = lowcut
        self.highcut = highcut
        self.fs = fs
        self.order = order
        self.sos = None
        self.zi = None
        
        if SCIPY_AVAILABLE:
            # Create second-order sections representation
            nyquist = 0.5 * fs
            low = lowcut / nyquist
            high = highcut / nyquist
            
            # Ensure cutoffs are valid
            if low <= 0:
                low = 0.01
            if high >= 1:
                high = 0.99
            
            self.sos = signal.butter(order, [low, high], btype='band', output='sos')
            # Initialize filter state
            self.zi = signal.sosfilt_zi(self.sos)
    
    def filter(self, data):
        """
        Apply bandpass filter to data.
        
        Args:
            data: 1D numpy array of samples
            
        Returns:
            Filtered data
        """
        if not SCIPY_AVAILABLE or self.sos is None:
            return data
        
        # Apply filter with state
        filtered, self.zi = signal.sosfilt(self.sos, data, zi=self.zi)
        return filtered


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
            
            # WORKAROUND: Try to fix encoding issue in TMSi legacy_device.py
            # The library incorrectly decodes UTF-16LE channel names
            def fix_channel_name(corrupted_name, channel_index):
                """
                Fix corrupted UTF-16LE channel names from TMSi legacy device.
                The TMSi library treats uint16 values as bytes, corrupting UTF-16LE decoding.
                
                For Porti7 (38 channels):
                - Channels 0-15: UNI1-UNI16 (unipolar)
                - Channels 16-31: BIP1-BIP16 (bipolar inputs 1-16)
                - Channels 32-35: BIP17-BIP20 (bipolar inputs 17-20)
                - Channel 36: STATUS
                - Channel 37: COUNTER
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
            
            # Fix ALL channels first (before displaying)
            print(f"   Fixing channel names for encoding issues...")
            channels_fixed = 0
            for i, ch in enumerate(channels):
                try:
                    ch_name = ch.get_channel_name()
                    # Check if name has encoding issues (non-ASCII characters)
                    if any(ord(c) > 127 for c in ch_name):
                        # Fix the corrupted name
                        ch_name_fixed = fix_channel_name(ch_name, i)
                        # Override the channel's internal name
                        ch._alt_name = ch_name_fixed
                        ch._def_name = ch_name_fixed
                        channels_fixed += 1
                except:
                    pass
            
            if channels_fixed > 0:
                print(f"   Fixed {channels_fixed} channel names with encoding issues")
            
            # Show up to first 32 channels for easier mapping
            max_preview = min(32, len(channels))
            for i, ch in enumerate(channels[:max_preview]):
                try:
                    ch_name = ch.get_channel_name()
                    ch_unit = ch.get_channel_unit_name()
                    # Legacy devices may expose a physical port number/name
                    ch_port = None
                    ch_port_name = None
                    try:
                        if hasattr(ch, "get_channel_port"):
                            ch_port = ch.get_channel_port()
                        if hasattr(ch, "get_channel_port_name"):
                            ch_port_name = ch.get_channel_port_name()
                    except Exception:
                        ch_port = None
                        ch_port_name = None

                    ch_name_display = f"{ch_name:20s}"
                    if ch_port is not None and ch_port != -1:
                        port_str = f"port={int(ch_port):>2d}"
                        if ch_port_name:
                            port_str += f" ({ch_port_name})"
                        print(f"     [{i:3d}] {ch_name_display} - {ch_unit:10s}  {port_str}")
                    else:
                        print(f"     [{i:3d}] {ch_name_display} - {ch_unit:10s}")
                except Exception as e:
                    print(f"     [{i:3d}] <error reading channel: {e}>")
            
            if len(channels) > max_preview:
                print(f"     ... and {len(channels) - max_preview} more channels:")
                # Show remaining channels
                for i in range(max_preview, len(channels)):
                    try:
                        ch = channels[i]
                        ch_name = ch.get_channel_name()
                        ch_unit = ch.get_channel_unit_name()
                        ch_port = None
                        ch_port_name = None
                        try:
                            if hasattr(ch, "get_channel_port"):
                                ch_port = ch.get_channel_port()
                            if hasattr(ch, "get_channel_port_name"):
                                ch_port_name = ch.get_channel_port_name()
                        except Exception:
                            ch_port = None
                            ch_port_name = None
                        ch_name_display = f"{ch_name:20s}"
                        if ch_port is not None and ch_port != -1:
                            port_str = f"port={int(ch_port):>2d}"
                            if ch_port_name:
                                port_str += f" ({ch_port_name})"
                            print(f"     [{i:3d}] {ch_name_display} - {ch_unit:10s}  {port_str}")
                        else:
                            print(f"     [{i:3d}] {ch_name_display} - {ch_unit:10s}")
                    except Exception as e:
                        print(f"     [{i:3d}] <error reading channel: {e}>")

            # Resolve EMG input port / channel
            selected_channel_index = None
            selected_channel_name = None

            # Build a simple name -> index map (now with fixed names)
            name_to_index = {}
            for i, ch in enumerate(channels):
                try:
                    name_to_index[ch.get_channel_name()] = i
                    name_to_index[ch.get_channel_name().upper()] = i
                except:
                    pass

            # 0) Explicit channel index takes precedence (most reliable if names are duplicated)
            if EMG_CHANNEL_INDEX is not None:
                try:
                    idx = int(EMG_CHANNEL_INDEX)
                    if 0 <= idx < len(channels):
                        selected_channel_index = idx
                        selected_channel_name = channels[idx].get_channel_name()
                        print(f"\n[OK] Using EMG_CHANNEL_INDEX={idx} -> '{selected_channel_name}'")
                    else:
                        print(f"\n[!] WARNING: EMG_CHANNEL_INDEX={idx} is out of range (0..{len(channels)-1})")
                except Exception as e:
                    print(f"\n[!] WARNING: Could not use EMG_CHANNEL_INDEX={EMG_CHANNEL_INDEX}: {e}")

            # 1) Explicit channel name takes precedence
            if EMG_CHANNEL_NAME is not None:
                if EMG_CHANNEL_NAME in name_to_index:
                    selected_channel_index = name_to_index[EMG_CHANNEL_NAME]
                    selected_channel_name = EMG_CHANNEL_NAME
                elif EMG_CHANNEL_NAME.upper() in name_to_index:
                    selected_channel_index = name_to_index[EMG_CHANNEL_NAME.upper()]
                    selected_channel_name = channels[selected_channel_index].get_channel_name()
                else:
                    print(f"\n[!] WARNING: EMG_CHANNEL_NAME='{EMG_CHANNEL_NAME}' "
                          f"not found in device channels.")

            # 2) Otherwise, try to find channel from physical input port
            if selected_channel_index is None and EMG_INPUT_PORT is not None:
                port_str = str(EMG_INPUT_PORT)
                target_name = f"BIP{port_str}"
                
                # Try direct match with fixed names
                if target_name in name_to_index:
                    selected_channel_index = name_to_index[target_name]
                    selected_channel_name = channels[selected_channel_index].get_channel_name()
                    print(f"\n[OK] Matched EMG_INPUT_PORT={EMG_INPUT_PORT} to channel '{selected_channel_name}' (index {selected_channel_index})")
                else:
                    # Prefer matching on the actual legacy Port field if available
                    try:
                        for i, ch in enumerate(channels):
                            if hasattr(ch, "get_channel_port") and int(ch.get_channel_port()) == int(EMG_INPUT_PORT):
                                selected_channel_index = i
                                selected_channel_name = ch.get_channel_name()
                                port_name = ch.get_channel_port_name() if hasattr(ch, "get_channel_port_name") else ""
                                print(f"\n[OK] Matched EMG_INPUT_PORT={EMG_INPUT_PORT} to channel '{selected_channel_name}' "
                                      f"(index {selected_channel_index}, port_name='{port_name}')")
                                break
                    except Exception:
                        pass

                    # Try pattern matching
                    for i, ch in enumerate(channels):
                        try:
                            ch_name = ch.get_channel_name()
                            upper_name = ch_name.upper()
                            
                            # Match BIP17, IN17, etc.
                            if any(pattern in upper_name for pattern in [f"BIP{port_str}", f"IN{port_str}"]):
                                selected_channel_index = i
                                selected_channel_name = ch_name
                                break
                        except:
                            continue

            if selected_channel_index is not None:
                print("\n   EMG channel configuration:")
                print(f"     -> Selected channel index: {selected_channel_index}")
                print(f"     -> Selected channel name : {selected_channel_name}")
                print("     NOTE: In the data array 'samples', this is column "
                      f"{selected_channel_index}.")
            else:
                print("\n   EMG channel configuration:")
                print("     No EMG input port/channel was successfully resolved.")
                print("     You can configure this at the top of the script via "
                      "EMG_CHANNEL_INDEX, EMG_CHANNEL_NAME, or EMG_INPUT_PORT.")
            
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
    
    # Initialize visualization (if enabled)
    plot_enabled = False
    fig, axes, lines = None, None, {}
    emg_data_buffer = None
    emg_filtered_buffer = None
    time_buffer = None
    time_filtered_buffer = None
    bandpass_filter = None

    def _init_realtime_plot(measurement, selected_desc):
        """Initialize the plot after we know which channel to display."""
        nonlocal plot_enabled, fig, axes, lines, emg_data_buffer, emg_filtered_buffer, time_buffer, time_filtered_buffer, bandpass_filter

        if plot_enabled:
            return
        if not ENABLE_REALTIME_PLOT:
            return
        if not MATPLOTLIB_AVAILABLE:
            return
        if not selected_desc:
            return

        try:
            print(f"\n4b. Initializing real-time plot...")

            sample_rate = measurement.get_device_sample_rate()
            buffer_size = int(PLOT_WINDOW_SECONDS * sample_rate)

            # Initialize data buffers
            emg_data_buffer = deque(maxlen=buffer_size)
            time_buffer = deque(maxlen=buffer_size)

            if APPLY_BANDPASS_FILTER and SCIPY_AVAILABLE:
                emg_filtered_buffer = deque(maxlen=buffer_size)
                time_filtered_buffer = deque(maxlen=buffer_size)
                bandpass_filter = BandpassFilter(
                    BANDPASS_LOW_CUTOFF,
                    BANDPASS_HIGH_CUTOFF,
                    sample_rate,
                    FILTER_ORDER
                )
                print(f"   Bandpass filter: {BANDPASS_LOW_CUTOFF}-{BANDPASS_HIGH_CUTOFF} Hz, Order {FILTER_ORDER}")

            # Create figure and subplots
            if APPLY_BANDPASS_FILTER and SCIPY_AVAILABLE:
                fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
                axes = [ax1, ax2]

                ax1.set_title(f'Raw EMG Signal - {selected_desc}', fontsize=12, fontweight='bold')
                ax1.set_ylabel('Amplitude (µV)', fontsize=10)
                ax1.grid(True, alpha=0.3)
                line1, = ax1.plot([], [], 'b-', linewidth=0.8, label='Raw')
                ax1.legend(loc='upper right')
                lines['raw'] = line1

                ax2.set_title(f'Filtered EMG Signal ({BANDPASS_LOW_CUTOFF}-{BANDPASS_HIGH_CUTOFF} Hz)',
                              fontsize=12, fontweight='bold')
                ax2.set_xlabel('Time (s)', fontsize=10)
                ax2.set_ylabel('Amplitude (µV)', fontsize=10)
                ax2.grid(True, alpha=0.3)
                line2, = ax2.plot([], [], 'r-', linewidth=0.8, label='Filtered')
                ax2.legend(loc='upper right')
                lines['filtered'] = line2
            else:
                fig, ax1 = plt.subplots(1, 1, figsize=(12, 6))
                axes = [ax1]

                ax1.set_title(f'EMG Signal - {selected_desc} @ {sample_rate} Hz',
                              fontsize=12, fontweight='bold')
                ax1.set_xlabel('Time (s)', fontsize=10)
                ax1.set_ylabel('Amplitude (µV)', fontsize=10)
                ax1.grid(True, alpha=0.3)
                line1, = ax1.plot([], [], 'b-', linewidth=0.8, label='EMG')
                ax1.legend(loc='upper right')
                lines['raw'] = line1

            plt.tight_layout()
            plt.ion()
            plt.show(block=False)

            plot_enabled = True
            print(f"[OK] Real-time plot initialized")
            print(f"   Window: {PLOT_WINDOW_SECONDS} seconds")
            print(f"   Sample rate: {sample_rate} Hz")
        except Exception as e:
            print(f"[!] Warning: Could not initialize plot: {e}")
            plot_enabled = False
    
    # Only initialize plot now if we already know the channel; otherwise we will
    # initialize it lazily after auto-detect selects a channel.
    if ENABLE_REALTIME_PLOT and not MATPLOTLIB_AVAILABLE:
        print("\n[!] Real-time plot disabled: matplotlib not available")
    elif ENABLE_REALTIME_PLOT and selected_channel_index is not None:
        _init_realtime_plot(measurement, f"idx{selected_channel_index} '{selected_channel_name}'")
    
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
        plot_time_offset = 0.0
        
        # NOTE: selected_channel_index/selected_channel_name were defined
        # in the device-info block above. If that block failed, they may
        # not exist; handle this gracefully.
        try:
            selected_channel_index
        except NameError:
            selected_channel_index = None
            selected_channel_name = None

        emg_autoselected = False
        selected_desc = ""

        def _resolve_emg_pair_indices(n_channels=None):
            """
            Resolve configured EMG pair into 0-based numpy indices.

            If EMG_PAIR_CHANNELS_ARE_1_BASED=True, treat EMG_PAIR_POS_CHANNEL/NEG_CHANNEL
            as human channel numbers (1..N) and convert to indices (0..N-1).
            """
            try:
                pos_raw = int(EMG_PAIR_POS_CHANNEL)
                neg_raw = int(EMG_PAIR_NEG_CHANNEL)
            except Exception as e:
                raise ValueError(f"Invalid EMG_PAIR_POS/NEG settings: {e}")

            if EMG_PAIR_CHANNELS_ARE_1_BASED:
                pos_i = pos_raw - 1
                neg_i = neg_raw - 1
            else:
                pos_i = pos_raw
                neg_i = neg_raw

            if n_channels is not None:
                if not (0 <= pos_i < n_channels) or not (0 <= neg_i < n_channels):
                    raise ValueError(f"EMG pair indices out of range: pos={pos_i}, neg={neg_i}, n_channels={n_channels}")

            return pos_i, neg_i

        # If configured, lock to the differential pair immediately (consistent behavior)
        if USE_EMG_DIFFERENTIAL_PAIR:
            try:
                pos_i, neg_i = _resolve_emg_pair_indices()
                selected_desc = f"Diff idx{pos_i} - idx{neg_i} (ch{EMG_PAIR_POS_CHANNEL}-ch{EMG_PAIR_NEG_CHANNEL})"
                _init_realtime_plot(measurement, selected_desc)
                print(f"[OK] Using EMG differential pair: {selected_desc}")
            except Exception as e:
                print(f"[!] Could not set EMG differential pair: {e}")
                selected_desc = ""

        while (time.time() - start_time) < TEST_DURATION:
            try:
                # Read samples from measurement
                samples = measurement.get_samples(blocking=False)
                
                if samples is not None and len(samples) > 0:
                    read_count += 1
                    n_samples = samples.shape[0]
                    sample_count += n_samples

                    # If no channel was configured (and differential mode is off),
                    # try a one-time auto-detect: pick the uV channel with the
                    # highest variability (ignores NaNs).
                    if (not USE_EMG_DIFFERENTIAL_PAIR) and (selected_channel_index is None) and (not emg_autoselected):
                        try:
                            with warnings.catch_warnings():
                                warnings.simplefilter("ignore", RuntimeWarning)
                                per_ch_std = np.nanstd(samples, axis=0)
                            candidates = []
                            for ci, stdv in enumerate(per_ch_std):
                                try:
                                    unit = channels[ci].get_channel_unit_name()
                                except Exception:
                                    unit = ""
                                stdv = float(stdv)
                                # Only consider channels that actually have finite variability.
                                if not np.isfinite(stdv):
                                    continue
                                if unit.lower().endswith("v"):  # uV/mV/V
                                    candidates.append((stdv, ci))
                            candidates.sort(key=lambda x: x[0], reverse=True)

                            print("\n[INFO] Auto-detect EMG candidates (highest std, voltage-like units):")
                            if not candidates:
                                print("  (No non-NaN voltage-like channels found in this chunk.)")
                            for rank, (stdv, ci) in enumerate(candidates[:8], start=1):
                                try:
                                    nm = channels[ci].get_channel_name()
                                    un = channels[ci].get_channel_unit_name()
                                except Exception:
                                    nm, un = "?", "?"
                                print(f"  #{rank}: idx={ci:2d}  name='{nm}'  unit='{un}'  std={stdv:.3f}")

                            if candidates:
                                best_std, best_ci = candidates[0]
                                selected_channel_index = best_ci
                                selected_channel_name = channels[best_ci].get_channel_name()
                                emg_autoselected = True
                                selected_desc = f"idx{best_ci} '{selected_channel_name}'"
                                print(f"[OK] Auto-selected EMG channel {selected_desc} based on std={best_std:.3f}\n")
                                # Now that we know the channel, enable the plot.
                                _init_realtime_plot(measurement, selected_desc)
                        except Exception as e:
                            print(f"\n[!] Auto-detect failed: {e}\n")
                    
                    # Print sample info
                    elapsed = time.time() - start_time
                    print(f"[{elapsed:6.2f}s] Read #{read_count}: Got {n_samples} samples")
                    print(f"            Shape: {samples.shape}")
                    
                    # Print first sample values (first 5 channels)
                    num_channels_to_show = min(5, samples.shape[1])
                    first_sample = samples[0, :num_channels_to_show]
                    print(f"            First sample (first {num_channels_to_show} channels): {first_sample}")

                    # Compute EMG signal for this chunk (either differential pair or single channel)
                    emg_chunk = None
                    if USE_EMG_DIFFERENTIAL_PAIR:
                        try:
                            pos_i, neg_i = _resolve_emg_pair_indices(samples.shape[1])
                            if pos_i < samples.shape[1] and neg_i < samples.shape[1]:
                                emg_chunk = samples[:, pos_i] - samples[:, neg_i]
                                emg_value = emg_chunk[0]
                                print(f"            EMG (diff) first sample: {emg_value}")
                        except Exception:
                            emg_chunk = None
                    else:
                        if selected_channel_index is not None and selected_channel_index < samples.shape[1]:
                            emg_chunk = samples[:, selected_channel_index]
                            emg_value = emg_chunk[0]
                            print(f"            EMG channel [{selected_channel_index}] "
                                  f"('{selected_channel_name}') first sample: {emg_value}")

                    if emg_chunk is not None:
                        try:
                            with warnings.catch_warnings():
                                warnings.simplefilter("ignore", RuntimeWarning)
                                ch_mean = float(np.nanmean(emg_chunk))
                                ch_std = float(np.nanstd(emg_chunk))
                            print(f"            EMG stats (chunk): mean={ch_mean:.3f}, std={ch_std:.3f}")
                        except Exception:
                            pass

                        # Update plot data if visualization is enabled
                        if plot_enabled:
                            try:
                                # EMG data for plotting (single or differential)
                                emg_channel_data = emg_chunk

                                # Create time values for this chunk
                                sample_rate = measurement.get_device_sample_rate()
                                dt = 1.0 / sample_rate
                                chunk_times = plot_time_offset + np.arange(n_samples) * dt
                                plot_time_offset = chunk_times[-1] + dt

                                # Add to raw buffers (skip NaNs)
                                for t, val in zip(chunk_times, emg_channel_data):
                                    if not np.isnan(val):
                                        time_buffer.append(t)
                                        emg_data_buffer.append(val)

                                # Apply filtering if enabled
                                if bandpass_filter is not None and len(emg_channel_data) > 0:
                                    valid_mask = ~np.isnan(emg_channel_data)
                                    if np.any(valid_mask):
                                        filtered_chunk = bandpass_filter.filter(emg_channel_data[valid_mask])
                                        for t, val in zip(chunk_times[valid_mask], filtered_chunk):
                                            emg_filtered_buffer.append(val)
                                            time_filtered_buffer.append(t)

                                # Update plot
                                if len(time_buffer) > 1:
                                    time_array = np.array(time_buffer)
                                    raw_array = np.array(emg_data_buffer)

                                    if 'raw' in lines:
                                        lines['raw'].set_data(time_array, raw_array)
                                        axes[0].relim()
                                        axes[0].autoscale_view()
                                        if time_array[-1] > PLOT_WINDOW_SECONDS:
                                            axes[0].set_xlim(time_array[-1] - PLOT_WINDOW_SECONDS, time_array[-1])

                                if 'filtered' in lines and emg_filtered_buffer is not None and len(emg_filtered_buffer) > 1:
                                    tf = np.array(time_filtered_buffer)
                                    yf = np.array(emg_filtered_buffer)
                                    if len(tf) == len(yf) and len(tf) > 1:
                                        lines['filtered'].set_data(tf, yf)
                                        axes[1].relim()
                                        axes[1].autoscale_view()
                                        if tf[-1] > PLOT_WINDOW_SECONDS:
                                            axes[1].set_xlim(tf[-1] - PLOT_WINDOW_SECONDS, tf[-1])

                                # Redraw (plt.pause tends to work more reliably than flush_events)
                                fig.canvas.draw_idle()
                                plt.pause(0.001)

                            except Exception as plot_error:
                                # Don't crash on plot errors; show periodically
                                if read_count <= 3 or (read_count % 25 == 0):
                                    print(f"            [!] Plot update error: {plot_error}")
                    elif USE_EMG_DIFFERENTIAL_PAIR:
                        # Most common cause: one of the selected channels is all-NaN (not connected / wrong index)
                        try:
                            pos_i, neg_i = _resolve_emg_pair_indices(samples.shape[1])
                            pos_valid = int(np.sum(~np.isnan(samples[:, pos_i])))
                            neg_valid = int(np.sum(~np.isnan(samples[:, neg_i])))
                            print(f"            [!] EMG diff unavailable: valid samples pos_idx={pos_i} -> {pos_valid}/{n_samples}, "
                                  f"neg_idx={neg_i} -> {neg_valid}/{n_samples}")
                            if EMG_PAIR_CHANNELS_ARE_1_BASED:
                                print("            [!] Tip: If you intended numpy indices, set EMG_PAIR_CHANNELS_ARE_1_BASED = False.")
                                print("            [!] Tip: If you intended channel numbers (1-based), keep it True and set EMG_PAIR_POS/NEG to 3 & 4 etc.")
                        except Exception:
                            pass
                    
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
        # Save plot if enabled
        if plot_enabled and AUTO_SAVE_PLOT:
            try:
                # Add timestamp to filename
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                base, ext = os.path.splitext(PLOT_SAVE_PATH)
                save_path = f"{base}_{timestamp}{ext}"
                
                print(f"\nSaving plot to '{save_path}'...")
                fig.savefig(save_path, dpi=150, bbox_inches='tight')
                print(f"[OK] Plot saved successfully")
            except Exception as e:
                print(f"[!] Error saving plot: {e}")
        
        # Keep plot open briefly if enabled (user can manually save if needed)
        if plot_enabled and not AUTO_SAVE_PLOT:
            print("\n[INFO] Plot is still open. Close the plot window to continue,")
            print("       or manually save it using the matplotlib toolbar.")
            try:
                plt.show(block=True)  # Wait for user to close
            except Exception:
                pass
        
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
