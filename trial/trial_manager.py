"""
Trial Manager Module
====================
Core orchestration for trial data collection.

Coordinates:
- Synchronized EMG + IMU acquisition
- Real-time preprocessing
- GUI updates
- Data storage
- State machine
"""

import time
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any
from collections import deque
import threading
import sys
import struct

# Import trial modules
from . import setup_trial
from .trial_gui import TrialGUI, TrialState
from .data_storage import TrialDataStorage

try:
    import serial
except ImportError:
    serial = None

# Import source modules
try:
    from src.emg_processing import EMGPreprocessor
    from src.dummy_acquisition import DummyAcquisition
    from src.stm32_emg_sync import SyncDelayEstimator
    from src.stm32_reader import SampleSTM32, BIN_FORMAT, BIN_PACKET_SIZE, BIN_SYNC1, BIN_SYNC2
    # find_arduino_port will be imported locally if needed
except ImportError:
    print("\u26a0 Warning: Could not import src modules. Mock mode will be used.")
    EMGPreprocessor = None
    DummyAcquisition = None
    SyncDelayEstimator = None
    SampleSTM32 = BIN_FORMAT = BIN_PACKET_SIZE = BIN_SYNC1 = BIN_SYNC2 = None

try:
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
    from TMSiSDK.device.devices.legacy.measurements.signal_measurement import SignalMeasurement
    _TMSI_AVAILABLE = True
except ImportError:
    LegacyDevice = None
    SignalMeasurement = None
    _TMSI_AVAILABLE = False

def _extract_trig_binary(v_raw: np.ndarray, is_status: bool, trig_bit: Optional[int]) -> np.ndarray:
    if is_status and trig_bit is not None:
        # nan_to_num prevents RuntimeWarning: invalid value encountered in cast
        v_clean = np.nan_to_num(v_raw, nan=0)
        return ((v_clean.astype(np.int64) >> trig_bit) & 1).astype(np.float64)
    lo, hi = np.nanmin(v_raw), np.nanmax(v_raw)
    thr = (lo + hi) / 2.0 if (hi - lo) > 1e-9 else 0.5
    return (v_raw > thr).astype(np.float64)


def _stm32_to_imu_sample(stm_sample):
    """
    Convert a SampleSTM32 (dual BNO085 via binary protocol) into a duck-typed
    object that matches the IMUSample API used by _on_imu_sample().

    SampleSTM32 fields used:
        t_ms        - Arduino millis() timestamp
        yaw1/pitch1/roll1 - IMU1 Euler angles in degrees (from BNO085 RVC)
        ax1/ay1/az1       - IMU1 acceleration in m/s²
        yaw2/pitch2/roll2 - IMU2 Euler angles in degrees
        ax2/ay2/az2       - IMU2 acceleration in m/s²
        imu1_ok / imu2_ok - health flags
    """
    from types import SimpleNamespace

    try:
        t = time.perf_counter()

        # Acceleration in m/s² (already in correct units from STM32 firmware)
        accel1 = np.array([stm_sample.ax1, stm_sample.ay1, stm_sample.az1], dtype=float)
        accel2 = np.array([stm_sample.ax2, stm_sample.ay2, stm_sample.az2], dtype=float)

        # Gyro not available in BNO085 RVC mode — use zeros
        gyro1 = np.zeros(3, dtype=float)
        gyro2 = np.zeros(3, dtype=float)

        # Euler angles (yaw, pitch, roll) in degrees from BNO085 RVC
        euler1 = (float(stm_sample.yaw1), float(stm_sample.pitch1), float(stm_sample.roll1))
        euler2 = (float(stm_sample.yaw2), float(stm_sample.pitch2), float(stm_sample.roll2))

        # Health derived from ok flags
        health = SimpleNamespace(
            imu1_online=bool(stm_sample.imu1_ok),
            imu2_online=bool(stm_sample.imu2_ok),
            imu1_zero_data=False,
            imu2_zero_data=False,
        )

        reading = SimpleNamespace(
            accel1=accel1,
            gyro1=gyro1,
            accel2=accel2,
            gyro2=gyro2,
            quat1=None,
            quat2=None,
            euler1=euler1,
            euler2=euler2,
            ok1=bool(stm_sample.imu1_ok),
            ok2=bool(stm_sample.imu2_ok),
            health=health,
            t_us=int(stm_sample.t_ms * 1000),
            seq=0,
        )

        return SimpleNamespace(t=t, reading=reading, t_hardware_s=stm_sample.t_ms / 1000.0)

    except Exception:
        return None


# =============================================================================
# TRIAL TMSI THREAD  (mirrors RawTMSiThread in signal_acquisition_testing.py)
# =============================================================================

class TrialTMSiThread(threading.Thread):
    """
    Robust Porti7 acquisition thread with retry + 'USB -' filtering.
    Mirrors the RawTMSiThread pattern from signal_acquisition_testing.py.

    Delivers EMGChunk-like SimpleNamespace objects to `on_emg` callback:
        chunk.t            - midpoint timestamp (perf_counter)
        chunk.sample_t     - per-sample timestamps (np.ndarray)
        chunk.data['raw']  - raw samples (n_samples x n_channels)
    """

    MAX_DISCOVERY_RETRIES = 5
    MAX_OPEN_RETRIES = 3

    def __init__(
        self,
        sample_rate: int = 2048,
        raw_channels: Optional[List[int]] = None,
        estimator=None,
        on_emg=None,
    ):
        super().__init__(name="TrialTMSiThread", daemon=True)
        self.sample_rate = sample_rate
        self.raw_channels = raw_channels  # None = keep all channels
        self.estimator = estimator
        self.on_emg = on_emg
        self.running = False
        self.error: Optional[str] = None
        self.channels: List[str] = []
        self.trig_idx: int = -1
        self.estimated_rate_hz: float = 0.0
        self._device = None
        self._measurement = None

    def run(self):
        from types import SimpleNamespace
        if not _TMSI_AVAILABLE:
            self.error = "TMSi SDK not available"
            print("[TMSi] SDK not available")
            return
        try:
            # --- 1. Discover with retries, filtering 'USB -' placeholders ---
            self._device = None
            for attempt in range(self.MAX_DISCOVERY_RETRIES):
                print(f"[TMSi] Discovery attempt {attempt + 1}/{self.MAX_DISCOVERY_RETRIES}...")
                try:
                    LegacyDevice.cleanup()
                except Exception:
                    pass
                devices = LegacyDevice.discover("usb")
                valid = [
                    d for d in devices
                    if d._device_name and len(d._device_name) > 5
                    and "-" not in d._device_name[4:]
                ]
                if valid:
                    self._device = valid[0]
                    break
                if devices:
                    print(f"[TMSi] Found {len(devices)} device(s) but name invalid "
                          f"(e.g. '{devices[0]._device_name}'). Retrying...")
                else:
                    print("[TMSi] No devices found. Retrying...")
                if attempt < self.MAX_DISCOVERY_RETRIES - 1:
                    time.sleep(2.0)

            if not self._device:
                raise RuntimeError("Failed to find a valid TMSi device after "
                                   f"{self.MAX_DISCOVERY_RETRIES} attempts.")

            # --- 2. Open with retries ---
            opened = False
            for attempt in range(self.MAX_OPEN_RETRIES):
                try:
                    print(f"[TMSi] Opening {self._device._device_name} "
                          f"(attempt {attempt + 1})...")
                    self._device.open()
                    opened = True
                    break
                except Exception as e:
                    print(f"[TMSi] Open failed: {e}")
                    if attempt < self.MAX_OPEN_RETRIES - 1:
                        time.sleep(1.5)
            if not opened:
                raise RuntimeError(f"Failed to open {self._device._device_name}")

            # --- 3. Find channels ---
            self.channels = [
                ch.get_channel_name() for ch in self._device.get_device_channels()
            ]
            for i, name in enumerate(self.channels):
                if name.lower() in ("dig", "status", "trig"):
                    self.trig_idx = i
                    break
            if self.trig_idx == -1:
                print("[TMSi] WARNING: trigger channel not found in:", self.channels)
            else:
                print(f"[TMSi] Trigger channel: '{self.channels[self.trig_idx]}'")

            # --- 4. Start measurement ---
            self._measurement = SignalMeasurement(self._device)
            self._measurement.set_sample_rate(self.sample_rate)
            self._measurement.start()
            self.running = True
            print(f"[TMSi] Acquisition started @ {self.sample_rate} Hz")

            dt = 1.0 / self.sample_rate
            last_print = time.perf_counter()
            count = 0

            while self.running:
                samples = self._measurement.get_samples(blocking=True)
                if samples is None or len(samples) == 0:
                    continue

                now = time.perf_counter()
                n = len(samples)
                count += n

                # Per-sample timestamps (uniform spacing anchored to receive time)
                t_arr = now - (n - 1 - np.arange(n)) * dt

                # Feed PRBS estimator (last channel = trigger if available)
                if self.estimator is not None and self.trig_idx != -1:
                    try:
                        trig_vals = samples[:, self.trig_idx]
                        self.estimator.add_emg_trig_chunk(t_arr, trig_vals)
                    except Exception:
                        pass

                # Select channels
                if self.raw_channels:
                    raw = samples[:, self.raw_channels]
                else:
                    raw = samples

                # Deliver EMGChunk-compatible SimpleNamespace
                if self.on_emg:
                    try:
                        chunk = SimpleNamespace(
                            t=float(t_arr[n // 2]),
                            sample_t=t_arr,
                            data={'raw': raw},
                            counter_raw_last=None,
                        )
                        self.on_emg(chunk)
                    except Exception:
                        pass

                # Rate tracking (1s smoothing)
                if now - last_print >= 1.0:
                    self.estimated_rate_hz = count / (now - last_print)
                    count = 0
                    last_print = now

        except Exception as e:
            self.error = str(e)
            print(f"[TMSi] Error: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop acquisition (mirrors RawTMSiThread.stop())."""
        self.running = False
        if self._measurement:
            try:
                self._measurement.stop()
            except Exception:
                pass
        if self._device:
            try:
                self._device.close()
            except Exception:
                pass


class TrialSTM32Thread(threading.Thread):
    """Direct binary acquisition from STM32, ported from signal_acquisition_testing.py."""
    def __init__(self, port: str, baud: int = 921600, estimator=None, on_sample=None):
        super().__init__(name="TrialSTM32Thread", daemon=True)
        self.port = port
        self.baud = baud
        self.estimator = estimator
        self.on_sample = on_sample
        self.running = False
        self.error: Optional[str] = None
        self.sample_count = 0
        self.snapshot_buffer = deque(maxlen=int(5.0 * 550)) # 5 seconds at ~550 Hz

    def run(self):
        if serial is None:
            self.error = "pyserial not installed"
            return
            
        self.running = True
        buffer = bytearray()
        ser = None
        last_rx = time.perf_counter()

        while self.running:
            if ser is None or not ser.is_open:
                try:
                    if ser is not None:
                        try: ser.close()
                        except Exception: pass
                    
                    print(f"[STM32] Connecting to {self.port} at {self.baud}...")
                    ser = serial.Serial(self.port, self.baud, timeout=0.1)
                    buffer.clear()
                    last_rx = time.perf_counter()
                    print(f"[STM32] Connected to {self.port}")
                except Exception as e:
                    self.error = f"Connection failed: {e}"
                    time.sleep(1.0)
                    continue

            if time.perf_counter() - last_rx > 3.0:
                print("[STM32] Stall detected (>3s no data), reconnecting...")
                try: ser.close()
                except Exception: pass
                ser = None
                continue

            try:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    time.sleep(0.001)
                    continue
                
                last_rx = time.perf_counter()
                buffer.extend(chunk)
                
                while len(buffer) >= BIN_PACKET_SIZE:
                    idx = buffer.find(bytes([BIN_SYNC1, BIN_SYNC2]))
                    if idx == -1:
                        if len(buffer) > 0 and buffer[-1] == BIN_SYNC1:
                            buffer = buffer[-1:]
                        else:
                            buffer.clear()
                        break
                    
                    if idx > 0:
                        buffer = buffer[idx:]
                    if len(buffer) < BIN_PACKET_SIZE:
                        break
                    
                    packet = buffer[:BIN_PACKET_SIZE]
                    cs = 0
                    for b in packet[2:-1]:
                        cs ^= b
                    
                    if cs != packet[-1]:
                        buffer = buffer[2:]
                        continue

                    buffer = buffer[BIN_PACKET_SIZE:]
                    
                    try:
                        vals = struct.unpack(BIN_FORMAT, packet)
                        sample = SampleSTM32(
                            t_ms=float(vals[2]),
                            imu1_ok=(vals[3] & 1),
                            imu2_ok=((vals[3] >> 1) & 1),
                            yaw1=vals[4], pitch1=vals[5], roll1=vals[6], ax1=vals[7], ay1=vals[8], az1=vals[9],
                            yaw2=vals[10], pitch2=vals[11], roll2=vals[12], ax2=vals[13], ay2=vals[14], az2=vals[15],
                            keys_mask=int(vals[16]),
                            keys_rise=int(vals[17]),
                            keys_fall=int(vals[18]),
                            prbs_tick=int(vals[19]),
                            prbs_lvl=(vals[20] & 1),
                            in_mark=((vals[20] >> 1) & 1)
                        )
                        
                        self.sample_count += 1
                        
                        # Add to snapshot buffer
                        self.snapshot_buffer.append({
                            "pc_time": time.perf_counter(),
                            "t_ms": sample.t_ms,
                            "prbs_level": sample.prbs_lvl,
                            "keys_mask": sample.keys_mask,
                            "imu1_ok": sample.imu1_ok,
                            "imu2_ok": sample.imu2_ok,
                        })

                        if self.estimator:
                            self.estimator.add_stm32_samples([sample])
                        
                        if self.on_sample:
                            self.on_sample(sample)
                            
                    except Exception as e:
                        print(f"[STM32] Parse error: {e}")
                        continue
                        
            except (serial.SerialException, OSError) as e:
                print(f"[STM32] Serial error: {e}")
                ser = None
                time.sleep(1.0)
                
        if ser is not None:
            try: ser.close()
            except Exception: pass

    def stop(self):
        self.running = False

    def get_snapshot(self) -> Dict[str, np.ndarray]:
        """Returns a dictionary of numpy arrays for the current snapshot buffer."""
        if not self.snapshot_buffer:
            return {}
        
        # Convert deque of dicts to dict of lists, then to dict of numpy arrays
        snapshot_dict = {key: [d[key] for d in self.snapshot_buffer] for key in self.snapshot_buffer[0]}
        
        # Convert lists to numpy arrays
        for key, value in snapshot_dict.items():
            snapshot_dict[key] = np.array(value)
        
        return snapshot_dict


# =============================================================================
# TRIAL MANAGER
# =============================================================================

class TrialManager:
    """
    Main trial orchestration class.
    
    Manages:
    - Device initialization and synchronization
    - Trial state machine
    - Real-time data collection and preprocessing
    - GUI updates
    - Data storage
    """
    
    def __init__(self, config_module=None, mock_mode: bool = False):
        """
        Initialize trial manager.
        
        Args:
            config_module: Configuration module (default: setup_trial)
            mock_mode: If True, run without hardware (for testing)
        """
        # Configuration
        self.config = config_module or setup_trial
        self.mock_mode = mock_mode or (not _TMSI_AVAILABLE and DummyAcquisition is not None)
        
        if self.mock_mode:
            print("⚠ Running in MOCK MODE (no hardware required)")
        
        # Validate configuration
        self.config.validate_config()
        
        # State
        self.state = TrialState.IDLE
        self.current_trial_idx = 0
        self.trial_start_time = None
        self.countdown_start = None
        self.recording_data = False
        
        # Protocol
        self.protocol = self._load_protocol()
        self.expanded_protocol = self._expand_protocol(self.protocol)
        
        # Data buffers for current trial
        self.emg_buffer_times = []
        self.emg_buffer_raw = []
        self.emg_buffer_preprocessed = []
        self.imu_buffer_times = []
        self.imu_buffer_data = []
        
        # Devices
        self.acquisition = None   # DummyAcquisition (mock mode only)
        self._tmsi_thread = None  # TrialTMSiThread (real hardware)
        self._stm32_thread = None # TrialSTM32Thread (real hardware)
        self.estimator = None
        self.preprocessor = None
        self.storage = None
        self.gui = None
        
        # Statistics
        self.stats = {
            'emg_samples_received': 0,
            'imu_samples_received': 0,
            'emg_sample_rate_measured': 0.0,
            'imu_sample_rate_measured': 0.0,
            'last_emg_time': None,
            'last_imu_time': None,
        }
        
        # Thread control
        self.running = False
        self.data_thread = None
        self.buffer_lock = threading.Lock()
        self.scheduled_callbacks = []  # Track all scheduled callbacks for cleanup
        # Tkinter is NOT thread-safe: all GUI updates must run on main thread.
        # We therefore schedule periodic manager updates via root.after().
        self._manager_update_interval_ms = 33 # Faster update for smoother plots
        # Throttle plot updates: acquisition callbacks can fire very frequently.
        # We coalesce updates so Tk doesn't get flooded with after_idle callbacks.
        self._pending_emg_plot = None  # (timestamps, data)
        self._pending_imu_plot = None  # (timestamps, accel, gyro)
        self._emg_plot_update_scheduled = False
        self._imu_plot_update_scheduled = False
        self._pending_imu_health = None
        self._imu_health_update_scheduled = False
        self._last_emg_rate_t = time.perf_counter()
        self._emg_samples_since_rate = 0
        self._last_imu_rate_t = time.perf_counter()
        self._imu_samples_since_rate = 0

        # Quit coordination
        # If quit is requested while recording/saving, we finish saving the current trial first.
        self._quit_requested = False
        self._quit_after_save = False
        
        print(f"\n✓ Trial Manager initialized")
        print(f"  Participant: {self.config.PARTICIPANT_ID}")
        print(f"  Session: {self.config.SESSION_ID}")
        print(f"  Protocol: {len(self.expanded_protocol)} trials")
    
    def _load_protocol(self) -> List[Dict[str, Any]]:
        """Load trial protocol."""
        return self.config.TRIAL_EXERCISES

    def _expand_protocol(self, protocol: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Expand protocol by repeating exercises according to 'repetitions' field.
        """
        expanded = []
        for exercise in protocol:
            repetitions = exercise.get('repetitions', 1)
            for rep in range(repetitions):
                ex_copy = exercise.copy()
                ex_copy['repetition'] = rep + 1
                ex_copy['total_repetitions'] = repetitions
                if repetitions > 1:
                    ex_copy['display_name'] = f"{exercise.get('display_name', exercise.get('name', ''))} (Rep {rep + 1}/{repetitions})"
                else:
                    ex_copy['display_name'] = exercise.get('display_name', exercise.get('name', ''))
                expanded.append(ex_copy)
        return expanded

    def initialize(self):
        """Initialize all components."""
        print("\n" + "="*70)
        print("INITIALIZING TRIAL SYSTEM")
        print("="*70)
        
        # Create session directory
        session_dir = self.config.get_session_dir()
        session_dir.mkdir(parents=True, exist_ok=True)
        print(f"\n✓ Session directory: {session_dir}")
        
        # Initialize storage
        self.storage = TrialDataStorage(
            session_dir=session_dir,
            compression=self.config.COMPRESSION,
            compression_level=self.config.COMPRESSION_LEVEL
        )
        
        # Set session metadata
        self.storage.set_session_metadata(
            participant_info=self.config.PARTICIPANT_INFO,
            session_info=self.config.SESSION_INFO,
            emg_config=self.config.EMG_CONFIG,
            imu_config=self.config.IMU_CONFIG,
            preprocessing_config=self.config.PREPROCESSING
        )
        
        # Save session metadata file
        self.storage.save_session_metadata()
        
        # Initialize GUI
        print("\n✓ Creating GUI...")
        self.gui = TrialGUI(
            config=self.config.GUI_CONFIG,
            on_key_press=self._handle_key_press,
            on_test_hardware=self._handle_test_hardware
        )
        
        # Set initial GUI state
        self.gui.set_state(TrialState.IDLE)
        self.gui.set_trial_info(0, len(self.expanded_protocol))
        
        # Initialize acquisition
        print("\n✓ Initializing acquisition system...")
        use_dummy = self.config.SYNC_CONFIG.get('use_dummy_signals', False) or self.mock_mode

        if use_dummy:
            # Initialize with dummy acquisition
            print("  Using DUMMY signals (simulated)")
            self.acquisition = DummyAcquisition(
                emg_sample_rate=self.config.EMG_CONFIG.get('sample_rate', 2048),
                imu_sample_rate=200,
                emg_channels=len(self.config.EMG_CONFIG.get('raw_channels', [0, 1, 2, 3])),
                emg_amplitude=self.config.SYNC_CONFIG.get('dummy_emg_amplitude', 50.0),
                emg_noise_level=self.config.SYNC_CONFIG.get('dummy_emg_noise_level', 5.0),
                imu_motion=self.config.SYNC_CONFIG.get('dummy_imu_motion', True),
                on_emg=self._on_emg_chunk,
                on_imu=self._on_imu_sample
            )
            self.acquisition.start()
        else:
            # Try real hardware acquisition
            try:
                # Use real hardware acquisition
                print("  Using REAL hardware signals")
                print("  Connecting to devices (this may take up to 30 seconds)...")
                
                # --- STM32 (IMU/PRBS) thread ---
                from src.arduino_connection import find_arduino_port
                stm32_port = self.config.SYNC_CONFIG.get('stm32_port')
                if not stm32_port and find_arduino_port:
                    stm32_port = find_arduino_port()
                
                if not stm32_port:
                    raise RuntimeError("STM32 port not found (and not set in SYNC_CONFIG)")

                # --- PRBS Sync Estimator (created before TMSi thread so both can share it) ---
                if SyncDelayEstimator is not None:
                    try:
                        emg_sr = float(self.config.EMG_CONFIG.get('sample_rate', 2048))
                        self.estimator = SyncDelayEstimator(
                            emg_sample_rate=emg_sr,
                            sync_window_s=0.5,
                            sync_step_s=0.1,
                        )
                        self.estimator._trig_is_status = True
                        self.estimator._detected_trig_bit = 0
                        print("  \u2713 SyncDelayEstimator started")
                    except Exception as e:
                        print(f"  \u2717 Estimator failed to start: {e}")

                self._stm32_thread = TrialSTM32Thread(
                    port=stm32_port,
                    baud=921600,
                    estimator=self.estimator,
                    on_sample=self._on_stm32_sample
                )
                self._stm32_thread.start()
                print(f"  \u2713 STM32 Reader thread started on {stm32_port}")

                # --- TMSi (Porti7) EMG thread with retry + USB-placeholder filter ---
                self._tmsi_thread = TrialTMSiThread(
                    sample_rate=self.config.EMG_CONFIG.get('sample_rate', 2048),
                    raw_channels=self.config.EMG_CONFIG.get('raw_channels', None),
                    estimator=self.estimator,
                    on_emg=self._on_emg_chunk,
                )
                self._tmsi_thread.start()

                # Wait up to 30 s for TMSi to either connect or fail hard
                ready_timeout = self.config.SYNC_CONFIG.get('ready_timeout_s', 30.0)
                t0 = time.perf_counter()
                while not self._tmsi_thread.running and not self._tmsi_thread.error:
                    if time.perf_counter() - t0 > ready_timeout:
                        break
                    time.sleep(0.2)

                if self._tmsi_thread.error:
                    raise RuntimeError(f"TMSi thread: {self._tmsi_thread.error}")

                print("  \u2713 TMSi EMG thread started")
                print("  \u2713 Acquisition started successfully")

                
            except TimeoutError as e:
                print(f"  ✗ Hardware connection timeout: {e}")
                print("  ✗ Make sure EMG and IMU devices are connected and powered on")
                print("  Falling back to DUMMY mode")
                self.mock_mode = True
                # Initialize with dummy instead
                print("  Initializing dummy acquisition...")
                self.acquisition = DummyAcquisition(
                    emg_sample_rate=self.config.EMG_CONFIG.get('sample_rate', 2048),
                    imu_sample_rate=200,
                    emg_channels=len(self.config.EMG_CONFIG.get('raw_channels', [0, 1, 2, 3])),
                    emg_amplitude=self.config.SYNC_CONFIG.get('dummy_emg_amplitude', 50.0),
                    emg_noise_level=self.config.SYNC_CONFIG.get('dummy_emg_noise_level', 5.0),
                    imu_motion=self.config.SYNC_CONFIG.get('dummy_imu_motion', True),
                    on_emg=self._on_emg_chunk,
                    on_imu=self._on_imu_sample
                )
                self.acquisition.start()
                
            except Exception as e:
                print(f"  ✗ Failed to initialize acquisition: {e}")
                import traceback
                traceback.print_exc()
                self.mock_mode = True
                print("  Falling back to DUMMY mode")
                # Initialize with dummy instead
                print("  Initializing dummy acquisition...")
                self.acquisition = DummyAcquisition(
                    emg_sample_rate=self.config.EMG_CONFIG.get('sample_rate', 2048),
                    imu_sample_rate=200,
                    emg_channels=len(self.config.EMG_CONFIG.get('raw_channels', [0, 1, 2, 3])),
                    emg_amplitude=self.config.SYNC_CONFIG.get('dummy_emg_amplitude', 50.0),
                    emg_noise_level=self.config.SYNC_CONFIG.get('dummy_emg_noise_level', 5.0),
                    imu_motion=self.config.SYNC_CONFIG.get('dummy_imu_motion', True),
                    on_emg=self._on_emg_chunk,
                    on_imu=self._on_imu_sample
                )
                self.acquisition.start()
        
        # Initialize preprocessor
        if self.config.PREPROCESSING:
            self.preprocessor = EMGPreprocessor(
                fs=self.config.EMG_CONFIG['sample_rate'] or 2048,
                mvc_value=self.config.PREPROCESSING.get('mvc_value'),
                bandpass_low=self.config.PREPROCESSING['bandpass_low'],
                bandpass_high=self.config.PREPROCESSING['bandpass_high'],
                notch_freq=self.config.PREPROCESSING.get('notch_freq'),
                envelope_cutoff=self.config.PREPROCESSING['envelope_cutoff']
            )
            print(f"\n✓ Preprocessor initialized: {self.preprocessor}")
        
        # Set ready state
        self._set_state(TrialState.READY)
        self._update_gui_for_current_trial()
        
        print("\n" + "="*70)
        print("SYSTEM READY")
        print("="*70)
        print("\nPress SPACE to start first trial")
        print("Press T to test hardware (launch visualization)")
        print("Press C to calibrate IMU")
        print("Press Q to quit")
        print("="*70 + "\n")
    
    def _handle_key_press(self, key: str):
        """Handle keyboard input from GUI."""
        if key == 'space':
            self._handle_space()
        elif key == 'n':
            self._handle_next_trial()
        elif key == 'r':
            self._handle_repeat_trial()
        elif key == 'c':
            self._handle_calibrate()
        elif key == 'q' or key == 'escape':
            self._handle_quit()
    
    def _handle_space(self):
        """Handle SPACE key (start/stop recording)."""
        if self.state == TrialState.READY:
            # Start trial
            self._start_trial()
        
        elif self.state == TrialState.RECORDING:
            # Stop trial early
            self._stop_trial()
    
    def _handle_next_trial(self):
        """Handle N key (next trial)."""
        if self.state in [TrialState.READY, TrialState.IDLE]:
            if self.current_trial_idx < len(self.expanded_protocol):
                self.current_trial_idx += 1
                if self.current_trial_idx < len(self.expanded_protocol):
                    self._set_state(TrialState.READY)
                    self._update_gui_for_current_trial()
                else:
                    self._set_state(TrialState.COMPLETED)
                    self.gui.set_exercise("All Trials Complete!", 
                                         "Thank you for participating!")
    
    def _handle_repeat_trial(self):
        """Handle R key (repeat current trial)."""
        if self.state == TrialState.READY and self.current_trial_idx > 0:
            self.current_trial_idx -= 1
            self._update_gui_for_current_trial()
    
    def _handle_calibrate(self):
        """Handle C key (calibrate IMU)."""
        if self.state != TrialState.READY:
            return

        # In dummy-signal mode there's no real IMU to calibrate.
        if self.config.SYNC_CONFIG.get('use_dummy_signals', False):
            try:
                self.gui.show_message("IMU Calibration", "Calibration is not available while using dummy signals.")
            except Exception:
                pass
            return

        if not self.mock_mode:
            self._calibrate_imu()
    
    def _handle_test_hardware(self):
        """Handle T key / Test Hardware button (launch visualization tool)."""
        # Only allow when in READY or IDLE state
        if self.state not in [TrialState.READY, TrialState.IDLE]:
            self.gui.show_message("Hardware Test", 
                                 "Cannot test hardware during a trial.\n"
                                 "Please wait for the current trial to complete.")
            return
        
        # Check if using dummy signals
        use_dummy = self.config.SYNC_CONFIG.get('use_dummy_signals', False)
        
        # Launch visualization in a separate thread/process
        import subprocess
        import sys
        
        try:
            self.gui.set_status("Launching hardware test visualization...")
            
            # Build command
            cmd = [sys.executable, '-m', 'src.synchronized_visualization']
            if use_dummy:
                cmd.append('--dummy')
            
            # Launch in separate process (non-blocking)
            subprocess.Popen(
                cmd,
                cwd=str(Path(__file__).parent.parent),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.gui.show_message("Hardware Test Launched", 
                                 "A separate window will open showing real-time signals.\n\n"
                                 "This allows you to:\n"
                                 "• Verify EMG electrode connections\n"
                                 "• Check IMU sensor readings\n"
                                 "• Ensure proper signal quality\n\n"
                                 "Close that window when done to return here.")
            
        except Exception as e:
            self.gui.show_error("Launch Error", 
                               f"Failed to launch hardware test:\n{e}\n\n"
                               f"Try running manually:\n"
                               f"python -m src.synchronized_visualization")
    
    def _handle_quit(self):
        """Handle Q key (quit)."""
        if self.gui.ask_yes_no("Quit", "Are you sure you want to quit?"):
            self._request_quit()

    def _request_quit(self):
        """
        Graceful quit:
        - If recording: stop + save current trial, then quit
        - If saving: quit after save finishes
        - Otherwise: quit immediately
        """
        self._quit_requested = True

        # If we're already saving, just exit once save thread completes.
        if self.state == TrialState.SAVING:
            self._quit_after_save = True
            try:
                self.gui.set_status("Quitting after saving current trial...")
            except Exception:
                pass
            return

        # If currently recording, stop (which triggers async save thread) then quit.
        if self.state == TrialState.RECORDING:
            self._quit_after_save = True
            try:
                self.gui.set_status("Stopping and saving current trial before quitting...")
            except Exception:
                pass
            self._stop_trial()
            return

        # If we're in countdown, we won't have a complete trial yet.
        # Just close; cleanup will stop acquisition and cancel callbacks.
        self.running = False
        try:
            self.gui.request_close.emit()
        except Exception:
            pass
    
    def _start_trial(self):
        """Start recording current trial."""
        if self.state != TrialState.READY:
            return
        
        print(f"\n{'='*70}")
        print(f"Starting Trial {self.current_trial_idx + 1}/{len(self.expanded_protocol)}")
        print(f"{'='*70}")
        
        # Get current exercise
        exercise = self.expanded_protocol[self.current_trial_idx]
        print(f"Exercise: {exercise['display_name']}")
        print(f"Duration: {exercise['duration']}s")
        
        # Start countdown (non-blocking)
        self._set_state(TrialState.COUNTDOWN)
        self.countdown_start = time.time()
        self._countdown_step(self.config.COUNTDOWN_DURATION)
    
    def _countdown_step(self, remaining: int):
        """Non-blocking countdown step."""
        if remaining <= 0:
            # Countdown complete, start recording
            self._begin_recording()
            return
        
        # Update countdown display
        exercise = self.expanded_protocol[self.current_trial_idx]
        self.gui.set_exercise(exercise['display_name'], f"Starting in {remaining}...")
        
        # Schedule next countdown step
        timer = self.gui.schedule_task(1000, lambda: self._countdown_step(remaining - 1))
        self.scheduled_callbacks.append(timer)
    
    def _begin_recording(self):
        """Begin recording after countdown."""
        exercise = self.expanded_protocol[self.current_trial_idx]
        
        # Clear buffers
        self._clear_trial_buffers()
        self.gui.clear_plots()
        # Reset filter states per trial. We preprocess after recording (in the save thread)
        # to avoid heavy CPU load during recording that can stall the GUI.
        if self.preprocessor:
            try:
                self.preprocessor.reset()
            except Exception:
                pass
        
        # Start recording
        self._set_state(TrialState.RECORDING)
        self.trial_start_time = time.time()
        self.recording_data = True
        
        self.gui.set_exercise(exercise['display_name'], exercise['instruction'])
        print(f"Recording started at {datetime.now().strftime('%H:%M:%S')}")
        
        # Schedule auto-stop
        duration = exercise['duration']
        timer = self.gui.schedule_task(int(duration * 1000), self._stop_trial)
        self.scheduled_callbacks.append(timer)
    
    def _stop_trial(self):
        """Stop recording and save trial."""
        if self.state != TrialState.RECORDING:
            return
        
        self.recording_data = False
        trial_duration = time.time() - self.trial_start_time
        
        print(f"\nRecording stopped (duration: {trial_duration:.2f}s)")
        
        # Save trial in background thread to prevent GUI freeze
        self._set_state(TrialState.SAVING)
        self.gui.set_exercise("Saving Data", "Please wait...")
        self.gui.set_status("Saving trial data to disk...")
        
        # Run save in background thread
        def save_and_continue():
            try:
                self._save_current_trial()
                print("✓ Trial saved successfully")
                
                # Schedule GUI updates in main thread
                # PyQt can use QTimer.singleShot for deferred execution on the main thread
                self.gui.schedule_task(0, self._after_save_success)
                
            except Exception as e:
                print(f"✗ Error saving trial: {e}")
                import traceback
                traceback.print_exc()
                
                # Schedule error dialog in main thread
                self.gui.schedule_task(0, lambda e_in=e: self._after_save_error(e_in))
        
        # Start save thread
        save_thread = threading.Thread(target=save_and_continue, daemon=True)
        save_thread.start()
    
    def _after_save_success(self):
        """Called in main thread after successful save."""
        # If user requested quit during recording/saving, exit cleanly now.
        if self._quit_after_save:
            self.running = False
            try:
                self.gui.request_close.emit()
            except Exception:
                pass
            return

        # Move to next trial
        self.current_trial_idx += 1
        
        if self.current_trial_idx < len(self.expanded_protocol):
            # More trials remaining
            self._set_state(TrialState.READY)
            
            # Rest period
            exercise = self.expanded_protocol[self.current_trial_idx - 1]
            rest_duration = exercise.get('rest_after', 0)
            if rest_duration > 0:
                print(f"\nRest period: {rest_duration}s")
                self.gui.set_exercise("Rest", f"Relax for {rest_duration} seconds")
                self.gui.set_status(f"Rest period: {rest_duration}s")
                
                # Schedule end of rest period
                def end_rest():
                    self._update_gui_for_current_trial()
                
                timer = self.gui.schedule_task(int(rest_duration * 1000), end_rest)
                self.scheduled_callbacks.append(timer)
            else:
                self._update_gui_for_current_trial()
        else:
            # All trials complete
            self._set_state(TrialState.COMPLETED)
            self.gui.set_exercise("All Trials Complete!", 
                                 "Thank you for participating!\n\nPress Q to quit")
            self.gui.set_status("All trials completed successfully!")
            print(f"\n{'='*70}")
            print("ALL TRIALS COMPLETED")
            print(f"{'='*70}\n")
    
    def _after_save_error(self, error):
        """Called in main thread after save error."""
        self.gui.show_error("Save Error", f"Failed to save trial:\n{error}")

        # If user requested quit, exit even if save failed (raw file may still exist).
        if self._quit_after_save:
            self.running = False
            try:
                self.gui.request_close.emit()
            except Exception:
                pass
            return
        
        # Still advance to prevent getting stuck
        self.current_trial_idx += 1
        
        if self.current_trial_idx < len(self.expanded_protocol):
            self._set_state(TrialState.READY)
            self._update_gui_for_current_trial()
        else:
            self._set_state(TrialState.COMPLETED)
            self.gui.set_exercise("Session Complete (with errors)", 
                                 "Some trials failed to save.\nCheck terminal for details.")
            self.gui.set_status("Completed with errors - check terminal")
    
    def _save_current_trial(self):
        """Save current trial data to HDF5."""
        trial_num = self.current_trial_idx
        exercise = self.expanded_protocol[trial_num]
        
        print(f"\nSaving trial {trial_num + 1}...")
        
        # CRITICAL: Copy data from buffers QUICKLY under lock, then release
        # Heavy processing (vstacking, dict building) happens OUTSIDE lock
        with self.buffer_lock:
            # Just copy the lists - very fast
            emg_times_copy = list(self.emg_buffer_times)
            emg_raw_copy = list(self.emg_buffer_raw)
            imu_times_copy = list(self.imu_buffer_times)
            imu_data_copy = list(self.imu_buffer_data)
        # Lock released immediately - callbacks can continue
        
        # Prepare EMG data (heavy operation, no lock needed)
        emg_data = None
        if len(emg_raw_copy) > 0 and len(emg_times_copy) > 0:
            emg_data = {
                'timestamps': np.array(emg_times_copy),
                'raw': np.vstack(emg_raw_copy),
                'sample_rate': self.stats['emg_sample_rate_measured'] or 2048,
                'channel_names': self.config.EMG_CONFIG.get('channel_names', 
                                                            [f'CH{i}' for i in range(4)]),
            }
            print(f"  EMG: {emg_data['raw'].shape[0]} samples")
        else:
            print(f"  EMG: No data collected")
        
        # Prepare IMU data (heavy operation, no lock needed)
        imu_data = None
        if len(imu_data_copy) > 0 and len(imu_times_copy) > 0:
            imu_dict = self._consolidate_imu_buffer_from_copy(imu_data_copy)
            imu_data = {
                'timestamps': np.array(imu_times_copy),
                'accel1': imu_dict['accel1'],
                'gyro1': imu_dict['gyro1'],
                'accel2': imu_dict['accel2'],
                'gyro2': imu_dict['gyro2'],
                'quat1': imu_dict.get('quat1'),
                'quat2': imu_dict.get('quat2'),
                'euler1': imu_dict.get('euler1'),
                'euler2': imu_dict.get('euler2'),
                'sample_rate': self.stats['imu_sample_rate_measured'] or 200,
            }
            print(f"  IMU: {len(imu_times_copy)} samples")
        else:
            print(f"  IMU: No data collected")
        
        # Synchronization info
        sync_info = {
            'method': 'hardware_timestamps',
            'emg_samples': len(emg_times_copy),
            'imu_samples': len(imu_times_copy),
        }
        
        # Exercise info
        exercise_info = {
            'name': exercise['name'],
            'display_name': exercise['display_name'],
            'duration': exercise['duration'],
            'instruction': exercise['instruction'],
            'timestamp_start': datetime.fromtimestamp(self.trial_start_time).isoformat(),
            'timestamp_end': datetime.now().isoformat(),
        }
        
        # Save raw data
        if self.config.SAVE_RAW:
            self.storage.save_trial_raw(
                trial_number=trial_num + 1,
                exercise_name=exercise['name'],
                exercise_info=exercise_info,
                emg_data=emg_data,
                imu_data=imu_data,
                sync_info=sync_info
            )
        
        # Save preprocessed data
        # IMPORTANT: preprocess here (save thread), not during recording.
        if self.config.SAVE_PREPROCESSED and self.preprocessor and emg_data is not None:
            emg_preprocessed = self._prepare_preprocessed_emg_from_raw(emg_times_copy, emg_data['raw'])
            self.storage.save_trial_preprocessed(
                trial_number=trial_num + 1,
                exercise_name=exercise['name'],
                emg_preprocessed=emg_preprocessed,
                imu_data=imu_data
            )
    
    def _prepare_preprocessed_emg(self) -> Dict[str, Any]:
        """Prepare preprocessed EMG data dictionary (called under lock)."""
        return self._prepare_preprocessed_emg_from_copy(self.emg_buffer_times, self.emg_buffer_preprocessed)
    
    def _prepare_preprocessed_emg_from_copy(self, times_copy: List, prep_copy: List) -> Dict[str, Any]:
        """Prepare preprocessed EMG from copies (no lock needed)."""
        # Stack all preprocessed data
        all_stages = {}
        for item in prep_copy:
            for key, value in item.items():
                if key not in all_stages:
                    all_stages[key] = []
                all_stages[key].append(value)
        
        # Convert to arrays
        result = {
            'timestamps': np.array(times_copy),
            'sample_rate': self.stats['emg_sample_rate_measured'] or 2048,
            'channel_names': self.config.EMG_CONFIG.get('channel_names', 
                                                        [f'CH{i}' for i in range(4)]),
        }
        
        for key, value_list in all_stages.items():
            if len(value_list) > 0:
                result[key] = np.vstack(value_list)
        
        return result

    def _prepare_preprocessed_emg_from_raw(self, times_copy: List, raw_array: np.ndarray) -> Dict[str, Any]:
        """
        Prepare preprocessed EMG dict from the full raw array.
        Runs in the save thread (no locks), so it doesn't affect GUI responsiveness.
        """
        processed = self.preprocessor.process(raw_array, return_all_stages=True)
        return {
            'timestamps': np.array(times_copy),
            'filtered': processed.get('filtered'),
            'envelope': processed.get('envelope'),
            'normalized': processed.get('normalized'),
            'sample_rate': self.stats['emg_sample_rate_measured'] or 2048,
            'channel_names': self.config.EMG_CONFIG.get(
                'channel_names',
                [f'CH{i}' for i in range(raw_array.shape[1] if raw_array.ndim == 2 else 1)]
            ),
        }
    
    def _consolidate_imu_buffer(self) -> Dict[str, np.ndarray]:
        """Consolidate IMU buffer into arrays (called under lock)."""
        return self._consolidate_imu_buffer_from_copy(self.imu_buffer_data)
    
    def _consolidate_imu_buffer_from_copy(self, imu_data_copy: List) -> Dict[str, np.ndarray]:
        """Consolidate IMU buffer from a copy (no lock needed)."""
        result = {
            'accel1': [],
            'gyro1': [],
            'accel2': [],
            'gyro2': [],
            'quat1': [],
            'quat2': [],
            'euler1': [],
            'euler2': [],
        }
        
        for sample in imu_data_copy:
            result['accel1'].append(sample.reading.accel1)
            result['gyro1'].append(sample.reading.gyro1)
            result['accel2'].append(sample.reading.accel2)
            result['gyro2'].append(sample.reading.gyro2)
            
            # IMUSample stores the IMUReading in `sample.reading`
            if sample.reading.quat1 is not None:
                result['quat1'].append(sample.reading.quat1)
            if sample.reading.quat2 is not None:
                result['quat2'].append(sample.reading.quat2)
            if sample.reading.euler1 is not None:
                result['euler1'].append(sample.reading.euler1)
            if sample.reading.euler2 is not None:
                result['euler2'].append(sample.reading.euler2)
        
        # Convert to arrays
        for key in result:
            if len(result[key]) > 0:
                result[key] = np.array(result[key])
            else:
                result[key] = None
        
        return result
    
    def _calibrate_imu(self):
        """Calibrate IMU gyroscope bias."""
        if self.mock_mode:
            return
        
        self._set_state(TrialState.CALIBRATING)
        self.gui.set_exercise("IMU Calibration", 
                             "Place IMUs FLAT and STILL\n\nCalibrating...")
        
        print("\nCalibrating IMU...")
        
        try:
            # Access IMU device from acquisition
            if hasattr(self.acquisition, '_imu_device') and self.acquisition._imu_device:
                self.acquisition._imu_device.calibrate(
                    samples=self.config.IMU_CONFIG['calibration_samples']
                )
                print("✓ IMU calibration complete")
            else:
                print("⚠ IMU device not available for calibration")
        
        except Exception as e:
            print(f"✗ Calibration failed: {e}")
            self.gui.show_error("Calibration Error", f"Failed to calibrate IMU:\n{e}")
        
        self._set_state(TrialState.READY)
        self._update_gui_for_current_trial()
    
    def _on_emg_chunk(self, chunk):
        """Callback for EMG data chunks."""
        # Update statistics
        self.stats['emg_samples_received'] += len(chunk.sample_t)
        self._emg_samples_since_rate += len(chunk.sample_t)
        
        now = time.perf_counter()
        dt = now - self._last_emg_rate_t
        if dt >= 1.0:
            self.stats['emg_sample_rate_measured'] = self._emg_samples_since_rate / dt
            self._emg_samples_since_rate = 0
            self._last_emg_rate_t = now
            
        self.stats['last_emg_time'] = chunk.t
        
        # Extract EMG signal
        if 'raw' in chunk.data:
            emg_signal = chunk.data['raw']
        elif 'pairs' in chunk.data:
            emg_signal = chunk.data['pairs']
        else:
            return
            
        if self.estimator is not None and not self.mock_mode:
            try:
                # pass status channel to estimator
                if 'raw' in chunk.data:
                    trig_vals = chunk.data['raw'][:, -1]
                    self.estimator.add_emg_trig_chunk(chunk.sample_t, trig_vals)
            except Exception:
                pass
        
        # Buffer for recording
        if self.recording_data:
            with self.buffer_lock:
                # Store raw data
                for i, t in enumerate(chunk.sample_t):
                    self.emg_buffer_times.append(t)
                self.emg_buffer_raw.append(emg_signal)
        
        # Update GUI plots (downsample for performance)
        if len(chunk.sample_t) > 0 and self.gui:
            try:
                # Coalesce EMG plot updates: keep only latest chunk and schedule one GUI update.
                self._pending_emg_plot = (chunk.sample_t, emg_signal)
                if not self._emg_plot_update_scheduled:
                    self._emg_plot_update_scheduled = True
                    timer = self.gui.schedule_task(0, self._flush_emg_plot_update)
                    self.scheduled_callbacks.append(timer)
            except Exception as e:
                # GUI may be closed, ignore
                pass
    
    def _on_imu_sample(self, sample):
        """Callback for IMU samples."""
        # Update statistics
        self.stats['imu_samples_received'] += 1
        self._imu_samples_since_rate += 1
        
        now = time.perf_counter()
        dt = now - self._last_imu_rate_t
        if dt >= 1.0:
            self.stats['imu_sample_rate_measured'] = self._imu_samples_since_rate / dt
            self._imu_samples_since_rate = 0
            self._last_imu_rate_t = now
            
        self.stats['last_imu_time'] = sample.t
        
        # Buffer for recording
        if self.recording_data:
            with self.buffer_lock:
                self.imu_buffer_times.append(sample.t)
                self.imu_buffer_data.append(sample)
        
        # Update GUI plots
        if self.gui:
            try:
                # Coalesce IMU plot updates: keep only latest sample and schedule one GUI update.
                ts = np.array([sample.t])
                accel = sample.reading.accel1.reshape(1, 3)
                gyro = sample.reading.gyro1.reshape(1, 3)
                self._pending_imu_plot = (ts, accel, gyro)
                if not self._imu_plot_update_scheduled:
                    self._imu_plot_update_scheduled = True
                    timer = self.gui.schedule_task(0, self._flush_imu_plot_update)
                    self.scheduled_callbacks.append(timer)

                # Coalesce IMU health updates too (IMU1/IMU2 online/zero stream)
                self._pending_imu_health = getattr(sample.reading, "health", None)
                if not self._imu_health_update_scheduled:
                    self._imu_health_update_scheduled = True
                    timer = self.gui.schedule_task(0, self._flush_imu_health_update)
                    self.scheduled_callbacks.append(timer)
            except Exception as e:
                # GUI may be closed, ignore
                pass

    def _on_stm32_sample(self, sample):
        """Callback for STM32 samples (PRBS + IMU)."""
        # Bridge BNO085 orientation/accel into IMU pipeline
        try:
            imu_sample = _stm32_to_imu_sample(sample)
            if imu_sample is not None:
                self._on_imu_sample(imu_sample)
        except Exception:
            pass

    def _flush_emg_plot_update(self):
        """Run a coalesced EMG plot update (main thread)."""
        self._emg_plot_update_scheduled = False
        pending = self._pending_emg_plot
        self._pending_emg_plot = None
        if pending is None:
            return
        timestamps, data = pending
        self._safe_update_emg_plot(timestamps, data)

    def _flush_imu_plot_update(self):
        """Run a coalesced IMU plot update (main thread)."""
        self._imu_plot_update_scheduled = False
        pending = self._pending_imu_plot
        self._pending_imu_plot = None
        if pending is None:
            return
        timestamps, accel, gyro = pending
        self._safe_update_imu_plot(timestamps, accel, gyro)

    def _flush_imu_health_update(self):
        """Run a coalesced IMU health update (main thread)."""
        self._imu_health_update_scheduled = False
        health = self._pending_imu_health
        self._pending_imu_health = None
        if health is None:
            return
        try:
            if self.gui and hasattr(self.gui, "update_imu_health"):
                self.gui.update_imu_health(
                    imu1_online=getattr(health, "imu1_online", False),
                    imu1_zero_data=getattr(health, "imu1_zero_data", False),
                    imu2_online=getattr(health, "imu2_online", False),
                    imu2_zero_data=getattr(health, "imu2_zero_data", False),
                )
        except Exception:
            pass
    
    def _clear_trial_buffers(self):
        """Clear data buffers for new trial."""
        with self.buffer_lock:
            self.emg_buffer_times.clear()
            self.emg_buffer_raw.clear()
            self.emg_buffer_preprocessed.clear()
            self.imu_buffer_times.clear()
            self.imu_buffer_data.clear()
    
    def _safe_update_emg_plot(self, timestamps, data):
        """Safely update EMG plot from main thread."""
        try:
            if self.gui and hasattr(self.gui, 'update_emg_data'):
                self.gui.update_emg_data(timestamps, data)
        except Exception as e:
            # Silently ignore GUI update errors
            pass
    
    def _safe_update_imu_plot(self, timestamps, accel, gyro):
        """Safely update IMU plot from main thread."""
        try:
            if self.gui and hasattr(self.gui, 'update_imu_data'):
                self.gui.update_imu_data(timestamps, accel, gyro)
        except Exception as e:
            # Silently ignore GUI update errors
            pass
    
    def _set_state(self, state: TrialState):
        """Set trial state."""
        self.state = state
        self.gui.set_state(state)
    
    def _update_gui_for_current_trial(self):
        """Update GUI with current trial information."""
        if self.current_trial_idx >= len(self.expanded_protocol):
            return
        
        exercise = self.expanded_protocol[self.current_trial_idx]
        
        self.gui.set_trial_info(self.current_trial_idx + 1, len(self.expanded_protocol))
        self.gui.set_exercise(
            exercise['display_name'],
            "Ready to start\n\nPress SPACE when ready"
        )
        self.gui.set_progress(0.0)
    
    def _update_trial_progress(self):
        """Update trial progress bar."""
        if self.state == TrialState.RECORDING and self.trial_start_time:
            if self.current_trial_idx < len(self.expanded_protocol):
                exercise = self.expanded_protocol[self.current_trial_idx]
                elapsed = time.time() - self.trial_start_time
                progress = min(1.0, elapsed / exercise['duration'])
                
                if self.gui and hasattr(self.gui, 'set_progress'):
                    try:
                        self.gui.set_progress(progress)
                        
                        # Update status
                        remaining = max(0, exercise['duration'] - elapsed)
                        self.gui.set_status(f"Recording... {remaining:.1f}s remaining")
                    except Exception as e:
                        # GUI may be closed, ignore
                        pass
    
    def _update_signal_quality(self):
        """Update signal quality indicators."""
        with self.buffer_lock:
            if len(self.emg_buffer_raw) > 0:
                last_chunk = self.emg_buffer_raw[-1]
                rms = np.sqrt(np.mean(last_chunk**2))
                
                # Check variance for simple health flags
                variances = np.var(last_chunk, axis=0)
                health_flags = [bool(variances[i] > 1e-6) for i in range(min(8, len(variances)))]
                while len(health_flags) < 8:
                    health_flags.append(False)
            else:
                rms = 0.0
                health_flags = [False] * 8
        
        if self.gui and hasattr(self.gui, 'update_signal_quality'):
            try:
                self.gui.update_signal_quality(
                    emg_rate=self.stats['emg_sample_rate_measured'],
                    imu_rate=self.stats['imu_sample_rate_measured'],
                    emg_rms=rms
                )
            except Exception as e:
                # GUI may be closed, ignore
                pass
                
        if self.gui and hasattr(self.gui, 'update_emg_health'):
            try:
                self.gui.update_emg_health(health_flags)
            except Exception:
                pass

    def _manager_update_tick(self):
        """
        Periodic manager-driven GUI updates (runs on Qt main thread).
        IMPORTANT: UI widgets must only be touched from the main thread.
        """
        if not self.running or not self.gui:
            return
        try:
            if self.state == TrialState.RECORDING:
                self._update_trial_progress()
            self._update_signal_quality()

            # If IMU stream stalls, update health labels to OFFLINE (even without new samples).
            if self.gui and hasattr(self.gui, "update_imu_health"):
                last_t = self.stats.get("last_imu_time", None)
                if last_t is None:
                    self.gui.update_imu_health(False, False, False, False)
                else:
                    now_t = time.perf_counter()
                    if (now_t - float(last_t)) > 1.0:
                        self.gui.update_imu_health(False, False, False, False)
        except Exception:
            # Never let periodic GUI update crash the app
            pass

        # === UPDATE PRBS AND BUTTON MATRIX ===
        try:
            if self.gui and hasattr(self.gui, 'update_stm32_data') and self._stm32_thread is not None:
                stm32_snap = self._stm32_thread.get_snapshot()
                
                emg_chunks = None
                if self.estimator is not None:
                    # Run background sync update periodically
                    now = time.perf_counter()
                    if self.estimator.should_update(now):
                        self.estimator.update(now)
                        
                    emg_chunks = self.estimator.get_emg_trig_snapshot()
                    
                    res = self.estimator._last_result
                    if res is not None:
                        delay = self.estimator._current_delay_ms
                        conf = res.confidence
                        self.gui.update_sync_data(now, delay, conf)
                        
                if stm32_snap and "pc_time" in stm32_snap:
                    keys_mask = stm32_snap["keys_mask"][-1] if len(stm32_snap["keys_mask"]) > 0 else 0
                    self.gui.update_stm32_data(
                        stm32_snap["pc_time"],
                        stm32_snap["prbs_level"],
                        int(keys_mask)
                    )
                    
                if emg_chunks:
                    t_raw = np.concatenate([c[0] for c in emg_chunks])
                    v_raw = np.concatenate([c[1] for c in emg_chunks])
                    
                    is_status = getattr(self.estimator, "_trig_is_status", True)
                    trig_bit = getattr(self.estimator, "_detected_trig_bit", None)
                    y_plot = _extract_trig_binary(v_raw, is_status, trig_bit)
                    self.gui.update_emg_prbs_data(t_raw, y_plot)
        except Exception:
            pass

        try:
            timer = self.gui.schedule_task(self._manager_update_interval_ms, self._manager_update_tick)
            self.scheduled_callbacks.append(timer)
        except Exception:
            # GUI may be closing
            pass
    
    def run(self, app):
        """Run the trial manager (blocking)."""
        self.running = True
        # Schedule periodic manager updates on the GUI main thread
        if self.gui:
            try:
                timer = self.gui.schedule_task(self._manager_update_interval_ms, self._manager_update_tick)
                self.scheduled_callbacks.append(timer)
            except Exception:
                pass
        
        # Show GUI and run the Qt event loop (blocking)
        try:
            self.gui.show()
            app.exec()
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources."""
        print("\n\nCleaning up...")
        
        self.running = False
        self.recording_data = False
        
        # Cancel all scheduled PyQt timers
        if self.gui:
            for timer in self.scheduled_callbacks:
                try:
                    timer.stop()
                except:
                    pass
            self.scheduled_callbacks.clear()
            
        # Stop TMSi thread (real hardware)
        if self._tmsi_thread is not None:
            try:
                self._tmsi_thread.stop()
                self._tmsi_thread.join(timeout=3)
                print("✓ TMSi thread stopped")
            except Exception as e:
                print(f"⚠ Error stopping TMSi thread: {e}")
            try:
                if LegacyDevice is not None:
                    LegacyDevice.cleanup()
            except Exception:
                pass

        # Stop STM32
        if self._stm32_thread is not None:
            try:
                self._stm32_thread.stop()
                self._stm32_thread.join(timeout=2)
                print("✓ STM32 thread stopped")
            except Exception as e:
                print(f"⚠ Error stopping STM32 thread: {e}")
        
        # Stop dummy acquisition (mock mode)
        if self.acquisition:
            try:
                self.acquisition.stop()
                print("✓ Dummy acquisition stopped")
            except Exception as e:
                print(f"⚠ Error stopping acquisition: {e}")
        
        # Close GUI
        if self.gui:
            try:
                self.gui.close()
            except:
                pass
        
        print("✓ Cleanup complete")


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """Main entry point for trial data collection."""
    print("\n" + "="*70)
    print("EMG + IMU TRIAL DATA COLLECTION SYSTEM")
    print("="*70)
    
    # Print configuration summary
    setup_trial.print_config_summary()
    
    # Check for mock mode flag
    mock_mode = '--mock' in sys.argv or '--test' in sys.argv
    
    # Initialize QApplication before any PyQt6 widgets are constructed
    from PyQt6 import QtWidgets
    app = QtWidgets.QApplication(sys.argv)
    
    # Create and initialize manager
    manager = TrialManager(mock_mode=mock_mode)
    
    try:
        manager.initialize()
        manager.run(app)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\n\n✗ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        manager.cleanup()
    
    print("\nSession complete. Goodbye!")


if __name__ == '__main__':
    main()
