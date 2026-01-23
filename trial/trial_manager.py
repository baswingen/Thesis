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

# Import trial modules
from . import setup_trial
from .trial_gui import TrialGUI, TrialState
from .data_storage import TrialDataStorage
from .trial_protocols import get_protocol, expand_protocol, validate_protocol

# Import source modules
try:
    from src.synchronized_acquisition import SynchronizedAcquisition
    from src.emg_processing import EMGPreprocessor
    from src.dummy_acquisition import DummyAcquisition
except ImportError:
    print("⚠ Warning: Could not import src modules. Mock mode will be used.")
    SynchronizedAcquisition = None
    EMGPreprocessor = None
    DummyAcquisition = None


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
        self.mock_mode = mock_mode or (SynchronizedAcquisition is None)
        
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
        self.expanded_protocol = expand_protocol(self.protocol)
        
        # Data buffers for current trial
        self.emg_buffer_times = []
        self.emg_buffer_raw = []
        self.emg_buffer_preprocessed = []
        self.imu_buffer_times = []
        self.imu_buffer_data = []
        
        # Devices
        self.acquisition = None
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
        self._manager_update_interval_ms = 100
        # Throttle plot updates: acquisition callbacks can fire very frequently.
        # We coalesce updates so Tk doesn't get flooded with after_idle callbacks.
        self._pending_emg_plot = None  # (timestamps, data)
        self._pending_imu_plot = None  # (timestamps, accel, gyro)
        self._emg_plot_update_scheduled = False
        self._imu_plot_update_scheduled = False

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
        if self.config.PROTOCOL_NAME == 'custom':
            protocol = self.config.TRIAL_EXERCISES
        else:
            protocol = get_protocol(self.config.PROTOCOL_NAME)
        
        validate_protocol(protocol)
        return protocol
    
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
        
        if not self.mock_mode:
            # Initialize acquisition
            print("\n✓ Initializing acquisition system...")
            try:
                # Check if dummy signals are enabled
                use_dummy = self.config.SYNC_CONFIG.get('use_dummy_signals', False)
                
                if use_dummy and DummyAcquisition is not None:
                    # Use dummy acquisition
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
                else:
                    # Use real hardware acquisition
                    print("  Using REAL hardware signals")
                    print("  Connecting to devices (this may take up to 30 seconds)...")
                    
                    # Reduce timeout for faster failure if hardware not connected
                    sync_config = self.config.get_sync_config()
                    # Override timeout if too long
                    if sync_config.imu.imu_config.timeout > 2.0:
                        sync_config.imu.imu_config.timeout = 2.0
                    
                    self.acquisition = SynchronizedAcquisition(
                        config=sync_config,
                        on_emg=self._on_emg_chunk,
                        on_imu=self._on_imu_sample
                    )
                
                print("  Starting acquisition threads...")
                self.acquisition.start(ready_timeout_s=30.0)  # Reduced timeout
                
                # Check for errors
                if hasattr(self.acquisition, 'errors') and self.acquisition.errors:
                    error_msgs = [str(e) for e in self.acquisition.errors]
                    raise RuntimeError(f"Acquisition errors: {'; '.join(error_msgs)}")
                
                print("  ✓ Acquisition started successfully")
                
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
            self.gui.close()
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
        callback_id = self.gui.root.after(1000, lambda: self._countdown_step(remaining - 1))
        self.scheduled_callbacks.append(callback_id)
    
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
        callback_id = self.gui.root.after(int(duration * 1000), self._stop_trial)
        self.scheduled_callbacks.append(callback_id)
    
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
                self.gui.root.after(0, self._after_save_success)
                
            except Exception as e:
                print(f"✗ Error saving trial: {e}")
                import traceback
                traceback.print_exc()
                
                # Schedule error dialog in main thread
                self.gui.root.after(0, lambda: self._after_save_error(e))
        
        # Start save thread
        save_thread = threading.Thread(target=save_and_continue, daemon=True)
        save_thread.start()
    
    def _after_save_success(self):
        """Called in main thread after successful save."""
        # If user requested quit during recording/saving, exit cleanly now.
        if self._quit_after_save:
            self.running = False
            try:
                self.gui.close()
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
                
                callback_id = self.gui.root.after(int(rest_duration * 1000), end_rest)
                self.scheduled_callbacks.append(callback_id)
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
                self.gui.close()
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
            
            if sample.quat1 is not None:
                result['quat1'].append(sample.quat1)
            if sample.quat2 is not None:
                result['quat2'].append(sample.quat2)
            if sample.euler1 is not None:
                result['euler1'].append(sample.euler1)
            if sample.euler2 is not None:
                result['euler2'].append(sample.euler2)
        
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
        
        if self.stats['last_emg_time'] is not None:
            dt = chunk.t - self.stats['last_emg_time']
            if dt > 0:
                rate = len(chunk.sample_t) / dt
                # Exponential moving average
                alpha = 0.1
                self.stats['emg_sample_rate_measured'] = (
                    alpha * rate + (1 - alpha) * self.stats['emg_sample_rate_measured']
                )
        self.stats['last_emg_time'] = chunk.t
        
        # Extract EMG signal
        if 'raw' in chunk.data:
            emg_signal = chunk.data['raw']
        elif 'pairs' in chunk.data:
            emg_signal = chunk.data['pairs']
        else:
            return
        
        # Buffer for recording
        if self.recording_data:
            with self.buffer_lock:
                # Store raw data
                for i, t in enumerate(chunk.sample_t):
                    self.emg_buffer_times.append(t)
                self.emg_buffer_raw.append(emg_signal)
        
        # Update GUI plots (downsample for performance)
        if len(chunk.sample_t) > 0 and self.gui and self.gui.root:
            try:
                # Coalesce EMG plot updates: keep only latest chunk and schedule one GUI update.
                self._pending_emg_plot = (chunk.sample_t, emg_signal)
                if not self._emg_plot_update_scheduled:
                    self._emg_plot_update_scheduled = True
                    callback_id = self.gui.root.after(0, self._flush_emg_plot_update)
                    self.scheduled_callbacks.append(callback_id)
            except Exception as e:
                # GUI may be closed, ignore
                pass
    
    def _on_imu_sample(self, sample):
        """Callback for IMU samples."""
        # Update statistics
        self.stats['imu_samples_received'] += 1
        
        if self.stats['last_imu_time'] is not None:
            dt = sample.t - self.stats['last_imu_time']
            if dt > 0:
                rate = 1.0 / dt
                alpha = 0.1
                self.stats['imu_sample_rate_measured'] = (
                    alpha * rate + (1 - alpha) * self.stats['imu_sample_rate_measured']
                )
        self.stats['last_imu_time'] = sample.t
        
        # Buffer for recording
        if self.recording_data:
            with self.buffer_lock:
                self.imu_buffer_times.append(sample.t)
                self.imu_buffer_data.append(sample)
        
        # Update GUI plots
        if self.gui and self.gui.root:
            try:
                # Coalesce IMU plot updates: keep only latest sample and schedule one GUI update.
                ts = np.array([sample.t])
                accel = sample.reading.accel1.reshape(1, 3)
                gyro = sample.reading.gyro1.reshape(1, 3)
                self._pending_imu_plot = (ts, accel, gyro)
                if not self._imu_plot_update_scheduled:
                    self._imu_plot_update_scheduled = True
                    callback_id = self.gui.root.after(0, self._flush_imu_plot_update)
                    self.scheduled_callbacks.append(callback_id)
            except Exception as e:
                # GUI may be closed, ignore
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
            else:
                rms = 0.0
        
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

    def _manager_update_tick(self):
        """
        Periodic manager-driven GUI updates (runs on Tk main thread).
        IMPORTANT: Tkinter widgets must only be touched from the main thread.
        """
        if not self.running or not self.gui or not self.gui.root:
            return
        try:
            if self.state == TrialState.RECORDING:
                self._update_trial_progress()
            self._update_signal_quality()
        except Exception:
            # Never let periodic GUI update crash the app
            pass

        try:
            callback_id = self.gui.root.after(self._manager_update_interval_ms, self._manager_update_tick)
            self.scheduled_callbacks.append(callback_id)
        except Exception:
            # GUI may be closing
            pass
    
    def run(self):
        """Run the trial manager (blocking)."""
        self.running = True
        # Schedule periodic manager updates on the Tk main thread
        if self.gui and self.gui.root:
            try:
                callback_id = self.gui.root.after(self._manager_update_interval_ms, self._manager_update_tick)
                self.scheduled_callbacks.append(callback_id)
            except Exception:
                pass
        
        # Run GUI (blocking)
        try:
            self.gui.run()
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources."""
        print("\n\nCleaning up...")
        
        self.running = False
        self.recording_data = False
        
        # Cancel all scheduled callbacks
        if self.gui and self.gui.root:
            for callback_id in self.scheduled_callbacks:
                try:
                    self.gui.root.after_cancel(callback_id)
                except:
                    pass
            self.scheduled_callbacks.clear()
        
        # Stop acquisition
        if self.acquisition:
            try:
                self.acquisition.stop()
                print("✓ Acquisition stopped")
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
    
    # Create and initialize manager
    manager = TrialManager(mock_mode=mock_mode)
    
    try:
        manager.initialize()
        manager.run()
    
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
