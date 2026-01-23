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
except ImportError:
    print("⚠ Warning: Could not import src modules. Mock mode will be used.")
    SynchronizedAcquisition = None
    EMGPreprocessor = None


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
            on_key_press=self._handle_key_press
        )
        
        # Set initial GUI state
        self.gui.set_state(TrialState.IDLE)
        self.gui.set_trial_info(0, len(self.expanded_protocol))
        
        if not self.mock_mode:
            # Initialize acquisition
            print("\n✓ Initializing acquisition system...")
            try:
                sync_config = self.config.get_sync_config()
                self.acquisition = SynchronizedAcquisition(
                    config=sync_config,
                    on_emg=self._on_emg_chunk,
                    on_imu=self._on_imu_sample
                )
                
                print("  Starting acquisition...")
                self.acquisition.start()
                
                # Check for errors
                if self.acquisition.errors:
                    raise RuntimeError(f"Acquisition errors: {self.acquisition.errors}")
                
                print("  ✓ Acquisition started successfully")
                
            except Exception as e:
                print(f"  ✗ Failed to initialize acquisition: {e}")
                self.mock_mode = True
                print("  Falling back to mock mode")
        
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
        if self.state == TrialState.READY and not self.mock_mode:
            self._calibrate_imu()
    
    def _handle_quit(self):
        """Handle Q key (quit)."""
        if self.gui.ask_yes_no("Quit", "Are you sure you want to quit?"):
            self.running = False
            self.gui.close()
    
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
        
        # Countdown
        self._set_state(TrialState.COUNTDOWN)
        self.countdown_start = time.time()
        
        for i in range(self.config.COUNTDOWN_DURATION, 0, -1):
            self.gui.set_exercise(exercise['display_name'], f"Starting in {i}...")
            time.sleep(1)
        
        # Clear buffers
        self._clear_trial_buffers()
        self.gui.clear_plots()
        
        # Start recording
        self._set_state(TrialState.RECORDING)
        self.trial_start_time = time.time()
        self.recording_data = True
        
        self.gui.set_exercise(exercise['display_name'], exercise['instruction'])
        print(f"Recording started at {datetime.now().strftime('%H:%M:%S')}")
        
        # Schedule auto-stop
        duration = exercise['duration']
        self.gui.root.after(int(duration * 1000), self._stop_trial)
    
    def _stop_trial(self):
        """Stop recording and save trial."""
        if self.state != TrialState.RECORDING:
            return
        
        self.recording_data = False
        trial_duration = time.time() - self.trial_start_time
        
        print(f"\nRecording stopped (duration: {trial_duration:.2f}s)")
        
        # Save trial
        self._set_state(TrialState.SAVING)
        self.gui.set_exercise("Saving Data", "Please wait...")
        
        try:
            self._save_current_trial()
            print("✓ Trial saved successfully")
        except Exception as e:
            print(f"✗ Error saving trial: {e}")
            self.gui.show_error("Save Error", f"Failed to save trial:\n{e}")
        
        # Move to next trial
        self.current_trial_idx += 1
        
        if self.current_trial_idx < len(self.expanded_protocol):
            # More trials remaining
            time.sleep(1)  # Brief pause
            self._set_state(TrialState.READY)
            self._update_gui_for_current_trial()
            
            # Rest period
            exercise = self.expanded_protocol[self.current_trial_idx - 1]
            rest_duration = exercise.get('rest_after', 0)
            if rest_duration > 0:
                print(f"\nRest period: {rest_duration}s")
                self.gui.set_exercise("Rest", f"Relax for {rest_duration} seconds")
                time.sleep(rest_duration)
                self._update_gui_for_current_trial()
        else:
            # All trials complete
            self._set_state(TrialState.COMPLETED)
            self.gui.set_exercise("All Trials Complete!", 
                                 "Thank you for participating!\n\nPress Q to quit")
            print(f"\n{'='*70}")
            print("ALL TRIALS COMPLETED")
            print(f"{'='*70}\n")
    
    def _save_current_trial(self):
        """Save current trial data to HDF5."""
        trial_num = self.current_trial_idx
        exercise = self.expanded_protocol[trial_num]
        
        print(f"\nSaving trial {trial_num + 1}...")
        
        # Prepare EMG data
        emg_data = None
        if len(self.emg_buffer_raw) > 0:
            emg_data = {
                'timestamps': np.array(self.emg_buffer_times),
                'raw': np.vstack(self.emg_buffer_raw),
                'sample_rate': self.stats['emg_sample_rate_measured'] or 2048,
                'channel_names': self.config.EMG_CONFIG.get('channel_names', 
                                                            [f'CH{i}' for i in range(4)]),
            }
            print(f"  EMG: {emg_data['raw'].shape[0]} samples")
        
        # Prepare IMU data
        imu_data = None
        if len(self.imu_buffer_data) > 0:
            imu_dict = self._consolidate_imu_buffer()
            imu_data = {
                'timestamps': np.array(self.imu_buffer_times),
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
            print(f"  IMU: {len(self.imu_buffer_times)} samples")
        
        # Synchronization info
        sync_info = {
            'method': 'hardware_timestamps',
            'emg_samples': len(self.emg_buffer_times),
            'imu_samples': len(self.imu_buffer_times),
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
        if self.config.SAVE_PREPROCESSED and len(self.emg_buffer_preprocessed) > 0:
            emg_preprocessed = self._prepare_preprocessed_emg()
            self.storage.save_trial_preprocessed(
                trial_number=trial_num + 1,
                exercise_name=exercise['name'],
                emg_preprocessed=emg_preprocessed,
                imu_data=imu_data
            )
    
    def _prepare_preprocessed_emg(self) -> Dict[str, Any]:
        """Prepare preprocessed EMG data dictionary."""
        # Stack all preprocessed data
        all_stages = {}
        for item in self.emg_buffer_preprocessed:
            for key, value in item.items():
                if key not in all_stages:
                    all_stages[key] = []
                all_stages[key].append(value)
        
        # Convert to arrays
        result = {
            'timestamps': np.array(self.emg_buffer_times),
            'sample_rate': self.stats['emg_sample_rate_measured'] or 2048,
            'channel_names': self.config.EMG_CONFIG.get('channel_names', 
                                                        [f'CH{i}' for i in range(4)]),
        }
        
        for key, value_list in all_stages.items():
            if len(value_list) > 0:
                result[key] = np.vstack(value_list)
        
        return result
    
    def _consolidate_imu_buffer(self) -> Dict[str, np.ndarray]:
        """Consolidate IMU buffer into arrays."""
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
        
        for sample in self.imu_buffer_data:
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
            # Store raw data
            for i, t in enumerate(chunk.sample_t):
                self.emg_buffer_times.append(t)
            self.emg_buffer_raw.append(emg_signal)
            
            # Preprocess if enabled
            if self.preprocessor:
                try:
                    processed = self.preprocessor.process(emg_signal, return_all_stages=True)
                    self.emg_buffer_preprocessed.append(processed)
                except Exception as e:
                    pass
        
        # Update GUI plots (downsample for performance)
        if len(chunk.sample_t) > 0:
            try:
                self.gui.update_emg_data(chunk.sample_t, emg_signal)
            except Exception as e:
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
            self.imu_buffer_times.append(sample.t)
            self.imu_buffer_data.append(sample)
        
        # Update GUI plots
        try:
            self.gui.update_imu_data(
                np.array([sample.t]),
                sample.reading.accel1.reshape(1, 3),
                sample.reading.gyro1.reshape(1, 3)
            )
        except Exception as e:
            pass
    
    def _clear_trial_buffers(self):
        """Clear data buffers for new trial."""
        self.emg_buffer_times.clear()
        self.emg_buffer_raw.clear()
        self.emg_buffer_preprocessed.clear()
        self.imu_buffer_times.clear()
        self.imu_buffer_data.clear()
    
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
            exercise = self.expanded_protocol[self.current_trial_idx]
            elapsed = time.time() - self.trial_start_time
            progress = min(1.0, elapsed / exercise['duration'])
            self.gui.set_progress(progress)
            
            # Update status
            remaining = max(0, exercise['duration'] - elapsed)
            self.gui.set_status(f"Recording... {remaining:.1f}s remaining")
    
    def _update_signal_quality(self):
        """Update signal quality indicators."""
        if len(self.emg_buffer_raw) > 0:
            last_chunk = self.emg_buffer_raw[-1]
            rms = np.sqrt(np.mean(last_chunk**2))
        else:
            rms = 0.0
        
        self.gui.update_signal_quality(
            emg_rate=self.stats['emg_sample_rate_measured'],
            imu_rate=self.stats['imu_sample_rate_measured'],
            emg_rms=rms
        )
    
    def _status_update_loop(self):
        """Background thread for status updates."""
        while self.running:
            if self.state == TrialState.RECORDING:
                self._update_trial_progress()
            
            self._update_signal_quality()
            
            time.sleep(0.1)
    
    def run(self):
        """Run the trial manager (blocking)."""
        self.running = True
        
        # Start status update thread
        self.data_thread = threading.Thread(target=self._status_update_loop, daemon=True)
        self.data_thread.start()
        
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
