"""
Trial Execution Framework
=========================

Main entry point for running a clinical/experimental trial. 
Records data from STM32 and EMG, synchronized via PRBS, and saves 
it directly to an HDF5 database.
"""

import argparse
import sys
import time
import queue
import threading
from pathlib import Path
from typing import Optional, Dict, Any
import numpy as np

# Ensure src/ and setup_scripts/ are accessible
sys.path.append(str(Path(__file__).parent.parent))

from src.stm32_reader import SampleSTM32
from src.stm32_emg_sync import SyncDelayEstimator, CHIP_RATE_HZ, MIN_OVERLAP_RATIO
from src.emg_processing import EMGProcessor
from setup_scripts.signal_acquisition_testing import (
    RawSTM32Thread, RawTMSiThread, _SyncWorker, _TMSI_AVAILABLE, STM32_BAUD, STM32_PORT,
    EMG_SAMPLE_RATE, VERBOSE, PRBS_UPDATE_INTERVAL_S, PRBS_CORRELATION_WINDOW_S
)
from trial.hdf5_logger import HDF5TrialLogger

# Try to import PyQt for the GUI status window
try:
    from PyQt6 import QtWidgets, QtCore
    import pyqtgraph as pg
    _QT_AVAILABLE = True
except ImportError:
    _QT_AVAILABLE = False
    print("PyQt6/pyqtgraph not found. Falling back to headless mode inherently.")

# Fallback for STM32 Port auto-detect
def get_stm32_port() -> str:
    if STM32_PORT is not None:
        return STM32_PORT
    try:
        from src.arduino_connection import find_arduino_port
        port = find_arduino_port(verbose=VERBOSE)
        if port:
            return port
    except ImportError:
        pass
    raise RuntimeError("Could not auto-detect STM32 Port. Specify in CONFIG.")


class TrialManager:
    """Orchestrates the acquisition, synchronization, and logging of a single Trial."""
    
    def __init__(self, participant_id: str, trial_num: int, duration_s: Optional[float] = None, metadata: Optional[Dict[str, Any]] = None):
        self.participant_id = participant_id
        self.trial_num = trial_num
        self.duration_s = duration_s
        self.metadata = metadata or {}
        
        # Add technical metadata
        self.metadata.update({
            'stm32_baud': STM32_BAUD,
            'emg_target_sample_rate': EMG_SAMPLE_RATE,
            'prbs_chip_rate_hz': CHIP_RATE_HZ
        })
        
        # Base directory for the database
        self.db_path = str(Path(__file__).parent.parent / 'database')
        
        self.logger = HDF5TrialLogger(
            self.db_path, participant_id, trial_num, self.metadata
        )
        
        # Acquisition components
        self.estimator = SyncDelayEstimator(
            chip_rate_hz=CHIP_RATE_HZ,
            emg_sample_rate=float(EMG_SAMPLE_RATE or 2000),
            sync_window_s=PRBS_CORRELATION_WINDOW_S,
            update_interval_s=PRBS_UPDATE_INTERVAL_S,
        )
        self.stm32_thread = RawSTM32Thread(port=get_stm32_port(), baud=STM32_BAUD, estimator=self.estimator)
        self.tmsi_thread = RawTMSiThread(sample_rate=(EMG_SAMPLE_RATE or 2000), estimator=self.estimator)
        self.sync_worker = _SyncWorker(self.estimator)
        
        # Background worker for polling and dumping data to HDF5
        self._running = False
        self._log_thread = threading.Thread(target=self._logging_loop, daemon=True)
        
        self.start_time = 0.0
        
        # Processing setup for synced signals
        self.emg_processor = EMGProcessor(fs=float(EMG_SAMPLE_RATE or 2000))
        
    def start(self):
        print(f"\n[TRIAL] Starting Trial {self.trial_num} for Participant {self.participant_id}")
        self._running = True
        
        # 1. Start Hardware Threads
        self.stm32_thread.start()
        self.tmsi_thread.start()
        
        # Wait for EMG to initialize
        wait_t = time.perf_counter()
        while not self.tmsi_thread.running and not self.tmsi_thread.error:
            if time.perf_counter() - wait_t > 10.0:
                print("[TRIAL] ERR: Timed out waiting for TMSi Porti7 to start.")
                self.stop()
                sys.exit(1)
            time.sleep(0.1)
            
        if self.tmsi_thread.error:
            print(f"[TRIAL] ERR: TMSi initialization failed: {self.tmsi_thread.error}")
            self.stop()
            sys.exit(1)
            
        # 2. Start Sync Worker
        self.sync_worker.start()
        
        # 3. Start HDF5 Logger Thread
        self.start_time = time.perf_counter()
        self._log_thread.start()
        
        print("[TRIAL] All systems running. Recording...\n")

    def stop(self):
        print("\n[TRIAL] Stopping components and saving trial data...")
        self._running = False
        
        # Signal stops
        if hasattr(self, 'tmsi_thread'): self.tmsi_thread.stop()
        if hasattr(self, 'stm32_thread'): self.stm32_thread.stop()
        if hasattr(self, 'sync_worker'): self.sync_worker.stop()
        
        # Wait for threads
        if hasattr(self, 'stm32_thread') and self.stm32_thread.is_alive(): self.stm32_thread.join(timeout=2)
        if hasattr(self, 'tmsi_thread') and self.tmsi_thread.is_alive(): self.tmsi_thread.join(timeout=2)
        if hasattr(self, '_log_thread') and self._log_thread.is_alive(): self._log_thread.join(timeout=5)
        
        # Run final synchronization alignment and processing
        self._finalize_trial_data()
        
        print(f"[TRIAL] Saved HDF5 file: {self.logger.filename}")
        print("[TRIAL] Complete.")

    def _logging_loop(self):
        """Periodically polls threads for their full history buffers incrementally."""
        # Pointers to track how much of the thread history we've already written
        stm32_idx = 0
        emg_idx = 0
        
        last_sync_update = time.perf_counter()
        
        while self._running:
            time.sleep(0.1) # Poll at ~10Hz
            
            # --- STM32 Logging ---
            with self.stm32_thread._lock:
                current_stm32_hist = list(self.stm32_thread._history) # Snapshot
            
            n_stm = len(current_stm32_hist)
            if n_stm > stm32_idx:
                new_samples = current_stm32_hist[stm32_idx:n_stm]
                stm32_idx = n_stm
                
                # Format to (N, 22) array
                # (t_pc, t_stm32, imu1_ok, imu2_ok, yaw1, pitch1, roll1, ax1, ay1, az1, 
                #  yaw2, pitch2, roll2, ax2, ay2, az2, keys_mask, keys_rise, keys_fall, 
                #  prbs_tick, prbs_lvl, in_mark)
                out = np.zeros((len(new_samples), 22), dtype=np.float64)
                for i, (t_pc, s) in enumerate(new_samples):
                    out[i] = [
                        t_pc, s.t_ms, s.imu1_ok, s.imu2_ok, 
                        s.yaw1, s.pitch1, s.roll1, s.ax1, s.ay1, s.az1,
                        s.yaw2, s.pitch2, s.roll2, s.ax2, s.ay2, s.az2,
                        s.keys_mask, s.keys_rise, s.keys_fall,
                        s.prbs_tick, s.prbs_lvl, s.in_mark
                    ]
                self.logger.append_stm32_data(out)
                
            # --- EMG Logging ---
            # TMSiThread does not expose a list history by default, we use the estimator's buffer
            # since it already cleanly stores (timestamps, samples) arrays
            with self.estimator._lock:
                emg_chunks = list(self.estimator._emg_trig_buf)
                
            n_emg = len(emg_chunks)
            if n_emg > emg_idx:
                new_chunks = emg_chunks[emg_idx:n_emg]
                emg_idx = n_emg
                
                ts_arr = np.concatenate([c[0] for c in new_chunks])
                dat_arr = np.concatenate([c[1] for c in new_chunks])
                
                if len(ts_arr) > 0:
                    out = np.column_stack((ts_arr, dat_arr)) # t_pc, ch1...chn
                    self.logger.append_emg_data(out)
                    
            # --- Sync Logic Trigger / Metrics Logging ---
            now = time.perf_counter()
            if now - last_sync_update > PRBS_UPDATE_INTERVAL_S:
                last_sync_update = now
                self.sync_worker.trigger()
                
                res = self.estimator.get_result()
                if res is not None:
                    arr = np.array([[now, self.estimator.get_delay_ms(), res.confidence]])
                    self.logger.append_sync_metrics(arr)

    def _finalize_trial_data(self):
        """
        Called on stop(). Reconstructs the entire trial timeline using the 
        final overall delay, effectively computing `/stm32/synced`, `/emg/synced`,
        and applying filtering for `/emg/synced_processed`.
        """
        print("[TRIAL] Post-processing and finalizing synchronized datasets...")
        res = self.estimator.get_result()

        if res is None:
            print("[TRIAL] WARN: PRBS sync result unavailable — falling back to delay=0 ms, drift=0 ppm.")
            print("[TRIAL] The synced matrix will be written using raw PC timestamps only (no clock correction).")
            final_delay_ms = 0.0
            final_drift_ppm = 0.0
        else:
            print(f"[TRIAL] Found valid sync result: Confidence={res.confidence:.2f}")
            final_delay_ms = self.estimator.get_delay_ms()
            final_drift_ppm = self.estimator.get_drift_rate_ppm()
        
        # 1. Gather all raw STM32 and EMG data from the HDF5 file
        import h5py
        with h5py.File(self.logger.filename, 'r') as f:
            if '_raw/stm32' not in f or '_raw/emg' not in f:
                print("[TRIAL] ERR: Raw data groups missing from file!")
                return
            t_stm32_pc = f['_raw/stm32'][:, 0]
            stm32_data = f['_raw/stm32'][:, :]
            
            t_emg_pc = f['_raw/emg'][:, 0]
            emg_data = f['_raw/emg'][:, 2:] 
        
        if len(t_stm32_pc) == 0 or len(t_emg_pc) == 0:
            print(f"[TRIAL] ERR: Empty streams! STM32:{len(t_stm32_pc)}, EMG:{len(t_emg_pc)}")
            return
            
        print(f"[TRIAL] Aligning {len(t_emg_pc)} EMG samples to {len(t_stm32_pc)} STM32 samples with a Kalman delay of {final_delay_ms:.2f} ms (Drift: {final_drift_ppm:.1f} ppm).")

        # 2. Time shifting (Shared Clock)
        # We establish the PC Wall Clock (EMG timestamps) as the master reference.
        # Relationship: t_stm32_aligned = t_emg_pc - delay(t)
        # delay(t) = initial_delay + (t - t0) * drift_rate
        
        t_ref = t_emg_pc[0]
        drift_rate = final_drift_ppm / 1e6
        
        # Calculate time-varying delay for each STM32 sample
        dt_stm32 = t_stm32_pc - t_ref
        current_delays_s = (final_delay_ms / 1000.0) + (dt_stm32 * drift_rate)
        t_stm32_aligned = t_stm32_pc + current_delays_s

        # 3. Interpolate STM32 data onto the EMG timestamps (the master grid)
        stm32_synced = np.zeros((len(t_emg_pc), stm32_data.shape[1]), dtype=np.float64)
        
        for col in range(stm32_data.shape[1]):
            stm32_synced[:, col] = np.interp(
                t_emg_pc,
                t_stm32_aligned,
                stm32_data[:, col],
                left=np.nan, right=np.nan
            )
            
        valid_mask = ~np.isnan(stm32_synced[:, 0])
        
        t_common = t_emg_pc[valid_mask]
        stm32_synced = stm32_synced[valid_mask, :]
        emg_synced = emg_data[valid_mask, :] 
        
        # 4. Construct Unified Matrix (STM32 + EMG concatenated)
        unified_matrix = np.column_stack((stm32_synced, emg_synced))

        # 5. Process the synchronized EMG data
        print("[TRIAL] Filtering and processing EMG envelope...")
        
        # 6. Save the synchronized datasets (Simplified Logger)
        self.logger.save_synchronized_datasets(
            t_common=t_common,
            unified_matrix=unified_matrix,
            kalman_delay_ms=final_delay_ms,
            kalman_drift_ppm=final_drift_ppm
        )

# =============================================================================
# Simple GUI Status Framework
# =============================================================================

class SimpleTrialStatusWidget(QtWidgets.QWidget):
    """Modular GUI framework for trial status."""
    def __init__(self, manager: TrialManager):
        super().__init__()
        self.manager = manager
        
        self.setWindowTitle(f"Trial Status - {manager.participant_id} (Trial {manager.trial_num})")
        self.resize(500, 300)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Headline
        self.lbl_headline = QtWidgets.QLabel(f"RECORDING: Participant {manager.participant_id} | Trial {manager.trial_num}")
        font = self.lbl_headline.font()
        font.setPointSize(14)
        font.setBold(True)
        self.lbl_headline.setFont(font)
        self.lbl_headline.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_headline)
        
        layout.addSpacing(20)
        
        # Metrics
        f_layout = QtWidgets.QFormLayout()
        self.lbl_time = QtWidgets.QLabel("0.0s")
        self.lbl_stm32 = QtWidgets.QLabel("0 pkts")
        self.lbl_emg = QtWidgets.QLabel("0 Hz")
        self.lbl_sync = QtWidgets.QLabel("Waiting...")
        
        f_layout.addRow("Elapsed Time:", self.lbl_time)
        f_layout.addRow("STM32 Packets:", self.lbl_stm32)
        f_layout.addRow("EMG Approx Rate:", self.lbl_emg)
        f_layout.addRow("Sync Status:", self.lbl_sync)
        
        layout.addLayout(f_layout)
        
        layout.addStretch()
        
        # Stop button
        self.btn_stop = QtWidgets.QPushButton("STOP TRIAL")
        self.btn_stop.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; padding: 10px;")
        self.btn_stop.clicked.connect(self.close)
        layout.addWidget(self.btn_stop)
        
        # Update Timer (UI tick)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_status)
        self.timer.start(100) # 10Hz UI refresh
        
    def _update_status(self):
        # Update metrics from underlying manager
        elapsed = time.perf_counter() - self.manager.start_time
        self.lbl_time.setText(f"{elapsed:.1f} s")
        
        n_stm32 = len(self.manager.stm32_thread._history)
        self.lbl_stm32.setText(f"{n_stm32}")
        
        emg_rate = self.manager.tmsi_thread.estimated_rate_hz
        self.lbl_emg.setText(f"{emg_rate:.1f} Hz")
        
        res = self.manager.estimator.get_result()
        if res:
            self.lbl_sync.setText(f"Delay: {self.manager.estimator.get_delay_ms():.1f}ms | Confidence: {res.confidence:.2f}")
            if res.confidence > 0.4:
                self.lbl_sync.setStyleSheet("color: green; font-weight: bold;")
            else:
                self.lbl_sync.setStyleSheet("color: orange;")
                
        # Handle duration stop
        if self.manager.duration_s and elapsed > self.manager.duration_s:
            self.close()

    def closeEvent(self, event):
        self.timer.stop()
        self.manager.stop()
        event.accept()

# =============================================================================
# CLI Entry Point
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Run an experimental trial with synchronized data collection.")
    parser.add_argument("--participant", type=str, default="P00", help="Participant ID (e.g., P01)")
    parser.add_argument("--trial", type=int, default=0, help="Trial number (e.g., 1)")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds to automatically stop the trial.")
    parser.add_argument("--headless", action="store_true", help="Run without the GUI status window.")
    
    # Physiological Metadata arguments
    parser.add_argument("--age", type=int, default=25, help="Participant age (default: 25)")
    parser.add_argument("--gender", type=str, default="Unknown", help="Participant gender (default: Unknown)")
    parser.add_argument("--dominant_arm", type=str, default="Right", help="Participant dominant arm (e.g. Left, Right) (default: Right)")
    parser.add_argument("--measured_arm", type=str, default="Right", help="Participant measured arm (e.g. Left, Right) (default: Right)")
    parser.add_argument("--arm_size_cm", type=float, default=30.0, help="Participant arm circumference in cm (default: 30.0)")
    
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    
    metadata = {
        'age': args.age,
        'gender': args.gender,
        'dominant_arm': args.dominant_arm,
        'measured_arm': args.measured_arm,
        'arm_size_cm': args.arm_size_cm
    }
    
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run acquisition.")
        sys.exit(1)
        
    manager = TrialManager(
        participant_id=args.participant,
        trial_num=args.trial,
        duration_s=args.duration,
        metadata=metadata
    )
    
    manager.start()
    
    if not args.headless and _QT_AVAILABLE:
        app = QtWidgets.QApplication(sys.argv)
        # Apply dark theme fusion style
        app.setStyle("Fusion")
        palette = app.palette()
        palette.setColor(app.palette().ColorRole.Window, QtCore.Qt.GlobalColor.darkGray)
        app.setPalette(palette)
        
        gui = SimpleTrialStatusWidget(manager)
        gui.show()
        app.exec() # Block until window close, which triggers manager.stop()
    else:
        try:
            # Headless loop
            print("[TRIAL] Running in headless mode. Press Ctrl+C to stop.")
            start = time.perf_counter()
            while True:
                if args.duration and (time.perf_counter() - start) > args.duration:
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[TRIAL] Interrupted by user.")
        finally:
            manager.stop()
