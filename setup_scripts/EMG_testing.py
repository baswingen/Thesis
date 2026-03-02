"""
TMSi Porti7/REFA Signal Acquisition (PyQt Version)
====================================================

High-performance signal acquisition script for legacy TMSi devices 
(Porti7, REFA, REFA Extended).
Features:
- Threaded acquisition pipeline for stable sampling (avoids blocking main thread)
- PyQtGraph-based real-time visualization
- Optional digital bandpass filtering using scipy
- Clean and robust device discovery and connection

Prerequisites:
- TMSi device connected via USB
- PyQt6 and pyqtgraph installed (`pip install pyqtgraph PyQt6`)
- Optional: scipy (`pip install scipy`) for filtering
"""

import sys
import os
import time
import threading
from collections import deque
from typing import Optional, List, Tuple
import numpy as np
import warnings

# Add project root and TMSi Python Interface to path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

tmsi_interface_path = os.path.join(PROJECT_ROOT, 'tmsi-python-interface')
if os.path.exists(tmsi_interface_path) and tmsi_interface_path not in sys.path:
    sys.path.insert(0, tmsi_interface_path)

try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    import pyqtgraph as pg
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    print("Error: PyQt6 and pyqtgraph are required.")
    print("Run: pip install PyQt6 pyqtgraph")
    sys.exit(1)

try:
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
    from TMSiSDK.device.devices.legacy.measurements.signal_measurement import SignalMeasurement
    _TMSI_AVAILABLE = True
except ImportError as e:
    print("="*60)
    print("ERROR: TMSi Python Interface not found!")
    print("="*60)
    print(f"\nImport error: {e}")
    sys.exit(1)

# Optional filtering
try:
    from scipy import signal
    from src.emg_processing import EMGProcessor
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False
    print("Warning: scipy or src.emg_processing not available - filtering disabled")


# ============================================================================
# SETTINGS
# ============================================================================
DEVICE_INTERFACE = "usb"  # "usb" or "bluetooth"
SAMPLE_RATE = 2000        # Try to request this sample rate

# EMG channel configuration
USE_EMG_DIFFERENTIAL_PAIR = True
EMG_PAIR_CHANNELS_ARE_1_BASED = True
EMG_PAIR_POS_CHANNEL = 1   # Channel number (1-based)
EMG_PAIR_NEG_CHANNEL = 2   # Channel number (1-based)

# Filter Settings
APPLY_BANDPASS_FILTER = True
BANDPASS_LOW_CUTOFF = 20.0
BANDPASS_HIGH_CUTOFF = 450.0
FILTER_ORDER = 4

PLOT_WINDOW_SECONDS = 5.0
SMOOTHING_WINDOW_SECONDS = 0.2

# Filter settings moved to internal configuration

# ============================================================================
# ACQUISITION THREAD
# ============================================================================
class TMSiAcquisitionThread(threading.Thread):
    """Background thread handling connection and data reading for TMSi device."""
    def __init__(self, sample_rate=2000):
        super().__init__(name="TMSiAcqThread", daemon=True)
        self.sample_rate = sample_rate
        self.device: Optional[LegacyDevice] = None
        self.measurement: Optional[SignalMeasurement] = None
        self.running = False
        self.channels_info = []
        
        # Buffer passing data to UI. Stores tuples of (t_array, data_array)
        self.data_queue = deque(maxlen=200) 
        
        # Stats
        self.estimated_rate_hz = 0.0
        self.sample_count = 0
        self.error: str = ""
        self.device_name = ""

    def run(self):
        self.running = True
        try:
            # Discovery
            print("[TMSi] Scanning for Legacy Devices...")
            devices = LegacyDevice.discover(DEVICE_INTERFACE)
            valid_devices = [d for d in devices if d._device_name and len(d._device_name) > 5 and "-" not in d._device_name[4:]]

            if not valid_devices:
                self.error = "No valid TMSi devices found."
                print(f"[TMSi] {self.error}")
                self.running = False
                return
            
            self.device = valid_devices[0]
            self.device_name = self.device._device_name
            print(f"[TMSi] Opening device: {self.device_name}")
            
            self.device.open()
            print("[TMSi] Device opened successfully.")
            
            # Extract channel info for mapping
            for i, ch in enumerate(self.device.get_device_channels()):
                try:
                    cname = ch.get_channel_name()
                    cunit = ch.get_channel_unit_name()
                except:
                    cname = f"CH{i}"
                    cunit = ""
                self.channels_info.append({"index": i, "name": cname, "unit": cunit})
                
            self.measurement = SignalMeasurement(self.device)
            self.measurement.set_sample_rate(self.sample_rate)
            self.measurement.start()
            
            actual_sr = self.measurement.get_device_sample_rate()
            print(f"[TMSi] Measurement started. Rate: {actual_sr} Hz")
            
            last_stats_t = time.perf_counter()
            count_since_last = 0
            t_offset = 0.0
            dt = 1.0 / actual_sr if actual_sr > 0 else 0.0005
            
            while self.running:
                try:
                    # blocking=True so thread yields if no data
                    samples = self.measurement.get_samples(blocking=True)
                    
                    if samples is not None and len(samples) > 0:
                        n = samples.shape[0]
                        self.sample_count += n
                        count_since_last += n
                        
                        t_chunk = t_offset + np.arange(n) * dt
                        t_offset = t_chunk[-1] + dt
                        
                        # Offload to UI queue
                        self.data_queue.append((t_chunk, samples))
                        
                        now = time.perf_counter()
                        if now - last_stats_t > 1.0:
                            self.estimated_rate_hz = count_since_last / (now - last_stats_t)
                            count_since_last = 0
                            last_stats_t = now
                except Exception as e:
                    print(f"[TMSi] Warning: get_samples failed: {e}")
                    time.sleep(0.1)

        except Exception as e:
            self.error = f"Pipeline Error: {e}"
            print(f"[TMSi] {self.error}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        if self.measurement:
            try: self.measurement.stop()
            except: pass
        if self.device:
            try: self.device.close()
            except: pass


# ============================================================================
# UI APPLICATION
# ============================================================================
class EMGWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-time EMG Acquisition (TMSi)")
        self.resize(1100, 700)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; }
            QLabel { color: #ecf0f1; font-family: 'Segoe UI', sans-serif; }
            QFrame#stats { background-color: #34495e; border-radius: 6px; padding: 10px; }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # --- Sidebar ---
        sidebar = QtWidgets.QFrame()
        sidebar.setFixedWidth(260)
        side_layout = QtWidgets.QVBoxLayout(sidebar)
        side_layout.setContentsMargins(0,0,0,0)
        
        title = QtWidgets.QLabel("EMG SETUP")
        title.setStyleSheet("font-weight: bold; font-size: 16px; color: #3498db; margin-bottom: 10px;")
        side_layout.addWidget(title)
        
        self.stats_frame = QtWidgets.QFrame()
        self.stats_frame.setObjectName("stats")
        sf_layout = QtWidgets.QVBoxLayout(self.stats_frame)
        
        self.lbl_status = QtWidgets.QLabel("Status: Initializing...")
        self.lbl_status.setStyleSheet("font-weight: bold;")
        self.lbl_device = QtWidgets.QLabel("Device: --")
        self.lbl_srate = QtWidgets.QLabel("Rate: -- Hz")
        self.lbl_samples = QtWidgets.QLabel("Total: 0 samples")
        self.lbl_ch_info = QtWidgets.QLabel("Config: Pending...")
        self.lbl_ch_info.setWordWrap(True)
        self.lbl_ch_info.setStyleSheet("color: #f39c12; font-size: 11px;")

        for w in [self.lbl_status, self.lbl_device, self.lbl_srate, self.lbl_samples, self.lbl_ch_info]:
            sf_layout.addWidget(w)
            
        side_layout.addWidget(self.stats_frame)
        side_layout.addStretch()
        
        layout.addWidget(sidebar)
        
        # --- Main Plotting Area ---
        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout, stretch=1)
        
        # Plot 1: Raw
        self.plot_raw = self.graphics_layout.addPlot(row=0, col=0, title="Raw EMG Signal")
        self.plot_raw.setLabel("left", "Voltage", units="µV")
        self.plot_raw.showGrid(x=True, y=True, alpha=0.3)
        self.curve_raw = self.plot_raw.plot(pen=pg.mkPen('#3498db', width=1.5))
        
        # Plot 2: Processed
        self.plot_filtered = self.graphics_layout.addPlot(row=1, col=0, title="Processed EMG Signal (Bandpass + Rectified + Smoothed)")
        self.plot_filtered.setLabel("left", "Voltage", units="µV")
        self.plot_filtered.setLabel("bottom", "Time", units="s")
        self.plot_filtered.showGrid(x=True, y=True, alpha=0.3)
        self.curve_filtered = self.plot_filtered.plot(pen=pg.mkPen('#e74c3c', width=1.5))
        
        self.plot_filtered.setXLink(self.plot_raw)
        
        # --- State ---
        self.tmsi_thread = TMSiAcquisitionThread(sample_rate=SAMPLE_RATE)
        self.processor: Optional[EMGProcessor] = None
        
        self.pos_idx = -1
        self.neg_idx = -1
        self.emg_configured = False
        
        # Render buffers
        max_pts = int(PLOT_WINDOW_SECONDS * SAMPLE_RATE) + 1000
        self.time_buf = deque(maxlen=max_pts)
        self.raw_buf = deque(maxlen=max_pts)
        self.filt_buf = deque(maxlen=max_pts)

        # Timers
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(33) # ~30FPS refresh

        # Start acquisition
        self.tmsi_thread.start()


    def configure_emg_channel(self):
        """Map the configured channel numbers/indices to the actual layout."""
        if not self.tmsi_thread.channels_info:
            return False
            
        n_ch = len(self.tmsi_thread.channels_info)
        
        try:
            p_raw = int(EMG_PAIR_POS_CHANNEL)
            n_raw = int(EMG_PAIR_NEG_CHANNEL)
        except ValueError:
            self.lbl_ch_info.setText("Config Error: Invalid POS/NEG channels.")
            return False

        if EMG_PAIR_CHANNELS_ARE_1_BASED:
            p_idx = p_raw - 1
            n_idx = n_raw - 1
        else:
            p_idx = p_raw
            n_idx = n_raw
            
        if not (0 <= p_idx < n_ch) or not (0 <= n_idx < n_ch):
            self.lbl_ch_info.setText(f"Config Error: Indices out of range (0-{n_ch-1}).")
            return False
            
        self.pos_idx = p_idx
        self.neg_idx = n_idx
        self.emg_configured = True
        
        c1 = self.tmsi_thread.channels_info[p_idx]['name']
        c2 = self.tmsi_thread.channels_info[n_idx]['name']
        
        info = f"Sampling: Differential\n+ : [{p_idx}] {c1}\n- : [{n_idx}] {c2}"
        self.lbl_ch_info.setText(info)
        
        # Update Raw Plot Title with channel info
        self.plot_raw.setTitle(f"Raw EMG: {c1} - {c2}")
        
        if APPLY_BANDPASS_FILTER and _SCIPY_AVAILABLE:
            sr = self.tmsi_thread.sample_rate # fallback
            if self.tmsi_thread.measurement:
                sr = self.tmsi_thread.measurement.get_device_sample_rate()
            
            # Use centralized EMGProcessor for filtering and envelope extraction
            envelope_cutoff = 1.0 / SMOOTHING_WINDOW_SECONDS if SMOOTHING_WINDOW_SECONDS > 0 else 10.0
            self.processor = EMGProcessor(
                fs=sr,
                bandpass_low=BANDPASS_LOW_CUTOFF,
                bandpass_high=BANDPASS_HIGH_CUTOFF,
                bandpass_order=FILTER_ORDER,
                envelope_cutoff=envelope_cutoff
            )
            # Premium formatted title reflecting the src.emg_processing pipeline
            self.plot_filtered.setTitle(
                f"Processed EMG ({BANDPASS_LOW_CUTOFF}-{BANDPASS_HIGH_CUTOFF}Hz BP "
                f"\u2192 Rectified \u2192 {envelope_cutoff:.1f}Hz LP Envelope)"
            )
        
        return True


    def update_ui(self):
        if not self.tmsi_thread.running and not self.tmsi_thread.error:
            return
            
        if self.tmsi_thread.error:
            self.lbl_status.setText(f"ERROR: {self.tmsi_thread.error}")
            self.lbl_status.setStyleSheet("color: #e74c3c; font-weight: bold;")
            return
            
        self.lbl_status.setText("Status: Recording")
        self.lbl_status.setStyleSheet("color: #2ecc71; font-weight: bold;")
        self.lbl_device.setText(f"Device: {self.tmsi_thread.device_name}")
        self.lbl_srate.setText(f"Rate: {self.tmsi_thread.estimated_rate_hz:.1f} Hz")
        self.lbl_samples.setText(f"Total: {self.tmsi_thread.sample_count}")
        
        if not self.emg_configured:
            if not self.configure_emg_channel():
                return
                
        # Drain queue
        chunks_t = []
        chunks_v = []
        
        while self.tmsi_thread.data_queue:
            t, chunk = self.tmsi_thread.data_queue.popleft()
            if len(chunk) > 0 and chunk.shape[1] > max(self.pos_idx, self.neg_idx):
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", RuntimeWarning)
                    emg_diff = chunk[:, self.pos_idx] - chunk[:, self.neg_idx]
                    
                chunks_t.append(t)
                chunks_v.append(emg_diff)

        if not chunks_t:
            return
            
        # Concatenate and filter out NaNs
        t_batch = np.concatenate(chunks_t)
        v_batch = np.concatenate(chunks_v)
        
        valid_mask = ~np.isnan(v_batch)
        if not np.any(valid_mask):
            return
            
        t_valid = t_batch[valid_mask]
        v_valid = v_batch[valid_mask]
        
        self.time_buf.extend(t_valid)
        self.raw_buf.extend(v_valid)
        
        if self.processor is not None:
            _, envelope = self.processor.process(v_valid, return_envelope=True)
            self.filt_buf.extend(envelope)
            
        # Redraw
        if len(self.time_buf) > 0:
            t_arr = np.array(self.time_buf)
            
            # Use relative time for nicer x-axis
            t_rel = t_arr - t_arr[0]
            if t_arr[-1] - t_arr[0] > PLOT_WINDOW_SECONDS:
                t_rel = t_arr - (t_arr[-1] - PLOT_WINDOW_SECONDS)
                
            self.curve_raw.setData(t_rel, np.array(self.raw_buf))
            self.plot_raw.setXRange(max(0, t_rel[-1] - PLOT_WINDOW_SECONDS), t_rel[-1])
            
            if self.processor is not None and len(self.filt_buf) == len(self.time_buf):
                self.curve_filtered.setData(t_rel, np.array(self.filt_buf))


    def closeEvent(self, event):
        self.update_timer.stop()
        self.tmsi_thread.cleanup()
        event.accept()

# ============================================================================
# MAIN
# ============================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    
    # We use a slightly different style from dual_BNO085 but consistent dark theme
    app.setStyle("Fusion")
    
    win = EMGWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
