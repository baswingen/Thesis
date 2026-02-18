"""
Synchronized STM32 + EMG Signal Acquisition (Real-time Visualization)
=====================================================================

Single mode: run both acquisition devices (STM32 + EMG) and PRBS synchronization
together with PyQtGraph real-time visualization.

Hardware:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

Run:
    python signal_acquisition_testing.py

Tuning: edit the CONFIG block below (duration, PRBS window, update interval, chip rate).
"""

from __future__ import annotations

import sys
import time
import struct
import threading
import queue
from collections import deque
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

import numpy as np
try:
    import serial
except ImportError:
    serial = None

# -----------------------------------------------------------------------------
# CONFIG – edit these to tune duration and PRBS sync (no CLI)
# -----------------------------------------------------------------------------
DURATION_S: Optional[float] = None  # Run indefinitely until window closed
PRBS_CORRELATION_WINDOW_S: float = 1  # Search range (chips = rate × window)
PRBS_UPDATE_INTERVAL_S: float = 0.1   # Update every 500 ms
PRBS_CHIP_RATE_HZ: float = 100.0  # Must match STM32 firmware (100 Hz chip rate)
PRBS_VIZ_WINDOW_S: float = 1  # Sliding window width in seconds for the plot
STM32_PORT: Optional[str] = None # Auto-detect
STM32_BAUD: int = 921600
EMG_CONNECTION_TYPE: str = "usb"
EMG_SAMPLE_RATE: Optional[int] = 2000
VERBOSE: bool = True
# -----------------------------------------------------------------------------

# PyQtGraph required
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    _PYQTGRAPH_AVAILABLE = False

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

try:
    from src.stm32_emg_sync import SyncDelayEstimator
    from src.stm32_reader import SampleSTM32, BIN_FORMAT, BIN_PACKET_SIZE, BIN_SYNC1, BIN_SYNC2
    
    # TMSi SDK Imports
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
    from TMSiSDK.device.devices.legacy.measurements.signal_measurement import SignalMeasurement
    _TMSI_AVAILABLE = True
except ImportError as e:
    print(f"Import Error: {e}")
    _TMSI_AVAILABLE = False


# =============================================================================
# RAW ACQUISITION THREADS
# =============================================================================

class RawTMSiThread(threading.Thread):
    """Direct acquisition from TMSi Porti7."""
    def __init__(self, sample_rate: int = 2000, estimator: Optional[SyncDelayEstimator] = None):
        super().__init__(name="RawTMSiThread", daemon=True)
        self.sample_rate = sample_rate
        self.estimator = estimator
        self.device: Optional[LegacyDevice] = None
        self.measurement: Optional[SignalMeasurement] = None
        self.running = False
        self.error: Optional[str] = None
        self.channels: List[str] = []
        self.trig_idx: int = -1
        self.estimated_rate_hz: float = 0.0

    def run(self):
        try:
            # 1. Discover devices with retries
            MAX_DISCOVERY_RETRIES = 5
            self.device = None
            
            for attempt in range(MAX_DISCOVERY_RETRIES):
                print(f"[TMSi] Discovery attempt {attempt + 1}/{MAX_DISCOVERY_RETRIES}...")
                
                # Ensure fresh start
                try: LegacyDevice.cleanup()
                except Exception: pass
                
                devices = LegacyDevice.discover("usb")
                
                # Filter for "real" names. 
                # "USB -" is a placeholder indicating the device is present but not fully ready.
                valid_devices = [d for d in devices if d._device_name and len(d._device_name) > 5 and "-" not in d._device_name[4:]]
                
                if valid_devices:
                    self.device = valid_devices[0]
                    break
                
                if devices:
                    print(f"[TMSi] Found {len(devices)} device(s), but names seem invalid (e.g. '{devices[0]._device_name}'). Device might be booting.")
                else:
                    print("[TMSi] No devices found.")
                
                if attempt < MAX_DISCOVERY_RETRIES - 1:
                    time.sleep(2.0) # Wait for device to stabilize
            
            if not self.device:
                raise RuntimeError("Failed to find a valid TMSi device after multiple attempts.")

            # 2. Open device with retries
            MAX_OPEN_RETRIES = 3
            opened = False
            for attempt in range(MAX_OPEN_RETRIES):
                try:
                    print(f"[TMSi] Opening {self.device._device_name} (attempt {attempt + 1})...")
                    self.device.open()
                    opened = True
                    break
                except Exception as e:
                    print(f"[TMSi] Open failed: {e}")
                    if attempt < MAX_OPEN_RETRIES - 1:
                        time.sleep(1.5)
            
            if not opened:
                raise RuntimeError(f"Failed to open device {self.device._device_name} after {MAX_OPEN_RETRIES} attempts.")
            
            # Identify TRIG channel (usually 'Dig' or 'STATUS')
            self.channels = [ch.get_channel_name() for ch in self.device.get_device_channels()]
            for i, name in enumerate(self.channels):
                if name.lower() in ("dig", "status", "trig"):
                    self.trig_idx = i
                    break
            
            if self.trig_idx == -1:
                print("[TMSi] WARNING: Trigger channel not found in:", self.channels)
            else:
                print(f"[TMSi] Using channel '{self.channels[self.trig_idx]}' for trigger")

            # Start Measurement
            self.measurement = SignalMeasurement(self.device)
            self.measurement.set_sample_rate(self.sample_rate)
            self.measurement.start()
            self.running = True
            
            last_print = time.perf_counter()
            count = 0
            
            while self.running:
                samples = self.measurement.get_samples(blocking=True)
                if samples is not None and len(samples) > 0:
                    now = time.perf_counter()
                    count += len(samples)
                    
                    # Feed estimator
                    if self.estimator is not None and self.trig_idx != -1:
                        # Timestamps are approximated from host clock + uniform
                        # spacing.  USB chunk arrival has 1-10 ms jitter, which
                        # propagates into the delay estimate.  A hardware-
                        # timestamped source would remove this limitation.
                        dt = 1.0 / self.sample_rate
                        t_arr = now - (len(samples) - 1 - np.arange(len(samples))) * dt
                        trig_vals = samples[:, self.trig_idx]
                        self.estimator.add_emg_trig_chunk(t_arr, trig_vals)
                    
                    if now - last_print > 1.0:
                        self.estimated_rate_hz = count / (now - last_print)
                        count = 0
                        last_print = now
                        
        except Exception as e:
            self.error = str(e)
            print(f"[TMSi] Error: {e}")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.measurement:
            try: self.measurement.stop()
            except Exception: pass
        if self.device:
            try: self.device.close()
            except Exception: pass


class RawSTM32Thread(threading.Thread):
    """Direct binary acquisition from STM32."""
    def __init__(self, port: str, baud: int = 921600, estimator: Optional[SyncDelayEstimator] = None):
        super().__init__(name="RawSTM32Thread", daemon=True)
        self.port = port
        self.baud = baud
        self.estimator = estimator
        self.running = False
        self.error: Optional[str] = None
        self.sample_count = 0
        # Scale history to ~6× the viz window to allow comfortable scrollback
        self._history = deque(maxlen=int(PRBS_CHIP_RATE_HZ * PRBS_VIZ_WINDOW_S * 6) + 1000)
        self._lock = threading.Lock()

    def run(self):
        if serial is None:
            self.error = "pyserial not installed"
            return
            
        self.running = True
        buffer = bytearray()
        ser = None
        last_rx = time.perf_counter()

        while self.running:
            # Reconnection logic if ser is None or closed
            if ser is None or not ser.is_open:
                try:
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                    
                    print(f"[STM32] Connecting to {self.port} at {self.baud}...")
                    ser = serial.Serial(self.port, self.baud, timeout=0.1)
                    buffer.clear()
                    last_rx = time.perf_counter()
                    print(f"[STM32] Connected to {self.port}")
                except Exception as e:
                    self.error = f"Connection failed: {e}"
                    time.sleep(1.0)
                    continue

            # Stall detection
            if time.perf_counter() - last_rx > 3.0:
                print("[STM32] Stall detected (>3s no data), reconnecting...")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                continue

            try:
                # Read data
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    time.sleep(0.001)
                    continue
                
                last_rx = time.perf_counter()
                buffer.extend(chunk)
                
                # Sync logic
                while len(buffer) >= BIN_PACKET_SIZE:
                    # Look for sync marker 0xAA 0x55
                    idx = -1
                    # Optimization: use bytearray.find
                    idx = buffer.find(bytes([BIN_SYNC1, BIN_SYNC2]))
                    
                    if idx == -1:
                        # No sync found, keep last byte in case it's 0xAA
                        if len(buffer) > 0 and buffer[-1] == BIN_SYNC1:
                            buffer = buffer[-1:]
                        else:
                            buffer.clear()
                        break
                    
                    if idx > 0:
                        # Trash pre-sync bytes
                        buffer = buffer[idx:]
                        
                    if len(buffer) < BIN_PACKET_SIZE:
                        break # Need more data
                    
                    # Parse packet
                    packet = buffer[:BIN_PACKET_SIZE]
                    
                    # Simple checksum validation (if implemented in firmware)
                    # XOR of bytes 2 to size-2 (matching src/stm32_reader.py logic)
                    cs = 0
                    for b in packet[2:-1]:
                        cs ^= b
                    
                    if cs != packet[-1]:
                        # Checksum failed, skip the sync and continue
                        buffer = buffer[2:]
                        continue

                    # Valid packet, consume from buffer
                    buffer = buffer[BIN_PACKET_SIZE:]
                    
                    try:
                        vals = struct.unpack(BIN_FORMAT, packet)
                        # t_ms is at index 2
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
                        
                        pc_time = time.perf_counter()
                        
                        with self._lock:
                            self._history.append((pc_time, sample))
                            self.sample_count += 1
                        
                        if self.estimator:
                            self.estimator.add_stm32_samples([sample])
                            
                    except Exception as e:
                        print(f"[STM32] Parse error: {e}")
                        continue
                        
            except (serial.SerialException, OSError) as e:
                print(f"[STM32] Serial error: {e}")
                ser = None # Trigger reconnection
                time.sleep(1.0)
                
        # Final cleanup
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def get_snapshot(self, n=5000):
        with self._lock:
            data = list(self._history)
            if not data: return {}
            count = min(len(data), n)
            subset = data[-count:]
            return {
                # t_sec: STM32 boot-relative time (millis()/1000), NOT PC wall time.
                "t_sec": np.array([s.t_ms / 1000.0 for t, s in subset]),
                "pc_time": np.array([t for t, s in subset]),
                "prbs_level": np.array([s.prbs_lvl for t, s in subset])
            }

    def stop(self):
        self.running = False


# Removed _build_config – initialization now inline in run()


def _extract_trig_binary(
    v_raw: np.ndarray, is_status: bool, trig_bit: Optional[int]
) -> np.ndarray:
    """Convert raw TRIG values to binary 0/1 array.

    Shared helper used by both the raw EMG plot and the
    synchronized overlay, avoiding duplicated logic.
    """
    if is_status and trig_bit is not None:
        return ((v_raw.astype(np.int64) >> trig_bit) & 1).astype(np.float64)
    lo, hi = np.nanmin(v_raw), np.nanmax(v_raw)
    thr = (lo + hi) / 2.0 if (hi - lo) > 1e-9 else 0.5
    return (v_raw > thr).astype(np.float64)


class _SyncWorker(threading.Thread):
    """Persistent background thread for PRBS cross-correlation updates.

    Replaces the previous pattern of spawning a new daemon thread every
    sync-tick (up to 20 Hz).  A single long-lived thread waits on an
    event and runs the estimator update when triggered.
    """

    def __init__(self, estimator: SyncDelayEstimator):
        super().__init__(name="PRBS-Sync-Worker", daemon=True)
        self.estimator = estimator
        self._trigger = threading.Event()
        self._running = True

    def trigger(self) -> None:
        """Signal the worker to attempt a sync update."""
        self._trigger.set()

    def stop(self) -> None:
        self._running = False
        self._trigger.set()  # unblock wait()

    def run(self) -> None:
        while self._running:
            self._trigger.wait()
            self._trigger.clear()
            if not self._running:
                break
            if self.estimator.should_update():
                self.estimator.update()


class PRBSVisualizationWindow(QtWidgets.QWidget):
    """Real-time PRBS visualization with shared timeline (STM32 time)."""

    def __init__(
        self,
        stm32_thread: RawSTM32Thread,
        tmsi_thread: RawTMSiThread,
        estimator: SyncDelayEstimator,
        duration_s: Optional[float] = None,
    ):
        super().__init__()
        self.stm32_thread = stm32_thread
        self.tmsi_thread = tmsi_thread
        self.estimator = estimator
        self.duration_s = duration_s
        self.start_time = time.perf_counter()
        self.prbs_window_s = PRBS_VIZ_WINDOW_S

        self.timer: Optional[QtCore.QTimer] = None
        self.sync_timer: Optional[QtCore.QTimer] = None
        self.duration_timer: Optional[QtCore.QTimer] = None

        self._last_stm32_len: int = 0
        self._last_stm32_hash: int = 0
        self._last_emg_len: int = 0
        self._last_emg_hash: int = 0
        self._last_prbs_err: Optional[str] = None

        # History buffers for delay and confidence time-series (~120 s at 30 fps)
        self._delay_history: deque = deque(maxlen=4000)
        self._raw_delay_history: deque = deque(maxlen=4000)
        self._confidence_history: deque = deque(maxlen=4000)

        self.setWindowTitle("PRBS Signal Monitor – STM32 + EMG")
        self.resize(1100, 950)
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        self.status_label = QtWidgets.QLabel("Acquiring PRBS signals...")
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; padding: 10px; "
            "background-color: #2c3e50; color: white;"
        )
        layout.addWidget(self.status_label)

        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout)

        # Plot 1: STM32 PRBS
        self.plot_stm32 = self.graphics_layout.addPlot(row=0, col=0, title="STM32 PRBS (Raw)")
        self.plot_stm32.setLabel("left", "Level")
        self.plot_stm32.setLabel("bottom", "Time", units="s")
        self.plot_stm32.setYRange(-1.2, 1.2)
        self.plot_stm32.showGrid(x=True, y=True, alpha=0.3)
        self.curve_stm32 = self.plot_stm32.plot(
            pen=pg.mkPen("#2ecc71", width=2), stepMode=True
        )

        # Plot 2: EMG TRIG
        self.plot_emg = self.graphics_layout.addPlot(row=1, col=0, title="Porti7 TRIG (EMG)")
        self.plot_emg.setLabel("left", "Level")
        self.plot_emg.setLabel("bottom", "Time", units="s")
        self.plot_emg.setYRange(-1.2, 1.2)
        self.plot_emg.showGrid(x=True, y=True, alpha=0.3)
        self.curve_emg = self.plot_emg.plot(
            pen=pg.mkPen("#3498db", width=2), stepMode=True
        )

        # Plot 3: Cross-Correlation Delay (ms)
        self.plot_delay = self.graphics_layout.addPlot(
            row=2, col=0, title="Cross-Correlation Delay"
        )
        self.plot_delay.setLabel("left", "Delay", units="ms")
        self.plot_delay.setLabel("bottom", "Time", units="s")
        self.plot_delay.showGrid(x=True, y=True, alpha=0.3)
        self.curve_delay = self.plot_delay.plot(
            pen=pg.mkPen(color="#e74c3c", width=2), name="Kalman"
        )
        self.curve_raw_delay = self.plot_delay.plot(
            pen=pg.mkPen(color="#e74c3c", width=1, style=QtCore.Qt.PenStyle.DashLine),
            name="Raw",
        )
        self.plot_delay.addLegend(offset=(10, 10))

        # Plot 4: Cross-Correlation Confidence
        self.plot_conf = self.graphics_layout.addPlot(
            row=3, col=0, title="Correlation Confidence"
        )
        self.plot_conf.setLabel("left", "Confidence")
        self.plot_conf.setLabel("bottom", "Time", units="s")
        self.plot_conf.setYRange(0, 1.05)
        self.plot_conf.showGrid(x=True, y=True, alpha=0.3)
        self.curve_conf = self.plot_conf.plot(
            pen=pg.mkPen(color="#f39c12", width=2),
            fillLevel=0,
            fillBrush=pg.mkBrush(243, 156, 18, 40),
        )
        # Confidence threshold line
        threshold_pen = pg.mkPen(color="#95a5a6", width=1, style=QtCore.Qt.PenStyle.DashLine)
        self.conf_threshold_line = pg.InfiniteLine(
            pos=0.15, angle=0, pen=threshold_pen, label="threshold",
            labelOpts={"position": 0.95, "color": "#95a5a6"},
        )
        self.plot_conf.addItem(self.conf_threshold_line)

        # Plot 5: Synchronized Overlay — EMG shifted by estimated delay
        self.plot_synced = self.graphics_layout.addPlot(
            row=4, col=0, title="Synchronized Overlay (delay-corrected)"
        )
        self.plot_synced.setLabel("left", "Level")
        self.plot_synced.setLabel("bottom", "Time", units="s")
        self.plot_synced.setYRange(-0.3, 1.5)
        self.plot_synced.showGrid(x=True, y=True, alpha=0.3)
        self.curve_sync_stm32 = self.plot_synced.plot(
            pen=pg.mkPen("#2ecc71", width=2), stepMode=True, name="STM32"
        )
        self.curve_sync_emg = self.plot_synced.plot(
            pen=pg.mkPen("#3498db", width=2), stepMode=True, name="EMG (shifted)"
        )
        self.plot_synced.addLegend(offset=(10, 10))

        # Link X axes for all plots
        self.plot_emg.setXLink(self.plot_stm32)
        self.plot_delay.setXLink(self.plot_stm32)
        self.plot_conf.setXLink(self.plot_stm32)
        self.plot_synced.setXLink(self.plot_stm32)

        # Only apply clipToView to non-stepMode curves.
        # stepMode curves require len(x) == len(y)+1; pyqtgraph's view-range
        # clipping can break that invariant and cause a crash.
        for curve in [self.curve_delay, self.curve_raw_delay, self.curve_conf]:
            curve.setClipToView(True)

        # Visualization update timer (~30 fps)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._on_viz_tick)
        self.timer.start(33)

        # Persistent sync worker (replaces thread-per-tick pattern)
        self._sync_worker = _SyncWorker(estimator)
        self._sync_worker.start()

        # Sync estimator update timer (triggers the persistent worker)
        sync_interval_ms = int(PRBS_UPDATE_INTERVAL_S * 1000)
        self.sync_timer = QtCore.QTimer()
        self.sync_timer.timeout.connect(self._on_sync_tick)
        self.sync_timer.start(sync_interval_ms)

        if self.duration_s is not None:
            self.duration_timer = QtCore.QTimer()
            self.duration_timer.timeout.connect(self._check_duration)
            self.duration_timer.start(1000)

    def _check_duration(self) -> None:
        if self.duration_s is not None:
            elapsed = time.perf_counter() - self.start_time
            if elapsed >= self.duration_s:
                if VERBOSE:
                    print(f"\n[VIZ] Duration {self.duration_s:.0f}s reached, closing...")
                self.close()

    def _on_sync_tick(self) -> None:
        """Trigger a cross-correlation update via the persistent worker."""
        if self.estimator is None:
            return

        self._sync_worker.trigger()

        # Log update attempt diagnostics
        if VERBOSE:
            elapsed = time.perf_counter() - self.start_time
            n_emg = self.estimator.emg_buf_len
            n_stm = self.estimator.stm32_buf_len
            updates = self.estimator._update_count
            print(
                f"[SYNC-TICK] t={elapsed:.1f}s  "
                f"emg_chips={n_emg}  stm32_chips={n_stm}  "
                f"updates={updates}"
            )

    def _on_viz_tick(self) -> None:
        """Update PRBS signal plots and cross-correlation plots."""
        # Capture snapshot data ONCE per tick so all plots share the same
        # time reference and data — prevents ~10ms window shift between plots.
        now = time.perf_counter()
        n_samples_stm32 = int(self.prbs_window_s * 550)
        stm32_snap = self.stm32_thread.get_snapshot(n=n_samples_stm32)

        emg_chunks = None
        if self.estimator is not None:
            emg_chunks = self.estimator.get_emg_trig_snapshot()

        self._update_prbs_plots(now, stm32_snap, emg_chunks)
        self._update_sync_plots()
        self._update_synced_plot(now, stm32_snap, emg_chunks)

        # Update status
        elapsed = now - self.start_time
        
        stm32_info = "STM32: --"
        if self.stm32_thread is not None:
            n_samples = self.stm32_thread.sample_count
            rate = n_samples / elapsed if elapsed > 0.5 else 0
            stm32_info = f"STM32: {n_samples} ({rate:.0f} Hz)"

        emg_info = "EMG: --"
        sync_info = "Sync: waiting"
        if self.estimator is not None:
            n_bufs = self.estimator.emg_buf_len
            true_rate = self.tmsi_thread.estimated_rate_hz
            emg_info = f"EMG: {true_rate:.1f} Hz"
            result = self.estimator.get_result()
            if result is not None:
                delay = self.estimator.get_delay_ms()
                sync_info = (
                    f"Delay: {delay:+.1f} ms | "
                    f"Conf: {result.confidence:.2f} | "
                    f"Updates: {self.estimator._update_count}"
                )
            else:
                sync_info = f"Sync: accumulating ({n_bufs} chips)"

        self.status_label.setText(
            f"{stm32_info} | {emg_info} | {sync_info} | {elapsed:.0f}s"
        )

        # DEBUG: Print status every ~2s
        if int(elapsed / 2) > getattr(self, "_last_debug_s", -1):
            self._last_debug_s = int(elapsed / 2)
            print(f"[DEBUG] {stm32_info} | {emg_info} | {sync_info}")

    def _update_prbs_plots(self, now: float, stm32_snap: dict, emg_chunks) -> None:
        """Fetch latest data and update curves."""
        try:
            # --- STM32 Data ---
            if stm32_snap and "t_sec" in stm32_snap and len(stm32_snap["t_sec"]) > 0:
                t_host = stm32_snap.get("pc_time", None)
                prbs_lvl = stm32_snap.get("prbs_level", None) 
                
                if t_host is not None and prbs_lvl is not None:
                    mask = t_host > (now - self.prbs_window_s)
                    t_plot = t_host[mask]
                    y_plot = prbs_lvl[mask]
                    
                    if len(t_plot) > 1:
                        t_rel = t_plot - self.start_time
                        # Pad frame for stepMode (len(x) = len(y) + 1)
                        if len(t_rel) > 1:
                            dt = t_rel[-1] - t_rel[-2]
                            t_padded = np.append(t_rel, t_rel[-1] + dt)
                            self.curve_stm32.setData(t_padded, y_plot)
            
            # --- EMG Data ---
            if emg_chunks:
                t_raw = np.concatenate([c[0] for c in emg_chunks])
                v_raw = np.concatenate([c[1] for c in emg_chunks])
                
                is_status = getattr(self.estimator, "_trig_is_status", True)
                trig_bit = getattr(self.estimator, "_detected_trig_bit", None)
                y_plot = _extract_trig_binary(v_raw, is_status, trig_bit)
                
                mask = t_raw > (now - self.prbs_window_s)
                t_plot = t_raw[mask]
                y_plot = y_plot[mask]
                
                if len(t_plot) > 1:
                    t_rel = t_plot - self.start_time
                    # Pad frame for stepMode (len(x) = len(y) + 1)
                    if len(t_rel) > 1:
                        dt = t_rel[-1] - t_rel[-2]
                        t_padded = np.append(t_rel, t_rel[-1] + dt)
                        self.curve_emg.setData(t_padded, y_plot)

            # Update X Range to latest time
            current_rel = now - self.start_time
            self.plot_stm32.setXRange(max(0, current_rel - self.prbs_window_s), current_rel + 0.5)

        except Exception as exc:
            if self._last_prbs_err != str(exc):
                print(f"[VIZ] PRBS update error: {exc}")
                self._last_prbs_err = str(exc)

    def _update_sync_plots(self) -> None:
        """Read cross-correlation results and update delay / confidence plots."""
        if self.estimator is None:
            return

        elapsed = time.perf_counter() - self.start_time
        result = self.estimator.get_result()
        if result is None:
            return

        # Current Kalman-smoothed delay
        delay_ms = self.estimator.get_delay_ms()
        raw_delay_ms = result.delay_ms
        confidence = result.confidence

        # Append to history
        self._delay_history.append((elapsed, delay_ms))
        self._raw_delay_history.append((elapsed, raw_delay_ms))
        self._confidence_history.append((elapsed, confidence))

        # Delay plot
        if self._delay_history:
            arr = np.array(self._delay_history)
            self.curve_delay.setData(arr[:, 0], arr[:, 1])
        if self._raw_delay_history:
            arr = np.array(self._raw_delay_history)
            self.curve_raw_delay.setData(arr[:, 0], arr[:, 1])

        # Confidence plot
        if self._confidence_history:
            arr = np.array(self._confidence_history)
            self.curve_conf.setData(arr[:, 0], arr[:, 1])

    def _update_synced_plot(self, now: float, stm32_snap: dict, emg_chunks) -> None:
        """Update the synchronized overlay plot with delay-corrected EMG."""
        if self.estimator is None:
            return

        # Use Kalman-smoothed delay — more reliable than single-update RAW
        delay_ms = self.estimator.get_delay_ms()
        self.plot_synced.setTitle(f"Synchronized Overlay (Kalman delay: {delay_ms:.1f} ms)")
             
        delay_s = delay_ms / 1000.0

        # --- STM32 Data (same snapshot as raw plot) ---
        if stm32_snap and "t_sec" in stm32_snap:
            t_host = stm32_snap.get("pc_time", None)
            prbs_lvl = stm32_snap.get("prbs_level", None)
            
            if t_host is not None and prbs_lvl is not None:
                mask = t_host > (now - self.prbs_window_s)
                t_plot = t_host[mask]
                y_plot = prbs_lvl[mask]
                
                if len(t_plot) > 1:
                    t_stm32_rel = t_plot - self.start_time
                    # Pad for stepMode
                    if len(t_stm32_rel) > 1:
                        dt = t_stm32_rel[-1] - t_stm32_rel[-2]
                        t_padded = np.append(t_stm32_rel, t_stm32_rel[-1] + dt)
                        self.curve_sync_stm32.setData(t_padded, y_plot)

        # --- EMG Data (same chunks as raw plot) ---
        if emg_chunks:
            t_raw = np.concatenate([c[0] for c in emg_chunks])
            v_raw = np.concatenate([c[1] for c in emg_chunks])
            
            is_status = getattr(self.estimator, "_trig_is_status", True)
            trig_bit = getattr(self.estimator, "_detected_trig_bit", None)
            y_plot = _extract_trig_binary(v_raw, is_status, trig_bit)
            
            mask = t_raw > (now - self.prbs_window_s)
            t_plot = t_raw[mask]
            y_plot = y_plot[mask]
            
            if len(t_plot) > 1:
                # No group-delay correction: we use raw sample timestamps, not downsampled chips
                t_rel = (t_plot - delay_s) - self.start_time
                
                # Pad for stepMode
                if len(t_rel) > 1:
                    dt = t_rel[-1] - t_rel[-2]
                    t_padded = np.append(t_rel, t_rel[-1] + dt)
                    self.curve_sync_emg.setData(t_padded, y_plot)

    def closeEvent(self, event) -> None:
        if self.timer:
            self.timer.stop()
        if self.sync_timer:
            self.sync_timer.stop()
        if self.duration_timer:
            self.duration_timer.stop()
        if hasattr(self, '_sync_worker'):
            self._sync_worker.stop()
        event.accept()


def run() -> None:
    """Run real-time visualization with raw acquisition threads."""
    if not _PYQTGRAPH_AVAILABLE:
        print("\nERROR: PyQtGraph is required. Install with: pip install pyqtgraph PyQt6")
        sys.exit(1)
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run acquisition.")
        sys.exit(1)
    if serial is None:
        print("\nERROR: pyserial is required. Install with: pip install pyserial")
        sys.exit(1)

    print("=" * 60)
    print("STM32 + EMG PRBS Real-time Monitor (RAW ACQUISITION)")
    print("=" * 60)
    print(f"  Duration: {DURATION_S if DURATION_S else 'Indefinite'} s")

    # 1. Initialize Sync Estimator
    estimator = SyncDelayEstimator(
        chip_rate_hz=PRBS_CHIP_RATE_HZ,
        emg_sample_rate=float(EMG_SAMPLE_RATE or 2000),
        sync_window_s=PRBS_CORRELATION_WINDOW_S,
        update_interval_s=PRBS_UPDATE_INTERVAL_S,
    )

    # 2. STM32 Thread
    stm32_port = STM32_PORT
    if stm32_port is None:
        try:
            from src.arduino_connection import find_arduino_port
            stm32_port = find_arduino_port(verbose=VERBOSE)
        except ImportError:
            pass
    
    if stm32_port is None:
        print("[ERROR] STM32 port not found. Specify STM32_PORT in CONFIG.")
        sys.exit(1)

    stm32_thread = RawSTM32Thread(port=stm32_port, baud=STM32_BAUD, estimator=estimator)
    
    # 3. TMSi Thread
    tmsi_thread = RawTMSiThread(sample_rate=(EMG_SAMPLE_RATE or 2000), estimator=estimator)

    try:
        # Start Threads
        stm32_thread.start()
        tmsi_thread.start()
        
        # Wait for TMSi to be ready or fail
        t_start = time.perf_counter()
        while not tmsi_thread.running and not tmsi_thread.error:
            if time.perf_counter() - t_start > 10:
                break
            time.sleep(0.1)
        
        if tmsi_thread.error:
            raise RuntimeError(f"TMSi Thread Error: {tmsi_thread.error}")

        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv)
            
        app.setStyle("Fusion")
        # Dark theme
        from pyqtgraph.Qt import QtGui
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor(25, 25, 25))
        palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.ToolTipBase, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.ToolTipText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Text, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.BrightText, QtCore.Qt.GlobalColor.red)
        palette.setColor(QtGui.QPalette.ColorRole.Link, QtGui.QColor(42, 130, 218))
        palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(42, 130, 218))
        palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtCore.Qt.GlobalColor.black)
        app.setPalette(palette)

        window = PRBSVisualizationWindow(
            stm32_thread=stm32_thread,
            tmsi_thread=tmsi_thread,
            estimator=estimator,
            duration_s=DURATION_S
        )
        window.show()

        app.exec() if hasattr(app, "exec") else app.exec_()
        
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user.")
    except Exception as e:
        print(f"\n[MAIN] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[MAIN] Stopping threads...")
        stm32_thread.stop()
        tmsi_thread.stop()
        stm32_thread.join(timeout=2)
        tmsi_thread.join(timeout=2)
        
        # Final SDK cleanup
        try:
            LegacyDevice.cleanup()
        except Exception:
            pass
        print("[MAIN] Done.")

if __name__ == "__main__":
    # Import QtGui here for the palette code above
    from pyqtgraph.Qt import QtGui
    run()
