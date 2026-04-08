"""
run_inference.py
================

Entry point for the real-time weight inference system.

Usage
-----
Demo mode (no hardware, random signals):
    python inference/run_inference.py --demo

Hardware mode:
    python inference/run_inference.py
"""

from __future__ import annotations

import sys
import os

# ---------------------------------------------------------------------------
# Windows DLL & MKL Bootstrap
# ---------------------------------------------------------------------------
if sys.platform == "win32":
    # 1. Allow duplicate MKL libs (common on Windows with multiple envs)
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
    
    # 2. Add System32 to DLL search path (required for some torch dependencies)
    _sys32 = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32")
    if os.path.isdir(_sys32):
        try:
            os.add_dll_directory(_sys32)
        except (AttributeError, OSError):
            pass
            
    # 3. Pre-import torch to trigger DLL loading early
    try:
        import torch
    except ImportError:
        pass

import argparse
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup — project root must be on sys.path for imports
# ---------------------------------------------------------------------------
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from inference.inference_config import (
    MODEL_RUN_DIR, WINDOW_S, INFERENCE_STEP_S,
    PREDICTION_EMA_ALPHA, SYNC_MIN_CONFIDENCE, ZERO_FILL_MISSING,
    STM32_PORT, STM32_BAUD, EMG_SAMPLE_RATE, EMG_CONNECTION_TYPE,
    PRBS_CHIP_RATE_HZ, PRBS_CORRELATION_WINDOW_S, PRBS_UPDATE_INTERVAL_S,
    EMG_BUFFER_S, STM32_BUFFER_S,
)

# Optional hardware imports
try:
    from setup_scripts.signal_acquisition_testing import (
        RawSTM32Thread, RawTMSiThread, _SyncWorker,
        _TMSI_AVAILABLE,
    )
    from src.stm32_emg_sync import SyncDelayEstimator
    from src.arduino_connection import find_arduino_port
    _HW_AVAILABLE = _TMSI_AVAILABLE
except ImportError as _e:
    print(f"[run_inference] Hardware imports failed ({_e}) — only demo mode available.")
    _HW_AVAILABLE = False
    RawSTM32Thread = RawTMSiThread = _SyncWorker = SyncDelayEstimator = None  # type: ignore

from inference.model_loader import ModelLoader
from inference.dashboard import launch_dashboard

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Real-time CNN-LSTM weight inference dashboard.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--model", type=str, default=MODEL_RUN_DIR, help="Path to model run directory.")
    parser.add_argument("--window", type=float, default=WINDOW_S, help="Window duration (s).")
    parser.add_argument("--step", type=float, default=INFERENCE_STEP_S, help="Inference interval (s).")
    parser.add_argument("--ema", type=float, default=PREDICTION_EMA_ALPHA, help="EMA alpha.")
    parser.add_argument("--demo", action="store_true", help="Demo mode.")
    parser.add_argument("--headless", action="store_true", help="Headless mode.")
    parser.add_argument("--port", type=str, default=None, help="STM32 port.")
    return parser.parse_args()

def _run_headless_demo(model: ModelLoader, args: argparse.Namespace):
    import random
    weights = [0.0, 0.9, 2.0, 2.94, 5.92]
    print("[HEADLESS DEMO] Running. Ctrl+C to stop.\n")
    ema = None
    try:
        while True:
            raw = random.choice(weights) + random.gauss(0, 0.1)
            raw = max(0.0, raw)
            ema = args.ema * raw + (1 - args.ema) * (ema if ema is not None else raw)
            print(f"[DEMO] raw={raw:.3f} kg | smoothed={ema:.3f} kg")
            time.sleep(args.step)
    except KeyboardInterrupt:
        print("\n[HEADLESS DEMO] Stopped.")

def _run_headless_hardware(engine, args):
    print("[HEADLESS] Running. Ctrl+C to stop.\n")
    prev_count = 0
    try:
        while True:
            time.sleep(0.5)
            raw, sm, ts = engine.get_prediction()
            if engine.inference_count != prev_count and sm is not None:
                prev_count = engine.inference_count
                print(f"[INFERENCE #{engine.inference_count}] raw={raw:.3f}kg | sm={sm:.3f}kg | sync={engine.sync_confidence:.2f}")
    except KeyboardInterrupt:
        print("\n[HEADLESS] Stopped.")

def main():
    args = _parse_args()
    print("=" * 60)
    print("  CNN-LSTM Real-Time Inference")
    print("=" * 60)
    
    # 1. Load model
    model = ModelLoader(run_dir=args.model)
    
    # 2. Demo mode
    if args.demo:
        if args.headless:
            _run_headless_demo(model, args)
        else:
            launch_dashboard(model=model, window_s=args.window, step_s=args.step, demo=True)
        return

    # 3. Hardware mode
    if not _HW_AVAILABLE:
        print("Hardware dependencies missing (TMSi SDK/pyserial). Use --demo.")
        sys.exit(1)

    stm32_port = args.port or STM32_PORT
    if stm32_port is None:
        try: stm32_port = find_arduino_port()
        except: pass
    if stm32_port is None:
        print("ERROR: No STM32 port found.")
        sys.exit(1)

    estimator = SyncDelayEstimator(
        chip_rate_hz=PRBS_CHIP_RATE_HZ,
        emg_sample_rate=float(EMG_SAMPLE_RATE),
        sync_window_s=PRBS_CORRELATION_WINDOW_S,
        update_interval_s=PRBS_UPDATE_INTERVAL_S,
    )

    stm32_thread = RawSTM32Thread(port=stm32_port, baud=STM32_BAUD, estimator=estimator)
    stm32_thread._history = []
    tmsi_thread = RawTMSiThread(sample_rate=EMG_SAMPLE_RATE, estimator=estimator)
    sync_worker = _SyncWorker(estimator)

    stm32_thread.start()
    tmsi_thread.start()

    # Wait for TMSi
    t_start = time.perf_counter()
    while not tmsi_thread.running and not tmsi_thread.error:
        if time.perf_counter() - t_start > 15: break
        time.sleep(0.1)

    if tmsi_thread.error or not tmsi_thread.running:
        print("TMSi initialization failed.")
        stm32_thread.stop()
        sys.exit(1)

    from inference.signal_buffer import SignalBuffer
    buffer = SignalBuffer(
        estimator=estimator, emg_sample_rate=EMG_SAMPLE_RATE,
        emg_buffer_s=EMG_BUFFER_S, stm32_buffer_s=STM32_BUFFER_S,
        zero_fill_missing=ZERO_FILL_MISSING,
    )
    buffer.set_trig_idx(tmsi_thread.trig_idx)

    import threading
    _running = [True]
    def _feed_loop():
        e_idx = 0; s_idx = 0
        last_sync_t = 0.0
        while _running[0]:
            time.sleep(0.02)
            now = time.perf_counter()

            # 1. Sync trigger
            if now - last_sync_t >= PRBS_UPDATE_INTERVAL_S:
                sync_worker.trigger()
                last_sync_t = now

            # 2. EMG Data
            with tmsi_thread._lock:
                chunk = list(tmsi_thread._history[e_idx:])
                e_idx = len(tmsi_thread._history)
            for t_arr, s_arr in chunk: buffer.add_emg_chunk(t_arr, s_arr)
            
            # 3. STM32 Data
            with stm32_thread._lock:
                chunk = list(stm32_thread._history[s_idx:])
                s_idx = len(stm32_thread._history)
            for pc, s in chunk: buffer.add_stm32_sample(pc, s)

    feed_thread = threading.Thread(target=_feed_loop, daemon=True)
    sync_worker.start()

    from inference.inference_engine import InferenceEngine
    engine = InferenceEngine(
        model=model, buffer=buffer, estimator=estimator,
        window_s=args.window, step_s=args.step, ema_alpha=args.ema,
        sync_min_confidence=SYNC_MIN_CONFIDENCE,
    )

    feed_thread.start()
    engine.start_engine()

    try:
        if args.headless:
            _run_headless_hardware(engine, args)
        else:
            launch_dashboard(
                engine=engine, buffer=buffer, estimator=estimator,
                stm32_thread=stm32_thread, tmsi_thread=tmsi_thread,
                model=model, window_s=args.window, step_s=args.step, demo=False,
            )
    finally:
        _running[0] = False
        engine.stop()
        sync_worker.stop()
        tmsi_thread.stop()
        stm32_thread.stop()

if __name__ == "__main__":
    main()
