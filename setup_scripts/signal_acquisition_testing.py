"""
Synchronized STM32 + EMG Signal Acquisition (performance testing)
================================================================

CLI wrapper around the native signal acquisition module in src.
Uses the integrated PRBS synchronization (stm32_emg_sync) with:
- Real-time mode: Kalman-smoothed delay estimation during live acquisition
- Postprocessing mode: Records both streams, computes delay signal for offline alignment

Performance tests for real-time synchronized acquisition of:
- Dual BNO085 IMU + button matrix + PRBS (via STM32F401, 500 Hz)
- Multi-channel EMG differential pairs (via TMSi Porti7, ~2000 Hz)

PRBS synchronization (10 Hz chip rate, PRBS-15):
    STM32 outputs PRBS on PA8 -> Porti7 TRIG. Cross-correlates EMG TRIG
    with STM32 chip stream for clock offset. Real-time mode uses Kalman
    smoothing; postprocessing outputs full (t_s, delay_ms) for alignment.

Hardware setup:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

Run modes:
    --mode sequence     Run in order: STM32, EMG, combined (default).
    --mode stm32        STM32 only (500 Hz).
    --mode emg          EMG only (~2000 Hz).
    --mode prbs         Combined STM32 + EMG + PRBS sync (alias for all).
    --mode all          Combined STM32 + EMG + real-time PRBS sync.
    --mode postprocess  Record for duration, then compute delay signal.

Sync mode (for all/prbs):
    --sync-mode realtime     Kalman-smoothed delay during acquisition (default).
    --sync-mode postprocessing  Collect data, compute delay signal at end.

Visualization (default for prbs/all with realtime sync):
    Real-time plot: STM32 PRBS, Porti7 PRBS, sync offset, drift, confidence.
    Use --no-visualize to disable.

Examples:
    python signal_acquisition_testing.py --mode sequence --duration 10
    python signal_acquisition_testing.py --mode all --duration 15
    python signal_acquisition_testing.py --mode postprocess --duration 30

Programmatic access:
    from src import SignalAcquisition, SignalAcquisitionConfig
    config = SignalAcquisitionConfig(sync_mode="realtime")
    acq = SignalAcquisition(config)
    acq.start()
    delay_ms = acq.get_sync_delay_ms()
    acq.stop()
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np

# Try to import PyQtGraph for high-performance visualization
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    _PYQTGRAPH_AVAILABLE = False

# Ensure project root is on path when running from setup_scripts
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.signal_acquisition import (
    SignalAcquisition,
    SignalAcquisitionConfig,
    run_stm32_only,
    run_emg_only,
    run_prbs_test,
    run_combined,
)

try:
    from src.signal_acquisition import _TMSI_AVAILABLE
except ImportError:
    _TMSI_AVAILABLE = False


def run_postprocessing(config: SignalAcquisitionConfig, duration_s: float) -> None:
    """Record STM32 + EMG, compute delay signal for postprocessing alignment."""
    config = SignalAcquisitionConfig(
        enable_stm32=True,
        enable_emg=True,
        enable_prbs_sync=True,
        sync_mode="postprocessing",
        verbose=config.verbose,
        stm32_port=config.stm32_port,
        stm32_baud=config.stm32_baud,
        emg_connection_type=config.emg_connection_type,
        emg_differential_pairs=config.emg_differential_pairs,
        emg_sample_rate=config.emg_sample_rate,
    )
    acq = SignalAcquisition(config)
    acq.start()
    session = acq.record_for_postprocessing(duration_s=duration_s)
    if session is not None and config.verbose:
        print(f"\n  [POSTPROCESSING] Delay signal: {len(session.delay_signal.t_s)} points")
        print(f"  [POSTPROCESSING] Median delay: {session.sync_result.delay_ms:.2f} ms")
        print(f"  [POSTPROCESSING] EMG chunks: {len(session.emg_chunks)}, STM32 samples: {len(session.stm32_samples)}")
    elif session is None and config.verbose:
        print("\n  [POSTPROCESSING] Could not compute delay (insufficient data)")


# Target 30 fps for PRBS visualization
_VIZ_FPS = 30
_VIZ_FRAME_S = 1.0 / _VIZ_FPS


def _init_prbs_artists(
    fig: plt.Figure, axes: List[plt.Axes]
) -> Dict[str, object]:
    """Create reusable line artists for blitted updates."""
    # PRBS step plots (use drawstyle='steps-mid')
    stm32_line, = axes[0].plot([], [], drawstyle="steps-mid", color="#2ecc71", linewidth=0.8)
    emg_line, = axes[1].plot([], [], drawstyle="steps-mid", color="#3498db", linewidth=0.8)
    offset_line, = axes[2].plot([], [], color="#9b59b6", linewidth=1.2)
    drift_line, = axes[3].plot([], [], color="#e74c3c", linewidth=1.2)
    confidence_line, = axes[4].plot([], [], color="#f39c12", linewidth=1.2)

    for ax in axes:
        ax.set_autoscale_on(False)
    axes[0].set_ylabel("STM32 PRBS")
    axes[0].set_ylim(-0.2, 1.2)
    axes[0].set_xlim(0, 500)
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("Porti7 PRBS")
    axes[1].set_ylim(-0.2, 1.2)
    axes[1].set_xlim(0, 500)
    axes[1].grid(True, alpha=0.3)
    axes[2].set_ylabel("Offset (ms)")
    axes[2].set_ylim(-100, 100)  # Initial range, will auto-update
    axes[2].grid(True, alpha=0.3)
    axes[3].set_ylabel("Drift (ppm)")
    axes[3].set_ylim(-5000, 5000)  # Cover observed range ±3000 ppm
    axes[3].grid(True, alpha=0.3)
    axes[4].set_ylabel("Confidence")
    axes[4].set_ylim(0, 1.1)
    axes[4].set_xlabel("Time (s)")
    axes[4].grid(True, alpha=0.3)

    waiting_text = fig.text(0.5, 0.15, "waiting for sync...", ha="center", va="center", fontsize=10)
    
    # Status indicator text at top
    status_text = fig.text(
        0.5, 0.96, "", 
        ha="center", va="top", 
        fontsize=12, fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="gray", alpha=0.8, edgecolor="none")
    )

    return {
        "stm32_line": stm32_line,
        "emg_line": emg_line,
        "offset_line": offset_line,
        "drift_line": drift_line,
        "confidence_line": confidence_line,
        "waiting_text": waiting_text,
        "status_text": status_text,
    }


def _update_prbs_visualization(
    acq: SignalAcquisition,
    axes: List[plt.Axes],
    artists: Dict[str, object],
    history: Dict[str, List[float]],
    start_time: float,
) -> None:
    """Update PRBS plot via set_data (no clear/redraw) for 30 fps."""
    estimator = getattr(acq, "sync_delay_estimator", None)
    if estimator is None:
        return

    now = time.perf_counter()
    sync_state = acq.get_sync_state()

    stm32_chips, emg_chips = estimator.get_chip_streams_for_display(max_chips=300)

    # Apply time alignment using measured offset
    if stm32_chips is not None and len(stm32_chips) > 0:
        stm32_x = np.arange(len(stm32_chips))
        artists["stm32_line"].set_data(stm32_x, stm32_chips)
        
    if emg_chips is not None and len(emg_chips) > 0:
        # get_chip_streams_for_display already time-aligns the arrays by
        # integer chip lag.  Only apply the fractional chip remainder.
        frac_offset = 0.0
        if sync_state is not None:
            total_offset = sync_state.offset_ms / 1000.0 * estimator.chip_rate_hz
            frac_offset = total_offset - round(total_offset)
        
        emg_x = np.arange(len(emg_chips)) + frac_offset
        artists["emg_line"].set_data(emg_x, emg_chips)
        
        # Update x-axis limits to show both signals (scrolling window)
        if stm32_chips is not None and len(stm32_chips) > 0:
            max_x = max(stm32_x[-1] if len(stm32_x) > 0 else 0,
                       emg_x[-1] if len(emg_x) > 0 else 0)
            axes[0].set_xlim(max(0, max_x - 300), max_x + 10)
            axes[1].set_xlim(max(0, max_x - 300), max_x + 10)

    if sync_state is not None:
        history["t"].append(now - start_time)
        history["offset"].append(sync_state.offset_ms)
        history["drift"].append(sync_state.drift_rate_ppm)
        history["confidence"].append(sync_state.confidence)
        max_history = 200
        for key in ("t", "offset", "drift", "confidence"):
            history[key] = history[key][-max_history:]

        artists["waiting_text"].set_visible(False)
        t_hist = np.array(history["t"], dtype=np.float64)
        artists["offset_line"].set_data(t_hist, history["offset"])
        artists["drift_line"].set_data(t_hist, history["drift"])
        artists["confidence_line"].set_data(t_hist, history["confidence"])
        
        # Update x-axis and y-axis limits for time series plots
        for i in (2, 3, 4):
            ax = axes[i]
            if len(t_hist) > 1:
                ax.set_xlim(t_hist[0], t_hist[-1] + 0.01)
        
        # Dynamic y-axis scaling for offset and drift
        if len(history["offset"]) > 2:
            offset_vals = history["offset"]
            offset_range = max(abs(min(offset_vals)), abs(max(offset_vals)))
            # Add 20% margin
            axes[2].set_ylim(-offset_range * 1.2, offset_range * 1.2)
            
        if len(history["drift"]) > 2:
            drift_vals = history["drift"]
            drift_min = min(drift_vals)
            drift_max = max(drift_vals)
            drift_range = drift_max - drift_min
            # Add 20% margin, ensure minimum range
            margin = max(drift_range * 0.2, 100)
            axes[3].set_ylim(drift_min - margin, drift_max + margin)
        
        # Update status indicator with color coding
        confidence = sync_state.confidence
        drift = abs(sync_state.drift_rate_ppm)
        
        # Determine status color
        if confidence > 0.6 and drift < 1000:
            color = "#27ae60"  # Green - good sync
            status = "GOOD"
        elif confidence > 0.3 or drift < 3000:
            color = "#f39c12"  # Orange - moderate sync
            status = "MODERATE"
        else:
            color = "#e74c3c"  # Red - poor sync
            status = "POOR"
        
        status_str = (
            f"SYNC {status}  |  "
            f"Offset: {sync_state.offset_ms:+.2f} ms  |  "
            f"Drift: {sync_state.drift_rate_ppm:+.1f} ppm  |  "
            f"Confidence: {sync_state.confidence:.3f}"
        )
        artists["status_text"].set_text(status_str)
        artists["status_text"].set_bbox(dict(boxstyle="round,pad=0.5", facecolor=color, alpha=0.9, edgecolor="none"))
        artists["status_text"].set_color("white")
    else:
        artists["waiting_text"].set_visible(True)
        artists["status_text"].set_text("Waiting for sync data...")
        artists["status_text"].set_bbox(dict(boxstyle="round,pad=0.5", facecolor="gray", alpha=0.8, edgecolor="none"))


def run_with_prbs_visualization(
    config: SignalAcquisitionConfig,
    duration_s: Optional[float],
    mode: str,
    use_pyqtgraph: Optional[bool] = None,
) -> None:
    """Run PRBS or combined acquisition with real-time visualization.
    
    Args:
        config: Acquisition configuration
        duration_s: Optional duration in seconds
        mode: Run mode ('prbs' or 'all')
        use_pyqtgraph: If True, force PyQtGraph. If False, force matplotlib.
                       If None (default), use PyQtGraph if available, else matplotlib.
    """
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run PRBS visualization.")
        return
    
    # Decide which visualization to use
    if use_pyqtgraph is None:
        use_pg = _PYQTGRAPH_AVAILABLE
    else:
        use_pg = use_pyqtgraph and _PYQTGRAPH_AVAILABLE
    
    if use_pg:
        print("[VIZ] Using PyQtGraph (high-performance mode)")
        _run_with_pyqtgraph_viz(config, duration_s, mode)
    else:
        if use_pyqtgraph:
            print("[VIZ] PyQtGraph not available, falling back to matplotlib")
        else:
            print("[VIZ] Using matplotlib")
        _run_with_matplotlib_viz(config, duration_s, mode)


def _run_with_pyqtgraph_viz(
    config: SignalAcquisitionConfig,
    duration_s: Optional[float],
    mode: str,
) -> None:
    """Run visualization using PyQtGraph (imported from separate module)."""
    try:
        # Import the PyQtGraph visualization module
        import signal_acquisition_testing_pyqtgraph as pg_viz
        pg_viz.run_with_pyqtgraph_visualization(config, duration_s, mode)
    except Exception as e:
        print(f"[ERROR] PyQtGraph visualization failed: {e}")
        print("[VIZ] Falling back to matplotlib...")
        _run_with_matplotlib_viz(config, duration_s, mode)


def _run_with_matplotlib_viz(
    config: SignalAcquisitionConfig,
    duration_s: Optional[float],
    mode: str,
) -> None:
    """Run PRBS or combined acquisition with matplotlib visualization (fallback)."""
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run PRBS visualization.")
        return

    cfg = SignalAcquisitionConfig(
        enable_stm32=True,
        enable_emg=True,
        enable_prbs_sync=True,
        sync_mode="realtime",
        verbose=config.verbose,
        stm32_port=config.stm32_port,
        stm32_baud=config.stm32_baud,
        emg_connection_type=config.emg_connection_type,
        emg_differential_pairs=config.emg_differential_pairs,
        emg_sample_rate=config.emg_sample_rate,
        prbs_correlation_window_s=config.prbs_correlation_window_s,
        prbs_update_interval_s=config.prbs_update_interval_s,
    )
    if mode == "all":
        cfg = SignalAcquisitionConfig(
            enable_stm32=config.enable_stm32,
            enable_emg=config.enable_emg,
            enable_prbs_sync=True,
            sync_mode="realtime",
            verbose=config.verbose,
            stm32_port=config.stm32_port,
            stm32_baud=config.stm32_baud,
            emg_connection_type=config.emg_connection_type,
            emg_differential_pairs=config.emg_differential_pairs,
            emg_sample_rate=config.emg_sample_rate,
            prbs_correlation_window_s=config.prbs_correlation_window_s,
            prbs_update_interval_s=config.prbs_update_interval_s,
        )

    # With visualization: run until Ctrl+C unless --duration is given
    duration = duration_s if duration_s is not None else None

    if config.verbose:
        if mode == "prbs":
            dur_msg = f"Duration: {duration:.0f} s. " if duration is not None else ""
            print("\n" + "=" * 50 + "\nPhase: PRBS (EMG TRIG) + visualization\n" + "=" * 50 +
                  f"\n  Target: PRBS on TRIG (10 Hz). {dur_msg}Press Ctrl+C to stop.\n")
        else:
            dur_msg = f"Duration: {duration:.0f} s. " if duration is not None else ""
            print("\n" + "=" * 70 + "\nSYNCHRONIZED STM32 + EMG (with visualization)\n" + "=" * 70 +
                  f"\n  Combined STM32 + EMG + PRBS sync. {dur_msg}Press Ctrl+C to stop.\n")

    acq = SignalAcquisition(cfg)
    acq.start()
    time.sleep(1.0)

    plt.ion()
    fig, axes = plt.subplots(5, 1, figsize=(10, 8), sharex=False)
    fig.suptitle("PRBS Real-Time Sync", fontsize=12)
    plt.tight_layout()
    artists = _init_prbs_artists(fig, axes)
    history: Dict[str, List[float]] = {"t": [], "offset": [], "drift": [], "confidence": []}

    fig.canvas.draw()
    try:
        bg = fig.canvas.copy_from_bbox(fig.bbox)
    except Exception:
        bg = None

    stm32_count_at_start = acq.stm32_reader.sample_count if acq.stm32_reader else 0
    start = time.perf_counter()
    last_log = start
    bg_captured_with_data = False

    line_artists = [
        artists["stm32_line"], artists["emg_line"],
        artists["offset_line"], artists["drift_line"], artists["confidence_line"],
    ]

    try:
        while duration is None or (time.perf_counter() - start) < duration:
            now = time.perf_counter()
            acq.trigger_prbs_update()
            _update_prbs_visualization(acq, axes, artists, history, start)

            if bg is not None and not artists["waiting_text"].get_visible():
                if not bg_captured_with_data:
                    fig.canvas.draw()
                    try:
                        bg = fig.canvas.copy_from_bbox(fig.bbox)
                        bg_captured_with_data = True
                    except Exception:
                        pass
                if bg_captured_with_data:
                    fig.canvas.restore_region(bg)
                    for ax, line in zip(axes, line_artists):
                        ax.draw_artist(line)
                    fig.canvas.blit(fig.bbox)
                else:
                    fig.canvas.draw_idle()
            else:
                fig.canvas.draw_idle()

            elapsed = time.perf_counter() - now
            plt.pause(max(0.001, _VIZ_FRAME_S - elapsed))

            if config.verbose and (now - last_log) >= 1.0:
                last_log = now
                elapsed = now - start
                if acq.stm32_reader is not None:
                    n = acq.stm32_reader.sample_count - stm32_count_at_start
                    rate = n / elapsed if elapsed > 0 else 0
                    print(f"  [PERF] STM32: {rate:.1f} Hz")
                if acq.emg_thread is not None:
                    es = acq.emg_thread.get_stats()
                    print(f"  [PERF] EMG: {es['rate']:.0f} Hz")
                ss = acq.get_sync_state()
                if ss is not None:
                    print(f"  [PERF] PRBS: offset={ss.offset_ms:+.2f} ms, conf={ss.confidence:.3f}, drift={ss.drift_rate_ppm:.1f} ppm")
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[MAIN] Interrupted")
    finally:
        acq.stop()

    if config.verbose:
        ss = acq.get_sync_state()
        if ss is not None:
            print(f"\n  [SUMMARY] PRBS: offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}")

    plt.show(block=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="STM32 + EMG signal acquisition with integrated PRBS sync.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python signal_acquisition_testing.py --mode sequence --duration 10
  python signal_acquisition_testing.py --mode all --duration 15
  python signal_acquisition_testing.py --mode postprocess --duration 30 --sync-mode postprocessing
""",
    )
    parser.add_argument(
        "--mode",
        choices=["stm32", "emg", "prbs", "all", "sequence", "postprocess"],
        default="sequence",
        help="Run mode (postprocess=record then compute delay signal)",
    )
    parser.add_argument("--duration", type=float, default=None, metavar="SEC", help="Duration per phase (seconds)")
    parser.add_argument(
        "--sync-mode",
        choices=["realtime", "postprocessing"],
        default="realtime",
        help="PRBS sync: realtime=Kalman during acquisition; postprocessing=delay signal at end (for all/prbs)",
    )
    parser.add_argument(
        "--no-visualize",
        action="store_true",
        help="Disable real-time PRBS visualization (enabled by default for prbs/all)",
    )
    parser.add_argument(
        "--viz",
        choices=["auto", "pyqtgraph", "matplotlib"],
        default="auto",
        help="Visualization backend: auto (PyQtGraph if available, else matplotlib), pyqtgraph, or matplotlib",
    )
    args = parser.parse_args()

    config = SignalAcquisitionConfig(
        sync_mode=args.sync_mode,
    )

    use_viz = (not args.no_visualize) and args.sync_mode == "realtime"
    
    # Determine visualization backend
    use_pyqtgraph = None  # Auto
    if args.viz == "pyqtgraph":
        use_pyqtgraph = True
    elif args.viz == "matplotlib":
        use_pyqtgraph = False

    if args.mode == "stm32":
        run_stm32_only(config, args.duration)
    elif args.mode == "emg":
        run_emg_only(config, args.duration)
    elif args.mode == "prbs":
        if use_viz:
            run_with_prbs_visualization(config, args.duration, "prbs", use_pyqtgraph=use_pyqtgraph)
        else:
            run_prbs_test(config, args.duration)
    elif args.mode == "all":
        if use_viz:
            run_with_prbs_visualization(config, args.duration, "all", use_pyqtgraph=use_pyqtgraph)
        else:
            run_combined(config, args.duration)
    elif args.mode == "postprocess":
        duration = args.duration if args.duration is not None else 30.0
        run_postprocessing(config, duration)
    else:
        run_stm32_only(config, args.duration)
        run_emg_only(config, args.duration)
        if use_viz:
            run_with_prbs_visualization(config, args.duration, "all", use_pyqtgraph=use_pyqtgraph)
        else:
            run_combined(config, args.duration)


if __name__ == "__main__":
    main()
