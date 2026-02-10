"""
STM32 signal verification – real-time visualization
====================================================

High-performance real-time dashboard for the STM32 acquisition stream.
Uses the threaded ``STM32Reader`` from ``all_STM32_acquisition`` so the
serial port is read at full 500 Hz in a background thread while
matplotlib runs at ~30 fps in the main thread with manual blitting.

Displays:
- IMU1 orientation (yaw, pitch, roll)
- IMU2 orientation (yaw, pitch, roll)
- Button matrix (keys_mask vs time)
- PRBS (prbs_level as ±1 signal, in_mark overlay)

Usage:
  python stm32_visualization_test.py [--port PORT] [--baud BAUD]
                                     [--window SEC] [--duration SEC]
                                     [--debug]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Project root
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from all_STM32_acquisition import STM32Reader  # noqa: E402
import numpy as np  # noqa: E402

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required.  pip install matplotlib numpy")
    sys.exit(1)

# ============================================================================
# CLI
# ============================================================================

def parse_args():
    p = argparse.ArgumentParser(description="Real-time STM32 signal verification")
    p.add_argument("--port", type=str, default="auto")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--window", type=float, default=10.0, help="Rolling window (s)")
    p.add_argument("--duration", type=float, default=None, help="Auto-exit after N s")
    p.add_argument("--debug", action="store_true")
    return p.parse_args()

# Max points to actually *draw* per line (more = prettier but slower)
MAX_DISPLAY_POINTS = 600


def downsample(arr: np.ndarray, n: int) -> np.ndarray:
    """Downsample array to at most *n* points using stride slicing (no copy)."""
    if len(arr) <= n:
        return arr
    step = max(1, len(arr) // n)
    return arr[::step]


# ============================================================================
# Figure setup  (no sharex – manual xlim is faster)
# ============================================================================

def setup_figure():
    fig, axes = plt.subplots(4, 1, figsize=(13, 9))
    fig.suptitle("STM32 signal verification", fontsize=12, fontweight="bold")

    # ---- IMU1 ----
    ax = axes[0]
    ax.set_ylabel("deg"); ax.set_title("IMU1", fontsize=10)
    ax.set_ylim(-185, 185); ax.grid(True, alpha=0.3)
    ly1, = ax.plot([], [], "r-", lw=1, label="yaw")
    lp1, = ax.plot([], [], "g-", lw=1, label="pitch")
    lr1, = ax.plot([], [], "b-", lw=1, label="roll")
    ax.legend(loc="upper right", fontsize=7)

    # ---- IMU2 ----
    ax = axes[1]
    ax.set_ylabel("deg"); ax.set_title("IMU2", fontsize=10)
    ax.set_ylim(-185, 185); ax.grid(True, alpha=0.3)
    ly2, = ax.plot([], [], "r-", lw=1, label="yaw")
    lp2, = ax.plot([], [], "g-", lw=1, label="pitch")
    lr2, = ax.plot([], [], "b-", lw=1, label="roll")
    ax.legend(loc="upper right", fontsize=7)

    # ---- Buttons ----
    ax = axes[2]
    ax.set_ylabel("mask"); ax.set_title("Buttons", fontsize=10)
    ax.set_ylim(-0.5, 10); ax.grid(True, alpha=0.3)
    lk, = ax.plot([], [], "k-", lw=1)

    # ---- PRBS ----
    ax = axes[3]
    ax.set_xlabel("Time (s)"); ax.set_ylabel("±1"); ax.set_title("PRBS", fontsize=10)
    ax.set_ylim(-1.5, 1.5); ax.grid(True, alpha=0.3)
    lprbs, = ax.plot([], [], "b-", lw=0.6, label="prbs ±1")
    lmark, = ax.plot([], [], color="orange", lw=1.2, label="in_mark")
    ax.legend(loc="upper right", fontsize=7)

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])

    # Draw static background once
    fig.canvas.draw()

    # Cache background for each axis (for blitting)
    backgrounds = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axes]

    return {
        "fig": fig,
        "axes": axes,
        "backgrounds": backgrounds,
        "lines": [ly1, lp1, lr1, ly2, lp2, lr2, lk, lprbs, lmark],
        "line_keys": [
            ("yaw1", 0), ("pitch1", 0), ("roll1", 0),
            ("yaw2", 1), ("pitch2", 1), ("roll2", 1),
            ("keys_mask", 2),
            ("prbs_signal", 3), ("in_mark", 3),
        ],
    }


# ============================================================================
# Main loop with manual blit
# ============================================================================

def main():
    args = parse_args()
    port = None if (args.port or "auto").lower() == "auto" else args.port
    window_sec = args.window
    debug = args.debug
    n_fetch = int(window_sec * 500)  # samples to fetch from ring buffer

    print("Close Arduino Serial Monitor before running this script.")

    reader = STM32Reader(port=port, baud=args.baud, verbose=True)
    reader.start()

    ctx = setup_figure()
    fig = ctx["fig"]
    axes = ctx["axes"]
    backgrounds = ctx["backgrounds"]
    lines = ctx["lines"]
    line_keys = ctx["line_keys"]

    start_time = time.time()
    last_status_time = start_time
    frame_count = 0
    fps_t0 = time.perf_counter()

    # Status text (updated infrequently — avoids redrawing ticks)
    status_txt = fig.text(0.01, 0.005, "", fontsize=8, family="monospace")

    plt.ion()
    plt.show(block=False)
    fig.canvas.flush_events()

    try:
        while plt.fignum_exists(fig.number):
            t_frame_start = time.perf_counter()

            # --- Check duration ---
            if args.duration and (time.time() - start_time) >= args.duration:
                print(f"\nDuration {args.duration}s reached.")
                break

            # --- Get data snapshot ---
            snap = reader.get_snapshot(n_fetch)
            t_sec = snap["t_sec"]
            n = len(t_sec)

            if n < 2:
                fig.canvas.flush_events()
                time.sleep(0.033)
                continue

            # --- Compute x-limits ---
            t_max = t_sec[-1]
            t_min = max(t_sec[0], t_max - window_sec)

            # --- Downsample t once ---
            t_ds = downsample(t_sec, MAX_DISPLAY_POINTS)

            # --- Restore backgrounds and draw lines ---
            for i, ax in enumerate(axes):
                ax.set_xlim(t_min, t_max)
                fig.canvas.restore_region(backgrounds[i])

            # Set data on each line (downsampled)
            for idx, (key, ax_idx) in enumerate(line_keys):
                y_ds = downsample(snap[key], MAX_DISPLAY_POINTS)
                lines[idx].set_data(t_ds[:len(y_ds)], y_ds)
                axes[ax_idx].draw_artist(lines[idx])

            # Autoscale buttons y-axis
            km = snap["keys_mask"]
            km_max = float(np.nanmax(km)) if n else 0
            axes[2].set_ylim(-0.5, max(10, km_max + 2))

            # Blit all axes
            for ax in axes:
                fig.canvas.blit(ax.bbox)

            # --- Status text (throttled to ~2 Hz) ---
            now = time.time()
            if now - last_status_time >= 0.5:
                last_status_time = now
                frame_count += 1
                elapsed_fps = time.perf_counter() - fps_t0
                fps = frame_count / elapsed_fps if elapsed_fps > 0.5 else 0
                rate = reader.get_sample_rate()
                s = reader.latest
                mask_val = int(km[-1]) if n else 0
                status_txt.set_text(
                    f"FPS={fps:.0f}  serial={rate:.0f} Hz  "
                    f"samples={reader.sample_count}  mask={mask_val}  "
                    f"fails={reader.parse_fail_count}"
                )
                # Full redraw for text + recache backgrounds
                fig.canvas.draw()
                backgrounds[:] = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axes]
                frame_count = 0
                fps_t0 = time.perf_counter()

                if debug and s:
                    print(f"[DEBUG] t_ms={s.t_ms:.0f} yaw1={s.yaw1:.1f} "
                          f"keys={s.keys_mask} prbs_lvl={s.prbs_lvl} "
                          f"FPS={fps:.0f} serial={rate:.0f}Hz")

            fig.canvas.flush_events()

            # --- Frame rate limiter: sleep remainder of 33ms frame ---
            elapsed = time.perf_counter() - t_frame_start
            sleep_time = 0.033 - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        reader.stop()
        print("Done.")


if __name__ == "__main__":
    main()
