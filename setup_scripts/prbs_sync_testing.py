"""
Basic PRBS synchronization test: STM32 + EMG TRIG recording.

This script keeps synchronization intentionally simple:
1) Record STM32 and EMG TRIG in parallel.
2) Convert both to 100 Hz chip streams.
3) Run one full-stream PRBS cross-correlation.
4) Report lag and correlation peak.
5) Optionally show/save diagnostics with lag-over-time.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Project root for src and tmsi-python-interface
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

_tmsi_path = _project_root / "tmsi-python-interface"
if _tmsi_path.exists() and str(_tmsi_path) not in sys.path:
    sys.path.insert(0, str(_tmsi_path))

# Optional matplotlib
try:
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

try:
    from scipy import signal as sp_signal

    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

# TMSi SDK
try:
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
    from TMSiSDK.device.tmsi_device_enums import (
        DeviceInterfaceType,
        DeviceType,
        MeasurementType,
    )
    from TMSiSDK.tmsi_sdk import TMSiSDK
except ImportError as e:
    print("ERROR: TMSi Python Interface not found. Ensure tmsi-python-interface is in project root.")
    print(f"  {e}")
    sys.exit(1)

from src.stm32_reader import STM32Reader, SampleSTM32

# =============================================================================
# Constants
# =============================================================================
CHIP_RATE_HZ = 50
EMG_SAMPLE_RATE = 2000
SAMPLES_PER_CHIP = EMG_SAMPLE_RATE // CHIP_RATE_HZ  # 40
RECORD_DURATION_S = 60
MIN_OVERLAP_RATIO = 0.6
SYNC_WINDOW_S = 20.0
SYNC_STEP_S = 2.0


def fix_channel_name(channel_index: int) -> str:
    if channel_index < 16:
        return f"UNI{channel_index + 1}"
    if channel_index < 36:
        return f"BIP{channel_index - 15}"
    if channel_index == 36:
        return "STATUS"
    if channel_index == 37:
        return "COUNTER"
    return f"CH{channel_index}"


def fix_all_channel_names(channels) -> None:
    for i, ch in enumerate(channels):
        try:
            name = ch.get_channel_name()
            if any(ord(c) > 127 for c in name):
                corrected = fix_channel_name(i)
                ch._alt_name = corrected
                ch._def_name = corrected
        except Exception:
            pass


def find_trig_channel(channels):
    for i, ch in enumerate(channels):
        if "TRIG" in ch.get_channel_name().upper():
            return i, False
    for i, ch in enumerate(channels):
        if ch.get_channel_name().upper().startswith("DIG"):
            return i, True
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "STATUS" in name or "STATU" in name:
            return i, True
    if len(channels) > 36:
        return 36, True
    return None, None


def extract_trig_bits(raw: np.ndarray, is_status: bool = False):
    """Extract binary TRIG signal from raw STATUS/TRIG channel data.

    Args:
        raw: Raw channel data (float64 from TMSi SDK).
        is_status: If True, this is a STATUS register channel. Bit-0 is
                   preferred but only used if it has a plausible transition
                   count (the SDK's gain/offset conversion may alter the
                   integer representation, making bit-0 extraction unreliable).

    Returns:
        (signal, method_name, transition_count, all_method_stats)
    """
    raw_int = raw.astype(np.int64)
    bit0 = (raw_int & 1).astype(np.float64)
    lo, hi = np.nanmin(raw), np.nanmax(raw)
    threshold = (raw > (lo + hi) / 2.0).astype(np.float64) if (hi - lo) > 1e-9 else bit0
    nonzero = (raw_int != 0).astype(np.float64)
    candidates = {"bit0": bit0, "threshold": threshold, "nonzero": nonzero}

    # Compute transition counts for all methods (for diagnostics)
    method_stats: dict[str, int] = {}
    for name, sig in candidates.items():
        method_stats[name] = int(np.sum(np.abs(np.diff(sig))))

    # For STATUS channels, prefer bit-0 only if it has a plausible number
    # of transitions (at least 100). The TMSi SDK may apply gain/offset to
    # the STATUS channel, making the integer bit layout unreliable.
    MIN_PLAUSIBLE_TRANSITIONS = 100
    if is_status and method_stats["bit0"] >= MIN_PLAUSIBLE_TRANSITIONS:
        return bit0, "bit0", method_stats["bit0"], method_stats

    # Heuristic fallback: pick the method with the most transitions
    best_name = "bit0"
    best_sig = bit0
    best_transitions = -1
    for name, sig in candidates.items():
        if method_stats[name] > best_transitions:
            best_name = name
            best_sig = sig
            best_transitions = method_stats[name]
    return best_sig, best_name, best_transitions, method_stats


def downsample_to_100hz(trig: np.ndarray) -> np.ndarray:
    n_chips = len(trig) // SAMPLES_PER_CHIP
    out = np.empty(n_chips, dtype=np.float64)
    for k in range(n_chips):
        block = trig[k * SAMPLES_PER_CHIP : (k + 1) * SAMPLES_PER_CHIP]
        out[k] = 1.0 if np.mean(block) > 0.5 else 0.0
    return out



def stm32_samples_to_chipstream(samples: list) -> np.ndarray:
    """Convert 500 Hz STM32 samples to ~100 Hz chip stream using prbs_lvl directly.

    Instead of reconstructing PRBS bits from tick indices via a reference table,
    this uses the actual prbs_lvl value that the STM32 reports for each sample.
    One chip is emitted each time prbs_tick changes.
    """
    if not samples:
        return np.array([], dtype=np.float64)

    out: list[float] = []
    prev_tick = samples[0].prbs_tick
    out.append(float(samples[0].prbs_lvl))

    for s in samples[1:]:
        if s.prbs_tick != prev_tick:
            out.append(float(s.prbs_lvl))
            prev_tick = s.prbs_tick

    return np.asarray(out, dtype=np.float64)


def cross_correlate_fullstream(signal_a: np.ndarray, signal_b: np.ndarray, min_overlap_ratio: float = MIN_OVERLAP_RATIO):
    M, N = len(signal_a), len(signal_b)
    a = 2.0 * signal_a - 1.0
    b = 2.0 * signal_b - 1.0
    if SCIPY_AVAILABLE:
        corr_raw = sp_signal.correlate(a, b, mode="full")
    else:
        corr_raw = np.correlate(a, b, mode="full")

    lags = np.arange(len(corr_raw)) - (M - 1)
    # Correct overlap for potentially unequal-length signals:
    #   lag >= 0: overlap = min(M - lag, N)
    #   lag <  0: overlap = min(M, N + lag)
    overlap = np.where(
        lags >= 0,
        np.minimum(M - lags, N),
        np.minimum(M, N + lags),
    )
    overlap = np.maximum(overlap.astype(np.float64), 1.0)
    corr = corr_raw / overlap

    min_overlap = max(1, int(np.ceil(min_overlap_ratio * min(M, N))))
    valid = overlap >= min_overlap
    abs_corr = np.abs(corr)
    abs_corr_valid = np.where(valid, abs_corr, 0.0)
    peak_idx = int(np.argmax(abs_corr_valid))
    lag = int(lags[peak_idx])
    peak = float(corr[peak_idx])
    overlap_at_peak = int(overlap[peak_idx])
    return lag, peak, corr, lags, overlap_at_peak


def sliding_window_lag_series(
    signal_a: np.ndarray,
    signal_b: np.ndarray,
    window_chips: int,
    step_chips: int,
    min_overlap_ratio: float = MIN_OVERLAP_RATIO,
):
    n = min(len(signal_a), len(signal_b))
    if n < window_chips or window_chips <= 0:
        return np.array([]), np.array([]), np.array([]), np.array([])

    starts = list(range(0, max(1, n - window_chips + 1), step_chips))
    if starts[-1] != (n - window_chips):
        starts.append(n - window_chips)

    centers_s = []
    lags = []
    peaks = []
    overlaps = []
    for s in starts:
        a_win = signal_a[s : s + window_chips]
        b_win = signal_b[s : s + window_chips]
        lag, peak, _, _, overlap = cross_correlate_fullstream(
            a_win, b_win, min_overlap_ratio=min_overlap_ratio
        )
        centers_s.append((s + 0.5 * window_chips) / CHIP_RATE_HZ)
        lags.append(lag)
        peaks.append(peak)
        overlaps.append(overlap)

    return (
        np.asarray(centers_s, dtype=np.float64),
        np.asarray(lags, dtype=np.int64),
        np.asarray(peaks, dtype=np.float64),
        np.asarray(overlaps, dtype=np.int64),
    )


def main():
    parser = argparse.ArgumentParser(description="Basic PRBS sync test (single full-stream cross-correlation)")
    parser.add_argument("--duration", type=float, default=RECORD_DURATION_S, help="Recording duration in seconds")
    parser.add_argument("--sync-window-s", type=float, default=SYNC_WINDOW_S, help="Window length used for lag estimation (seconds)")
    parser.add_argument("--sync-step-s", type=float, default=SYNC_STEP_S, help="Sliding-window step size (seconds)")
    parser.add_argument("--save-plot", type=str, default=None, help="Save figure to this path")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting")
    args = parser.parse_args()

    duration_s = float(args.duration)
    sync_window_s = max(2.0, float(args.sync_window_s))
    sync_step_s = max(0.2, float(args.sync_step_s))
    save_plot = args.save_plot
    do_plot = (not args.no_plot) and MATPLOTLIB_AVAILABLE

    print("=" * 60)
    print("  Basic PRBS Synchronization Test")
    print("=" * 60)
    print(f"  Duration  : {duration_s:.1f} s")
    print(f"  Chip rate : {CHIP_RATE_HZ} Hz")
    print(f"  EMG rate  : {EMG_SAMPLE_RATE} Hz")
    print()

    device = None
    measurement = None
    reader = None

    try:
        print("[1/4] Discovering TMSi Porti7...")
        sdk = TMSiSDK()
        devices, _ = sdk.discover(DeviceType.legacy, dr_interface=DeviceInterfaceType.usb)
        if not devices:
            print("  ERROR: No TMSi legacy device found.")
            return

        device = devices[0]
        device.open()
        print(f"  Connected: {device.get_device_name()}")

        channels = device.get_device_channels()
        fix_all_channel_names(channels)
        trig_idx, is_status = find_trig_channel(channels)
        if trig_idx is None:
            print("  ERROR: TRIG channel not found.")
            return
        print(f"  TRIG channel index: {trig_idx}")

        print("\n[2/4] Starting measurement and STM32 reader...")
        measurement = MeasurementType.LEGACY_SIGNAL(device)
        measurement.set_reference_calculation(False)
        print(f"  EMG sample rate: {measurement.get_device_sample_rate()} Hz")

        stm32_samples: list[SampleSTM32] = []

        def on_stm32_sample(sample: SampleSTM32):
            stm32_samples.append(sample)

        reader = STM32Reader(port=None, baud=115200, on_sample=on_stm32_sample, verbose=True)
        reader.start()
        measurement.start()

        print(f"\n[3/4] Recording for {duration_s:.1f} s...")
        trig_chunks = []
        t_start = time.time()
        while (time.time() - t_start) < duration_s:
            samples = measurement.get_samples(blocking=False)
            if samples is not None and len(samples) > 0:
                trig_chunks.append(samples[:, trig_idx].copy())
            time.sleep(0.002)

        elapsed = time.time() - t_start
        trig_raw = np.concatenate(trig_chunks) if trig_chunks else np.array([])
        n_emg = len(trig_raw)
        n_stm32 = len(stm32_samples)
        print(f"  Done. EMG: {n_emg} samples, STM32: {n_stm32} samples in {elapsed:.2f} s")
        if n_emg < 1000 or n_stm32 < 100:
            print("  ERROR: Not enough data. Check cables and STM32.")
            return

        print("\n[4/4] Running single PRBS cross-correlation...")
        trig_bin, ext_method, n_trans, method_stats = extract_trig_bits(
            trig_raw, is_status=bool(is_status)
        )
        print(f"  TRIG extraction: {ext_method} (is_status={is_status}), transitions: {n_trans}")
        print(f"  All methods: {', '.join(f'{k}={v}' for k, v in method_stats.items())}")
        lo, hi = np.nanmin(trig_raw), np.nanmax(trig_raw)
        print(f"  Raw TRIG stats: min={lo:.4g}, max={hi:.4g}, mean={np.nanmean(trig_raw):.4g}, "
              f"unique_ints={len(np.unique(trig_raw.astype(np.int64)))}")

        emg_chips = downsample_to_100hz(trig_bin)
        stm32_chips = stm32_samples_to_chipstream(stm32_samples)
        print(f"  Chip streams: EMG={len(emg_chips)} chips, STM32={len(stm32_chips)} chips")

        n_common = min(len(emg_chips), len(stm32_chips))
        if n_common < 200:
            print("  ERROR: Not enough chip overlap for sync.")
            return

        emg_sync = emg_chips[:n_common]
        stm32_sync = stm32_chips[:n_common]
        sync_window_chips = min(n_common, int(round(sync_window_s * CHIP_RATE_HZ)))
        sync_step_chips = max(1, int(round(sync_step_s * CHIP_RATE_HZ)))

        lag_t_s, lag_series_chips, peak_series, overlap_series = sliding_window_lag_series(
            emg_sync,
            stm32_sync,
            window_chips=sync_window_chips,
            step_chips=sync_step_chips,
            min_overlap_ratio=MIN_OVERLAP_RATIO,
        )
        if len(lag_series_chips) == 0:
            print("  ERROR: Sliding-window lag estimation failed (window too large for available data).")
            return

        strong_mask = np.abs(peak_series) >= 0.10
        selected_idx = np.where(strong_mask)[0] if np.any(strong_mask) else np.arange(len(lag_series_chips))
        lag_chips = int(np.round(np.median(lag_series_chips[selected_idx])))
        representative_i = int(selected_idx[np.argmin(np.abs(lag_series_chips[selected_idx] - lag_chips))])
        corr_peak = float(peak_series[representative_i])
        overlap_at_peak = int(overlap_series[representative_i])
        lag_ms = lag_chips / CHIP_RATE_HZ * 1000.0
        polarity_note = "normal"
        if corr_peak < 0:
            polarity_note = "inverted"
        lag_std = float(np.std(lag_series_chips[selected_idx])) if len(selected_idx) > 1 else 0.0

        rep_start_chip = int(np.clip(int(round(lag_t_s[representative_i] * CHIP_RATE_HZ - 0.5 * sync_window_chips)), 0, n_common - sync_window_chips))
        rep_a = emg_sync[rep_start_chip : rep_start_chip + sync_window_chips]
        rep_b = stm32_sync[rep_start_chip : rep_start_chip + sync_window_chips]
        _, _, corr, lags, _ = cross_correlate_fullstream(rep_a, rep_b, min_overlap_ratio=MIN_OVERLAP_RATIO)

        mean_abs_peak = float(np.mean(np.abs(peak_series[selected_idx])))
        if mean_abs_peak >= 0.50:
            assessment = "Reliable"
        elif mean_abs_peak >= 0.25:
            assessment = "Moderate"
        else:
            assessment = "Weak"

        print(
            f"  Result: lag={lag_chips} chips ({lag_ms:.1f} ms), representative_peak={corr_peak:.4f}, "
            f"overlap={overlap_at_peak}/{sync_window_chips}, polarity={polarity_note}, "
            f"window={sync_window_s:.1f}s, step={sync_step_s:.1f}s"
        )
        print(
            f"  Sliding lag stats: median={lag_chips} chips, std={lag_std:.2f} chips, "
            f"windows_used={len(selected_idx)}/{len(lag_series_chips)}, mean|peak|={mean_abs_peak:.4f}"
        )
        print(f"  Assessment: {assessment}")

        if do_plot or save_plot:
            print("  Generating simple diagnostics plot...")
            fig, axes = plt.subplots(4, 1, figsize=(12, 11))
            fig.suptitle("Basic PRBS Synchronization Diagnostics", fontsize=14, fontweight="bold")

            show_n = min(1000, n_common)
            t_show = np.arange(show_n) / CHIP_RATE_HZ
            axes[0].plot(t_show, emg_sync[:show_n], color="steelblue", linewidth=0.8)
            axes[0].set_ylabel("EMG chips")
            axes[0].set_xlabel("Time (s)")
            axes[0].set_ylim(-0.1, 1.1)
            axes[0].set_title("Panel 1: EMG chip stream (first 10 s max)")

            axes[1].plot(t_show, stm32_sync[:show_n], color="green", linewidth=0.8)
            axes[1].set_ylabel("STM32 chips")
            axes[1].set_xlabel("Time (s)")
            axes[1].set_ylim(-0.1, 1.1)
            axes[1].set_title("Panel 2: STM32 chip stream (first 10 s max)")

            axes[2].plot(lags, corr, color="purple", linewidth=0.8)
            axes[2].axvline(lag_chips, color="red", linestyle="--", label=f"lag={lag_chips}")
            axes[2].set_ylabel("Correlation")
            axes[2].set_xlabel("Lag (chips)")
            axes[2].set_title(
                f"Panel 3: Correlation (representative window, peak={corr_peak:.4f}, overlap={overlap_at_peak})"
            )
            axes[2].legend()

            axes[3].plot(lag_t_s, lag_series_chips, color="darkorange", linewidth=1.2, label="window lag")
            axes[3].axhline(lag_chips, color="red", linestyle="--", label=f"median lag={lag_chips}")
            axes[3].set_ylabel("Lag (chips)")
            axes[3].set_xlabel("Window center time (s)")
            axes[3].set_title(
                f"Panel 4: Sliding-window lag over time ({sync_window_s:.1f}s window, {sync_step_s:.1f}s step)"
            )
            axes[3].legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            if save_plot:
                fig.savefig(save_plot, dpi=150, bbox_inches="tight")
                print(f"  Plot saved: {save_plot}")
            if do_plot:
                plt.show(block=True)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            if measurement is not None:
                measurement.stop()
        except Exception:
            pass
        try:
            if reader is not None:
                reader.stop()
        except Exception:
            pass
        try:
            if device is not None:
                device.close()
                LegacyDevice.cleanup()
        except Exception:
            pass
        print("\nDone.")


if __name__ == "__main__":
    main()
