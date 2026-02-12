"""
Basic PRBS synchronization test: STM32 + EMG TRIG recording.

This script keeps synchronization intentionally simple:
1) Record STM32 and EMG TRIG in parallel.
2) Convert both to chip streams and run PRBS cross-correlation (via stm32_emg_sync).
3) Report lag and correlation peak.
4) Optionally show/save diagnostics with lag-over-time.
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
from src.stm32_emg_sync import (
    CHIP_RATE_HZ,
    compute_sync_delay_signal,
    extract_trig_bits,
    downsample_to_chip_rate,
    stm32_samples_to_chipstream,
    cross_correlate_fullstream,
)
from src.signal_acquisition import fix_all_channel_names, find_trig_channel

# =============================================================================
# Constants
# =============================================================================
EMG_SAMPLE_RATE = 2000
RECORD_DURATION_S = 60
MIN_OVERLAP_RATIO = 0.9
SYNC_WINDOW_S = 10
SYNC_STEP_S = 0.1


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

        stm32_samples: list[SampleSTM32] = []

        def on_stm32_sample(sample: SampleSTM32):
            stm32_samples.append(sample)

        reader = STM32Reader(port=None, baud=115200, on_sample=on_stm32_sample, verbose=True)
        reader.start()
        measurement.start()

        # Use actual device sample rate for downsampling (avoids drift)
        actual_emg_rate = measurement.get_device_sample_rate()
        samples_per_chip = max(1, int(round(actual_emg_rate / CHIP_RATE_HZ)))
        print(f"  EMG sample rate (device): {actual_emg_rate} Hz -> {samples_per_chip} samples/chip")
        if abs(actual_emg_rate - EMG_SAMPLE_RATE) > 1:
            print(f"  Note: device rate differs from nominal {EMG_SAMPLE_RATE} Hz; using device rate for sync")

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

        print("\n[4/4] Running PRBS cross-correlation (stm32_emg_sync)...")
        trig_bin, ext_method, n_trans, method_stats = extract_trig_bits(
            trig_raw, is_status=bool(is_status)
        )
        print(f"  TRIG extraction: {ext_method} (is_status={is_status}), transitions: {n_trans}")
        print(f"  All methods: {', '.join(f'{k}={v}' for k, v in method_stats.items())}")
        lo, hi = np.nanmin(trig_raw), np.nanmax(trig_raw)
        print(f"  Raw TRIG stats: min={lo:.4g}, max={hi:.4g}, mean={np.nanmean(trig_raw):.4g}, "
              f"unique_ints={len(np.unique(trig_raw.astype(np.int64)))}")

        try:
            delay_signal, sync_result = compute_sync_delay_signal(
                emg_trig_raw=trig_raw,
                stm32_samples=stm32_samples,
                emg_sample_rate=actual_emg_rate,
                is_status=bool(is_status),
                sync_window_s=sync_window_s,
                sync_step_s=sync_step_s,
                min_overlap_ratio=MIN_OVERLAP_RATIO,
            )
        except ValueError as e:
            print(f"  ERROR: {e}")
            return

        lag_chips = sync_result.delay_chips
        lag_ms = sync_result.delay_ms
        corr_peak = sync_result.correlation_peak
        polarity_note = sync_result.polarity
        lag_t_s = sync_result.lag_series_t_s
        lag_series_chips = sync_result.lag_series_chips
        lag_std = float(np.std(lag_series_chips)) if len(lag_series_chips) > 1 else 0.0

        emg_chips = downsample_to_chip_rate(trig_bin, samples_per_chip)
        stm32_chips = stm32_samples_to_chipstream(stm32_samples)
        n_common = min(len(emg_chips), len(stm32_chips))
        emg_sync = emg_chips[:n_common]
        stm32_sync = stm32_chips[:n_common]
        sync_window_chips = min(n_common, int(round(sync_window_s * CHIP_RATE_HZ)))
        representative_i = min(
            len(lag_t_s) - 1,
            max(0, int(np.argmin(np.abs(lag_series_chips - lag_chips)))),
        )
        rep_start_chip = int(
            np.clip(
                int(round(lag_t_s[representative_i] * CHIP_RATE_HZ - 0.5 * sync_window_chips)),
                0,
                n_common - sync_window_chips,
            )
        )
        rep_a = emg_sync[rep_start_chip : rep_start_chip + sync_window_chips]
        rep_b = stm32_sync[rep_start_chip : rep_start_chip + sync_window_chips]
        _, _, corr, lags, overlap_at_peak = cross_correlate_fullstream(
            rep_a, rep_b, min_overlap_ratio=MIN_OVERLAP_RATIO
        )

        mean_abs_peak = sync_result.confidence
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
            f"windows_used={len(lag_series_chips)}, mean|peak|={mean_abs_peak:.4f}"
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
