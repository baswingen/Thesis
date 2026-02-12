"""
PRBS Synchronization Testing: 30 s STM32 + EMG recording and diagnostics
========================================================================

Records 30 s of STM32 (prbs_tick, prbs_level, in_mark) and EMG TRIG in parallel,
reconstructs the STM32 PRBS at 2000 Hz (one chip per EMG sample), cross-correlates
with the received PRBS on the EMG TRIG channel, and shows a 4-panel diagnostics plot.

Hardware setup:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

STM32 firmware (STM32_all_in_python.ino):
    - 2000 Hz chip rate (matches Porti7), gap every 1 s, LFSR reset at gap
    - CSV: ...,prbs_tick,prbs_level,in_mark  (prbs_tick = chip index 0..1999)

Usage:
    python setup_scripts/prbs_sync_testing.py --duration 30 --save-plot prbs_diagnostics.png
    python setup_scripts/prbs_sync_testing.py --no-plot
"""

from __future__ import annotations

import argparse
import os
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
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
except ImportError as e:
    print("ERROR: TMSi Python Interface not found. Ensure tmsi-python-interface is in project root.")
    print(f"  {e}")
    sys.exit(1)

from src.stm32_reader import STM32Reader, SampleSTM32

# =============================================================================
# Constants (match STM32 firmware)
# =============================================================================
CHIP_RATE_HZ = 100
EMG_SAMPLE_RATE = 2000
SAMPLES_PER_CHIP = EMG_SAMPLE_RATE // CHIP_RATE_HZ  # 20
PRBS15_SEED = 0x7ACE
CHIPS_PER_SEQUENCE = 1000   # 10 s segment
RECORD_DURATION_S = 30.0
# Gap on wire: 30 ms LOW at 2000 Hz = 60 samples; detect runs of LOW >= 60 (or use 16 for safety)
MIN_GAP_RUN = 60

# =============================================================================
# Helpers from trig_sync_testing (simplified)
# =============================================================================

def fix_channel_name(corrupted_name: str, channel_index: int) -> str:
    if channel_index < 16:
        return f"UNI{channel_index + 1}"
    elif channel_index < 36:
        return f"BIP{channel_index - 15}"
    elif channel_index == 36:
        return "STATUS"
    elif channel_index == 37:
        return "COUNTER"
    return f"CH{channel_index}"


def fix_all_channel_names(channels):
    fixed = 0
    for i, ch in enumerate(channels):
        try:
            name = ch.get_channel_name()
            if any(ord(c) > 127 for c in name):
                corrected = fix_channel_name(name, i)
                ch._alt_name = corrected
                ch._def_name = corrected
                fixed += 1
        except Exception:
            pass
    return fixed


def find_trig_channel(channels):
    for i, ch in enumerate(channels):
        if "TRIG" in ch.get_channel_name().upper():
            return i, False
    for i, ch in enumerate(channels):
        if ch.get_channel_name().upper().startswith("DIG"):
            return i, True
    for i, ch in enumerate(channels):
        if "STATUS" in ch.get_channel_name().upper() or "STATU" in ch.get_channel_name().upper():
            return i, True
    if len(channels) > 36:
        return 36, True
    return None, None


def extract_trig_bits(raw: np.ndarray, is_status_bit: bool):
    raw_int = raw.astype(np.int64)
    bit0 = (raw_int & 1).astype(np.float64)
    candidates = {"bit0": bit0}
    lo, hi = np.nanmin(raw), np.nanmax(raw)
    if hi - lo > 1e-9:
        thresh = (raw > (lo + hi) / 2.0).astype(np.float64)
        candidates["threshold"] = thresh
    nonzero = (raw_int != 0).astype(np.float64)
    candidates["nonzero"] = nonzero
    best_name, best_trans, best_sig = "bit0", -1, bit0
    for name, sig in candidates.items():
        t = int(np.sum(np.abs(np.diff(sig))))
        if t > best_trans:
            best_trans, best_name, best_sig = t, name, sig
    return best_sig, best_name, best_trans


def find_low_gaps(trig: np.ndarray, min_run: int = MIN_GAP_RUN):
    gaps = []
    n, i = len(trig), 0
    while i < n:
        if trig[i] == 0:
            j = i
            while j < n and trig[j] == 0:
                j += 1
            if (j - i) >= min_run:
                gaps.append((i, j))
            i = j
        else:
            i += 1
    return gaps


def generate_prbs15(seed: int = PRBS15_SEED, n_chips: int = CHIPS_PER_SEQUENCE) -> np.ndarray:
    lfsr = seed & 0x7FFF
    seq = np.empty(n_chips, dtype=np.float64)
    for i in range(n_chips):
        b14 = (lfsr >> 14) & 1
        b13 = (lfsr >> 13) & 1
        newbit = b14 ^ b13
        lfsr = ((lfsr << 1) | newbit) & 0x7FFF
        seq[i] = float(newbit)
    return seq


def cross_correlate_100hz(segment: np.ndarray, reference: np.ndarray):
    seg_bp = 2.0 * segment - 1.0
    ref_bp = 2.0 * reference - 1.0
    if SCIPY_AVAILABLE:
        corr = sp_signal.correlate(seg_bp, ref_bp, mode="full")
    else:
        corr = np.correlate(seg_bp, ref_bp, mode="full")
    norm = np.sqrt(np.sum(seg_bp ** 2) * np.sum(ref_bp ** 2))
    if norm > 0:
        corr = corr / norm
    peak_idx = np.argmax(np.abs(corr))
    peak_val = corr[peak_idx]
    abs_corr = np.abs(corr)
    guard = max(5, len(segment) // 50)
    mask = np.ones(len(abs_corr), dtype=bool)
    mask[max(0, peak_idx - guard):min(len(abs_corr), peak_idx + guard + 1)] = False
    sidelobes = abs_corr[mask]
    psr = abs_corr[peak_idx] / np.max(sidelobes) if (len(sidelobes) > 0 and np.max(sidelobes) > 0) else float("inf")
    best_lag = peak_idx - (len(segment) - 1)
    return best_lag, peak_val, psr, corr


def downsample_to_100hz(trig: np.ndarray, samples_per_chip: int = SAMPLES_PER_CHIP) -> np.ndarray:
    n = len(trig)
    n_chips = n // samples_per_chip
    out = np.empty(n_chips, dtype=np.float64)
    for k in range(n_chips):
        block = trig[k * samples_per_chip:(k + 1) * samples_per_chip]
        out[k] = 1.0 if np.mean(block) > 0.5 else 0.0
    return out


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="PRBS sync: 30 s STM32 + EMG record, cross-correlate, diagnostics plot")
    parser.add_argument("--duration", type=float, default=30.0, help="Recording duration in seconds (default: 30)")
    parser.add_argument("--save-plot", type=str, default=None, help="Save figure to this path")
    parser.add_argument("--no-plot", action="store_true", help="Skip visualisation")
    args = parser.parse_args()

    duration_s = args.duration
    save_plot = args.save_plot
    do_plot = (not args.no_plot) and MATPLOTLIB_AVAILABLE

    print("=" * 60)
    print("  PRBS Synchronization Testing (30 s record)")
    print("=" * 60)
    print(f"  Duration      : {duration_s:.1f} s")
    print(f"  Chip rate     : {CHIP_RATE_HZ} Hz")
    print(f"  EMG rate      : {EMG_SAMPLE_RATE} Hz")
    print(f"  Chips/segment : {CHIPS_PER_SEQUENCE} (10 s)")
    print()

    # ----- 1. Discover TMSi device -----
    print("[1/5] Discovering TMSi Porti7...")
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
        device.close()
        LegacyDevice.cleanup()
        return
    print(f"  TRIG channel index: {trig_idx}")

    # ----- 2. Create measurement and start STM32 -----
    print("\n[2/5] Creating measurement and starting STM32 reader...")
    measurement = MeasurementType.LEGACY_SIGNAL(device)
    measurement.set_reference_calculation(False)
    sample_rate = measurement.get_device_sample_rate()
    print(f"  EMG sample rate: {sample_rate} Hz")

    stm32_samples: list[SampleSTM32] = []

    def on_stm32_sample(s: SampleSTM32):
        stm32_samples.append(s)

    try:
        reader = STM32Reader(port=None, baud=115200, on_sample=on_stm32_sample, verbose=True)
        reader.start()
    except Exception as e:
        print(f"  ERROR: STM32Reader failed: {e}")
        print("  Ensure STM32 is connected and firmware is flashed (100 Hz chip, in_mark column).")
        device.close()
        LegacyDevice.cleanup()
        return

    measurement.start()
    trig_chunks = []
    t_start = time.time()

    # ----- 3. Record for duration_s -----
    print(f"\n[3/5] Recording for {duration_s:.1f} s (STM32 + EMG)...")
    try:
        while (time.time() - t_start) < duration_s:
            samples = measurement.get_samples(blocking=False)
            if samples is not None and len(samples) > 0:
                trig_chunks.append(samples[:, trig_idx].copy())
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("  Interrupted.")
    finally:
        measurement.stop()
        reader.stop()

    elapsed = time.time() - t_start
    trig_raw = np.concatenate(trig_chunks) if trig_chunks else np.array([])
    n_emg = len(trig_raw)
    n_stm32 = len(stm32_samples)
    print(f"  Done. EMG: {n_emg} samples, STM32: {n_stm32} samples in {elapsed:.2f} s")

    if n_emg < 1000 or n_stm32 < 100:
        print("  ERROR: Not enough data. Check cables and STM32.")
        device.close()
        LegacyDevice.cleanup()
        return

    # ----- 4. Extract binary TRIG, downsample, reconstruct STM32 PRBS -----
    print("\n[4/5] Extracting TRIG, reconstructing PRBS, cross-correlating...")
    trig_bin, ext_method, n_trans = extract_trig_bits(trig_raw, is_status)
    print(f"  TRIG extraction: {ext_method}, transitions: {n_trans}")

    # Downsample EMG TRIG to 100 Hz (one value per chip)
    emg_100hz = downsample_to_100hz(trig_bin)
    t_emg_s = np.arange(len(emg_100hz)) / CHIP_RATE_HZ

    # Reference: first 1000 chips of PRBS-15
    ref_1000 = generate_prbs15(seed=PRBS15_SEED, n_chips=CHIPS_PER_SEQUENCE)

    # Reconstructed STM32 PRBS from samples (each sample has prbs_tick 0..999)
    stm32_t_ms = np.array([s.t_ms for s in stm32_samples])
    stm32_prbs_tick = np.array([s.prbs_tick for s in stm32_samples])
    stm32_in_mark = np.array([s.in_mark for s in stm32_samples])
    stm32_reconstructed = ref_1000[(stm32_prbs_tick % CHIPS_PER_SEQUENCE).astype(int)]
    t_stm32_s = stm32_t_ms / 1000.0

    # Find gaps on EMG for alignment (optional)
    gaps = find_low_gaps(trig_bin, min_run=MIN_GAP_RUN)
    print(f"  Gaps detected on EMG TRIG: {len(gaps)}")

    # Cross-correlate: use first 10 s segment of downsampled EMG vs reference
    seg_len = min(CHIPS_PER_SEQUENCE, len(emg_100hz) // 2)
    segment = emg_100hz[:seg_len]
    ref_seg = ref_1000[:seg_len]
    best_lag, peak_val, psr, corr = cross_correlate_100hz(segment, ref_seg)
    print(f"  Correlation peak: {peak_val:.4f}, lag: {best_lag} chips, PSR: {psr:.1f}")

    if abs(peak_val) > 0.15 and psr > 1.5:
        reliability = "PRBS sync RELIABLE"
    elif abs(peak_val) > 0.05:
        reliability = "Weak but detectable correlation"
    else:
        reliability = "Weak correlation - check wiring and firmware"

    print(f"  Assessment: {reliability}")

    # ----- 5. Diagnostics plot -----
    if do_plot or save_plot:
        print("\n[5/5] Generating diagnostics plot...")
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))
        fig.suptitle("PRBS Synchronization Diagnostics", fontsize=14, fontweight="bold")

        # Panel 1: Incoming PRBS (EMG TRIG) - show last 2 s for clarity
        ax = axes[0]
        show_emg = min(int(2 * EMG_SAMPLE_RATE), len(trig_bin))
        t_show = np.arange(show_emg) / EMG_SAMPLE_RATE
        ax.plot(t_show, trig_bin[-show_emg:], linewidth=0.5, color="steelblue")
        base_idx = len(trig_bin) - show_emg
        for (gs, ge) in gaps:
            if gs < len(trig_bin) and ge > base_idx:
                x0 = max(0.0, (gs - base_idx) / EMG_SAMPLE_RATE)
                x1 = min(t_show[-1] if len(t_show) else 2.0, (ge - base_idx) / EMG_SAMPLE_RATE)
                ax.axvspan(x0, x1, alpha=0.25, color="salmon")
        ax.set_ylabel("TRIG (binary)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Panel 1: Incoming PRBS (EMG TRIG, last 2 s)")
        ax.set_ylim(-0.1, 1.3)

        # Panel 2: Reconstructed STM32 PRBS - last 2 s
        ax = axes[1]
        n_stm32_show = min(int(2 * 500), len(stm32_t_ms))
        t_s_show = stm32_t_ms[-n_stm32_show:] / 1000.0
        ax.plot(t_s_show, stm32_reconstructed[-n_stm32_show:], linewidth=0.5, color="green", alpha=0.8)
        ax.set_ylabel("PRBS (0/1)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Panel 2: Reconstructed STM32 PRBS (last 2 s)")
        ax.set_ylim(-0.1, 1.3)

        # Panel 3: Cross-correlation
        ax = axes[2]
        lags = np.arange(len(corr)) - (len(segment) - 1)
        hw = min(500, len(corr) // 2)
        peak_idx = np.argmax(np.abs(corr))
        cl, cr = max(0, peak_idx - hw), min(len(corr), peak_idx + hw)
        ax.plot(lags[cl:cr], corr[cl:cr], linewidth=0.8, color="steelblue")
        ax.axvline(best_lag, color="red", linestyle="--", label=f"peak lag = {best_lag}")
        ax.legend()
        ax.set_ylabel("Normalised correlation")
        ax.set_xlabel("Lag (chips)")
        ax.set_title(f"Panel 3: Cross-correlation (peak = {peak_val:.4f}, PSR = {psr:.1f})")

        # Panel 4: Summary
        ax = axes[3]
        ax.axis("off")
        summary = (
            f"Duration: {duration_s:.1f} s  |  EMG samples: {n_emg}  |  STM32 samples: {n_stm32}\n"
            f"Correlation peak: {peak_val:.4f}  |  Lag: {best_lag} chips  |  PSR: {psr:.1f}\n"
            f"Gaps on EMG: {len(gaps)}  |  STM32 in_mark range: [{stm32_in_mark.min()}, {stm32_in_mark.max()}]\n\n"
            f"Assessment: {reliability}"
        )
        ax.text(0.05, 0.5, summary, transform=ax.transAxes, fontsize=11, verticalalignment="center",
                fontfamily="monospace")

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        if save_plot:
            fig.savefig(save_plot, dpi=150, bbox_inches="tight")
            print(f"  Plot saved: {save_plot}")
        if do_plot:
            plt.show(block=True)

    # Cleanup
    try:
        device.close()
        LegacyDevice.cleanup()
    except Exception:
        pass
    print("\nDone.")


if __name__ == "__main__":
    main()
