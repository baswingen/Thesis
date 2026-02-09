"""
TRIG PRBS-15 Synchronization Testing
=====================================
Reads the PRBS-15 signal sent by an STM32F401 into the Porti7 TRIG input,
visually verifies reception, cross-correlates with a local reference, and
demonstrates that the frame markers can be used to measure clock drift between
the two devices.

Hardware setup:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

STM32 sketch parameters (must match the values below):
    - PRBS-15 LFSR: polynomial x^15 + x^14 + 1, seed 0x7ACE
    - Chip rate: 2000 Hz  (matches Porti7 base sample rate)
    - NRZ encoding: bit value -> HIGH / LOW
    - Frame marker: 30 ms LOW gap once per second (LFSR paused during gap)

Updated STM32 sketch (flash via Arduino IDE / STM32 core):
---------------------------------------------------------------
/*
  STM32F401 -> Porti7 TRIG PRBS-15 generator  (2000 Hz, LOW-gap marker)

  Wiring to Porti7 TRIG (LEMO FFA.00 coax):
    - Center pin: PRBS_OUT (through 330R..1k series resistor recommended)
    - Shield: GND
*/

#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PC13
#endif

static const uint8_t PRBS_PIN = PA8;

// --- PRBS + timing settings ---
static const uint32_t CHIP_RATE_HZ = 2000;   // matches Porti7 sample rate
static const uint32_t FRAME_HZ     = 1;      // marker once per second
static const uint32_t MARK_MS      = 30;     // 30 ms LOW-gap marker

// LFSR state (PRBS-15): x^15 + x^14 + 1
volatile uint16_t lfsr = 0x7ACE;              // non-zero seed

// Marker handling
volatile uint32_t tick_count      = 0;
volatile uint32_t chips_per_frame = 0;
volatile uint32_t mark_ticks      = 0;
volatile bool     in_mark         = false;

HardwareTimer *timer = nullptr;

static inline uint8_t lfsr_next_bit_prbs15(volatile uint16_t &s) {
  uint8_t b14 = (s >> 14) & 1;
  uint8_t b13 = (s >> 13) & 1;
  uint8_t newbit = b14 ^ b13;
  s = (uint16_t)((s << 1) | newbit);
  return newbit;
}

void onTick() {
  tick_count++;

  // Start marker at the beginning of each frame
  if (tick_count % chips_per_frame == 1) {
    in_mark = true;
    mark_ticks = (uint32_t)((CHIP_RATE_HZ * MARK_MS) / 1000);
    if (mark_ticks < 1) mark_ticks = 1;
  }

  if (in_mark) {
    // LOW gap marker -- force output LOW, do NOT advance LFSR
    digitalWrite(PRBS_PIN, LOW);
    if (mark_ticks > 0) {
      mark_ticks--;
    } else {
      in_mark = false;
    }
    return;
  }

  // PRBS output (NRZ)
  uint8_t bit = lfsr_next_bit_prbs15(lfsr);
  digitalWrite(PRBS_PIN, bit ? HIGH : LOW);
}

void setup() {
  pinMode(PRBS_PIN, OUTPUT);
  digitalWrite(PRBS_PIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  chips_per_frame = CHIP_RATE_HZ / FRAME_HZ;
  if (chips_per_frame < 2) chips_per_frame = 2;

  timer = new HardwareTimer(TIM2);
  timer->setOverflow(CHIP_RATE_HZ, HERTZ_FORMAT);
  timer->attachInterrupt(onTick);
  timer->resume();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static uint32_t last_ms = 0;
  if (millis() - last_ms > 500) {
    last_ms = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
---------------------------------------------------------------

Prerequisites:
- TMSi Porti7 connected via USB, drivers installed
- TMSiSDK.dll available in PATH or legacy module directory
- STM32F401 running the sketch above, TRIG cable connected

Author: Generated for Master Thesis
Date:   February 2026
"""

import sys
import os
import time
import argparse
import numpy as np
from datetime import datetime

# ---------------------------------------------------------------------------
# Optional imports
# ---------------------------------------------------------------------------
try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available -- visualisation disabled")

try:
    from scipy import signal as sp_signal
    SCIPY_AVAILABLE = True
except Exception as e:
    SCIPY_AVAILABLE = False
    print(f"Warning: scipy not available ({e}) -- correlation will use numpy (slower)")

# ---------------------------------------------------------------------------
# TMSi SDK
# ---------------------------------------------------------------------------
tmsi_interface_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                    "tmsi-python-interface")
if os.path.exists(tmsi_interface_path):
    sys.path.insert(0, tmsi_interface_path)

try:
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import (DeviceType,
                                                    DeviceInterfaceType,
                                                    MeasurementType)
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
except ImportError as e:
    print("=" * 60)
    print("ERROR: TMSi Python Interface not found!")
    print("=" * 60)
    print(f"\nImport error: {e}")
    print("Make sure tmsi-python-interface is in the project root.")
    print("=" * 60)
    sys.exit(1)

# ============================================================================
# STM32 sketch parameters -- keep in sync with the flashed firmware
# ============================================================================
CHIP_RATE_HZ = 2000          # PRBS chip rate (must equal Porti7 sample rate)
FRAME_HZ = 1                 # marker rate (once per second)
MARK_MS = 30                 # LOW-gap marker duration in ms
PRBS15_SEED = 0x7ACE         # LFSR seed (non-zero, 15-bit)
PRBS15_PERIOD = (1 << 15) - 1  # 32 767 chips

# Derived constants
MARK_CHIPS = int(CHIP_RATE_HZ * MARK_MS / 1000)          # 60
CHIPS_PER_FRAME = CHIP_RATE_HZ // FRAME_HZ                # 2000
PRBS_CHIPS_PER_FRAME = CHIPS_PER_FRAME - MARK_CHIPS       # ~1940
# Any run of zeros > MAX_NATURAL_ZERO_RUN is a marker gap
MAX_NATURAL_ZERO_RUN = 14    # for PRBS-15 the longest natural zero run is n-1=14
MIN_GAP_RUN = MAX_NATURAL_ZERO_RUN + 2  # detection threshold (16 consecutive 0s)


# ============================================================================
# PRBS-15 reference generator  (must match STM32 LFSR exactly)
# ============================================================================

def generate_prbs15(seed: int = PRBS15_SEED, n_chips: int = PRBS15_PERIOD) -> np.ndarray:
    """
    Generate a PRBS-15 bit sequence using the same LFSR as the STM32 sketch.

    Polynomial: x^15 + x^14 + 1  (taps at bits 14 and 13, 0-indexed).
    Shift direction: left-shift, new bit inserted at LSB.

    Returns
    -------
    np.ndarray of shape (n_chips,) with values 0.0 / 1.0
    """
    lfsr = seed & 0x7FFF
    seq = np.empty(n_chips, dtype=np.float64)
    for i in range(n_chips):
        b14 = (lfsr >> 14) & 1
        b13 = (lfsr >> 13) & 1
        newbit = b14 ^ b13
        lfsr = ((lfsr << 1) | newbit) & 0x7FFF
        seq[i] = float(newbit)
    return seq


# ============================================================================
# Channel helpers
# ============================================================================

def fix_channel_name(corrupted_name: str, channel_index: int) -> str:
    """
    Fix corrupted UTF-16LE channel names from TMSi legacy device.

    Porti7 (38 channels):
        0-15  : UNI1-UNI16
        16-35 : BIP1-BIP20
        36    : STATUS
        37    : COUNTER
    """
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
    """Apply encoding fix to all channels that need it."""
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
    """
    Locate the channel that carries the TRIG input.

    Strategy:
        1. Look for a channel explicitly named TRIG / TRIGGER.
        2. Look for a channel named DIG / DIGITAL (Porti7 names the trigger
           input channel "Dig" or "Digi" due to truncated UTF-16LE encoding).
        3. Fall back to the STATUS channel by name or index 36.

    Returns (channel_index, is_status_bit)
        is_status_bit: True  -> data is a bit-field, extract bit 0
                       False -> data is a direct digital value or voltage
    """
    # 1. Explicit TRIG channel
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "TRIG" in name:
            return i, False

    # 2. Digital channel (Porti7 "Dig" / "Digi" / "Digital")
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if name.startswith("DIG"):
            # "Dig" channel is the digital trigger port -- may be bit-field
            # or direct value; caller should try both extraction methods
            return i, True

    # 3. STATUS channel by name or hard-coded index 36
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "STATUS" in name or "STATU" in name:
            return i, True

    # 4. Last resort: index 36
    if len(channels) > 36:
        return 36, True

    return None, None


def find_counter_channel(channels):
    """Locate the COUNTER channel (index 37 on Porti7)."""
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "COUNTER" in name or i == 37:
            gain = None
            offset = None
            try:
                gain = ch.get_channel_unit_gain()
            except Exception:
                pass
            try:
                offset = ch.get_channel_unit_offset()
            except Exception:
                pass
            return i, gain, offset
    return None, None, None


# ============================================================================
# Signal processing helpers
# ============================================================================

def extract_trig_bits(raw: np.ndarray, is_status_bit: bool) -> np.ndarray:
    """
    Convert raw channel data to a binary TRIG signal (0 / 1).

    Tries multiple strategies and picks the one with the most transitions:
        1. Bit 0 extraction  (for STATUS / Digital bit-field channels)
        2. Midpoint threshold (for analogue / voltage channels)
        3. Non-zero detection (raw != 0)

    Returns the binary signal that shows the most transitions (i.e. the one
    most likely to contain the PRBS pattern).
    """
    candidates = {}

    # Strategy 1: bit 0
    raw_int = raw.astype(np.int64)
    bit0 = (raw_int & 1).astype(np.float64)
    candidates["bit0"] = bit0

    # Strategy 2: midpoint threshold
    lo, hi = np.nanmin(raw), np.nanmax(raw)
    if hi - lo > 1e-9:
        threshold = (lo + hi) / 2.0
        thresh = (raw > threshold).astype(np.float64)
        candidates["threshold"] = thresh

    # Strategy 3: non-zero
    nonzero = (raw_int != 0).astype(np.float64)
    candidates["nonzero"] = nonzero

    # Pick the candidate with the most transitions
    best_name = None
    best_trans = -1
    best_sig = bit0  # default fallback
    for name, sig in candidates.items():
        t = int(np.sum(np.abs(np.diff(sig))))
        if t > best_trans:
            best_trans = t
            best_name = name
            best_sig = sig

    return best_sig, best_name, best_trans


def find_low_gaps(trig: np.ndarray, min_run: int = MIN_GAP_RUN):
    """
    Find contiguous runs of LOW (0) longer than *min_run* samples.

    Returns list of (gap_start_idx, gap_end_idx) where gap_end_idx is the
    first sample *after* the gap (i.e. where PRBS resumes).
    """
    gaps = []
    n = len(trig)
    i = 0
    while i < n:
        if trig[i] == 0:
            j = i
            while j < n and trig[j] == 0:
                j += 1
            run_len = j - i
            if run_len >= min_run:
                gaps.append((i, j))
            i = j
        else:
            i += 1
    return gaps


def extract_prbs_segments(trig: np.ndarray, gaps):
    """
    Extract PRBS data segments between consecutive marker gaps.

    Each segment starts at gap_end of one marker and ends at gap_start of the
    next marker.  Returns list of (start_idx, segment_array).
    """
    segments = []
    for k in range(len(gaps) - 1):
        seg_start = gaps[k][1]      # first sample after this gap
        seg_end = gaps[k + 1][0]    # last sample before next gap
        if seg_end > seg_start + 10:  # need a minimum viable length
            segments.append((seg_start, trig[seg_start:seg_end]))
    return segments


def cross_correlate(segment: np.ndarray, reference: np.ndarray):
    """
    Cross-correlate a received PRBS segment against the full-period reference.

    Uses scipy.signal.correlate (FFT-based) when available, else numpy.

    Returns (best_lag, peak_value, peak_to_sidelobe_ratio, correlation_array).
    """
    # Convert to bipolar (+1 / -1) for sharper correlation
    seg_bp = 2.0 * segment - 1.0
    ref_bp = 2.0 * reference - 1.0

    if SCIPY_AVAILABLE:
        corr = sp_signal.correlate(seg_bp, ref_bp, mode="full")
    else:
        corr = np.correlate(seg_bp, ref_bp, mode="full")

    # Normalise
    norm = np.sqrt(np.sum(seg_bp ** 2) * np.sum(ref_bp ** 2))
    if norm > 0:
        corr = corr / norm

    peak_idx = np.argmax(np.abs(corr))
    peak_val = corr[peak_idx]

    # Peak-to-sidelobe ratio
    abs_corr = np.abs(corr)
    sorted_vals = np.sort(abs_corr)[::-1]
    if len(sorted_vals) > 1 and sorted_vals[1] > 0:
        psr = sorted_vals[0] / sorted_vals[1]
    else:
        psr = float("inf")

    # Lag: positive means reference is delayed relative to segment
    best_lag = peak_idx - (len(segment) - 1)
    return best_lag, peak_val, psr, corr


def analyse_marker_timing(gaps, sample_rate: float):
    """
    Compute inter-marker intervals and clock drift statistics.

    Uses gap-start positions as the timing reference (falling edge into the gap
    is very sharp).

    Returns dict with keys: intervals_samples, intervals_ms, expected_samples,
    drift_ppm, drift_ms_per_min, cumulative_offset_ms.
    """
    if len(gaps) < 2:
        return None

    starts = np.array([g[0] for g in gaps], dtype=np.float64)
    intervals = np.diff(starts)  # in samples

    expected = sample_rate / FRAME_HZ  # e.g. 2048 samples for 1 Hz markers

    mean_interval = np.mean(intervals)
    drift_frac = (mean_interval - expected) / expected  # fractional
    drift_ppm = drift_frac * 1e6
    drift_ms_per_min = drift_frac * 60_000.0

    cumulative_offsets = np.cumsum(intervals - expected) / sample_rate * 1000.0  # ms

    return {
        "intervals_samples": intervals,
        "intervals_ms": intervals / sample_rate * 1000.0,
        "expected_samples": expected,
        "mean_interval_samples": mean_interval,
        "drift_ppm": drift_ppm,
        "drift_ms_per_min": drift_ms_per_min,
        "cumulative_offset_ms": cumulative_offsets,
        "marker_times_s": starts / sample_rate,
    }


# ============================================================================
# Visualisation
# ============================================================================

def plot_results(trig, sample_rate, gaps, segments, corr_results, timing,
                 save_path=None):
    """Create the 4-subplot summary figure."""
    if not MATPLOTLIB_AVAILABLE:
        print("[WARN] matplotlib not available -- skipping plots")
        return

    t = np.arange(len(trig)) / sample_rate  # time axis in seconds

    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle("TRIG PRBS-15 Synchronization Verification", fontsize=14,
                 fontweight="bold")

    # --- Plot 1: full-duration raw TRIG signal ---
    ax = axes[0]
    ax.plot(t, trig, linewidth=0.4, color="steelblue")
    # Shade marker gaps
    for gs, ge in gaps:
        ax.axvspan(gs / sample_rate, ge / sample_rate,
                   alpha=0.25, color="salmon", label=None)
    ax.set_ylabel("TRIG (binary)")
    ax.set_xlabel("Time (s)")
    ax.set_title(f"Raw TRIG signal  ({len(gaps)} marker gaps detected)")
    ax.set_ylim(-0.1, 1.3)
    ax.set_xlim(t[0], t[-1])

    # --- Plot 2: zoomed view around first marker gap ---
    ax = axes[1]
    if gaps:
        # Centre the zoom on the first gap
        gap_centre = (gaps[0][0] + gaps[0][1]) // 2
        half_win = int(0.10 * sample_rate)  # +/- 100 ms
        z_start = max(0, gap_centre - half_win)
        z_end = min(len(trig), gap_centre + half_win)
        tz = t[z_start:z_end]
        ax.step(tz, trig[z_start:z_end], where="mid", linewidth=0.8,
                color="steelblue")
        for gs, ge in gaps:
            if z_start <= gs <= z_end or z_start <= ge <= z_end:
                ax.axvspan(max(gs, z_start) / sample_rate,
                           min(ge, z_end) / sample_rate,
                           alpha=0.25, color="salmon")
        ax.set_title("Zoomed view: PRBS chips + marker gap (first marker)")
    else:
        ax.text(0.5, 0.5, "No marker gaps found", transform=ax.transAxes,
                ha="center", va="center", fontsize=12, color="red")
        ax.set_title("Zoomed view (no markers)")
    ax.set_ylabel("TRIG (binary)")
    ax.set_xlabel("Time (s)")
    ax.set_ylim(-0.1, 1.3)

    # --- Plot 3: cross-correlation ---
    ax = axes[2]
    if corr_results:
        best_lag, peak_val, psr, corr = corr_results
        lags = np.arange(len(corr)) - (len(segments[0][1]) - 1)
        # Show a window around the peak
        peak_idx = np.argmax(np.abs(corr))
        hw = min(500, len(corr) // 2)
        cl = max(0, peak_idx - hw)
        cr = min(len(corr), peak_idx + hw)
        ax.plot(lags[cl:cr], corr[cl:cr], linewidth=0.6, color="steelblue")
        ax.axvline(best_lag, color="red", linestyle="--", linewidth=1,
                   label=f"peak lag = {best_lag} chips")
        ax.legend(fontsize=9)
        ax.set_title(f"Cross-correlation  (peak = {peak_val:.4f},  "
                     f"PSR = {psr:.1f})")
    else:
        ax.text(0.5, 0.5, "No PRBS segments for correlation",
                transform=ax.transAxes, ha="center", va="center",
                fontsize=12, color="red")
        ax.set_title("Cross-correlation (no data)")
    ax.set_ylabel("Normalised correlation")
    ax.set_xlabel("Lag (chips)")

    # --- Plot 4: marker timing stability ---
    ax = axes[3]
    if timing is not None:
        intervals = timing["intervals_samples"]
        expected = timing["expected_samples"]
        marker_idx = np.arange(1, len(intervals) + 1)
        ax.plot(marker_idx, intervals, "o-", markersize=4, linewidth=1,
                color="steelblue", label="measured")
        ax.axhline(expected, color="grey", linestyle="--", linewidth=1,
                   label=f"expected ({expected:.0f} samp)")
        # Trend line
        if len(intervals) > 2:
            z = np.polyfit(marker_idx, intervals, 1)
            ax.plot(marker_idx, np.polyval(z, marker_idx), color="red",
                    linewidth=1, linestyle=":", label="trend")
        ax.legend(fontsize=9)
        ax.set_title(f"Inter-marker interval  "
                     f"(drift = {timing['drift_ppm']:.1f} ppm)")
        ax.set_ylabel("Interval (samples)")
        ax.set_xlabel("Marker index")
    else:
        ax.text(0.5, 0.5, "Need >= 2 markers for timing analysis",
                transform=ax.transAxes, ha="center", va="center",
                fontsize=12, color="red")
        ax.set_title("Marker timing (insufficient data)")

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  [OK] Plot saved to: {save_path}")

    plt.show(block=True)


# ============================================================================
# Main
# ============================================================================

def main():
    # ------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="TRIG PRBS-15 synchronization verification")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Acquisition duration in seconds (default: 10)")
    parser.add_argument("--save-plot", type=str, default=None,
                        help="Save figure to this path (e.g. trig_test.png)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip visualisation")
    args = parser.parse_args()

    duration_s = args.duration
    save_plot = args.save_plot
    do_plot = (not args.no_plot) and MATPLOTLIB_AVAILABLE

    print("=" * 70)
    print("  TRIG PRBS-15 Synchronization Verification")
    print("=" * 70)
    print(f"  Acquisition duration : {duration_s:.1f} s")
    print(f"  Expected chip rate   : {CHIP_RATE_HZ} Hz")
    print(f"  Marker gap           : {MARK_MS} ms LOW ({MARK_CHIPS} chips)")
    print(f"  PRBS-15 period       : {PRBS15_PERIOD} chips")
    print(f"  PRBS chips per frame : ~{PRBS_CHIPS_PER_FRAME}")
    print()

    # ------------------------------------------------------------------
    # 1. Connect to Porti7
    # ------------------------------------------------------------------
    print("[1/6] Initialising TMSi SDK...")
    sdk = TMSiSDK()

    print("  Discovering legacy devices (Porti7/REFA)...")
    devices, _ = sdk.discover(DeviceType.legacy,
                              dr_interface=DeviceInterfaceType.usb)
    if not devices:
        print("  ERROR: No TMSi legacy devices found. Is the Porti7 powered on?")
        return

    device = devices[0]
    device.open()
    print(f"  Connected: {device.get_device_name()}")
    print(f"  Serial   : {device.get_device_serial_number()}")
    print(f"  Base rate : {device.get_device_base_sample_rate()} Hz")

    channels = device.get_device_channels()
    n_ch = len(channels)
    fixed = fix_all_channel_names(channels)
    if fixed:
        print(f"  Fixed {fixed} corrupted channel names")

    # Print channel summary
    print(f"  Channels  : {n_ch}")
    for i, ch in enumerate(channels):
        print(f"    [{i:2d}] {ch.get_channel_name():12s}  {ch.get_channel_unit_name()}")

    # Locate TRIG and COUNTER channels
    trig_idx, is_status = find_trig_channel(channels)
    if trig_idx is None:
        print("  ERROR: Cannot find TRIG or STATUS channel!")
        device.close()
        LegacyDevice.cleanup()
        return
    ch_name = channels[trig_idx].get_channel_name()
    source = f"bit-field extraction" if is_status else "direct digital"
    print(f"\n  TRIG channel : index {trig_idx}  (name: '{ch_name}', {source})")

    counter_idx, counter_gain, counter_offset = find_counter_channel(channels)
    if counter_idx is not None:
        print(f"  COUNTER channel : index {counter_idx}")
    else:
        print("  COUNTER channel : not found (timing from host clock)")

    # ------------------------------------------------------------------
    # 2. Create measurement
    # ------------------------------------------------------------------
    print("\n[2/6] Creating measurement...")
    measurement = MeasurementType.LEGACY_SIGNAL(device)
    measurement.set_reference_calculation(False)
    sample_rate = measurement.get_device_sample_rate()
    print(f"  Sample rate: {sample_rate} Hz")

    if abs(sample_rate - CHIP_RATE_HZ) > 1:
        print(f"  WARNING: sample rate ({sample_rate}) != chip rate ({CHIP_RATE_HZ}).")
        print(f"           Update the STM32 sketch CHIP_RATE_HZ to match,")
        print(f"           or cross-correlation results may be degraded.")

    # ------------------------------------------------------------------
    # 3. Acquire data
    # ------------------------------------------------------------------
    print(f"\n[3/6] Acquiring TRIG data for {duration_s:.1f} s ...")
    measurement.start()

    trig_chunks = []
    chunk_count = 0
    total_samples = 0
    t_start = time.time()

    try:
        while (time.time() - t_start) < duration_s:
            samples = measurement.get_samples(blocking=False)
            if samples is not None and len(samples) > 0:
                raw_trig = samples[:, trig_idx]
                trig_chunks.append(raw_trig.copy())
                chunk_count += 1
                total_samples += samples.shape[0]
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
    finally:
        measurement.stop()

    elapsed = time.time() - t_start
    print(f"  Done. {chunk_count} chunks, {total_samples} samples in {elapsed:.2f} s")
    print(f"  Effective rate: {total_samples / elapsed:.1f} sps")

    if total_samples < 100:
        print("  ERROR: Not enough data collected. Check TRIG cable and STM32.")
        device.close()
        LegacyDevice.cleanup()
        return

    # Concatenate into a single array
    trig_raw = np.concatenate(trig_chunks)

    # Debug: show raw channel statistics before extraction
    print(f"\n  Raw channel statistics:")
    print(f"    min = {np.nanmin(trig_raw):.6f}")
    print(f"    max = {np.nanmax(trig_raw):.6f}")
    print(f"    mean = {np.nanmean(trig_raw):.6f}")
    unique_vals = np.unique(trig_raw)
    if len(unique_vals) <= 20:
        print(f"    unique values ({len(unique_vals)}): {unique_vals}")
    else:
        print(f"    unique values: {len(unique_vals)} distinct  "
              f"(first 10: {unique_vals[:10]})")

    # Extract binary TRIG signal using best-of-multiple strategies
    trig, extraction_method, transitions = extract_trig_bits(trig_raw, is_status)

    ones_frac = np.mean(trig)
    print(f"\n  Extraction method  : {extraction_method}")
    print(f"  TRIG high fraction : {ones_frac * 100:.1f}%  "
          f"(expect ~45-55% for PRBS + markers)")
    print(f"  TRIG transitions   : {int(transitions)}  "
          f"(expect ~{int(total_samples * 0.40)}-{int(total_samples * 0.55)} "
          f"for PRBS-15)")

    if transitions < 10:
        print("\n  WARNING: Very few transitions detected!")
        print("  -> Is the STM32 running and the TRIG cable connected?")
        # Exhaustive bit analysis of the raw channel
        print("\n  Exhaustive bit analysis of raw channel:")
        raw_int = trig_raw.astype(np.int64)
        any_found = False
        best_bit = 0
        best_trans = 0
        for bit in range(32):
            bit_vals = (raw_int >> bit) & 1
            bt = int(np.sum(np.abs(np.diff(bit_vals))))
            bh = np.mean(bit_vals)
            if bt > 0:
                print(f"    bit {bit:2d}: {bt:6d} transitions, "
                      f"{bh * 100:.1f}% high")
                any_found = True
            if bt > best_trans:
                best_trans = bt
                best_bit = bit
        if not any_found:
            print("    (no bits show any transitions)")
        if best_trans > transitions and best_trans > 10:
            print(f"\n  Auto-selecting bit {best_bit} "
                  f"({best_trans} transitions)")
            trig = ((raw_int >> best_bit) & 1).astype(np.float64)
            transitions = best_trans

    # ------------------------------------------------------------------
    # 4. Detect frame markers (LOW gaps)
    # ------------------------------------------------------------------
    print(f"\n[4/6] Detecting frame markers (LOW gaps > {MIN_GAP_RUN} samples)...")
    gaps = find_low_gaps(trig, min_run=MIN_GAP_RUN)
    print(f"  Found {len(gaps)} marker gaps")

    if gaps:
        gap_lengths = np.array([g[1] - g[0] for g in gaps])
        print(f"  Gap lengths: mean={np.mean(gap_lengths):.1f}, "
              f"min={np.min(gap_lengths)}, max={np.max(gap_lengths)} samples")
        print(f"  Expected gap length: ~{MARK_CHIPS} samples "
              f"({MARK_MS} ms at {CHIP_RATE_HZ} Hz)")

    # ------------------------------------------------------------------
    # 5. Cross-correlate with PRBS-15 reference
    # ------------------------------------------------------------------
    print("\n[5/6] Cross-correlating with PRBS-15 reference...")
    prbs_ref = generate_prbs15(seed=PRBS15_SEED, n_chips=PRBS15_PERIOD)

    segments = extract_prbs_segments(trig, gaps)
    corr_results = None

    if segments:
        # Use the longest segment for correlation
        longest_idx = max(range(len(segments)), key=lambda k: len(segments[k][1]))
        seg_start, seg_data = segments[longest_idx]
        print(f"  Using segment {longest_idx} ({len(seg_data)} samples, "
              f"starts at sample {seg_start})")

        best_lag, peak_val, psr, corr = cross_correlate(seg_data, prbs_ref)
        corr_results = (best_lag, peak_val, psr, corr)

        print(f"  Correlation peak    : {peak_val:.4f}")
        print(f"  Peak lag            : {best_lag} chips")
        print(f"  Peak-to-sidelobe    : {psr:.1f}")

        if abs(peak_val) > 0.15:
            print("  -> PRBS-15 sequence RECOGNISED")
        else:
            print("  -> WARNING: weak correlation -- PRBS may not match reference")
    else:
        print("  No PRBS segments extracted (need >= 2 marker gaps).")
        # Try correlating the entire signal if no markers found
        if len(trig) > 100 and transitions > 10:
            print("  Attempting correlation on full signal (no marker segmentation)...")
            best_lag, peak_val, psr, corr = cross_correlate(trig, prbs_ref)
            corr_results = (best_lag, peak_val, psr, corr)
            # Make a fake segment for the plot
            segments = [(0, trig)]
            print(f"  Correlation peak    : {peak_val:.4f}")
            print(f"  Peak lag            : {best_lag} chips")
            print(f"  Peak-to-sidelobe    : {psr:.1f}")

    # ------------------------------------------------------------------
    # 6. Marker timing analysis (clock drift)
    # ------------------------------------------------------------------
    print("\n[6/6] Analysing marker timing (clock drift)...")
    timing = analyse_marker_timing(gaps, sample_rate)

    if timing is not None:
        ivl = timing["intervals_samples"]
        exp = timing["expected_samples"]
        print(f"  Expected interval   : {exp:.0f} samples  "
              f"({exp / sample_rate * 1000:.1f} ms)")
        print(f"  Measured mean       : {timing['mean_interval_samples']:.2f} samples  "
              f"({timing['mean_interval_samples'] / sample_rate * 1000:.2f} ms)")
        print(f"  Measured std        : {np.std(ivl):.2f} samples  "
              f"({np.std(ivl) / sample_rate * 1000:.3f} ms)")
        print(f"  Min / Max           : {np.min(ivl):.0f} / {np.max(ivl):.0f} samples")
        print(f"  Clock drift         : {timing['drift_ppm']:.1f} ppm")
        print(f"  Drift rate          : {timing['drift_ms_per_min']:.3f} ms/min")
        cum = timing["cumulative_offset_ms"]
        if len(cum) > 0:
            print(f"  Cumulative offset   : {cum[-1]:.3f} ms over {elapsed:.1f} s")
    else:
        print("  Not enough markers for timing analysis (need >= 2).")

    # ------------------------------------------------------------------
    # Summary / pass-fail
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("  SYNC FEASIBILITY ASSESSMENT")
    print("=" * 70)

    passed = True
    # Check 1: PRBS signal present
    if transitions < 10:
        print("  [FAIL] No PRBS signal detected on TRIG channel")
        passed = False
    else:
        print(f"  [PASS] PRBS signal detected ({int(transitions)} transitions)")

    # Check 2: Marker gaps detected
    if len(gaps) < 2:
        print(f"  [WARN] Only {len(gaps)} marker gaps (need >= 2 for drift analysis)")
    else:
        print(f"  [PASS] {len(gaps)} marker gaps detected")

    # Check 3: Cross-correlation quality
    if corr_results:
        _, pv, ps, _ = corr_results
        if abs(pv) > 0.15 and ps > 2.0:
            print(f"  [PASS] Cross-correlation confirms PRBS-15 "
                  f"(peak={pv:.3f}, PSR={ps:.1f})")
        elif abs(pv) > 0.05:
            print(f"  [WARN] Weak cross-correlation (peak={pv:.3f}, PSR={ps:.1f})")
        else:
            print(f"  [FAIL] Cross-correlation too weak (peak={pv:.3f})")
            passed = False
    else:
        print("  [FAIL] No cross-correlation computed")
        passed = False

    # Check 4: Clock drift
    if timing is not None:
        dppm = abs(timing["drift_ppm"])
        if dppm < 100:
            print(f"  [PASS] Clock drift {dppm:.1f} ppm (< 100 ppm) -- "
                  f"suitable for sync")
        elif dppm < 500:
            print(f"  [WARN] Clock drift {dppm:.1f} ppm -- "
                  f"sync possible but may need frequent correction")
        else:
            print(f"  [FAIL] Clock drift {dppm:.1f} ppm -- "
                  f"too high for reliable sync")
            passed = False

    if passed:
        print("\n  >>> OVERALL: PASS -- PRBS-based synchronization is FEASIBLE <<<")
    else:
        print("\n  >>> OVERALL: ISSUES DETECTED -- see above <<<")
    print("=" * 70)

    # ------------------------------------------------------------------
    # Clean up device
    # ------------------------------------------------------------------
    print("\nCleaning up...")
    try:
        device.close()
        print("  Device closed.")
    except Exception as e:
        print(f"  Error closing device: {e}")
    try:
        LegacyDevice.cleanup()
        print("  SDK cleaned up.")
    except Exception as e:
        print(f"  Error cleaning up SDK: {e}")

    # ------------------------------------------------------------------
    # Plot
    # ------------------------------------------------------------------
    if do_plot:
        print("\nGenerating plots...")
        plot_results(trig, sample_rate, gaps, segments, corr_results, timing,
                     save_path=save_plot)


if __name__ == "__main__":
    main()
