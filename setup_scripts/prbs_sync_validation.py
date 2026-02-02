"""
PRBS Synchronization Validation Script
=======================================

Comprehensive testing and validation of PRBS-based synchronization system.

Tests include:
1. Short-term accuracy (10 minutes)
2. Long-term stability (2 hours)
3. Controlled desync test (artificial drift injection)
4. Cross-correlation validation (offline vs real-time)

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple
import sys
from pathlib import Path

# Add project root to path
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.prbs_sync import PRBSGenerator, SynchronizationEngine, validate_prbs_quality


# =============================================================================
# TEST 1: PRBS QUALITY VALIDATION
# =============================================================================

def test_prbs_quality():
    """Test PRBS sequence quality (autocorrelation properties)."""
    print("\n" + "="*70)
    print("TEST 1: PRBS Quality Validation")
    print("="*70)
    
    # Test different sequence lengths
    lengths = [31, 63, 127, 255, 511]
    
    for length in lengths:
        print(f"\nTesting sequence length {length}:")
        prbs = PRBSGenerator(sequence_length=length, injection_rate_hz=10.0)
        quality = validate_prbs_quality(prbs.get_sequence())
        
        print(f"  Peak value: {quality['peak_value']:.3f}")
        print(f"  Peak-to-sidelobe ratio: {quality['peak_to_sidelobe_ratio']:.1f}")
        print(f"  Quality: {quality['quality']}")
        
        # Quality criteria
        if quality['peak_to_sidelobe_ratio'] > 50:
            print(f"  ✓ PASS: Good autocorrelation properties")
        else:
            print(f"  ✗ FAIL: Poor autocorrelation")
    
    print(f"\n{'='*70}")
    print("✓ Test 1 complete")


# =============================================================================
# TEST 2: SHORT-TERM ACCURACY (SIMULATED)
# =============================================================================

def test_short_term_accuracy():
    """Test short-term synchronization accuracy with known offset."""
    print("\n" + "="*70)
    print("TEST 2: Short-Term Accuracy (Simulated 10 minutes)")
    print("="*70)
    
    # Simulation parameters
    duration_s = 600.0  # 10 minutes
    emg_rate = 1000.0  # Hz
    imu_rate = 50.0  # Hz
    known_offset_ms = 15.3  # Ground truth offset
    
    print(f"\nSimulation parameters:")
    print(f"  Duration: {duration_s/60:.1f} minutes")
    print(f"  EMG rate: {emg_rate} Hz")
    print(f"  IMU rate: {imu_rate} Hz")
    print(f"  Known offset: {known_offset_ms:.1f} ms")
    
    # Create PRBS generator and sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=5.0
    )
    
    # Simulate data streams
    print(f"\nGenerating simulated data...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    t_imu_offset = t_imu + (known_offset_ms / 1000.0)  # Add known offset
    
    # Add PRBS markers
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t) for t in t_imu_offset])
    
    # Feed to sync engine (simulate real-time with downsampling)
    print(f"Running synchronization...")
    offset_estimates = []
    times = []
    
    for i in range(0, len(t_emg), 1000):  # Every second
        # Add batch of samples
        batch_size = min(1000, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(int(imu_rate), len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    sync_engine.add_imu_data(t_imu_offset[imu_idx+j], prbs_imu[imu_idx+j])
        
        # Update sync every 5 seconds
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                offset_estimates.append(sync_state.offset_ms)
                times.append(t_emg[i])
                
                if len(offset_estimates) % 12 == 0:  # Every minute
                    print(f"  t={t_emg[i]/60:.1f}min: offset={sync_state.offset_ms:.2f}ms "
                          f"(error={abs(sync_state.offset_ms - known_offset_ms):.2f}ms)")
    
    # Analyze results
    if offset_estimates:
        offset_estimates = np.array(offset_estimates)
        errors = np.abs(offset_estimates - known_offset_ms)
        
        print(f"\nResults:")
        print(f"  Mean estimated offset: {np.mean(offset_estimates):.2f} ms")
        print(f"  Known offset: {known_offset_ms:.2f} ms")
        print(f"  Mean error: {np.mean(errors):.2f} ms")
        print(f"  Max error: {np.max(errors):.2f} ms")
        print(f"  Std error: {np.std(errors):.2f} ms")
        
        if np.mean(errors) < 2.0:
            print(f"  ✓ PASS: Mean error < 2ms")
        else:
            print(f"  ✗ FAIL: Mean error too large")
    else:
        print(f"  ✗ FAIL: No offset estimates computed")
    
    print(f"\n{'='*70}")
    print("✓ Test 2 complete")
    
    return times, offset_estimates, known_offset_ms


# =============================================================================
# TEST 3: LONG-TERM STABILITY (SIMULATED)
# =============================================================================

def test_long_term_stability():
    """Test long-term synchronization with clock drift."""
    print("\n" + "="*70)
    print("TEST 3: Long-Term Stability (Simulated 2 hours with drift)")
    print("="*70)
    
    # Simulation parameters
    duration_s = 7200.0  # 2 hours
    emg_rate = 100.0  # Hz (downsampled for simulation speed)
    imu_rate = 50.0  # Hz
    base_offset_ms = 10.0  # Initial offset
    drift_rate_ppm = 20.0  # 20 parts per million (realistic crystal drift)
    
    print(f"\nSimulation parameters:")
    print(f"  Duration: {duration_s/3600:.1f} hours")
    print(f"  Base offset: {base_offset_ms:.1f} ms")
    print(f"  Drift rate: {drift_rate_ppm:.1f} ppm")
    print(f"  Expected drift over 2h: {drift_rate_ppm * 2:.1f} ms")
    
    # Create PRBS generator and sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=10.0  # Update every 10s
    )
    
    # Simulate data streams with drift
    print(f"\nGenerating simulated data with clock drift...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    
    # Add base offset + linear drift
    drift_s_per_s = drift_rate_ppm * 1e-6
    drift_offset = t_imu * drift_s_per_s  # Accumulating drift
    t_imu_offset = t_imu + (base_offset_ms / 1000.0) + drift_offset
    
    # Add PRBS markers
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t) for t in t_imu_offset])
    
    # Feed to sync engine
    print(f"Running synchronization with drift correction...")
    offset_estimates = []
    true_offsets = []
    times = []
    
    for i in range(0, len(t_emg), 1000):  # Every 10 seconds
        batch_size = min(1000, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(int(imu_rate * 10), len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    sync_engine.add_imu_data(t_imu_offset[imu_idx+j], prbs_imu[imu_idx+j])
        
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                current_time = t_emg[i]
                true_offset_ms = (base_offset_ms + drift_s_per_s * current_time * 1000.0)
                
                offset_estimates.append(sync_state.offset_ms)
                true_offsets.append(true_offset_ms)
                times.append(current_time)
                
                if len(offset_estimates) % 60 == 0:  # Every 10 minutes
                    print(f"  t={current_time/3600:.2f}h: est={sync_state.offset_ms:.2f}ms "
                          f"true={true_offset_ms:.2f}ms "
                          f"error={abs(sync_state.offset_ms - true_offset_ms):.2f}ms")
    
    # Analyze results
    if offset_estimates:
        offset_estimates = np.array(offset_estimates)
        true_offsets = np.array(true_offsets)
        errors = np.abs(offset_estimates - true_offsets)
        
        print(f"\nResults:")
        print(f"  Initial offset: {base_offset_ms:.2f} ms")
        print(f"  Final true offset: {true_offsets[-1]:.2f} ms")
        print(f"  Final estimated offset: {offset_estimates[-1]:.2f} ms")
        print(f"  Total drift: {true_offsets[-1] - base_offset_ms:.2f} ms")
        print(f"  Mean tracking error: {np.mean(errors):.2f} ms")
        print(f"  Max tracking error: {np.max(errors):.2f} ms")
        
        if np.max(errors) < 10.0:
            print(f"  ✓ PASS: Max error < 10ms over 2 hours")
        else:
            print(f"  ✗ FAIL: Drift correction insufficient")
    else:
        print(f"  ✗ FAIL: No offset estimates computed")
    
    print(f"\n{'='*70}")
    print("✓ Test 3 complete")
    
    return times, offset_estimates, true_offsets


# =============================================================================
# TEST 4: CONTROLLED DESYNC TEST
# =============================================================================

def test_controlled_desync():
    """Test detection and correction of sudden desynchronization."""
    print("\n" + "="*70)
    print("TEST 4: Controlled Desync Test")
    print("="*70)
    
    # Simulation parameters
    duration_s = 60.0  # 1 minute
    emg_rate = 1000.0  # Hz
    imu_rate = 50.0  # Hz
    initial_offset_ms = 5.0
    desync_time_s = 30.0  # Introduce sudden 20ms desync at t=30s
    desync_amount_ms = 20.0
    
    print(f"\nTest parameters:")
    print(f"  Duration: {duration_s} seconds")
    print(f"  Initial offset: {initial_offset_ms:.1f} ms")
    print(f"  Desync at t={desync_time_s}s: +{desync_amount_ms:.1f} ms")
    
    # Create sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=2.0  # Faster updates for desync detection
    )
    
    # Generate data
    print(f"\nGenerating test data...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    
    # Add initial offset, then sudden desync
    t_imu_offset = t_imu + (initial_offset_ms / 1000.0)
    desync_mask = t_imu >= desync_time_s
    t_imu_offset[desync_mask] += (desync_amount_ms / 1000.0)
    
    # Add PRBS markers
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t) for t in t_imu_offset])
    
    # Run sync
    print(f"Running desync detection test...")
    offset_estimates = []
    times = []
    
    for i in range(0, len(t_emg), 100):  # Every 0.1s
        batch_size = min(100, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(5, len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    sync_engine.add_imu_data(t_imu_offset[imu_idx+j], prbs_imu[imu_idx+j])
        
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                offset_estimates.append(sync_state.offset_ms)
                times.append(t_emg[i])
    
    # Analyze desync detection
    if offset_estimates and len(times) > 0:
        offset_estimates = np.array(offset_estimates)
        times = np.array(times)
        
        # Find estimates before and after desync
        pre_desync_mask = times < (desync_time_s - 2.0)
        post_desync_mask = times > (desync_time_s + 10.0)
        
        if np.any(pre_desync_mask) and np.any(post_desync_mask):
            pre_mean = np.mean(offset_estimates[pre_desync_mask])
            post_mean = np.mean(offset_estimates[post_desync_mask])
            detected_change = post_mean - pre_mean
            
            print(f"\nResults:")
            print(f"  Pre-desync mean offset: {pre_mean:.2f} ms")
            print(f"  Post-desync mean offset: {post_mean:.2f} ms")
            print(f"  Detected offset change: {detected_change:.2f} ms")
            print(f"  Actual desync amount: {desync_amount_ms:.2f} ms")
            print(f"  Detection error: {abs(detected_change - desync_amount_ms):.2f} ms")
            
            if abs(detected_change - desync_amount_ms) < 3.0:
                print(f"  ✓ PASS: Desync detected accurately")
            else:
                print(f"  ✗ FAIL: Desync detection error too large")
        else:
            print(f"  ✗ FAIL: Insufficient data for analysis")
    else:
        print(f"  ✗ FAIL: No offset estimates")
    
    print(f"\n{'='*70}")
    print("✓ Test 4 complete")
    
    return times, offset_estimates


# =============================================================================
# MAIN TEST SUITE
# =============================================================================

def run_all_tests():
    """Run complete test suite."""
    print("\n" + "="*70)
    print("PRBS SYNCHRONIZATION VALIDATION SUITE")
    print("="*70)
    print("\nThis script validates the PRBS-based synchronization system")
    print("through simulated tests.")
    
    # Run tests
    test_prbs_quality()
    times2, offsets2, known_offset = test_short_term_accuracy()
    times3, offsets3, true_offsets = test_long_term_stability()
    times4, offsets4 = test_controlled_desync()
    
    # Create summary plots
    print("\n" + "="*70)
    print("GENERATING VALIDATION PLOTS")
    print("="*70)
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('PRBS Synchronization Validation Results', fontsize=14, fontweight='bold')
    
    # Plot 1: Short-term accuracy
    if len(times2) > 0:
        ax = axes[0]
        errors = np.abs(np.array(offsets2) - known_offset)
        ax.plot(np.array(times2)/60, errors, 'b-', linewidth=1.5, label='Tracking Error')
        ax.axhline(y=2.0, color='orange', linestyle='--', label='2ms threshold')
        ax.axhline(y=5.0, color='red', linestyle='--', label='5ms threshold')
        ax.set_xlabel('Time (minutes)')
        ax.set_ylabel('Error (ms)')
        ax.set_title('Test 2: Short-Term Accuracy (10 min)')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    # Plot 2: Long-term stability
    if len(times3) > 0:
        ax = axes[1]
        times_h = np.array(times3) / 3600
        ax.plot(times_h, offsets3, 'b-', linewidth=1.5, label='Estimated Offset')
        ax.plot(times_h, true_offsets, 'r--', linewidth=1.5, label='True Offset (with drift)')
        ax.set_xlabel('Time (hours)')
        ax.set_ylabel('Offset (ms)')
        ax.set_title('Test 3: Long-Term Stability (2 hours with drift)')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    # Plot 3: Controlled desync
    if len(times4) > 0:
        ax = axes[2]
        ax.plot(times4, offsets4, 'b-', linewidth=1.5, label='Estimated Offset')
        ax.axvline(x=30, color='red', linestyle='--', label='Desync Event')
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Offset (ms)')
        ax.set_title('Test 4: Controlled Desync Detection')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    plt.tight_layout()
    
    # Save plot
    output_path = _project_root / "prbs_validation_results.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Validation plots saved to: {output_path}")
    
    plt.show()
    
    print("\n" + "="*70)
    print("✓ ALL TESTS COMPLETE")
    print("="*70)


if __name__ == "__main__":
    run_all_tests()
