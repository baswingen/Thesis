"""
EMG Preprocessing Module Usage Examples
========================================

Demonstrates how to use the native EMG preprocessing module (src.emg_processing)
for complete signal preprocessing including MVC normalization.

Author: Generated for Master Thesis
Date: January 2026
"""

import numpy as np
import sys
from pathlib import Path

# Add project root to path
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.emg_processing import (
    EMGPreprocessor,
    calibrate_mvc_from_data,
    BandpassFilter,
    NotchFilter,
    EMGEnvelopeExtractor
)


def example1_basic_preprocessing():
    """Example 1: Basic preprocessing without MVC normalization."""
    print("\n" + "="*70)
    print("EXAMPLE 1: Basic Preprocessing (No MVC)")
    print("="*70)
    
    # Create preprocessor
    preprocessor = EMGPreprocessor(
        fs=1024,  # 1024 Hz sample rate
        mvc_value=None,  # No MVC normalization
        bandpass_low=20.0,
        bandpass_high=450.0,
        notch_freq=50.0,
        envelope_cutoff=10.0
    )
    
    print(f"Preprocessor: {preprocessor}")
    
    # Generate fake EMG data (normally from acquisition)
    duration = 1.0  # 1 second
    fs = 1024
    t = np.linspace(0, duration, int(fs * duration))
    raw_emg = np.sin(2 * np.pi * 50 * t) + 0.5 * np.random.randn(len(t))
    
    # Process the signal
    normalized = preprocessor.process(raw_emg)  # Returns only normalized signal
    
    print(f"Input shape: {raw_emg.shape}")
    print(f"Output shape: {normalized.shape}")
    print(f"Output range: {np.min(normalized):.4f} to {np.max(normalized):.4f}")


def example2_preprocessing_with_mvc():
    """Example 2: Preprocessing with MVC normalization."""
    print("\n" + "="*70)
    print("EXAMPLE 2: Preprocessing with MVC Normalization")
    print("="*70)
    
    # Step 1: Collect MVC calibration data (simulated)
    fs = 1024
    mvc_duration = 5.0
    t_mvc = np.linspace(0, mvc_duration, int(fs * mvc_duration))
    mvc_recording = 10 * np.sin(2 * np.pi * 50 * t_mvc) + 2 * np.random.randn(len(t_mvc))
    
    # Step 2: Calculate MVC value
    mvc_value = calibrate_mvc_from_data(
        mvc_recording,
        fs=fs,
        percentile=95.0,
        bandpass_low=20.0,
        bandpass_high=450.0,
        notch_freq=50.0
    )
    
    print(f"MVC value: {mvc_value:.4f}")
    
    # Step 3: Create preprocessor with MVC
    preprocessor = EMGPreprocessor(
        fs=fs,
        mvc_value=mvc_value,
        bandpass_low=20.0,
        bandpass_high=450.0,
        notch_freq=50.0,
        envelope_cutoff=10.0
    )
    
    # Step 4: Process normal EMG data
    t = np.linspace(0, 1.0, fs)
    raw_emg = 5 * np.sin(2 * np.pi * 50 * t) + np.random.randn(len(t))
    
    normalized = preprocessor.process(raw_emg)
    
    print(f"Normalized range: {np.min(normalized):.4f} to {np.max(normalized):.4f}")
    print(f"Expected range: 0.0 to ~1.0 (relative to MVC)")


def example3_all_processing_stages():
    """Example 3: Access all processing stages."""
    print("\n" + "="*70)
    print("EXAMPLE 3: Accessing All Processing Stages")
    print("="*70)
    
    preprocessor = EMGPreprocessor(
        fs=1024,
        mvc_value=100.0,
        bandpass_low=20.0,
        bandpass_high=450.0,
        notch_freq=50.0,
        envelope_cutoff=10.0
    )
    
    # Generate fake data
    fs = 1024
    t = np.linspace(0, 1.0, fs)
    raw_emg = 10 * np.sin(2 * np.pi * 50 * t) + 2 * np.random.randn(len(t))
    
    # Get all stages
    result = preprocessor.process(raw_emg, return_all_stages=True)
    
    print(f"Available stages: {list(result.keys())}")
    print(f"Raw EMG RMS: {np.sqrt(np.mean(result['raw']**2)):.4f}")
    print(f"Filtered RMS: {np.sqrt(np.mean(result['filtered']**2)):.4f}")
    print(f"Envelope mean: {np.mean(result['envelope']):.4f}")
    print(f"Normalized mean: {np.mean(result['normalized']):.4f}")


def example4_real_time_processing():
    """Example 4: Real-time streaming simulation."""
    print("\n" + "="*70)
    print("EXAMPLE 4: Real-Time Streaming Simulation")
    print("="*70)
    
    preprocessor = EMGPreprocessor(
        fs=1024,
        mvc_value=50.0,
        bandpass_low=20.0,
        bandpass_high=450.0,
        notch_freq=50.0,
        envelope_cutoff=10.0
    )
    
    # Simulate real-time chunks
    fs = 1024
    chunk_size = 100  # 100 samples per chunk (~0.1 seconds at 1024 Hz)
    num_chunks = 10
    
    print(f"Processing {num_chunks} chunks of {chunk_size} samples each...")
    
    for i in range(num_chunks):
        # Simulate incoming chunk
        t = np.linspace(i * 0.1, (i + 1) * 0.1, chunk_size)
        chunk = 5 * np.sin(2 * np.pi * 50 * t) + 0.5 * np.random.randn(chunk_size)
        
        # Process chunk (filter states are maintained between calls)
        normalized = preprocessor.process(chunk)
        
        if i % 3 == 0:
            print(f"  Chunk {i+1}: mean={np.mean(normalized):.4f}, "
                  f"std={np.std(normalized):.4f}")
    
    print("Real-time processing complete!")


def example5_update_mvc_dynamically():
    """Example 5: Update MVC value dynamically."""
    print("\n" + "="*70)
    print("EXAMPLE 5: Dynamic MVC Update")
    print("="*70)
    
    preprocessor = EMGPreprocessor(
        fs=1024,
        mvc_value=50.0  # Initial MVC
    )
    
    print(f"Initial MVC: {preprocessor.mvc_value}")
    
    # Process some data
    raw_emg = np.random.randn(1024) * 10
    result1 = preprocessor.process(raw_emg)
    print(f"Normalized mean (MVC=50): {np.mean(result1):.4f}")
    
    # Update MVC value
    preprocessor.set_mvc_value(100.0)
    print(f"Updated MVC: {preprocessor.mvc_value}")
    
    # Reset filters and process again
    preprocessor.reset()
    result2 = preprocessor.process(raw_emg)
    print(f"Normalized mean (MVC=100): {np.mean(result2):.4f}")


def example6_individual_filters():
    """Example 6: Using individual filter components."""
    print("\n" + "="*70)
    print("EXAMPLE 6: Individual Filter Components")
    print("="*70)
    
    fs = 1024
    
    # Create individual filters
    bandpass = BandpassFilter(lowcut=20, highcut=450, fs=fs, order=4)
    notch = NotchFilter(freq=50, fs=fs, quality=30)
    envelope = EMGEnvelopeExtractor(fs=fs, cutoff=10, order=4)
    
    # Generate fake data
    t = np.linspace(0, 1.0, fs)
    raw_emg = 10 * np.sin(2 * np.pi * 50 * t) + 2 * np.random.randn(len(t))
    
    # Apply filters step by step
    filtered = bandpass.filter(raw_emg)
    print(f"After bandpass: RMS={np.sqrt(np.mean(filtered**2)):.4f}")
    
    filtered = notch.filter(filtered)
    print(f"After notch: RMS={np.sqrt(np.mean(filtered**2)):.4f}")
    
    env = envelope.extract(filtered)
    print(f"After envelope: mean={np.mean(env):.4f}")
    
    # Normalize manually
    mvc_value = 50.0
    normalized = env / mvc_value
    print(f"After normalization: mean={np.mean(normalized):.4f}")


def main():
    """Run all examples."""
    print("\n" + "="*70)
    print("EMG PREPROCESSING MODULE EXAMPLES")
    print("="*70)
    print("\nDemonstrates the native EMG preprocessing module")
    print("Location: src/emg_processing.py")
    
    example1_basic_preprocessing()
    example2_preprocessing_with_mvc()
    example3_all_processing_stages()
    example4_real_time_processing()
    example5_update_mvc_dynamically()
    example6_individual_filters()
    
    print("\n" + "="*70)
    print("ALL EXAMPLES COMPLETE")
    print("="*70)


if __name__ == '__main__':
    main()
