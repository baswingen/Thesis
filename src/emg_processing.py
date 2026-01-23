"""
EMG Signal Processing Module
============================
Signal processing utilities for EMG data analysis.

Features:
- Real-time bandpass filtering
- EMG envelope extraction
- Signal quality metrics
- Artifact detection
"""

import numpy as np
from typing import Optional, Tuple, Union
import warnings

# Optional scipy for filtering
try:
    from scipy import signal
    SCIPY_AVAILABLE = True
except Exception as e:
    SCIPY_AVAILABLE = False
    warnings.warn(f"scipy not available - filtering disabled: {e}")


class BandpassFilter:
    """
    Real-time bandpass filter using scipy's Butterworth filter.
    Maintains state across calls to avoid edge artifacts.
    
    Typical EMG frequency range: 20-450 Hz
    """
    
    def __init__(self, lowcut: float, highcut: float, fs: float, order: int = 4):
        """
        Initialize bandpass filter.
        
        Args:
            lowcut: Low cutoff frequency (Hz) - removes DC offset and motion artifacts
            highcut: High cutoff frequency (Hz) - removes high-frequency noise
            fs: Sample rate (Hz)
            order: Filter order (higher = sharper cutoff, but more phase distortion)
        
        Raises:
            RuntimeError: If scipy not available
        """
        if not SCIPY_AVAILABLE:
            raise RuntimeError("scipy is required for filtering. Install with: pip install scipy>=1.11.0")
        
        self.lowcut = lowcut
        self.highcut = highcut
        self.fs = fs
        self.order = order
        self.sos = None
        self.zi = None
        
        # Create second-order sections representation
        nyquist = 0.5 * fs
        low = lowcut / nyquist
        high = highcut / nyquist
        
        # Ensure cutoffs are valid
        if low <= 0:
            low = 0.01
        if high >= 1:
            high = 0.99
        
        if low >= high:
            raise ValueError(f"Invalid cutoff frequencies: low={lowcut}, high={highcut}, fs={fs}")
        
        self.sos = signal.butter(order, [low, high], btype='band', output='sos')
        # Initialize filter state
        self.zi = signal.sosfilt_zi(self.sos)
    
    def filter(self, data: np.ndarray) -> np.ndarray:
        """
        Apply bandpass filter to data.
        
        Args:
            data: 1D numpy array of samples
            
        Returns:
            Filtered data
        """
        if not SCIPY_AVAILABLE or self.sos is None:
            return data
        
        if len(data) == 0:
            return data
        
        # Handle NaN values
        if np.any(np.isnan(data)):
            valid_mask = ~np.isnan(data)
            if not np.any(valid_mask):
                return data
            
            # Filter only valid data
            filtered_valid, self.zi = signal.sosfilt(self.sos, data[valid_mask], zi=self.zi)
            
            # Reconstruct with NaNs
            filtered = np.full_like(data, np.nan)
            filtered[valid_mask] = filtered_valid
            return filtered
        
        # Apply filter with state
        filtered, self.zi = signal.sosfilt(self.sos, data, zi=self.zi)
        return filtered
    
    def reset(self):
        """Reset filter state."""
        if SCIPY_AVAILABLE and self.sos is not None:
            self.zi = signal.sosfilt_zi(self.sos)
    
    def __repr__(self):
        return f"BandpassFilter({self.lowcut}-{self.highcut}Hz, fs={self.fs}Hz, order={self.order})"


class NotchFilter:
    """
    Real-time notch filter for removing powerline interference (50/60 Hz).
    """
    
    def __init__(self, freq: float, fs: float, quality: float = 30.0):
        """
        Initialize notch filter.
        
        Args:
            freq: Frequency to remove (Hz) - typically 50 or 60
            fs: Sample rate (Hz)
            quality: Quality factor (higher = narrower notch)
        
        Raises:
            RuntimeError: If scipy not available
        """
        if not SCIPY_AVAILABLE:
            raise RuntimeError("scipy is required for filtering. Install with: pip install scipy>=1.11.0")
        
        self.freq = freq
        self.fs = fs
        self.quality = quality
        self.sos = None
        self.zi = None
        
        # Create notch filter
        self.sos = signal.iirnotch(freq, quality, fs)
        # Convert to second-order sections for stability
        self.sos = signal.tf2sos(*signal.iirnotch(freq, quality, fs))
        self.zi = signal.sosfilt_zi(self.sos)
    
    def filter(self, data: np.ndarray) -> np.ndarray:
        """
        Apply notch filter to data.
        
        Args:
            data: 1D numpy array of samples
            
        Returns:
            Filtered data
        """
        if not SCIPY_AVAILABLE or self.sos is None:
            return data
        
        if len(data) == 0:
            return data
        
        # Apply filter with state
        filtered, self.zi = signal.sosfilt(self.sos, data, zi=self.zi)
        return filtered
    
    def reset(self):
        """Reset filter state."""
        if SCIPY_AVAILABLE and self.sos is not None:
            self.zi = signal.sosfilt_zi(self.sos)
    
    def __repr__(self):
        return f"NotchFilter({self.freq}Hz, fs={self.fs}Hz, Q={self.quality})"


class EMGEnvelopeExtractor:
    """
    Extract EMG envelope using rectification and low-pass filtering.
    """
    
    def __init__(self, fs: float, cutoff: float = 10.0, order: int = 4):
        """
        Initialize envelope extractor.
        
        Args:
            fs: Sample rate (Hz)
            cutoff: Low-pass cutoff frequency (Hz) - typically 5-10 Hz for EMG envelope
            order: Filter order
        
        Raises:
            RuntimeError: If scipy not available
        """
        if not SCIPY_AVAILABLE:
            raise RuntimeError("scipy is required for filtering. Install with: pip install scipy>=1.11.0")
        
        self.fs = fs
        self.cutoff = cutoff
        self.order = order
        
        # Create low-pass filter for envelope
        nyquist = 0.5 * fs
        norm_cutoff = cutoff / nyquist
        
        if norm_cutoff >= 1:
            norm_cutoff = 0.99
        
        self.sos = signal.butter(order, norm_cutoff, btype='low', output='sos')
        self.zi = signal.sosfilt_zi(self.sos)
    
    def extract(self, data: np.ndarray) -> np.ndarray:
        """
        Extract EMG envelope.
        
        Args:
            data: 1D numpy array of EMG samples
            
        Returns:
            EMG envelope
        """
        if not SCIPY_AVAILABLE or self.sos is None:
            return np.abs(data)
        
        if len(data) == 0:
            return data
        
        # Rectify (absolute value)
        rectified = np.abs(data)
        
        # Low-pass filter
        envelope, self.zi = signal.sosfilt(self.sos, rectified, zi=self.zi)
        return envelope
    
    def reset(self):
        """Reset filter state."""
        if SCIPY_AVAILABLE and self.sos is not None:
            self.zi = signal.sosfilt_zi(self.sos)
    
    def __repr__(self):
        return f"EMGEnvelopeExtractor(cutoff={self.cutoff}Hz, fs={self.fs}Hz, order={self.order})"


class EMGProcessor:
    """
    Complete EMG processing pipeline.
    
    Combines bandpass filtering, notch filtering, and envelope extraction.
    """
    
    def __init__(self, 
                 fs: float,
                 bandpass_low: float = 20.0,
                 bandpass_high: float = 450.0,
                 bandpass_order: int = 4,
                 notch_freq: Optional[float] = None,
                 envelope_cutoff: Optional[float] = None,
                 envelope_order: int = 4):
        """
        Initialize EMG processor.
        
        Args:
            fs: Sample rate (Hz)
            bandpass_low: Bandpass low cutoff (Hz)
            bandpass_high: Bandpass high cutoff (Hz)
            bandpass_order: Bandpass filter order
            notch_freq: Notch frequency (Hz) - None to disable, typically 50 or 60
            envelope_cutoff: Envelope cutoff (Hz) - None to disable envelope extraction
            envelope_order: Envelope filter order
        
        Raises:
            RuntimeError: If scipy not available
        """
        if not SCIPY_AVAILABLE:
            raise RuntimeError("scipy is required for EMG processing")
        
        self.fs = fs
        
        # Create filters
        self.bandpass = BandpassFilter(bandpass_low, bandpass_high, fs, bandpass_order)
        
        self.notch = None
        if notch_freq is not None:
            self.notch = NotchFilter(notch_freq, fs)
        
        self.envelope_extractor = None
        if envelope_cutoff is not None:
            self.envelope_extractor = EMGEnvelopeExtractor(fs, envelope_cutoff, envelope_order)
    
    def process(self, data: np.ndarray, return_envelope: bool = False) -> Union[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
        """
        Process EMG data through pipeline.
        
        Args:
            data: 1D numpy array of raw EMG samples
            return_envelope: If True, return (filtered, envelope), else just filtered
            
        Returns:
            Filtered EMG data, or (filtered, envelope) if return_envelope=True
        """
        # Bandpass filter
        filtered = self.bandpass.filter(data)
        
        # Notch filter (if configured)
        if self.notch is not None:
            filtered = self.notch.filter(filtered)
        
        # Extract envelope (if configured)
        if return_envelope and self.envelope_extractor is not None:
            envelope = self.envelope_extractor.extract(filtered)
            return filtered, envelope
        
        return filtered
    
    def reset(self):
        """Reset all filter states."""
        self.bandpass.reset()
        if self.notch is not None:
            self.notch.reset()
        if self.envelope_extractor is not None:
            self.envelope_extractor.reset()
    
    def __repr__(self):
        parts = [str(self.bandpass)]
        if self.notch:
            parts.append(str(self.notch))
        if self.envelope_extractor:
            parts.append(str(self.envelope_extractor))
        return f"EMGProcessor({', '.join(parts)})"


# Signal quality metrics
def calculate_signal_quality(data: np.ndarray, fs: float) -> dict:
    """
    Calculate EMG signal quality metrics.
    
    Args:
        data: 1D numpy array of EMG samples
        fs: Sample rate (Hz)
        
    Returns:
        Dictionary with quality metrics
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        
        metrics = {
            'mean': float(np.nanmean(data)),
            'std': float(np.nanstd(data)),
            'rms': float(np.sqrt(np.nanmean(data**2))),
            'min': float(np.nanmin(data)),
            'max': float(np.nanmax(data)),
            'range': float(np.nanmax(data) - np.nanmin(data)),
            'num_samples': len(data),
            'num_valid': int(np.sum(~np.isnan(data))),
            'percent_valid': float(np.sum(~np.isnan(data)) / len(data) * 100) if len(data) > 0 else 0.0
        }
        
        # Calculate SNR estimate (simple method)
        if SCIPY_AVAILABLE and metrics['num_valid'] > 100:
            try:
                # High frequency content (noise estimate)
                sos_high = signal.butter(4, 0.8, btype='high', output='sos')
                noise = signal.sosfilt(sos_high, data[~np.isnan(data)])
                noise_power = np.mean(noise**2)
                
                # Signal power
                signal_power = np.mean(data[~np.isnan(data)]**2)
                
                if noise_power > 0:
                    snr = 10 * np.log10(signal_power / noise_power)
                    metrics['snr_db'] = float(snr)
            except:
                pass
    
    return metrics


# Type hint import
from typing import Union
