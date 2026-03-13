import warnings
import numpy as np
from scipy import signal, stats
import pandas as pd
from typing import Dict, List, Optional, Set, Tuple
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from model.config_model import FEATURE_CONFIG

class FeatureExtractor:
    def __init__(self, emg_fs: float = 2000,
                 imu_fs: float = 2000,
                 emg_threshold: float = FEATURE_CONFIG.get('emg_threshold', 1e-5),
                 emg_features: Optional[Dict[str, bool]] = None,
                 imu_features: Optional[Dict[str, bool]] = None):
        """
        Args:
            emg_fs: EMG sampling frequency in Hz.
            imu_fs: IMU sampling frequency in Hz.
            emg_threshold: Threshold for WAMP and Myopulse features.
            emg_features: Dict mapping feature name → enabled (True/False).
                          If None, all features are enabled.
            imu_features: Dict mapping feature name → enabled (True/False).
                          If None, all features are enabled.
        """
        self.emg_fs = emg_fs
        self.imu_fs = imu_fs
        self.emg_threshold = emg_threshold
        
        # Build sets of enabled feature keys for fast lookup
        self._emg_enabled: Optional[Set[str]] = None
        if emg_features is not None:
            self._emg_enabled = {k for k, v in emg_features.items() if v}
        
        self._imu_enabled: Optional[Set[str]] = None
        if imu_features is not None:
            self._imu_enabled = {k for k, v in imu_features.items() if v}

    def _emg_on(self, key: str) -> bool:
        """Check if an EMG feature is enabled."""
        return self._emg_enabled is None or key in self._emg_enabled
    
    def _imu_on(self, key: str) -> bool:
        """Check if an IMU feature is enabled."""
        return self._imu_enabled is None or key in self._imu_enabled

    def filter_active_emg(self, emg_data: np.ndarray, emg_cols: List[str]) -> Tuple[np.ndarray, List[str]]:
        """
        Removes EMG channels that consist entirely of zeros.
        """
        if emg_data.size == 0:
            return emg_data, emg_cols
            
        # A channel is considered inactive if all its values are exactly 0
        active_mask = ~np.all(emg_data == 0, axis=0)
        
        active_emg = emg_data[:, active_mask]
        active_cols = [col for col, is_active in zip(emg_cols, active_mask) if is_active]
        
        return active_emg, active_cols

    # ── helper: safe spectral entropy ──────────────────────────────────
    @staticmethod
    def _spectral_entropy(psd: np.ndarray, axis: int = 0) -> np.ndarray:
        """
        Compute spectral entropy along *axis*.
        psd shape: (n_freq, n_channels)  →  returns (n_channels,)
        """
        total = np.sum(psd, axis=axis, keepdims=True)
        total = np.where(total == 0, 1e-10, total)
        p_norm = psd / total
        p_norm = np.where(p_norm == 0, 1e-10, p_norm)
        return -np.sum(p_norm * np.log2(p_norm), axis=axis)

    # ══════════════════════════════════════════════════════════════════
    #  EMG FEATURES
    # ══════════════════════════════════════════════════════════════════
    def extract_emg_features(self, emg_data: np.ndarray, emg_cols: List[str]) -> Dict[str, float]:
        """
        Extracts comprehensive Time and Frequency domain features from EMG data.
        Returns a dictionary with feature names mapping to their computed values.
        
        Time-domain (14 features per channel):
            MAV, RMS, WL, ZC, SSC, VAR, WAMP, IEMG, LogDetector,
            Skewness, Kurtosis, Hjorth_Mobility, Hjorth_Complexity, Myopulse
        
        Frequency-domain (6 features per channel):
            MNF, MDF, Power, SpectralEntropy, PeakFreq, Bandwidth
        """
        features = {}
        if emg_data.size == 0:
            return features
            
        active_emg, active_cols = self.filter_active_emg(emg_data, emg_cols)
        
        if active_emg.size == 0:
            return features

        N = active_emg.shape[0]

        # ── Time Domain ─────────────────────────────────────────────
        # Existing features
        mav = np.mean(np.abs(active_emg), axis=0)
        rms = np.sqrt(np.mean(active_emg**2, axis=0))
        wl = np.sum(np.abs(np.diff(active_emg, axis=0)), axis=0)
        
        # Zero Crossings (ZC)
        zc = np.sum((active_emg[:-1, :] * active_emg[1:, :]) < 0, axis=0)
        
        # Slope Sign Changes (SSC)
        dx = np.diff(active_emg, axis=0)
        ssc = np.sum((dx[:-1, :] * dx[1:, :]) < 0, axis=0)
        
        variance = np.var(active_emg, axis=0)

        # ── NEW Time Domain ─────────────────────────────────────────
        # Willison Amplitude (WAMP): count of consecutive diffs exceeding threshold
        diffs = np.abs(np.diff(active_emg, axis=0))
        wamp = np.sum(diffs > self.emg_threshold, axis=0)

        # Integrated EMG (IEMG): sum of absolute values
        iemg = np.sum(np.abs(active_emg), axis=0)

        # Log Detector: geometric mean of absolute signal
        abs_emg = np.abs(active_emg)
        abs_emg_safe = np.where(abs_emg == 0, 1e-10, abs_emg)
        log_detector = np.exp(np.mean(np.log(abs_emg_safe), axis=0))

        # Skewness & Kurtosis (per channel) — replace NaN with 0.0
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            skew = np.nan_to_num(stats.skew(active_emg, axis=0, nan_policy='omit'), nan=0.0)
            kurt = np.nan_to_num(stats.kurtosis(active_emg, axis=0, nan_policy='omit'), nan=0.0)

        # Hjorth Parameters
        # Activity = variance (already computed)
        # Mobility = sqrt(var(dx) / var(x))
        var_dx = np.var(dx, axis=0)
        safe_var = np.where(variance == 0, 1e-10, variance)
        hjorth_mobility = np.sqrt(var_dx / safe_var)

        # Complexity = mobility(dx) / mobility(x)
        ddx = np.diff(dx, axis=0)
        var_ddx = np.var(ddx, axis=0)
        safe_var_dx = np.where(var_dx == 0, 1e-10, var_dx)
        mobility_dx = np.sqrt(var_ddx / safe_var_dx)
        safe_mobility = np.where(hjorth_mobility == 0, 1e-10, hjorth_mobility)
        hjorth_complexity = mobility_dx / safe_mobility

        # Myopulse Percentage Rate: fraction of samples exceeding threshold
        myopulse = np.sum(np.abs(active_emg) > self.emg_threshold, axis=0) / N

        # ── Frequency Domain ────────────────────────────────────────
        nperseg = min(256, active_emg.shape[0])
        freqs, psd = signal.welch(active_emg, fs=self.emg_fs, axis=0, nperseg=nperseg)
        
        total_power = np.sum(psd, axis=0)
        
        # Mean Frequency (MNF)
        safe_total_power = np.where(total_power == 0, 1e-10, total_power)
        mnf = np.sum(freqs[:, None] * psd, axis=0) / safe_total_power
        
        # Median Frequency (MDF)
        cum_power = np.cumsum(psd, axis=0)
        half_power = total_power / 2.0
        mdf_indices = np.nanargmax(cum_power >= half_power, axis=0)
        mdf = freqs[mdf_indices]

        # ── NEW Frequency Domain ────────────────────────────────────
        # Spectral Entropy
        spec_entropy = self._spectral_entropy(psd, axis=0)

        # Peak Frequency: frequency at maximum PSD
        peak_freq_indices = np.argmax(psd, axis=0)
        peak_freq = freqs[peak_freq_indices]

        # Bandwidth: frequency range containing 95% of total power
        cum_frac = cum_power / safe_total_power
        low_idx = np.nanargmax(cum_frac >= 0.025, axis=0)   # 2.5% lower
        high_idx = np.nanargmax(cum_frac >= 0.975, axis=0)   # 97.5% upper
        bandwidth = freqs[high_idx] - freqs[low_idx]

        # ── Populate features dict (only enabled features) ──────────
        for i, col in enumerate(active_cols):
            # Time-domain
            if self._emg_on('MAV'):      features[f"{col}_EMG_MAV"] = mav[i]
            if self._emg_on('RMS'):      features[f"{col}_EMG_RMS"] = rms[i]
            if self._emg_on('WL'):       features[f"{col}_EMG_WL"] = wl[i]
            if self._emg_on('ZC'):       features[f"{col}_EMG_ZC"] = zc[i]
            if self._emg_on('SSC'):      features[f"{col}_EMG_SSC"] = ssc[i]
            if self._emg_on('VAR'):      features[f"{col}_EMG_VAR"] = variance[i]
            if self._emg_on('WAMP'):     features[f"{col}_EMG_WAMP"] = wamp[i]
            if self._emg_on('IEMG'):     features[f"{col}_EMG_IEMG"] = iemg[i]
            if self._emg_on('LogDet'):   features[f"{col}_EMG_LogDet"] = log_detector[i]
            if self._emg_on('Skew'):     features[f"{col}_EMG_Skew"] = skew[i]
            if self._emg_on('Kurt'):     features[f"{col}_EMG_Kurt"] = kurt[i]
            if self._emg_on('HjMob'):    features[f"{col}_EMG_HjMob"] = hjorth_mobility[i]
            if self._emg_on('HjComp'):   features[f"{col}_EMG_HjComp"] = hjorth_complexity[i]
            if self._emg_on('Myopulse'): features[f"{col}_EMG_Myopulse"] = myopulse[i]
            # Frequency-domain
            if self._emg_on('MNF'):         features[f"{col}_EMG_MNF"] = mnf[i]
            if self._emg_on('MDF'):         features[f"{col}_EMG_MDF"] = mdf[i]
            if self._emg_on('Power'):       features[f"{col}_EMG_Power"] = total_power[i]
            if self._emg_on('SpecEntropy'): features[f"{col}_EMG_SpecEntropy"] = spec_entropy[i]
            if self._emg_on('PeakFreq'):    features[f"{col}_EMG_PeakFreq"] = peak_freq[i]
            if self._emg_on('BW'):          features[f"{col}_EMG_BW"] = bandwidth[i]

        return features

    # ══════════════════════════════════════════════════════════════════
    #  IMU FEATURES
    # ══════════════════════════════════════════════════════════════════
    def extract_imu_features(self, imu_data: np.ndarray, imu_cols: List[str]) -> Dict[str, float]:
        """
        Extracts comprehensive Time and Frequency domain features from IMU data.
        Returns a dictionary with feature names mapping to their computed values.

        Time-domain (14 features per channel):
            Mean, Var, Std, Max, Min, RMS, SMA, PeakToPeak, IQR,
            Skewness, Kurtosis, Jerk, ZC, Energy
        
        Frequency-domain (5 features per channel):
            DomFreq, SpecEnergy, MNF, MDF, SpectralEntropy
        
        Cross-channel (if ≥3 axes detected):
            SVM_mean, SVM_std per axis group (Acc / Gyr)
        """
        features = {}
        if imu_data.size == 0:
            return features

        N = imu_data.shape[0]

        # ── Time Domain ─────────────────────────────────────────────
        # Existing features
        mean_val = np.mean(imu_data, axis=0)
        var_val = np.var(imu_data, axis=0)
        std_val = np.std(imu_data, axis=0)
        max_val = np.max(imu_data, axis=0)
        min_val = np.min(imu_data, axis=0)
        rms_val = np.sqrt(np.mean(imu_data**2, axis=0))
        sma_val = np.sum(np.abs(imu_data), axis=0) / self.imu_fs

        # ── NEW Time Domain ─────────────────────────────────────────
        # Peak-to-Peak
        p2p = max_val - min_val

        # Interquartile Range
        q75 = np.percentile(imu_data, 75, axis=0)
        q25 = np.percentile(imu_data, 25, axis=0)
        iqr = q75 - q25

        # Skewness & Kurtosis — replace NaN with 0.0
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            skew = np.nan_to_num(stats.skew(imu_data, axis=0, nan_policy='omit'), nan=0.0)
            kurt = np.nan_to_num(stats.kurtosis(imu_data, axis=0, nan_policy='omit'), nan=0.0)

        # Mean Absolute Jerk (rate of change of signal)
        jerk = np.mean(np.abs(np.diff(imu_data, axis=0)), axis=0) * self.imu_fs

        # Zero Crossings
        centered = imu_data - mean_val
        zc = np.sum((centered[:-1, :] * centered[1:, :]) < 0, axis=0)

        # Energy (sum of squared values / N)
        energy = np.sum(imu_data**2, axis=0) / N

        # ── Frequency Domain ────────────────────────────────────────
        nperseg = min(256, imu_data.shape[0])
        freqs, psd = signal.welch(imu_data, fs=self.imu_fs, axis=0, nperseg=nperseg)
        
        spectral_energy = np.sum(psd, axis=0)
        dominant_freq_indices = np.argmax(psd, axis=0)
        dominant_freq = freqs[dominant_freq_indices]

        # ── NEW Frequency Domain ────────────────────────────────────
        # Mean Frequency (MNF)
        safe_spec_energy = np.where(spectral_energy == 0, 1e-10, spectral_energy)
        mnf = np.sum(freqs[:, None] * psd, axis=0) / safe_spec_energy

        # Median Frequency (MDF)
        cum_power = np.cumsum(psd, axis=0)
        half_power = spectral_energy / 2.0
        mdf_indices = np.nanargmax(cum_power >= half_power, axis=0)
        mdf = freqs[mdf_indices]

        # Spectral Entropy
        spec_entropy = self._spectral_entropy(psd, axis=0)

        # ── Populate per-channel features (only enabled) ─────────────
        for i, col in enumerate(imu_cols):
            # Time-domain
            if self._imu_on('Mean'):   features[f"{col}_IMU_Mean"] = mean_val[i]
            if self._imu_on('Var'):    features[f"{col}_IMU_Var"] = var_val[i]
            if self._imu_on('Std'):    features[f"{col}_IMU_Std"] = std_val[i]
            if self._imu_on('Max'):    features[f"{col}_IMU_Max"] = max_val[i]
            if self._imu_on('Min'):    features[f"{col}_IMU_Min"] = min_val[i]
            if self._imu_on('RMS'):    features[f"{col}_IMU_RMS"] = rms_val[i]
            if self._imu_on('SMA'):    features[f"{col}_IMU_SMA"] = sma_val[i]
            if self._imu_on('P2P'):    features[f"{col}_IMU_P2P"] = p2p[i]
            if self._imu_on('IQR'):    features[f"{col}_IMU_IQR"] = iqr[i]
            if self._imu_on('Skew'):   features[f"{col}_IMU_Skew"] = skew[i]
            if self._imu_on('Kurt'):   features[f"{col}_IMU_Kurt"] = kurt[i]
            if self._imu_on('Jerk'):   features[f"{col}_IMU_Jerk"] = jerk[i]
            if self._imu_on('ZC'):     features[f"{col}_IMU_ZC"] = zc[i]
            if self._imu_on('Energy'): features[f"{col}_IMU_Energy"] = energy[i]
            # Frequency-domain
            if self._imu_on('DomFreq'):    features[f"{col}_IMU_DomFreq"] = dominant_freq[i]
            if self._imu_on('SpecEnergy'): features[f"{col}_IMU_SpecEnergy"] = spectral_energy[i]
            if self._imu_on('MNF'):        features[f"{col}_IMU_MNF"] = mnf[i]
            if self._imu_on('MDF'):        features[f"{col}_IMU_MDF"] = mdf[i]
            if self._imu_on('SpecEntropy'):features[f"{col}_IMU_SpecEntropy"] = spec_entropy[i]

        # ── Cross-channel: Signal Vector Magnitude (SVM) ────────────
        # Group axes by sensor type (e.g. Acc_X, Acc_Y, Acc_Z → one SVM)
        axis_groups: Dict[str, List[int]] = {}
        for i, col in enumerate(imu_cols):
            # Try to extract group prefix (everything before the last axis letter)
            # e.g. "Acc_X" → "Acc", "Gyro_Z" → "Gyro", "AccX" → "Acc"
            prefix = col
            for suffix in ["_X", "_Y", "_Z", "_x", "_y", "_z", "X", "Y", "Z"]:
                if col.endswith(suffix):
                    prefix = col[: -len(suffix)]
                    break
            if prefix != col:  # only group if we can detect an axis suffix
                axis_groups.setdefault(prefix, []).append(i)

        for group_name, indices in axis_groups.items():
            if len(indices) >= 3:
                # Take first 3 axes (X, Y, Z)
                group_data = imu_data[:, indices[:3]]
                svm = np.sqrt(np.sum(group_data**2, axis=1))  # (N,)
                if self._imu_on('SVM_Mean'):
                    features[f"{group_name}_IMU_SVM_Mean"] = np.mean(svm)
                if self._imu_on('SVM_Std'):
                    features[f"{group_name}_IMU_SVM_Std"] = np.std(svm)

        return features

    def extract_features(self, emg_data: np.ndarray, emg_cols: List[str], 
                               imu_data: np.ndarray, imu_cols: List[str]) -> Dict[str, float]:
        """
        Extracts all features from a single segment containing both EMG and IMU data.
        """
        features = {}
        
        features.update(self.extract_emg_features(emg_data, emg_cols))
        features.update(self.extract_imu_features(imu_data, imu_cols))
        
        return features

    def extract_features_windowed(self, emg_data: np.ndarray, emg_cols: List[str], 
                                  imu_data: np.ndarray, imu_cols: List[str], 
                                  emg_window_size_sec: float, imu_window_size_sec: float,
                                  window_step_sec: float) -> List[Dict[str, float]]:
        """
        Splits the data into sliding windows and extracts features for each window.
        Anchors windows by their end time so that EMG and IMU features end exactly at the same moment.
        Returns a list of feature dictionaries, one for each window.
        """
        emg_window_size = int(emg_window_size_sec * self.emg_fs)
        imu_window_size = int(imu_window_size_sec * self.imu_fs)
        
        # Calculate the length of the segment in seconds
        emg_time_sec = emg_data.shape[0] / self.emg_fs if emg_data.size > 0 else 0
        imu_time_sec = imu_data.shape[0] / self.imu_fs if imu_data.size > 0 else 0
        
        # Use the maximum available time to dictate the segment bounds
        max_time_sec = max(emg_time_sec, imu_time_sec)
        
        # The first window finishes at the longest required window size
        t_end = max(emg_window_size_sec, imu_window_size_sec)
        
        windows_features = []
        
        while t_end <= max_time_sec + 1e-4:
            emg_end_idx = int(round(t_end * self.emg_fs))
            emg_start_idx = max(0, emg_end_idx - emg_window_size)
            
            imu_end_idx = int(round(t_end * self.imu_fs))
            imu_start_idx = max(0, imu_end_idx - imu_window_size)
            
            emg_win = emg_data[emg_start_idx:emg_end_idx] if emg_data.size > 0 else np.empty((0, 0))
            imu_win = imu_data[imu_start_idx:imu_end_idx] if imu_data.size > 0 else np.empty((0, 0))
            
            # Extract features only if we have at least half of the required samples for BOTH modalities
            if (emg_win.shape[0] >= emg_window_size // 2) and (imu_win.shape[0] >= imu_window_size // 2):
                features = self.extract_features(emg_win, emg_cols, imu_win, imu_cols)
                windows_features.append(features)
                
            t_end += window_step_sec
            
        return windows_features
