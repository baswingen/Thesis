import numpy as np
from scipy import signal
import pandas as pd
from typing import Dict, List, Tuple

class FeatureExtractor:
    def __init__(self, emg_fs: float = 4000.0, imu_fs: float = 4000.0):
        self.emg_fs = emg_fs
        self.imu_fs = imu_fs

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

    def extract_emg_features(self, emg_data: np.ndarray, emg_cols: List[str]) -> Dict[str, float]:
        """
        Extracts Time and Frequency domain features from EMG data.
        Returns a dictionary with feature names mapping to their computed values.
        """
        features = {}
        if emg_data.size == 0:
            return features
            
        active_emg, active_cols = self.filter_active_emg(emg_data, emg_cols)
        
        if active_emg.size == 0:
            return features

        # --- Time Domain Features ---
        mav = np.mean(np.abs(active_emg), axis=0)
        rms = np.sqrt(np.mean(active_emg**2, axis=0))
        wl = np.sum(np.abs(np.diff(active_emg, axis=0)), axis=0)
        
        # Zero Crossings (ZC)
        zc = np.sum((active_emg[:-1, :] * active_emg[1:, :]) < 0, axis=0)
        
        # Slope Sign Changes (SSC)
        dx = np.diff(active_emg, axis=0)
        ssc = np.sum((dx[:-1, :] * dx[1:, :]) < 0, axis=0)
        
        variance = np.var(active_emg, axis=0)

        # --- Frequency Domain Features ---
        nperseg = min(256, active_emg.shape[0])
        freqs, psd = signal.welch(active_emg, fs=self.emg_fs, axis=0, nperseg=nperseg)
        
        total_power = np.sum(psd, axis=0)
        
        # Mean Frequency (MNF)
        # Avoid division by zero
        safe_total_power = np.where(total_power == 0, 1e-10, total_power)
        mnf = np.sum(freqs[:, None] * psd, axis=0) / safe_total_power
        
        # Median Frequency (MDF)
        cum_power = np.cumsum(psd, axis=0)
        half_power = total_power / 2.0
        # Find the first index where cumulative power exceeds half total power for each column
        mdf_indices = np.nanargmax(cum_power >= half_power, axis=0)
        mdf = freqs[mdf_indices]

        # Populate features dict
        for i, col in enumerate(active_cols):
            features[f"{col}_EMG_MAV"] = mav[i]
            features[f"{col}_EMG_RMS"] = rms[i]
            features[f"{col}_EMG_WL"] = wl[i]
            features[f"{col}_EMG_ZC"] = zc[i]
            features[f"{col}_EMG_SSC"] = ssc[i]
            features[f"{col}_EMG_VAR"] = variance[i]
            features[f"{col}_EMG_MNF"] = mnf[i]
            features[f"{col}_EMG_MDF"] = mdf[i]
            features[f"{col}_EMG_Power"] = total_power[i]

        return features

    def extract_imu_features(self, imu_data: np.ndarray, imu_cols: List[str]) -> Dict[str, float]:
        """
        Extracts Time and Frequency domain features from IMU data.
        Returns a dictionary with feature names mapping to their computed values.
        """
        features = {}
        if imu_data.size == 0:
            return features

        # --- Time Domain Features ---
        mean_val = np.mean(imu_data, axis=0)
        var_val = np.var(imu_data, axis=0)
        std_val = np.std(imu_data, axis=0)
        max_val = np.max(imu_data, axis=0)
        min_val = np.min(imu_data, axis=0)
        rms_val = np.sqrt(np.mean(imu_data**2, axis=0))
        
        # Signal Magnitude Area (SMA) - typically sum of integrals of magnitude across axes, 
        # but here we compute per-axis SMA as sum of abs values divided by fs
        sma_val = np.sum(np.abs(imu_data), axis=0) / self.imu_fs

        # --- Frequency Domain Features ---
        nperseg = min(256, imu_data.shape[0])
        freqs, psd = signal.welch(imu_data, fs=self.imu_fs, axis=0, nperseg=nperseg)
        
        spectral_energy = np.sum(psd, axis=0)
        dominant_freq_indices = np.argmax(psd, axis=0)
        dominant_freq = freqs[dominant_freq_indices]

        # Populate features dict
        for i, col in enumerate(imu_cols):
            features[f"{col}_IMU_Mean"] = mean_val[i]
            features[f"{col}_IMU_Var"] = var_val[i]
            features[f"{col}_IMU_Std"] = std_val[i]
            features[f"{col}_IMU_Max"] = max_val[i]
            features[f"{col}_IMU_Min"] = min_val[i]
            features[f"{col}_IMU_RMS"] = rms_val[i]
            features[f"{col}_IMU_SMA"] = sma_val[i]
            features[f"{col}_IMU_DomFreq"] = dominant_freq[i]
            features[f"{col}_IMU_SpecEnergy"] = spectral_energy[i]

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
