# On-Device Feature Configuration Sweep Report

**Date:** 2026-05-20 15:59:32

**DEV_MODE:** True

**Total Sweep Execution Time:** 41.2 minutes

**Cross-Participant Validation Group:** held out subjects `P01, P02, P06, P17` for testing.

## Ranked Feature Configurations (Sorted by RMSE)

| Rank | Configuration | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time | Epochs |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | Config 3: No Frequency Domain | 220 | **0.5350** | 0.3034 | 0.9216 | 922,490 | 273.3s | 10 |
| 2 | Config 2: No Skewness & Kurtosis | 264 | **0.5608** | 0.2981 | 0.9138 | 927,770 | 518.6s | 20 |
| 3 | Config 6: EMG Only | 160 | **0.5796** | 0.3224 | 0.9079 | 563,930 | 108.3s | 9 |
| 4 | Config 5: Minimal Cheap (Ultra Real-Time) | 108 | **0.5909** | 0.3241 | 0.9043 | 909,050 | 353.7s | 12 |
| 5 | Config 4: Pure Time Domain | 180 | **0.5990** | 0.3266 | 0.9017 | 917,690 | 428.7s | 17 |
| 6 | Config 1: All Features (Baseline) | 304 | **0.6547** | 0.3600 | 0.8826 | 932,570 | 303.6s | 14 |
| 7 | Config 7: IMU Only | 144 | **1.3775** | 0.7922 | 0.4800 | 737,210 | 449.0s | 23 |

## Configuration Key Details & Interpretations

### Config 3: No Frequency Domain
* **Active Features:** 220
* **Val RMSE / MAE:** 0.5350 / 0.3034
* **Generalization R²:** 0.9216
* **On-Device Impact:**
  * **Eliminates ~19.5% of feature extraction compute.** Bypasses windowed Welch periodograms and spectral entropy calculations. Very strong efficiency gains.

### Config 2: No Skewness & Kurtosis
* **Active Features:** 264
* **Val RMSE / MAE:** 0.5608 / 0.2981
* **Generalization R²:** 0.9138
* **On-Device Impact:**
  * **Eliminates ~62.7% of Python's feature extraction overhead.** Removes stats.skew and stats.kurtosis. If accuracy is matching baseline, this is an automatic green light for real-time loops.

### Config 6: EMG Only
* **Active Features:** 160
* **Val RMSE / MAE:** 0.5796 / 0.3224
* **Generalization R²:** 0.9079
* **On-Device Impact:**
  * Analyzes the predictive capacity of EMG muscle signals alone. Eliminates all 12 IMU kinematic channels.

### Config 5: Minimal Cheap (Ultra Real-Time)
* **Active Features:** 108
* **Val RMSE / MAE:** 0.5909 / 0.3241
* **Generalization R²:** 0.9043
* **On-Device Impact:**
  * **Ultra Real-Time Optimal.** Restricts computations purely to $O(N)$ linear vectorized math with no logs, exponentials, or complex sorting. Highly recommended for ultra-low power devices.

### Config 4: Pure Time Domain
* **Active Features:** 180
* **Val RMSE / MAE:** 0.5990 / 0.3266
* **Generalization R²:** 0.9017
* **On-Device Impact:**
  * **Eliminates ~82.2% of total feature extraction compute.** No FFTs, no higher-order statistical moments. Relies solely on standard vectorized metrics (MAV, RMS, WL, etc.).

### Config 1: All Features (Baseline)
* **Active Features:** 304
* **Val RMSE / MAE:** 0.6547 / 0.3600
* **Generalization R²:** 0.8826
* **On-Device Impact:**
  * Baseline full dataset. Highest mathematical richness, but requires 318 features including expensive windowed Welch FFTs and third/fourth-order SciPy moments.

### Config 7: IMU Only
* **Active Features:** 144
* **Val RMSE / MAE:** 1.3775 / 0.7922
* **Generalization R²:** 0.4800
* **On-Device Impact:**
  * Analyzes the predictive capacity of IMU kinematics alone. Eliminates all 8 EMG muscle activation channels.

