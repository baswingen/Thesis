import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def analyze_interpolation():
    # Paths
    base_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
    trial_file = Path("/Volumes/Laurens SSD/BasData/participant_P01/session_01/trial_2_20260304_151841.h5")
    output_dir = base_dir / "data_analysis/results"
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Opening trial file: {trial_file.name}")
    
    with h5py.File(trial_file, 'r') as f:
        # 1. Load Raw Data
        raw_stm32 = f['_raw/stm32'][:]
        t_raw = raw_stm32[:, 0]  # t_pc
        ax_raw = raw_stm32[:, 7]  # ax1
        
        # 2. Load Synced/Interpolated Data
        synced_data = f['synced/data'][:]
        t_sync = synced_data[:, 0]  # t_pc_common
        ax_sync = synced_data[:, 8]  # ax1
        
    print(f"Loaded {len(t_raw)} raw samples and {len(t_sync)} synced samples.")
    
    # 3. Select a small window (e.g., 2 seconds in the middle of the trial)
    # Find a region with some variance if possible, or just pick a segment
    start_idx = len(t_sync) // 2
    end_idx = start_idx + 4000  # 2 seconds at 2000Hz
    
    t_sync_win = t_sync[start_idx:end_idx]
    ax_sync_win = ax_sync[start_idx:end_idx]
    
    # Filter raw data to match the time window
    mask_raw = (t_raw >= t_sync_win[0]) & (t_raw <= t_sync_win[-1])
    t_raw_win = t_raw[mask_raw]
    ax_raw_win = ax_raw[mask_raw]
    
    # 4. Compute Derivatives (Velocity/Jerk proxy)
    # We use np.diff and divide by time step to get actual rate of change
    dt_sync = np.diff(t_sync_win)
    dt_sync[dt_sync == 0] = 1e-6  # avoid div by zero
    jerk_sync = np.diff(ax_sync_win) / dt_sync
    
    # 5. Plotting
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # Plot 1: Raw vs Synced
    axs[0].plot(t_sync_win, ax_sync_win, label='Synced (Interpolated 2000Hz)', color='#3498db', linewidth=2)
    axs[0].scatter(t_raw_win, ax_raw_win, label='Raw IMU Points', color='#e74c3c', zorder=5, s=20)
    axs[0].set_title('IMU Acceleration (ax1) - Raw vs Interpolated')
    axs[0].set_ylabel('Acceleration (m/s^2)')
    axs[0].legend()
    axs[0].grid(True, alpha=0.3)
    
    # Plot 2: Zoomed in view
    zoom_len = 500  # 0.25 seconds
    t_sync_zoom = t_sync_win[:zoom_len]
    ax_sync_zoom = ax_sync_win[:zoom_len]
    mask_raw_zoom = (t_raw_win >= t_sync_zoom[0]) & (t_raw_win <= t_sync_zoom[-1])
    t_raw_zoom = t_raw_win[mask_raw_zoom]
    ax_raw_zoom = ax_raw_win[mask_raw_zoom]
    
    axs[1].plot(t_sync_zoom, ax_sync_zoom, 'o-', label='Synced (Zoomed)', color='#2ecc71', markersize=2)
    axs[1].scatter(t_raw_zoom, ax_raw_zoom, label='Raw Points (Zoomed)', color='#e74c3c', zorder=5, s=40)
    axs[1].set_title('Zoomed View (0.25 seconds)')
    axs[1].set_ylabel('Acceleration (m/s^2)')
    axs[1].legend()
    axs[1].grid(True, alpha=0.3)
    
    # Plot 3: Derivative (Jerk proxy)
    axs[2].plot(t_sync_win[:-1], jerk_sync, label='Jerk Proxy (d(ax)/dt)', color='#9b59b6')
    axs[2].set_title('Derivative of Interpolated Signal')
    axs[2].set_xlabel('Time (PC Clock)')
    axs[2].set_ylabel('Jerk')
    axs[2].legend()
    axs[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_path = output_dir / "imu_interpolation_analysis.png"
    plt.savefig(plot_path, dpi=300)
    print(f"Plot saved to: {plot_path}")
    
if __name__ == '__main__':
    analyze_interpolation()
