import sys
import h5py
import numpy as np
import time
import json
import argparse
from pathlib import Path

# Provide application root as correct path irrespective of where this is run from
sys.path.append(str(Path(__file__).parent.parent))

from src.stm32_emg_sync import SyncDelayEstimator
from src.stm32_reader import SampleSTM32
from setup_scripts.signal_acquisition_testing import PRBS_CHIP_RATE_HZ, PRBS_CORRELATION_WINDOW_S, PRBS_UPDATE_INTERVAL_S
from trial.hdf5_logger import HDF5TrialLogger

def fix_h5_sync(file_path: str):
    print(f"Reading raw datasets from {file_path}...")
    with h5py.File(file_path, 'r') as f:
        emg_data = f['_raw/emg'][:]
        stm32_data = f['_raw/stm32'][:]
        
        # Check if participant and trial metadata exists
        if 'participant_id' in f.attrs:
            participant_id = f.attrs['participant_id']
            trial_num = f.attrs['trial_num']
        else:
            participant_id = "UNKNOWN"
            trial_num = 0
            
        session_num = 1
        
        # get config metadata if available
        metadata = {}
        if 'config_participant' in f.attrs:
            try:
                metadata = json.loads(f.attrs['config_participant'])
            except json.JSONDecodeError:
                pass

    # Initialize Logger (it will just wrap the existing file and allow us to use its member functions)
    db_path = str(Path(file_path).parent.parent.parent)
    logger = HDF5TrialLogger(
        base_dir=db_path,
        participant_id=participant_id, 
        session_num=session_num,
        trial_num=trial_num,
        metadata=metadata
    )
    # Override the filename to point specifically to our given file
    logger.filename = Path(file_path)

    t_stm32_pc = stm32_data[:, 0]
    t_emg_pc = emg_data[:, 0]

    emg_sr = len(t_emg_pc) / (t_emg_pc[-1] - t_emg_pc[0]) if len(t_emg_pc) > 1 else 2000.0

    print(f"File EMG SR: {emg_sr:.2f}")

    estimator = SyncDelayEstimator(
        chip_rate_hz=PRBS_CHIP_RATE_HZ,
        emg_sample_rate=emg_sr,
        sync_window_s=PRBS_CORRELATION_WINDOW_S,
        update_interval_s=PRBS_UPDATE_INTERVAL_S
    )

    print("Feeding STM32 data...")
    # Direct overwrite
    estimator._stm32_samples = [(stm32_data[i, 0], SampleSTM32(
        t_ms=stm32_data[i, 1],
        imu1_ok=int(stm32_data[i, 2]),
        imu2_ok=int(stm32_data[i, 3]),
        yaw1=stm32_data[i, 4], pitch1=stm32_data[i, 5], roll1=stm32_data[i, 6], ax1=stm32_data[i, 7], ay1=stm32_data[i, 8], az1=stm32_data[i, 9],
        yaw2=stm32_data[i, 10], pitch2=stm32_data[i, 11], roll2=stm32_data[i, 12], ax2=stm32_data[i, 13], ay2=stm32_data[i, 14], az2=stm32_data[i, 15],
        keys_mask=int(stm32_data[i, 16]), keys_rise=int(stm32_data[i, 17]), keys_fall=int(stm32_data[i, 18]),
        prbs_tick=int(stm32_data[i, 19]), prbs_lvl=int(stm32_data[i, 20]), in_mark=int(stm32_data[i, 21])
    )) for i in range(len(stm32_data))]

    # Emg buffer is a list of (timestamps, chunk_array)
    # Feed it in chunks of 2000
    for i in range(0, len(t_emg_pc), 2000):
        t_chunk = t_emg_pc[i:i+2000]
        val_chunk = emg_data[i:i+2000, 34]
        estimator._emg_trig_buf.append((t_chunk, val_chunk))

    print("Running estimator update...")
    estimator._last_update_time = 0.0 # Force ready
    now_time = t_emg_pc[-1]
    success = getattr(estimator, '_do_update', lambda x: False)(now=now_time)

    if not success:
        # Try different intervals manually
        t = t_emg_pc[len(t_emg_pc)//2]
        success = getattr(estimator, '_do_update', lambda x: False)(now=t)

    print(f"Estimator update success: {success}")
    res = estimator._last_result
    if res is None:
        print("Failed to find sync parameters. Appending as 0-delay.")
        final_delay_ms = 0.0
        final_drift_ppm = 0.0
    else:
        print(f"Found valid sync result: Delay {res.delay_ms:.2f} ms")
        final_delay_ms = res.delay_ms
        final_drift_ppm = estimator._drift_rate_ppm if hasattr(estimator, '_drift_rate_ppm') else 0.0

    print(f"Applying Time Shifting...")
    t_ref = t_emg_pc[0]
    drift_rate = final_drift_ppm / 1e6

    dt_stm32 = t_stm32_pc - t_ref
    current_delays_s = (final_delay_ms / 1000.0) + (dt_stm32 * drift_rate)
    t_stm32_aligned = t_stm32_pc + current_delays_s

    stm32_synced = np.zeros((len(t_emg_pc), stm32_data.shape[1]), dtype=np.float64)

    for col in range(stm32_data.shape[1]):
        stm32_synced[:, col] = np.interp(
            t_emg_pc,
            t_stm32_aligned,
            stm32_data[:, col],
            left=np.nan, right=np.nan
        )
        
    valid_mask = ~np.isnan(stm32_synced[:, 0])

    t_common = t_emg_pc[valid_mask]
    stm32_synced = stm32_synced[valid_mask, :]
    emg_synced_data = emg_data[valid_mask, 2:] 

    unified_matrix = np.column_stack((stm32_synced, emg_synced_data))

    print("Writing datasets to HDF5 via Logger...")
    # This correctly writes to `synced/data` group via the HDF5TrialLogger module implementation
    logger.save_synchronized_datasets(
        t_common=t_common,
        unified_matrix=unified_matrix,
        kalman_delay_ms=final_delay_ms,
        kalman_drift_ppm=final_drift_ppm
    )
    print("Done!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline PRBS Synchronization Fixer for corrupted HDF5 files.")
    parser.add_argument("file_path", type=str, help="Absolute path to the .h5 trial file to be synchronized.")
    args = parser.parse_args()
    
    p = Path(args.file_path)
    if not p.exists() or p.suffix != '.h5':
        print(f"Error: Target file '{args.file_path}' does not exist or is not an .h5 file.")
        sys.exit(1)
        
    fix_h5_sync(args.file_path)
