"""Minimal debug test to verify PRBS correlation is working."""
import numpy as np
from src.prbs_sync import PRBSGenerator, SynchronizationEngine

# Create PRBS generator  
prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)

# Create sync engine
sync_engine = SynchronizationEngine(
    prbs_generator=prbs,
    correlation_window_s=5.0,
    update_interval_s=1.0,
    simulation_mode=True
)

# Generate 10 seconds of data
emg_rate = 1000
imu_rate = 50
duration_s = 10
known_offset_ms = 15.3

t_emg = np.arange(0, duration_s, 1.0/emg_rate)
t_imu = np.arange(0, duration_s, 1.0/imu_rate)

# Generate PRBS markers with offset in marker timing (CORRECTED approach)
prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
prbs_imu = np.array([prbs.get_marker_at_time(t + known_offset_ms / 1000.0) for t in t_imu])

print(f"Generated {len(t_emg)} EMG samples, {len(t_imu)} IMU samples")
print(f"EMG PRBS markers: min={prbs_emg.min()}, max={prbs_emg.max()}, unique={len(np.unique(prbs_emg))}")
print(f"IMU PRBS markers: min={prbs_imu.min()}, max={prbs_imu.max()}, unique={len(np.unique(prbs_imu))}")

# Check first few markers
print("\nFirst 20 EMG timestamps and markers:")
for i in range(min(20, len(t_emg))):
    print(f"  t={t_emg[i]:.4f}s -> marker={prbs_emg[i]:+.0f}")

print("\nFirst 20 IMU timestamps and markers:")
for i in range(min(20, len(t_imu))):
    print(f"  t={t_imu[i]:.4f}s (marker at t+offset={t_imu[i] + known_offset_ms/1000.0:.4f}s) -> marker={prbs_imu[i]:+.0f}")

# Feed all data to sync engine
for i in range(len(t_emg)):
    sync_engine.add_emg_data(t_emg[i], prbs_emg[i])
    
for i in range(len(t_imu)):
    sync_engine.add_imu_data(t_imu[i], prbs_imu[i])

print(f"\nBuffer sizes: EMG={len(sync_engine.emg_buffer)}, IMU={len(sync_engine.imu_buffer)}")

# Try to compute offset
if sync_engine.should_update():
    print("\nRunning synchronization...")
    sync_engine.update_sync()
    sync_state = sync_engine.get_sync_state()
    
    if sync_state:
        print(f"Estimated offset: {sync_state.offset_ms:.2f} ms")
        print(f"Known offset: {known_offset_ms:.2f} ms")
        print(f"Error: {abs(sync_state.offset_ms - known_offset_ms):.2f} ms")
        print(f"Confidence: {sync_state.confidence:.3f}")
    else:
        print("No offset estimate (insufficient data or correlation failed)")
else:
    print("Should not update yet (timing issue)")
