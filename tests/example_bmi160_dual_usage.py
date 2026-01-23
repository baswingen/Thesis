"""
Example usage of the bmi160_dual module.

This script demonstrates various ways to use the DualIMU class
for data acquisition from dual BMI160 sensors.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bmi160_dual import DualIMU, IMUConfig, quick_read


def example_basic():
    """Basic usage with defaults."""
    print("="*60)
    print("Example 1: Basic Usage")
    print("="*60)
    
    with DualIMU() as imu:
        # Calibrate
        print("\nPlace IMUs flat for calibration...")
        time.sleep(3)
        imu.calibrate(samples=200)
        
        # Read some data
        print("\nReading 50 samples...")
        for i, reading in enumerate(imu.read_stream(max_samples=50)):
            if i % 10 == 0:
                print(f"Sample {reading.seq:6d} | "
                      f"IMU1: R={reading.euler1[0]:+6.1f}° P={reading.euler1[1]:+6.1f}° Y={reading.euler1[2]:+6.1f}° | "
                      f"IMU2: R={reading.euler2[0]:+6.1f}° P={reading.euler2[1]:+6.1f}° Y={reading.euler2[2]:+6.1f}°")
    
    print("\n✓ Complete\n")


def example_custom_config():
    """Usage with custom configuration."""
    print("="*60)
    print("Example 2: Custom Configuration")
    print("="*60)
    
    # Create custom config
    config = IMUConfig(
        port=None,  # Auto-detect
        baud=230400,
        use_mahony=True,
        kp=2.0,
        ki=0.01,
        adaptive_gains=True
    )
    
    with DualIMU(config) as imu:
        print("\nCalibrating...")
        imu.calibrate(samples=100)
        
        print("\nReading for 5 seconds...")
        start = time.time()
        count = 0
        
        for reading in imu.read_stream(duration=5.0):
            count += 1
            if count % 50 == 0:
                elapsed = time.time() - start
                rate = count / elapsed
                print(f"  {count} samples, {rate:.1f} Hz", end='\r')
        
        print(f"\n✓ Collected {count} samples in {elapsed:.1f}s ({rate:.1f} Hz)\n")


def example_data_collection():
    """Collect data to numpy arrays for analysis."""
    print("="*60)
    print("Example 3: Data Collection for Analysis")
    print("="*60)
    
    with DualIMU() as imu:
        print("\nCalibrating...")
        imu.calibrate(samples=200)
        
        print("\nCollecting 200 samples...")
        
        timestamps = []
        accel1_data = []
        accel2_data = []
        euler1_data = []
        euler2_data = []
        
        for reading in imu.read_stream(max_samples=200):
            timestamps.append(reading.timestamp)
            accel1_data.append(reading.accel1)
            accel2_data.append(reading.accel2)
            euler1_data.append(reading.euler1)
            euler2_data.append(reading.euler2)
        
        # Convert to numpy arrays
        timestamps = np.array(timestamps)
        accel1_data = np.array(accel1_data)
        accel2_data = np.array(accel2_data)
        euler1_data = np.array(euler1_data)
        euler2_data = np.array(euler2_data)
        
        print(f"\n✓ Collected data:")
        print(f"  Shape: {accel1_data.shape}")
        print(f"  Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
        print(f"  Mean accel1: {np.mean(accel1_data, axis=0)}")
        print(f"  Std euler1: {np.std(euler1_data, axis=0)}")
        print()


def example_manual_loop():
    """Manual reading loop with error handling."""
    print("="*60)
    print("Example 4: Manual Loop with Error Handling")
    print("="*60)
    
    with DualIMU() as imu:
        print("\nCalibrating...")
        imu.calibrate(samples=100)
        
        print("\nReading with manual loop...")
        for i in range(100):
            reading = imu.read()
            if reading:
                if i % 20 == 0:
                    # Calculate relative orientation
                    if reading.quat1 is not None and reading.quat2 is not None:
                        from src.bmi160_dual import quat_mul, quat_conj, quat_to_euler
                        
                        q_rel = quat_mul(quat_conj(reading.quat1), reading.quat2)
                        euler_rel = quat_to_euler(q_rel)
                        
                        print(f"Sample {reading.seq}: Relative orientation = "
                              f"R={euler_rel[0]:+6.1f}° P={euler_rel[1]:+6.1f}° Y={euler_rel[2]:+6.1f}°")
            else:
                print(f"Sample {i}: No data received")
            
            time.sleep(0.01)
    
    print("\n✓ Complete\n")


def example_quick_read():
    """Using convenience function."""
    print("="*60)
    print("Example 5: Quick Read Function")
    print("="*60)
    
    print("\nUsing quick_read() convenience function...")
    readings = quick_read(port=None, samples=50, calibrate=True)
    
    print(f"\n✓ Collected {len(readings)} readings")
    print(f"  First euler1: {readings[0].euler1}")
    print(f"  Last euler1: {readings[-1].euler1}")
    print()


def example_csv_export():
    """Collect data and export to CSV."""
    print("="*60)
    print("Example 6: CSV Export")
    print("="*60)
    
    import csv
    from datetime import datetime
    
    output_file = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    with DualIMU() as imu:
        print("\nCalibrating...")
        imu.calibrate(samples=200)
        
        print(f"\nCollecting data to {output_file}...")
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'seq', 't_us',
                'ax1', 'ay1', 'az1', 'gx1', 'gy1', 'gz1',
                'ax2', 'ay2', 'az2', 'gx2', 'gy2', 'gz2',
                'roll1', 'pitch1', 'yaw1',
                'roll2', 'pitch2', 'yaw2'
            ])
            
            for reading in imu.read_stream(max_samples=100):
                writer.writerow([
                    reading.timestamp, reading.seq, reading.t_us,
                    reading.accel1[0], reading.accel1[1], reading.accel1[2],
                    reading.gyro1[0], reading.gyro1[1], reading.gyro1[2],
                    reading.accel2[0], reading.accel2[1], reading.accel2[2],
                    reading.gyro2[0], reading.gyro2[1], reading.gyro2[2],
                    reading.euler1[0], reading.euler1[1], reading.euler1[2],
                    reading.euler2[0], reading.euler2[1], reading.euler2[2]
                ])
        
        print(f"✓ Saved to {output_file}\n")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("Dual BMI160 Module Usage Examples")
    print("="*60 + "\n")
    
    examples = [
        ("Basic usage", example_basic),
        ("Custom config", example_custom_config),
        ("Data collection", example_data_collection),
        ("Manual loop", example_manual_loop),
        ("Quick read", example_quick_read),
        ("CSV export", example_csv_export)
    ]
    
    print("Available examples:")
    for i, (name, func) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    print(f"  0. Run all")
    
    try:
        choice = input("\nSelect example (0-6): ").strip()
        choice = int(choice)
        
        if choice == 0:
            for name, func in examples:
                func()
                time.sleep(1)
        elif 1 <= choice <= len(examples):
            examples[choice - 1][1]()
        else:
            print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
