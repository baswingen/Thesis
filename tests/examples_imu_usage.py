"""
Example Usage: IMU Acquisition Module

This file demonstrates various ways to use the IMU acquisition module
from the project level.

Author: Auto-generated
Date: January 2026
"""

import time
import numpy as np

# Import from src package
from src import IMUDevice, acquire_imu_data, IMUCalibration


# =============================================================================
# CONFIGURATION
# =============================================================================

# IMPORTANT: Update this to your serial port!
PORT = "/dev/cu.usbmodem9888E00A0CC02"  # macOS/Linux
# PORT = "COM3"  # Windows

BAUD = 115200


# =============================================================================
# EXAMPLE 1: Basic Data Acquisition
# =============================================================================

def example_1_basic_acquisition():
    """Simple example: connect and read raw IMU data."""
    print("\n" + "="*60)
    print("EXAMPLE 1: Basic Data Acquisition")
    print("="*60)
    
    # Create device
    device = IMUDevice(port=PORT, baud=BAUD)
    
    try:
        # Connect
        print("\nConnecting to IMU device...")
        device.connect()
        
        # Read 20 samples
        print("Reading 20 samples...\n")
        for i in range(20):
            reading = device.read_sample()
            
            if reading:
                # Calculate accelerometer magnitudes
                a1_mag = np.linalg.norm(reading.imu1_accel)
                a2_mag = np.linalg.norm(reading.imu2_accel)
                
                print(f"Sample {i+1:2d}: "
                      f"t={reading.timestamp_ms:6d}ms  "
                      f"IMU1_gyro=[{reading.imu1_gyro[0]:6.2f}, {reading.imu1_gyro[1]:6.2f}, {reading.imu1_gyro[2]:6.2f}]  "
                      f"accel_mag={a1_mag:.3f}g")
            
            time.sleep(0.05)
        
        print("\n[OK] Basic acquisition complete")
    
    finally:
        device.disconnect()


# =============================================================================
# EXAMPLE 2: Calibration and Corrected Data
# =============================================================================

def example_2_calibration():
    """Example showing gyro calibration and bias correction."""
    print("\n" + "="*60)
    print("EXAMPLE 2: Calibration and Bias Correction")
    print("="*60)
    
    device = IMUDevice(port=PORT, baud=BAUD)
    
    try:
        # Connect
        device.connect()
        
        # Calibrate
        print("\n" + "="*60)
        print("CALIBRATION REQUIRED")
        print("="*60)
        print("Place BOTH IMUs FLAT on a surface and keep them STILL")
        print("Calibration will start in 3 seconds...")
        time.sleep(3)
        
        calibration = device.calibrate_gyro(num_samples=150)
        
        print("\n[OK] Calibration complete!")
        print(f"  IMU1 bias: [{calibration.gyro_bias_imu1[0]:.3f}, "
              f"{calibration.gyro_bias_imu1[1]:.3f}, "
              f"{calibration.gyro_bias_imu1[2]:.3f}] deg/s")
        print(f"  IMU2 bias: [{calibration.gyro_bias_imu2[0]:.3f}, "
              f"{calibration.gyro_bias_imu2[1]:.3f}, "
              f"{calibration.gyro_bias_imu2[2]:.3f}] deg/s")
        
        # Read data with bias correction
        print("\nReading 10 samples (with bias correction)...\n")
        
        for i in range(10):
            reading = device.read_sample()
            
            if reading:
                # Apply calibration manually
                gyro1_corrected = reading.imu1_gyro - calibration.gyro_bias_imu1
                gyro2_corrected = reading.imu2_gyro - calibration.gyro_bias_imu2
                
                # Calculate magnitude
                g1_mag = np.linalg.norm(gyro1_corrected)
                g2_mag = np.linalg.norm(gyro2_corrected)
                
                print(f"Sample {i+1:2d}: "
                      f"IMU1_gyro_corrected=[{gyro1_corrected[0]:6.3f}, "
                      f"{gyro1_corrected[1]:6.3f}, {gyro1_corrected[2]:6.3f}]  "
                      f"mag={g1_mag:.3f} deg/s")
            
            time.sleep(0.1)
        
        print("\n[OK] Calibrated data acquisition complete")
        
        # Save calibration (optional)
        print("\nYou can save calibration for later use:")
        print("""
import pickle
with open('imu_calibration.pkl', 'wb') as f:
    pickle.dump(calibration, f)
        """)
    
    finally:
        device.disconnect()


# =============================================================================
# EXAMPLE 3: Orientation Tracking
# =============================================================================

def example_3_orientation_tracking():
    """Example showing orientation estimation with Madgwick filter."""
    print("\n" + "="*60)
    print("EXAMPLE 3: Orientation Tracking")
    print("="*60)
    
    device = IMUDevice(port=PORT, baud=BAUD)
    
    try:
        # Connect and calibrate
        device.connect()
        
        print("\nCalibrating... (keep IMUs flat and still)")
        time.sleep(2)
        calibration = device.calibrate_gyro(num_samples=100)
        
        # Enable orientation filter
        print("\n[OK] Enabling orientation filter (beta=0.08)")
        device.enable_orientation_filter(beta=0.08)
        
        # Track orientation for 15 seconds
        print("\n[OK] Tracking orientation for 15 seconds...")
        print("Try moving the IMUs to see orientation changes!\n")
        
        start_time = time.time()
        sample_count = 0
        
        while (time.time() - start_time) < 15.0:
            data = device.read_sample_with_orientation()
            
            if data and 'imu1_euler' in data:
                sample_count += 1
                
                # Get Euler angles and convert to degrees
                euler1 = np.rad2deg(data['imu1_euler'])
                euler2 = np.rad2deg(data['imu2_euler'])
                
                # Get quaternions
                q1 = data['imu1_quat']
                q2 = data['imu2_quat']
                
                # Display (update in place)
                elapsed = time.time() - start_time
                print(f"[{elapsed:5.1f}s] "
                      f"IMU1: Roll={euler1[0]:6.1f}° Pitch={euler1[1]:6.1f}° Yaw={euler1[2]:6.1f}°  "
                      f"IMU2: Roll={euler2[0]:6.1f}° Pitch={euler2[1]:6.1f}° Yaw={euler2[2]:6.1f}°  "
                      f"[{sample_count} samples]", end='\r')
            
            time.sleep(0.05)
        
        print(f"\n\n[OK] Orientation tracking complete ({sample_count} samples)")
    
    finally:
        device.disconnect()


# =============================================================================
# EXAMPLE 4: Relative Orientation Between IMUs
# =============================================================================

def example_4_relative_orientation():
    """Example showing relative orientation between two IMUs."""
    print("\n" + "="*60)
    print("EXAMPLE 4: Relative Orientation Between IMUs")
    print("="*60)
    
    from src.imu_acquisition import quat_mul, quat_inv, quat_to_euler
    
    device = IMUDevice(port=PORT, baud=BAUD)
    
    try:
        # Connect and setup
        device.connect()
        
        print("\nCalibrating...")
        time.sleep(2)
        device.calibrate_gyro(num_samples=100)
        device.enable_orientation_filter(beta=0.08)
        
        # Track relative orientation
        print("\n[OK] Tracking RELATIVE orientation (IMU2 relative to IMU1)")
        print("This is useful for measuring joint angles, etc.\n")
        
        for i in range(100):
            data = device.read_sample_with_orientation()
            
            if data and 'imu1_quat' in data:
                q1 = data['imu1_quat']
                q2 = data['imu2_quat']
                
                # Calculate relative orientation (IMU2 relative to IMU1)
                q_rel = quat_mul(quat_inv(q1), q2)
                
                # Convert to Euler angles
                roll_rel, pitch_rel, yaw_rel = quat_to_euler(q_rel)
                
                # Display
                print(f"Sample {i+1:3d}: "
                      f"Relative Roll={np.rad2deg(roll_rel):7.2f}°  "
                      f"Pitch={np.rad2deg(pitch_rel):7.2f}°  "
                      f"Yaw={np.rad2deg(yaw_rel):7.2f}°", end='\r')
            
            time.sleep(0.05)
        
        print("\n\n[OK] Relative orientation tracking complete")
    
    finally:
        device.disconnect()


# =============================================================================
# EXAMPLE 5: Convenience Function for Batch Acquisition
# =============================================================================

def example_5_convenience_function():
    """Example using the convenience function for easy data acquisition."""
    print("\n" + "="*60)
    print("EXAMPLE 5: Convenience Function (acquire_imu_data)")
    print("="*60)
    
    print("\nThe convenience function handles everything for you:")
    print("  - Connection")
    print("  - Calibration")
    print("  - Orientation filtering")
    print("  - Data collection")
    print("  - Automatic cleanup")
    
    print("\nIMPORTANT: Place IMUs flat for calibration")
    print("Starting in 3 seconds...")
    time.sleep(3)
    
    # Acquire 10 seconds of data
    readings, calibration = acquire_imu_data(
        port=PORT,
        duration=10.0,
        baud=BAUD,
        calibrate=True,
        enable_filter=True,
        filter_beta=0.08
    )
    
    if readings:
        print(f"\n[OK] Acquired {len(readings)} samples")
        
        # Analyze the data
        print("\nData Analysis:")
        
        # Extract Euler angles
        if isinstance(readings[0], dict) and 'imu1_euler' in readings[0]:
            euler1_data = np.array([r['imu1_euler'] for r in readings])
            euler1_deg = np.rad2deg(euler1_data)
            
            print(f"  IMU1 Roll:  mean={np.mean(euler1_deg[:, 0]):6.2f}°, "
                  f"std={np.std(euler1_deg[:, 0]):5.2f}°, "
                  f"range=[{np.min(euler1_deg[:, 0]):6.2f}, {np.max(euler1_deg[:, 0]):6.2f}]°")
            print(f"  IMU1 Pitch: mean={np.mean(euler1_deg[:, 1]):6.2f}°, "
                  f"std={np.std(euler1_deg[:, 1]):5.2f}°, "
                  f"range=[{np.min(euler1_deg[:, 1]):6.2f}, {np.max(euler1_deg[:, 1]):6.2f}]°")
            print(f"  IMU1 Yaw:   mean={np.mean(euler1_deg[:, 2]):6.2f}°, "
                  f"std={np.std(euler1_deg[:, 2]):5.2f}°, "
                  f"range=[{np.min(euler1_deg[:, 2]):6.2f}, {np.max(euler1_deg[:, 2]):6.2f}]°")
        
        # Calibration info
        if calibration:
            print(f"\n  Calibration:")
            print(f"    IMU1 bias magnitude: {np.linalg.norm(calibration.gyro_bias_imu1):.3f} deg/s")
            print(f"    IMU2 bias magnitude: {np.linalg.norm(calibration.gyro_bias_imu2):.3f} deg/s")
        
        print("\n[OK] Convenience function example complete")
    else:
        print("\n[FAIL] No data acquired")


# =============================================================================
# EXAMPLE 6: Save and Load Data
# =============================================================================

def example_6_save_load():
    """Example showing how to save and load IMU data."""
    print("\n" + "="*60)
    print("EXAMPLE 6: Save and Load Data")
    print("="*60)
    
    print("\nAcquiring data to save...")
    print("IMPORTANT: Place IMUs flat for calibration")
    time.sleep(2)
    
    # Acquire data
    readings, calibration = acquire_imu_data(
        port=PORT,
        duration=5.0,
        calibrate=True,
        enable_filter=True
    )
    
    if not readings:
        print("[FAIL] No data to save")
        return
    
    # Save to NPZ file
    print("\n[OK] Saving data to 'imu_data_example.npz'...")
    
    timestamps = np.array([r['reading'].timestamp_s for r in readings])
    imu1_gyro = np.array([r['reading'].imu1_gyro for r in readings])
    imu1_accel = np.array([r['reading'].imu1_accel for r in readings])
    imu2_gyro = np.array([r['reading'].imu2_gyro for r in readings])
    imu2_accel = np.array([r['reading'].imu2_accel for r in readings])
    imu1_quat = np.array([r['imu1_quat'] for r in readings])
    imu2_quat = np.array([r['imu2_quat'] for r in readings])
    
    np.savez('imu_data_example.npz',
             timestamps=timestamps,
             imu1_gyro=imu1_gyro,
             imu1_accel=imu1_accel,
             imu2_gyro=imu2_gyro,
             imu2_accel=imu2_accel,
             imu1_quat=imu1_quat,
             imu2_quat=imu2_quat,
             imu1_gyro_bias=calibration.gyro_bias_imu1,
             imu2_gyro_bias=calibration.gyro_bias_imu2)
    
    print("[OK] Data saved!")
    
    # Load and verify
    print("\n[OK] Loading data from 'imu_data_example.npz'...")
    data = np.load('imu_data_example.npz')
    
    print(f"[OK] Loaded {len(data['timestamps'])} samples")
    print(f"  Duration: {data['timestamps'][-1] - data['timestamps'][0]:.2f} seconds")
    print(f"  IMU1 gyro shape: {data['imu1_gyro'].shape}")
    print(f"  IMU1 accel shape: {data['imu1_accel'].shape}")
    print(f"  IMU1 quat shape: {data['imu1_quat'].shape}")
    print(f"  IMU1 gyro bias: {data['imu1_gyro_bias']}")
    
    # Example: Plot data (if matplotlib available)
    try:
        import matplotlib.pyplot as plt
        
        print("\n[OK] Generating plots...")
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 8))
        
        # Gyro data
        axes[0].plot(data['timestamps'], data['imu1_gyro'])
        axes[0].set_ylabel('Gyro (deg/s)')
        axes[0].set_title('IMU1 Gyroscope Data')
        axes[0].legend(['X', 'Y', 'Z'])
        axes[0].grid(True)
        
        # Accel data
        axes[1].plot(data['timestamps'], data['imu1_accel'])
        axes[1].set_ylabel('Accel (g)')
        axes[1].set_title('IMU1 Accelerometer Data')
        axes[1].legend(['X', 'Y', 'Z'])
        axes[1].grid(True)
        
        # Quaternion data
        axes[2].plot(data['timestamps'], data['imu1_quat'])
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Quaternion')
        axes[2].set_title('IMU1 Orientation (Quaternion)')
        axes[2].legend(['W', 'X', 'Y', 'Z'])
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig('imu_data_plot.png', dpi=150)
        print("[OK] Saved plot to 'imu_data_plot.png'")
        
        # plt.show()  # Uncomment to display plot
        
    except ImportError:
        print("\n[INFO] matplotlib not available - skipping plots")
    
    print("\n[OK] Save/load example complete")


# =============================================================================
# MAIN MENU
# =============================================================================

def main():
    """Main menu for running examples."""
    print("\n" + "="*60)
    print("IMU ACQUISITION MODULE - USAGE EXAMPLES")
    print("="*60)
    print(f"\nSerial Port: {PORT}")
    print(f"Baud Rate: {BAUD}")
    print("\nIMPORTANT: Ensure your IMU device is connected before running examples!")
    
    examples = {
        '1': ('Basic Data Acquisition', example_1_basic_acquisition),
        '2': ('Calibration and Bias Correction', example_2_calibration),
        '3': ('Orientation Tracking', example_3_orientation_tracking),
        '4': ('Relative Orientation Between IMUs', example_4_relative_orientation),
        '5': ('Convenience Function', example_5_convenience_function),
        '6': ('Save and Load Data', example_6_save_load),
    }
    
    while True:
        print("\n" + "="*60)
        print("SELECT AN EXAMPLE:")
        print("="*60)
        for key, (name, _) in examples.items():
            print(f"  {key}. {name}")
        print("  q. Quit")
        
        choice = input("\nEnter your choice: ").strip().lower()
        
        if choice == 'q':
            print("\nGoodbye!")
            break
        
        if choice in examples:
            name, func = examples[choice]
            try:
                func()
                input("\nPress Enter to continue...")
            except KeyboardInterrupt:
                print("\n\n[WARN] Example interrupted by user")
            except Exception as e:
                print(f"\n[ERROR] Example failed: {e}")
                import traceback
                traceback.print_exc()
                input("\nPress Enter to continue...")
        else:
            print(f"\n[ERROR] Invalid choice: {choice}")


if __name__ == "__main__":
    main()
