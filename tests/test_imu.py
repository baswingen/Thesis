"""
Test script for IMU acquisition module.

This script tests the functionality of the imu_acquisition module including:
- Serial connection
- Data parsing
- Gyro calibration
- Orientation filtering
- Data acquisition

Usage:
    python src/test_imu.py
    
Or from project root:
    python -m src.test_imu

IMPORTANT: Update the PORT variable to match your serial port!
"""

import time
import numpy as np
from imu_acquisition import (
    IMUDevice,
    acquire_imu_data,
    IMUReading,
    IMUCalibration,
    MadgwickIMU,
    parse_imu_line,
    quat_to_euler,
    quat_mul,
    quat_inv
)


# =============================================================================
# CONFIGURATION
# =============================================================================

# IMPORTANT: Update this to your serial port!
PORT = "/dev/cu.usbmodem9888E00A0CC02"  # macOS/Linux
# PORT = "COM3"  # Windows

BAUD = 115200


# =============================================================================
# TEST FUNCTIONS
# =============================================================================

def test_parse_line():
    """Test line parsing function."""
    print("\n" + "="*60)
    print("TEST: Line Parsing")
    print("="*60)
    
    # Valid line
    line = "1234 | 0.12 -0.34 0.56 0.01 -0.02 0.98 | -0.15 0.22 -0.44 0.00 0.01 1.02"
    reading = parse_imu_line(line)
    
    if reading:
        print("[OK] Successfully parsed valid line")
        print(f"  Timestamp: {reading.timestamp_ms} ms")
        print(f"  IMU1 Gyro: {reading.imu1_gyro}")
        print(f"  IMU1 Accel: {reading.imu1_accel}")
        print(f"  IMU2 Gyro: {reading.imu2_gyro}")
        print(f"  IMU2 Accel: {reading.imu2_accel}")
    else:
        print("[FAIL] Failed to parse valid line")
        return False
    
    # Invalid lines
    invalid_lines = [
        "",  # Empty
        "t_ms | header | line",  # Header
        "1234 | 0.1 0.2",  # Too few parts
        "1234 | 0.1 0.2 0.3 | 0.4 0.5",  # Too few values
    ]
    
    for invalid_line in invalid_lines:
        reading = parse_imu_line(invalid_line)
        if reading is None:
            print(f"[OK] Correctly rejected: '{invalid_line[:40]}'")
        else:
            print(f"[FAIL] Should have rejected: '{invalid_line}'")
            return False
    
    print("\n[OK] All parsing tests passed")
    return True


def test_madgwick_filter():
    """Test Madgwick filter."""
    print("\n" + "="*60)
    print("TEST: Madgwick Filter")
    print("="*60)
    
    # Create filter
    filter = MadgwickIMU(beta=0.1)
    print(f"[OK] Created filter with beta=0.1")
    print(f"  Initial quaternion: {filter.get_quaternion()}")
    
    # Simulate stationary IMU (only gravity, no rotation)
    dt = 0.01
    gx, gy, gz = 0.0, 0.0, 0.0  # No rotation
    ax, ay, az = 0.0, 0.0, 1.0  # Gravity pointing up (Z-axis)
    
    # Run for 100 iterations
    for i in range(100):
        q = filter.update(gx, gy, gz, ax, ay, az, dt)
    
    print(f"[OK] Updated filter 100 times")
    print(f"  Final quaternion: {q}")
    
    # Get Euler angles
    roll, pitch, yaw = filter.get_euler()
    print(f"  Euler angles (deg): Roll={np.rad2deg(roll):.2f}, "
          f"Pitch={np.rad2deg(pitch):.2f}, Yaw={np.rad2deg(yaw):.2f}")
    
    # Should be close to identity (small angles)
    if abs(roll) < 0.1 and abs(pitch) < 0.1:
        print("[OK] Orientation is close to identity (as expected)")
    else:
        print("[WARN] Orientation deviated more than expected")
    
    # Reset and test with rotation
    filter.reset()
    print("\n[OK] Reset filter")
    
    # Simulate rotation around Z-axis (yaw)
    gz = np.deg2rad(10)  # 10 deg/s
    for i in range(100):
        filter.update(0, 0, gz, 0, 0, 1, dt)
    
    roll, pitch, yaw = filter.get_euler()
    print(f"[OK] After simulated rotation:")
    print(f"  Yaw: {np.rad2deg(yaw):.2f}° (expected ~10°)")
    
    print("\n[OK] Madgwick filter tests passed")
    return True


def test_connection(port, baud):
    """Test serial connection."""
    print("\n" + "="*60)
    print("TEST: Serial Connection")
    print("="*60)
    
    try:
        device = IMUDevice(port=port, baud=baud)
        print(f"[OK] Created IMU device (port={port}, baud={baud})")
        
        device.connect()
        print("[OK] Connected successfully")
        
        # Check connection status
        if device.is_connected():
            print("[OK] Connection status verified")
        else:
            print("[FAIL] Device reports not connected")
            return False
        
        # Read a few samples to verify data flow
        print("\n[OK] Reading 5 test samples...")
        for i in range(5):
            reading = device.read_sample()
            if reading:
                print(f"  Sample {i+1}: t={reading.timestamp_ms}ms, "
                      f"IMU1_gyro={reading.imu1_gyro[:2]}...")
        
        device.disconnect()
        print("\n[OK] Disconnected successfully")
        
        if not device.is_connected():
            print("[OK] Connection status verified (disconnected)")
        else:
            print("[FAIL] Device still reports connected")
            return False
        
        print("\n[OK] Connection tests passed")
        return True
    
    except Exception as e:
        print(f"\n[FAIL] Connection test failed: {e}")
        return False


def test_calibration(port, baud):
    """Test gyro calibration."""
    print("\n" + "="*60)
    print("TEST: Gyro Calibration")
    print("="*60)
    
    print("\nIMPORTANT: Place both IMUs FLAT on a surface and keep them STILL")
    print("Starting calibration in 3 seconds...")
    time.sleep(3)
    
    try:
        device = IMUDevice(port=port, baud=baud)
        device.connect()
        
        # Calibrate with fewer samples for faster testing
        calibration = device.calibrate_gyro(num_samples=50)
        
        print("\n[OK] Calibration completed")
        print(f"  IMU1 bias: {calibration.gyro_bias_imu1}")
        print(f"  IMU2 bias: {calibration.gyro_bias_imu2}")
        print(f"  Samples: {calibration.num_samples}")
        
        # Verify calibration is reasonable (bias should be small)
        bias1_mag = np.linalg.norm(calibration.gyro_bias_imu1)
        bias2_mag = np.linalg.norm(calibration.gyro_bias_imu2)
        
        if bias1_mag < 5.0 and bias2_mag < 5.0:
            print("[OK] Bias magnitudes are reasonable (< 5 deg/s)")
        else:
            print(f"[WARN] Bias magnitudes seem high: IMU1={bias1_mag:.2f}, IMU2={bias2_mag:.2f}")
        
        # Test get/set calibration
        retrieved = device.get_calibration()
        if retrieved is not None:
            print("[OK] Retrieved calibration from device")
        else:
            print("[FAIL] Failed to retrieve calibration")
            device.disconnect()
            return False
        
        # Create new device and set calibration
        device.disconnect()
        device2 = IMUDevice(port=port, baud=baud)
        device2.connect()
        device2.set_calibration(calibration)
        
        if device2.get_calibration() is not None:
            print("[OK] Set calibration on new device instance")
        else:
            print("[FAIL] Failed to set calibration")
            device2.disconnect()
            return False
        
        device2.disconnect()
        
        print("\n[OK] Calibration tests passed")
        return True
    
    except Exception as e:
        print(f"\n[FAIL] Calibration test failed: {e}")
        return False


def test_orientation_filter(port, baud):
    """Test orientation filtering."""
    print("\n" + "="*60)
    print("TEST: Orientation Filter")
    print("="*60)
    
    try:
        device = IMUDevice(port=port, baud=baud)
        device.connect()
        
        # Enable filter
        device.enable_orientation_filter(beta=0.08)
        print("[OK] Orientation filter enabled")
        
        # Check that filters were created
        orientation = device.get_orientation()
        if orientation is not None:
            q1, q2 = orientation
            print(f"[OK] Got initial quaternions:")
            print(f"  IMU1: {q1}")
            print(f"  IMU2: {q2}")
        else:
            print("[FAIL] Failed to get orientation")
            device.disconnect()
            return False
        
        # Read samples with orientation
        print("\n[OK] Reading 10 samples with orientation...")
        for i in range(10):
            data = device.read_sample_with_orientation()
            if data and 'imu1_euler' in data:
                euler1 = np.rad2deg(data['imu1_euler'])
                euler2 = np.rad2deg(data['imu2_euler'])
                print(f"  Sample {i+1}: IMU1 Roll={euler1[0]:6.1f}°, "
                      f"Pitch={euler1[1]:6.1f}°, Yaw={euler1[2]:6.1f}°")
            time.sleep(0.05)
        
        # Get Euler angles
        euler = device.get_euler_angles()
        if euler is not None:
            euler1, euler2 = euler
            print(f"\n[OK] Got Euler angles:")
            print(f"  IMU1: Roll={np.rad2deg(euler1[0]):.1f}°, "
                  f"Pitch={np.rad2deg(euler1[1]):.1f}°, "
                  f"Yaw={np.rad2deg(euler1[2]):.1f}°")
        else:
            print("[FAIL] Failed to get Euler angles")
            device.disconnect()
            return False
        
        # Disable filter
        device.disable_orientation_filter()
        print("\n[OK] Orientation filter disabled")
        
        # Verify filter is disabled
        orientation = device.get_orientation()
        if orientation is None:
            print("[OK] Confirmed filter is disabled")
        else:
            print("[FAIL] Filter still active after disable")
            device.disconnect()
            return False
        
        device.disconnect()
        
        print("\n[OK] Orientation filter tests passed")
        return True
    
    except Exception as e:
        print(f"\n[FAIL] Orientation filter test failed: {e}")
        return False


def test_acquire_function(port, baud):
    """Test convenience acquire function."""
    print("\n" + "="*60)
    print("TEST: Acquire Function")
    print("="*60)
    
    print("\nTesting short acquisition (5 seconds)...")
    print("IMPORTANT: Place IMUs flat for calibration (starting in 3s)")
    time.sleep(3)
    
    try:
        readings, calibration = acquire_imu_data(
            port=port,
            duration=5.0,
            baud=baud,
            calibrate=True,
            enable_filter=True,
            filter_beta=0.08
        )
        
        if len(readings) > 0:
            print(f"\n[OK] Acquired {len(readings)} samples")
            
            # Check first sample
            first = readings[0]
            if isinstance(first, dict):
                print("[OK] Data includes orientation information")
                reading = first['reading']
                print(f"  First sample timestamp: {reading.timestamp_ms} ms")
                if 'imu1_euler' in first:
                    euler = np.rad2deg(first['imu1_euler'])
                    print(f"  First sample Euler: Roll={euler[0]:.1f}°, "
                          f"Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
            else:
                print("[OK] Data is raw readings")
            
            # Check calibration
            if calibration is not None:
                print(f"[OK] Calibration data returned")
                print(f"  IMU1 bias magnitude: {np.linalg.norm(calibration.gyro_bias_imu1):.2f} deg/s")
            else:
                print("[FAIL] No calibration data returned")
                return False
            
            print("\n[OK] Acquire function tests passed")
            return True
        else:
            print("\n[FAIL] No data acquired")
            return False
    
    except Exception as e:
        print(f"\n[FAIL] Acquire function test failed: {e}")
        return False


# =============================================================================
# MAIN TEST RUNNER
# =============================================================================

def run_all_tests():
    """Run all test functions."""
    print("\n" + "="*60)
    print("IMU ACQUISITION MODULE - TEST SUITE")
    print("="*60)
    print(f"\nSerial Port: {PORT}")
    print(f"Baud Rate: {BAUD}")
    
    results = {}
    
    # Test 1: Line parsing (no hardware required)
    results['parse'] = test_parse_line()
    
    # Test 2: Madgwick filter (no hardware required)
    results['madgwick'] = test_madgwick_filter()
    
    # Tests requiring hardware
    print("\n" + "="*60)
    print("HARDWARE TESTS")
    print("="*60)
    print("\nThe following tests require a connected IMU device.")
    print("Make sure your device is connected and the PORT is correct.")
    
    response = input("\nRun hardware tests? (y/n): ").strip().lower()
    
    if response == 'y':
        results['connection'] = test_connection(PORT, BAUD)
        
        if results['connection']:
            results['calibration'] = test_calibration(PORT, BAUD)
            results['orientation'] = test_orientation_filter(PORT, BAUD)
            results['acquire'] = test_acquire_function(PORT, BAUD)
        else:
            print("\n[SKIP] Skipping remaining tests due to connection failure")
            results['calibration'] = None
            results['orientation'] = None
            results['acquire'] = None
    else:
        print("\n[SKIP] Hardware tests skipped by user")
        results['connection'] = None
        results['calibration'] = None
        results['orientation'] = None
        results['acquire'] = None
    
    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    for test_name, result in results.items():
        if result is True:
            status = "[PASS]"
        elif result is False:
            status = "[FAIL]"
        else:
            status = "[SKIP]"
        print(f"{status} {test_name}")
    
    # Overall result
    passed = sum(1 for r in results.values() if r is True)
    failed = sum(1 for r in results.values() if r is False)
    skipped = sum(1 for r in results.values() if r is None)
    
    print(f"\nTotal: {passed} passed, {failed} failed, {skipped} skipped")
    
    if failed == 0:
        print("\n[OK] All tests passed!")
        return True
    else:
        print(f"\n[FAIL] {failed} test(s) failed")
        return False


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    print("\n" + "="*60)
    print("IMU ACQUISITION MODULE - TEST SCRIPT")
    print("="*60)
    print("\nThis script will test the IMU acquisition module.")
    print("\nBefore running hardware tests, ensure:")
    print("  1. Your IMU device is connected via USB")
    print("  2. The PORT variable is set correctly (currently: " + PORT + ")")
    print("  3. The device is powered on and transmitting data")
    
    input("\nPress Enter to start tests...")
    
    success = run_all_tests()
    
    print("\n" + "="*60)
    if success:
        print("ALL TESTS COMPLETED SUCCESSFULLY")
    else:
        print("SOME TESTS FAILED - CHECK OUTPUT ABOVE")
    print("="*60)
    print()
