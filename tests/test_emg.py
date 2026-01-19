"""
Quick test script for EMG acquisition module.
"""

from emg_acquisition import EMGDevice, acquire_emg_data


def test_basic_connection():
    """Test 1: Basic device connection"""
    print("="*60)
    print("Test 1: Device Connection")
    print("="*60)
    
    try:
        emg = EMGDevice(connection_type='usb')
        device_info = emg.connect()
        
        print(f"\n[OK] Connected successfully!")
        print(f"  Device: {device_info['name']}")
        print(f"  Channels: {device_info['num_channels']}")
        print(f"  Sample rate: {device_info['sample_rate']} Hz")
        
        emg.disconnect()
        print("[OK] Disconnected successfully!")
        return True
        
    except Exception as e:
        print(f"[FAIL] Connection test failed: {e}")
        return False


def test_short_acquisition():
    """Test 2: Short data acquisition"""
    print("\n" + "="*60)
    print("Test 2: Short Data Acquisition (3 seconds)")
    print("="*60)
    
    try:
        emg = EMGDevice()
        emg.connect()
        
        print("\nAcquiring data...")
        emg.start_acquisition(duration=3.0)
        
        data = emg.get_data()
        
        if data is not None:
            print(f"\n[OK] Acquired data successfully!")
            print(f"  Shape: {data.shape}")
            print(f"  Duration: {data.shape[0] / emg.sample_rate:.2f} seconds")
            print(f"  Data range: [{data.min():.2f}, {data.max():.2f}]")
        else:
            print("[FAIL] No data acquired")
            emg.disconnect()
            return False
        
        emg.disconnect()
        return True
        
    except Exception as e:
        print(f"[FAIL] Acquisition test failed: {e}")
        return False


def test_export():
    """Test 3: Data export"""
    print("\n" + "="*60)
    print("Test 3: Data Export")
    print("="*60)
    
    try:
        # Quick acquisition with export
        data, info = acquire_emg_data(
            duration=2.0,
            export_to='test_emg_data.npy'
        )
        
        print(f"\n[OK] Data exported successfully!")
        print(f"  File: test_emg_data.npy")
        print(f"  Size: {data.shape}")
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Export test failed: {e}")
        return False


def test_channel_info():
    """Test 4: Channel information"""
    print("\n" + "="*60)
    print("Test 4: Channel Information")
    print("="*60)
    
    try:
        with EMGDevice() as emg:
            emg.connect()
            
            channels = emg.get_channel_names()
            print(f"\n[OK] Retrieved {len(channels)} channels")
            print("\nFirst 5 channels:")
            for i, name in enumerate(channels[:5]):
                print(f"  [{i}] {name}")
            
            return True
            
    except Exception as e:
        print(f"[FAIL] Channel info test failed: {e}")
        return False


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print(" EMG ACQUISITION MODULE - TEST SUITE")
    print("="*70)
    
    tests = [
        test_basic_connection,
        test_short_acquisition,
        test_export,
        test_channel_info
    ]
    
    results = []
    
    for test_func in tests:
        try:
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"\nUnexpected error in {test_func.__name__}: {e}")
            results.append(False)
        
        print()
    
    # Summary
    print("="*70)
    print(" TEST SUMMARY")
    print("="*70)
    
    passed = sum(results)
    total = len(results)
    
    print(f"\nPassed: {passed}/{total} tests")
    
    if passed == total:
        print("\n[SUCCESS] All tests passed! ✓")
    else:
        print(f"\n[PARTIAL] {total - passed} test(s) failed")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    except Exception as e:
        print(f"\n\nFatal error: {e}")
        import traceback
        traceback.print_exc()
