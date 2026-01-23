"""
Simple Trial System Test (No GUI)
===================================
Basic test of trial system components without matplotlib/GUI dependencies.

Usage:
    python trial/test_simple.py
"""

import sys
import numpy as np
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_imports():
    """Test basic imports."""
    print("\n" + "="*70)
    print("TEST 1: Module Imports")
    print("="*70)
    
    try:
        from trial import setup_trial
        print("[OK] setup_trial imported")
        
        from trial.data_storage import TrialDataStorage
        print("[OK] data_storage imported")
        
        from trial.trial_protocols import get_protocol, expand_protocol
        print("[OK] trial_protocols imported")
        
        return True
    except Exception as e:
        print(f"[FAIL] Import test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_configuration():
    """Test configuration."""
    print("\n" + "="*70)
    print("TEST 2: Configuration")
    print("="*70)
    
    try:
        from trial import setup_trial
        
        # Validate config
        setup_trial.validate_config()
        print("[OK] Configuration valid")
        
        # Test helper functions
        session_dir = setup_trial.get_session_dir()
        print(f"[OK] Session directory: {session_dir}")
        
        return True
    except Exception as e:
        print(f"[FAIL] Configuration test failed: {e}")
        return False


def test_protocols():
    """Test protocol system."""
    print("\n" + "="*70)
    print("TEST 3: Protocols")
    print("="*70)
    
    try:
        from trial.trial_protocols import (
            get_protocol, 
            list_protocols, 
            expand_protocol,
            validate_protocol
        )
        
        # List protocols
        protocols = list_protocols()
        print(f"[OK] Available protocols: {', '.join(protocols)}")
        
        # Load basic protocol
        protocol = get_protocol('basic')
        print(f"[OK] Loaded 'basic' protocol: {len(protocol)} exercises")
        
        # Validate
        validate_protocol(protocol)
        print("[OK] Protocol validated")
        
        # Expand
        expanded = expand_protocol(protocol)
        print(f"[OK] Expanded to {len(expanded)} trials")
        
        return True
    except Exception as e:
        print(f"[FAIL] Protocol test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_data_storage():
    """Test HDF5 storage."""
    print("\n" + "="*70)
    print("TEST 4: Data Storage")
    print("="*70)
    
    try:
        from trial.data_storage import TrialDataStorage
        
        # Create test directory
        test_dir = Path("./database/test/simple_test")
        test_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize storage
        storage = TrialDataStorage(test_dir)
        print("[OK] Storage initialized")
        
        # Set metadata
        storage.set_session_metadata(
            participant_info={'id': 'TEST', 'age': 25},
            session_info={'id': 'TEST'},
            emg_config={'sample_rate': 2048},
            imu_config={'sample_rate': 200},
            preprocessing_config={'bandpass_low': 20.0}
        )
        print("[OK] Metadata set")
        
        # Generate mock data
        n_emg = 2048 * 2  # 2 seconds
        n_imu = 200 * 2
        
        emg_data = {
            'timestamps': np.linspace(0, 2, n_emg),
            'raw': np.random.randn(n_emg, 4) * 50,
            'sample_rate': 2048,
            'channel_names': ['CH1', 'CH2', 'CH3', 'CH4']
        }
        
        imu_data = {
            'timestamps': np.linspace(0, 2, n_imu),
            'accel1': np.random.randn(n_imu, 3),
            'gyro1': np.random.randn(n_imu, 3),
            'accel2': np.random.randn(n_imu, 3),
            'gyro2': np.random.randn(n_imu, 3),
            'sample_rate': 200
        }
        
        # Save data
        print("[INFO] Saving test data...")
        raw_file = storage.save_trial_raw(
            trial_number=1,
            exercise_name='Test',
            exercise_info={'name': 'Test', 'duration': 2.0},
            emg_data=emg_data,
            imu_data=imu_data,
            sync_info={'method': 'test'}
        )
        
        print(f"[OK] Data saved: {raw_file.name}")
        print(f"[INFO] File size: {raw_file.stat().st_size / 1024:.1f} KB")
        
        # Test loading
        print("[INFO] Loading data...")
        loaded = TrialDataStorage.load_trial_raw(raw_file)
        
        assert loaded['emg']['raw'].shape == (n_emg, 4), "EMG shape mismatch"
        assert loaded['imu']['accel1'].shape == (n_imu, 3), "IMU shape mismatch"
        print("[OK] Data loaded and verified")
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Storage test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_tests():
    """Run all tests."""
    print("\n" + "="*70)
    print("TRIAL SYSTEM - SIMPLE TESTS")
    print("="*70)
    print("\nTesting core components (no GUI)...")
    
    results = {
        'Imports': test_imports(),
        'Configuration': test_configuration(),
        'Protocols': test_protocols(),
        'Data Storage': test_data_storage(),
    }
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    for test_name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{test_name:20s}: {status}")
    
    total = len(results)
    passed = sum(results.values())
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n[SUCCESS] Core components working!")
        print("\nNext steps:")
        print("  1. Install dependencies: pip install -r requirements.txt")
        print("  2. Run full test: python trial/test_trial_system.py")
        print("  3. Run trial manager: python -m trial.trial_manager --mock")
    else:
        print("\n[WARNING] Some tests failed. Review errors above.")
    
    return passed == total


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
