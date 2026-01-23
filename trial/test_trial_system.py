"""
Trial System Integration Test
==============================
Test the trial data collection system in mock mode (no hardware required).

This script tests:
- Configuration loading
- Protocol expansion
- Data storage (HDF5)
- GUI initialization
- Mock data generation

Usage:
    python trial/test_trial_system.py
"""

import sys
import numpy as np
from pathlib import Path
import time

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from trial import setup_trial
from trial.data_storage import TrialDataStorage
from trial.trial_protocols import expand_protocol, print_protocol_summary
from trial.trial_gui import TrialGUI, TrialState


def test_configuration():
    """Test configuration loading and validation."""
    print("\n" + "="*70)
    print("TEST 1: Configuration")
    print("="*70)
    
    try:
        setup_trial.validate_config()
        print("✓ Configuration valid")
        
        # Print summary
        setup_trial.print_config_summary()
        
        return True
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        return False


def test_protocols():
    """Test protocol loading and expansion."""
    print("\n" + "="*70)
    print("TEST 2: Protocols")
    print("="*70)
    
    try:
        # Load protocol
        protocol = setup_trial.TRIAL_EXERCISES
        print(f"\n✓ Loaded protocol with {len(protocol)} exercises")
        
        # Expand protocol
        expanded = expand_protocol(protocol)
        print(f"✓ Expanded to {len(expanded)} trials (with repetitions)")
        
        # Print first few trials
        print("\nFirst 3 trials:")
        for i, trial in enumerate(expanded[:3], 1):
            print(f"  {i}. {trial['display_name']} ({trial['duration']}s)")
        
        return True
    except Exception as e:
        print(f"✗ Protocol test failed: {e}")
        return False


def test_data_storage():
    """Test HDF5 data storage."""
    print("\n" + "="*70)
    print("TEST 3: Data Storage")
    print("="*70)
    
    try:
        # Create test directory
        test_dir = Path("./database/test_participant/test_session")
        test_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize storage
        storage = TrialDataStorage(test_dir, compression='gzip')
        print("✓ Storage initialized")
        
        # Set metadata
        storage.set_session_metadata(
            participant_info={'id': 'TEST', 'age': 25},
            session_info={'id': 'TEST_S001'},
            emg_config={'sample_rate': 2048, 'channels': [0, 1, 2, 3]},
            imu_config={'sample_rate': 200},
            preprocessing_config={'bandpass_low': 20.0, 'bandpass_high': 450.0}
        )
        
        # Generate mock data
        print("\nGenerating mock data...")
        n_emg_samples = 2048 * 5  # 5 seconds at 2048 Hz
        n_imu_samples = 200 * 5   # 5 seconds at 200 Hz
        
        emg_data = {
            'timestamps': np.linspace(0, 5, n_emg_samples),
            'raw': np.random.randn(n_emg_samples, 4) * 100,
            'sample_rate': 2048,
            'channel_names': ['CH1', 'CH2', 'CH3', 'CH4']
        }
        
        imu_data = {
            'timestamps': np.linspace(0, 5, n_imu_samples),
            'accel1': np.random.randn(n_imu_samples, 3) * 0.1 + np.array([0, 0, 1]),
            'gyro1': np.random.randn(n_imu_samples, 3) * 0.5,
            'accel2': np.random.randn(n_imu_samples, 3) * 0.1 + np.array([0, 0, 1]),
            'gyro2': np.random.randn(n_imu_samples, 3) * 0.5,
            'sample_rate': 200,
        }
        
        sync_info = {
            'method': 'mock',
            'emg_samples': n_emg_samples,
            'imu_samples': n_imu_samples
        }
        
        exercise_info = {
            'name': 'Test_Exercise',
            'duration': 5.0,
            'instruction': 'Test instruction'
        }
        
        # Save raw data
        print("Saving raw data...")
        raw_file = storage.save_trial_raw(
            trial_number=1,
            exercise_name='Test_Exercise',
            exercise_info=exercise_info,
            emg_data=emg_data,
            imu_data=imu_data,
            sync_info=sync_info
        )
        print(f"✓ Raw data saved: {raw_file}")
        
        # Save preprocessed data (mock)
        print("Saving preprocessed data...")
        emg_preprocessed = {
            'timestamps': emg_data['timestamps'],
            'filtered': emg_data['raw'],  # Mock: use raw as filtered
            'envelope': np.abs(emg_data['raw']),  # Mock: abs as envelope
            'sample_rate': 2048,
            'channel_names': ['CH1', 'CH2', 'CH3', 'CH4']
        }
        
        prep_file = storage.save_trial_preprocessed(
            trial_number=1,
            exercise_name='Test_Exercise',
            emg_preprocessed=emg_preprocessed,
            imu_data=None
        )
        print(f"✓ Preprocessed data saved: {prep_file}")
        
        # Verify files exist
        assert raw_file.exists(), "Raw file not created"
        assert prep_file.exists(), "Preprocessed file not created"
        print(f"\n✓ Files verified:")
        print(f"  Raw: {raw_file.stat().st_size / 1024:.1f} KB")
        print(f"  Preprocessed: {prep_file.stat().st_size / 1024:.1f} KB")
        
        # Test loading
        print("\nTesting data loading...")
        loaded = TrialDataStorage.load_trial_raw(raw_file)
        assert loaded['emg']['raw'].shape == (n_emg_samples, 4), "EMG shape mismatch"
        assert loaded['imu']['accel1'].shape == (n_imu_samples, 3), "IMU shape mismatch"
        print("✓ Data loaded successfully")
        
        return True
        
    except Exception as e:
        print(f"✗ Data storage test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_gui_creation():
    """Test GUI creation (but don't run main loop)."""
    print("\n" + "="*70)
    print("TEST 4: GUI Creation")
    print("="*70)
    
    try:
        # Create GUI with custom callback
        def on_key(key):
            print(f"  Key pressed: {key}")
        
        gui = TrialGUI(on_key_press=on_key)
        print("✓ GUI created successfully")
        
        # Test state changes
        print("\nTesting state changes...")
        gui.set_state(TrialState.READY)
        print("  ✓ Set to READY state")
        
        gui.set_exercise("Test Exercise", "Test instruction")
        print("  ✓ Set exercise info")
        
        gui.set_trial_info(1, 10)
        print("  ✓ Set trial counter")
        
        gui.set_progress(0.5)
        print("  ✓ Set progress bar")
        
        # Test data updates
        print("\nTesting data updates...")
        timestamps = np.linspace(0, 1, 100)
        emg_data = np.random.randn(100, 4) * 100
        gui.update_emg_data(timestamps, emg_data)
        print("  ✓ EMG data updated")
        
        imu_accel = np.random.randn(50, 3)
        gui.update_imu_data(timestamps[:50], imu_accel)
        print("  ✓ IMU data updated")
        
        # Close GUI (don't run main loop)
        gui.close()
        print("\n✓ GUI test complete (window closed)")
        
        return True
        
    except Exception as e:
        print(f"✗ GUI test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all integration tests."""
    print("\n" + "="*70)
    print("TRIAL SYSTEM INTEGRATION TESTS")
    print("="*70)
    print("\nRunning tests without hardware (mock mode)...")
    
    results = {
        'Configuration': test_configuration(),
        'Protocols': test_protocols(),
        'Data Storage': test_data_storage(),
        'GUI Creation': test_gui_creation(),
    }
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test_name:20s}: {status}")
    
    total = len(results)
    passed = sum(results.values())
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All tests passed! System is ready to use.")
        print("\nTo run with hardware:")
        print("  python -m trial.trial_manager")
        print("\nTo run in mock mode:")
        print("  python -m trial.trial_manager --mock")
    else:
        print("\n⚠ Some tests failed. Please review errors above.")
    
    return passed == total


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
