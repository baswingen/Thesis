"""
Simple import test for bmi160_dual module.
Run this to verify the module is installed correctly.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

def test_imports():
    """Test that all imports work."""
    print("Testing bmi160_dual module imports...")
    
    try:
        from src.bmi160_dual import (
            DualIMU,
            IMUConfig,
            IMUReading,
            RawSample,
            CalibrationData,
            MahonyFilter,
            quick_read,
            quat_mul,
            quat_conj,
            quat_norm,
            quat_to_euler
        )
        print("✓ All imports successful")
        return True
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False


def test_config_creation():
    """Test IMUConfig creation."""
    print("\nTesting IMUConfig creation...")
    
    try:
        from src.bmi160_dual import IMUConfig
        
        # Default config
        config1 = IMUConfig()
        print(f"  ✓ Default config: baud={config1.baud}, use_mahony={config1.use_mahony}")
        
        # Custom config
        config2 = IMUConfig(
            port='/dev/ttyUSB0',
            baud=115200,
            kp=1.5,
            ki=0.005
        )
        print(f"  ✓ Custom config: port={config2.port}, kp={config2.kp}")
        
        return True
    except Exception as e:
        print(f"  ❌ Config creation failed: {e}")
        return False


def test_filter_creation():
    """Test MahonyFilter creation."""
    print("\nTesting MahonyFilter creation...")
    
    try:
        from src.bmi160_dual import MahonyFilter
        import numpy as np
        
        filter = MahonyFilter(kp=2.0, ki=0.01)
        print(f"  ✓ Filter created: q={filter.q}")
        
        # Test update (without sensor, just check it runs)
        gyro = np.array([0.0, 0.0, 0.0])
        accel = np.array([0.0, 0.0, -1.0])
        q = filter.update(gyro, accel, dt=0.01)
        print(f"  ✓ Filter update: q={q}")
        
        return True
    except Exception as e:
        print(f"  ❌ Filter test failed: {e}")
        return False


def test_quaternion_math():
    """Test quaternion operations."""
    print("\nTesting quaternion math...")
    
    try:
        from src.bmi160_dual import quat_mul, quat_conj, quat_norm, quat_to_euler
        import numpy as np
        
        q1 = np.array([1.0, 0.0, 0.0, 0.0])
        q2 = np.array([0.707, 0.707, 0.0, 0.0])
        
        q_prod = quat_mul(q1, q2)
        print(f"  ✓ Quaternion multiply: {q_prod}")
        
        q_conj = quat_conj(q2)
        print(f"  ✓ Quaternion conjugate: {q_conj}")
        
        q_norm = quat_norm(q2)
        print(f"  ✓ Quaternion normalize: norm={np.linalg.norm(q_norm):.6f}")
        
        euler = quat_to_euler(q2)
        print(f"  ✓ Quaternion to Euler: {euler}")
        
        return True
    except Exception as e:
        print(f"  ❌ Quaternion math failed: {e}")
        return False


def test_protocol_parsing():
    """Test ASCII protocol parsing."""
    print("\nTesting protocol parsing...")
    
    try:
        from src.bmi160_dual import parse_line, xor_checksum
        
        # Create valid test line
        payload = "123,456789,1000,2000,3000,100,200,300,1,1100,2200,3300,110,220,330,1"
        chk = xor_checksum(payload)
        line = f"${payload}*{chk:02X}"
        
        print(f"  Test line: {line[:40]}...")
        
        sample = parse_line(line)
        if sample:
            print(f"  ✓ Parsed: seq={sample.seq}, t_us={sample.t_us}, ok1={sample.ok1}")
            return True
        else:
            print(f"  ❌ Parse failed")
            return False
    except Exception as e:
        print(f"  ❌ Protocol test failed: {e}")
        return False


def test_package_import():
    """Test importing from main package."""
    print("\nTesting package-level imports...")
    
    try:
        from src import (
            DualIMU,
            IMUConfig,
            IMUReading,
            quick_read
        )
        print("  ✓ Package imports successful")
        return True
    except ImportError as e:
        print(f"  ❌ Package import failed: {e}")
        return False


if __name__ == "__main__":
    print("="*60)
    print("BMI160 Dual Module Import Tests")
    print("="*60)
    
    tests = [
        test_imports,
        test_config_creation,
        test_filter_creation,
        test_quaternion_math,
        test_protocol_parsing,
        test_package_import
    ]
    
    results = []
    for test in tests:
        result = test()
        results.append(result)
    
    print("\n" + "="*60)
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✓ All tests passed ({passed}/{total})")
        print("\nModule is ready to use!")
    else:
        print(f"⚠ Some tests failed ({passed}/{total} passed)")
        print("\nPlease check the errors above.")
    
    print("="*60)
