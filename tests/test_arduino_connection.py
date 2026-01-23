"""
Test and demonstrate the Arduino connection module.

This script shows how to use the arduino_connection module to
detect and connect to Arduino boards across different platforms.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.arduino_connection import (
    find_arduino_port,
    open_arduino_serial,
    list_all_ports,
    is_port_available
)


def test_list_ports():
    """Test listing all available serial ports"""
    print("\n" + "=" * 60)
    print("TEST 1: List all available serial ports")
    print("=" * 60)
    
    ports = list_all_ports()
    print(f"\nFound {len(ports)} port(s):")
    for port in ports:
        available = "✓" if is_port_available(port) else "✗"
        print(f"  {available} {port}")
    
    # List with details
    print("\nDetailed information:")
    ports_detailed = list_all_ports(include_details=True)
    for p in ports_detailed:
        vid_str = f"VID: {p.vid:#06x}" if p.vid else "VID: N/A"
        pid_str = f"PID: {p.pid:#06x}" if p.pid else "PID: N/A"
        print(f"\n  {p.device}")
        print(f"    Description: {p.description}")
        print(f"    Manufacturer: {p.manufacturer or 'N/A'}")
        print(f"    {vid_str}, {pid_str}")


def test_find_arduino():
    """Test auto-detection of Arduino"""
    print("\n" + "=" * 60)
    print("TEST 2: Auto-detect Arduino")
    print("=" * 60)
    
    port = find_arduino_port(verbose=True)
    
    if port:
        print(f"\n✓ SUCCESS: Arduino found on {port}")
        return port
    else:
        print("\n✗ FAILED: No Arduino detected")
        print("  Make sure your Arduino is connected and drivers are installed.")
        return None


def test_connect_arduino(port=None):
    """Test opening connection to Arduino"""
    print("\n" + "=" * 60)
    print("TEST 3: Connect to Arduino")
    print("=" * 60)
    
    try:
        if port:
            print(f"\nAttempting to connect to {port}...")
            ser = open_arduino_serial(
                port=port,
                baud=115200,
                wait_for_ready=True,
                verbose=True
            )
        else:
            print("\nAttempting auto-detect and connect...")
            ser = open_arduino_serial(
                port=None,
                baud=115200,
                wait_for_ready=True,
                verbose=True
            )
        
        print(f"\n✓ SUCCESS: Connected to Arduino on {ser.port}")
        print(f"  Baud rate: {ser.baudrate}")
        print(f"  Timeout: {ser.timeout}s")
        
        # Try to read a few lines
        print("\nReading first few lines from Arduino...")
        for i in range(5):
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    print(f"  [{i+1}] {line}")
            except Exception as e:
                print(f"  [{i+1}] Error reading: {e}")
        
        ser.close()
        print("\n✓ Connection closed successfully")
        return True
        
    except Exception as e:
        print(f"\n✗ FAILED: {e}")
        return False


def main():
    """Run all tests"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║  Arduino Connection Module - Test Suite" + " " * 17 + "║")
    print("╚" + "=" * 58 + "╝")
    
    # Test 1: List ports
    test_list_ports()
    
    # Test 2: Find Arduino
    detected_port = test_find_arduino()
    
    # Test 3: Connect to Arduino (skip if none found)
    if detected_port:
        user_input = input("\n\nWould you like to test connecting to the Arduino? (y/n): ").lower()
        if user_input == 'y':
            test_connect_arduino(detected_port)
        else:
            print("\nSkipping connection test.")
    else:
        print("\nSkipping connection test (no Arduino detected).")
    
    print("\n" + "=" * 60)
    print("All tests complete!")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
