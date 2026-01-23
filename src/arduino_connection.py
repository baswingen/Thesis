"""
Arduino Connection Utilities
=============================

Cross-platform Arduino serial port detection and connection management.
Works seamlessly on Windows, macOS, and Linux.

Usage:
    from src.arduino_connection import find_arduino_port, open_arduino_serial
    
    # Auto-detect and connect
    ser = open_arduino_serial(baud=230400)
    
    # Or just find the port
    port = find_arduino_port()
    if port:
        print(f"Arduino found on {port}")
"""

import time
import serial
from serial.tools import list_ports
from typing import Optional


def find_arduino_port(preferred_substr: Optional[str] = None, verbose: bool = True) -> Optional[str]:
    """
    Auto-detect Arduino serial port across Windows, macOS, and Linux.
    
    Detection priority:
    1. Preferred substring match (if provided)
    2. Arduino vendor ID (0x2341) and known descriptions
    3. Common USB-Serial chip identifiers (CH340, CP210x, FTDI)
    4. Generic USB serial ports
    5. First available port (fallback)
    
    Parameters
    ----------
    preferred_substr : str, optional
        Preferred substring to match in port device name or description
        (e.g., "usbmodem", "COM3", "arduino")
    verbose : bool, default=True
        If True, prints scanning information and found ports
    
    Returns
    -------
    str or None
        Device path of detected Arduino port (e.g., "COM3", "/dev/cu.usbmodem123")
        Returns None if no ports are available
    
    Examples
    --------
    >>> port = find_arduino_port()
    Scanning 3 available serial port(s)...
      - COM3: USB Serial Port (COM3)
      - COM5: Arduino Uno (COM5)
    'COM5'
    
    >>> port = find_arduino_port(preferred_substr="usbmodem")
    '/dev/cu.usbmodem14201'
    """
    ports = list(list_ports.comports())
    if not ports:
        if verbose:
            print("⚠ No serial ports found")
        return None

    if verbose:
        print(f"\nScanning {len(ports)} available serial port(s)...")
        for p in ports:
            print(f"  - {p.device}: {p.description}")

    # Priority 1: Preferred substring match
    if preferred_substr:
        preferred_substr = preferred_substr.lower()
        for p in ports:
            device_lower = (p.device or "").lower()
            desc_lower = (p.description or "").lower()
            if preferred_substr in device_lower or preferred_substr in desc_lower:
                if verbose:
                    print(f"✓ Matched preferred substring '{preferred_substr}': {p.device}")
                return p.device

    # Priority 2: Arduino-specific identifiers
    # Check for Arduino vendor IDs and common Arduino descriptions
    for p in ports:
        desc_lower = (p.description or "").lower()
        
        # Check vendor ID (0x2341 is Arduino's official VID)
        if p.vid == 0x2341:  # Arduino vendor ID
            if verbose:
                print(f"✓ Arduino detected by vendor ID: {p.device}")
            return p.device
        
        # Arduino keywords in description
        if any(keyword in desc_lower for keyword in [
            "arduino", "uno", "mega", "nano", "due", "leonardo"
        ]):
            if verbose:
                print(f"✓ Arduino detected by description: {p.device}")
            return p.device

    # Priority 3: Common USB-Serial chips used in Arduinos
    for p in ports:
        desc_lower = (p.description or "").lower()
        if any(chip in desc_lower for chip in [
            "ch340",  # Common USB-Serial chip on Arduino clones
            "ch341",
            "cp210",  # Silicon Labs CP210x USB-Serial
            "ftdi",   # FTDI USB-Serial on older Arduinos
        ]):
            if verbose:
                print(f"✓ Arduino-compatible USB-Serial chip detected: {p.device}")
            return p.device

    # Priority 4: Generic USB serial ports (might be Arduino)
    for p in ports:
        desc_lower = (p.description or "").lower()
        device_lower = (p.device or "").lower()
        if "usb" in desc_lower or "serial" in desc_lower or "usbmodem" in device_lower:
            if verbose:
                print(f"→ Generic USB serial port selected: {p.device}")
            return p.device

    # Priority 5: Last resort - return first available port
    if verbose:
        print(f"→ Using first available port: {ports[0].device}")
    return ports[0].device if ports else None


def open_arduino_serial(
    port: Optional[str] = None,
    baud: int = 115200,
    timeout: float = 0.25,
    wait_for_ready: bool = True,
    ready_timeout: float = 5.0,
    verbose: bool = True
) -> serial.Serial:
    """
    Open serial connection to Arduino with automatic port detection.
    
    Parameters
    ----------
    port : str, optional
        Serial port device path. If None, auto-detects Arduino port.
    baud : int, default=115200
        Baud rate for serial communication
    timeout : float, default=0.25
        Read timeout in seconds
    wait_for_ready : bool, default=True
        If True, waits for Arduino to send data before returning
    ready_timeout : float, default=5.0
        Maximum time to wait for Arduino ready signal (seconds)
    verbose : bool, default=True
        If True, prints connection information
    
    Returns
    -------
    serial.Serial
        Opened serial connection to Arduino
    
    Raises
    ------
    RuntimeError
        If no serial ports found during auto-detection
    serial.SerialException
        If unable to open the specified port
    
    Examples
    --------
    >>> ser = open_arduino_serial(baud=230400)
    Scanning 2 available serial port(s)...
      - COM5: Arduino Uno (COM5)
    Auto-detected port: COM5
    Opening serial port COM5 @ 230400 baud...
    ✓ Arduino data stream detected
    
    >>> # Manual port specification
    >>> ser = open_arduino_serial(port="COM3", baud=9600)
    Opening serial port COM3 @ 9600 baud...
    """
    # Auto-detect port if not specified
    if port is None:
        port = find_arduino_port(preferred_substr="usbmodem", verbose=verbose)
        if not port:
            raise RuntimeError(
                "No serial ports found. Please specify port manually or connect Arduino."
            )
        if verbose:
            print(f"Auto-detected port: {port}")
    
    if verbose:
        print(f"\nOpening serial port {port} @ {baud} baud...")
    
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException as e:
        raise serial.SerialException(
            f"Failed to open port {port}: {e}\n"
            f"Make sure the Arduino is connected and no other program is using the port."
        )
    
    # Opening the port resets many Arduinos; wait for boot
    time.sleep(1.5)
    ser.reset_input_buffer()
    
    if wait_for_ready:
        if verbose:
            print("Waiting for Arduino data stream...")
        
        t0 = time.time()
        lines_seen = 0
        
        while time.time() - t0 < ready_timeout:
            try:
                raw = ser.readline()
                if raw:
                    try:
                        line = raw.decode("ascii", errors="ignore").strip()
                        if line.startswith("#"):
                            # Status message from Arduino
                            if verbose:
                                print(f"  Arduino: {line}")
                            lines_seen += 1
                        elif line.startswith("$"):
                            # Data line - we're receiving properly
                            if verbose:
                                print("✓ Arduino data stream detected")
                            return ser
                    except Exception:
                        pass
            except serial.SerialException:
                break
            
            if lines_seen > 0:
                # We've seen some output, good sign
                break
        
        if lines_seen > 0:
            if verbose:
                print("✓ Arduino connected")
        else:
            if verbose:
                print("⚠ No Arduino output detected (might work anyway)")
    
    return ser


def list_all_ports(include_details: bool = False) -> list:
    """
    List all available serial ports on the system.
    
    Parameters
    ----------
    include_details : bool, default=False
        If True, returns full ListPortInfo objects with all details.
        If False, returns just the device paths as strings.
    
    Returns
    -------
    list
        List of port devices or ListPortInfo objects
    
    Examples
    --------
    >>> ports = list_all_ports()
    ['COM3', 'COM5']
    
    >>> ports = list_all_ports(include_details=True)
    >>> for p in ports:
    ...     print(f"{p.device}: {p.description} (VID: {p.vid:#06x})")
    COM5: Arduino Uno (VID: 0x2341)
    """
    ports = list(list_ports.comports())
    if include_details:
        return ports
    return [p.device for p in ports]


def is_port_available(port: str) -> bool:
    """
    Check if a specific serial port is available.
    
    Parameters
    ----------
    port : str
        Port device path (e.g., "COM3", "/dev/cu.usbmodem123")
    
    Returns
    -------
    bool
        True if port exists and is available, False otherwise
    
    Examples
    --------
    >>> is_port_available("COM3")
    True
    >>> is_port_available("COM99")
    False
    """
    ports = list_all_ports()
    return port in ports


if __name__ == "__main__":
    # Test/demo the module
    print("Arduino Connection Utilities - Test Mode")
    print("=" * 50)
    
    print("\n1. Listing all available ports:")
    ports = list_all_ports(include_details=True)
    if ports:
        for p in ports:
            vid_str = f"VID: {p.vid:#06x}" if p.vid else "VID: N/A"
            pid_str = f"PID: {p.pid:#06x}" if p.pid else "PID: N/A"
            print(f"  {p.device}")
            print(f"    Description: {p.description}")
            print(f"    {vid_str}, {pid_str}")
    else:
        print("  No ports found")
    
    print("\n2. Auto-detecting Arduino:")
    port = find_arduino_port(verbose=True)
    if port:
        print(f"\n✓ Arduino found on: {port}")
    else:
        print("\n✗ No Arduino detected")
    
    print("\n" + "=" * 50)
    print("Test complete. To connect to Arduino, use:")
    print("  from src.arduino_connection import open_arduino_serial")
    print("  ser = open_arduino_serial(baud=230400)")
