# Tests and Examples

This directory contains all test scripts and usage examples for the Thesis project.

## Contents

### Test Scripts

Test scripts validate the functionality of the modules:

- **`test_emg.py`** - Test suite for EMG acquisition module
  - Tests connection, data acquisition, and device management
  - Verifies calibration and measurements
  - Run: `python tests/test_emg.py`

- **`test_imu.py`** - Test suite for IMU acquisition module
  - Tests serial connection and data parsing
  - Verifies Madgwick filter and calibration
  - Validates orientation estimation
  - Run: `python tests/test_imu.py`

- **`test_example.py`** - Basic project test examples
  - Template for additional tests

### Example Scripts

Example scripts demonstrate how to use the modules:

- **`examples_emg_usage.py`** - Interactive EMG examples
  - 6 complete usage examples for EMG acquisition
  - Interactive menu for selecting examples
  - Run: `python tests/examples_emg_usage.py`

- **`examples_imu_usage.py`** - Interactive IMU examples
  - 6 complete usage examples for IMU acquisition
  - Shows calibration, orientation tracking, data export
  - Run: `python tests/examples_imu_usage.py`

- **`emg_example.py`** - Simple EMG demonstration
  - Quick and simple EMG acquisition example
  - Good starting point for new users
  - Run: `python tests/emg_example.py`

## Running Tests

### Run Individual Test Suite

```bash
# From project root
python tests/test_emg.py
python tests/test_imu.py
```

Or using Python module syntax:

```bash
python -m tests.test_emg
python -m tests.test_imu
```

### Run All Tests (if using pytest)

```bash
pytest tests/
```

## Running Examples

### EMG Examples

```bash
# Interactive menu
python tests/examples_emg_usage.py

# Simple example
python tests/emg_example.py
```

### IMU Examples

```bash
# Interactive menu
python tests/examples_imu_usage.py
```

## Hardware Requirements

### For EMG Tests/Examples
- TMSi Porti7 or REFA device
- USB or Bluetooth connection
- TMSiSDK.dll installed (see main README)

### For IMU Tests/Examples
- BMI160 dual IMU sensor system
- Serial connection (USB)
- Arduino or compatible microcontroller

## Important Notes

### Before Running Tests

1. **Update Serial Ports**: 
   - Edit PORT variable in test/example files to match your system
   - Windows: Usually `"COM3"`, `"COM4"`, etc.
   - macOS/Linux: Usually `"/dev/cu.usbmodem..."` or `"/dev/ttyUSB0"`

2. **Hardware Setup**:
   - Ensure devices are connected and powered on
   - For EMG: TMSiSDK.dll must be accessible
   - For IMU: Serial device must be transmitting data

3. **Calibration**:
   - EMG: Follow on-screen device setup instructions
   - IMU: Place sensors flat and still during calibration

### Test Organization

Tests are organized by functionality:
- **Unit tests**: Test individual components
- **Integration tests**: Test complete workflows
- **Example scripts**: Demonstrate real-world usage

## Contributing

When adding new tests or examples:
1. Follow the naming convention: `test_*.py` for tests, `examples_*.py` or `*_example.py` for examples
2. Include docstrings and usage instructions
3. Update this README with new file descriptions
4. Ensure tests can run independently

## Documentation

For detailed module documentation, see the `../README/` directory:
- [README_EMG.md](../README/README_EMG.md) - EMG module documentation
- [README_IMU.md](../README/README_IMU.md) - IMU module documentation

## Support

If tests fail:
1. Check hardware connections
2. Verify serial port configuration
3. Review error messages carefully
4. Consult module documentation in `../README/`
5. Check troubleshooting sections in module READMEs

---

**Last Updated**: January 19, 2026
