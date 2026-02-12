# Documentation Index

This folder contains all project-specific documentation and README files.

## Contents

### Module Documentation

- **[README_EMG.md](README_EMG.md)** - Complete guide for EMG acquisition module
  - API reference for `EMGDevice` class
  - Usage examples and troubleshooting
  - Hardware setup and calibration instructions

- **[README_IMU.md](README_IMU.md)** - Complete guide for IMU acquisition module
  - API reference for `IMUDevice` and `MadgwickIMU` classes
  - Orientation filtering and calibration guide
  - Quaternion utilities and examples

### Summaries

- **[IMU_MODULE_SUMMARY.md](IMU_MODULE_SUMMARY.md)** - Quick reference for IMU module
  - Overview of files and features
  - Quick start guide
  - Comparison with original script

## Main Project README

The main project README is located in the root directory: `../README.md`

## Quick Links

### For EMG Users
- Quick start: [README_EMG.md - Quick Start](README_EMG.md#quick-start)
- API Reference: [README_EMG.md - API Reference](README_EMG.md#api-reference)
- Troubleshooting: [README_EMG.md - Troubleshooting](README_EMG.md#troubleshooting)

### For IMU Users
- Quick start: [README_IMU.md - Quick Start](README_IMU.md#quick-start)
- API Reference: [README_IMU.md - API Reference](README_IMU.md#api-reference)
- Calibration: [README_IMU.md - Calibration](README_IMU.md#calibration)
- Troubleshooting: [README_IMU.md - Troubleshooting](README_IMU.md#troubleshooting)

- **[PORTI7_TRIG_WIRING.md](PORTI7_TRIG_WIRING.md)** - Porti7 TRIG input wiring and troubleshooting
  - TMSi interface and trigger behaviour (SAGA/APEX/legacy)
  - Recommended wiring (LEMO, 330 Ω, GND)
  - Why TRIG can stay flat (level, resistor, cable) and what to try (5 V level shifter, smaller R, manual)

## Additional Resources

### Tests and Examples
All test scripts and usage examples are located in the `../tests/` directory:
- `test_emg.py` - EMG module test suite
- `test_imu.py` - IMU module test suite
- `examples_emg_usage.py` - EMG usage examples
- `examples_imu_usage.py` - IMU usage examples
- `emg_example.py` - Simple EMG example

### Source Code
The main source code modules are in the `../src/` directory:
- `emg_acquisition.py` - EMG acquisition module
- `imu_acquisition.py` - IMU acquisition module

---

**Last Updated**: January 19, 2026
