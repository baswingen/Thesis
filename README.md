# EMG + IMU Data Acquisition System

Comprehensive system for synchronized EMG (electromyography) and IMU (inertial measurement unit) data acquisition, processing, and visualization.

## 🎯 Project Overview

This system enables researchers to collect synchronized biosignal data from multiple sources:
- **EMG Signals**: Muscle activity from TMSi Porti7/REFA devices
- **IMU Sensors**: Motion data from dual BMI160 sensors via Arduino
- **Synchronized Acquisition**: Hardware-timestamped data alignment
- **Real-time Processing**: Filtering, envelope extraction, normalization
- **ML-Ready Storage**: HDF5 format compatible with PyTorch/Keras

## 📁 Project Structure

```
├── src/                          # Core acquisition modules
│   ├── emg_acquisition.py       # TMSi EMG device interface
│   ├── imu_acquisition.py       # Dual BMI160 IMU interface
│   ├── synchronized_acquisition.py  # Synchronized EMG+IMU
│   ├── emg_processing.py        # Signal preprocessing
│   └── emg_visualization.py     # Real-time plotting
│
├── trial/                       # Trial data collection system ⭐ NEW
│   ├── setup_trial.py          # Configuration (edit this!)
│   ├── trial_manager.py        # Core orchestration
│   ├── trial_gui.py            # GUI interface
│   ├── data_storage.py         # HDF5 storage
│   ├── trial_protocols.py      # Protocol definitions
│   └── README.md               # Detailed trial documentation
│
├── database/                    # Trial data storage
│   ├── participant_XXX/
│   │   └── session_XXX/
│   │       ├── metadata.json
│   │       ├── trial_XXX_raw.h5
│   │       └── trial_XXX_preprocessed.h5
│   └── README.md
│
├── tests/                       # Example scripts
├── setup_scripts/               # Testing and setup scripts
├── arduino/                     # Arduino sketches for IMU
├── README/                      # Detailed documentation
└── requirements.txt            # Python dependencies
```

## 🚀 Quick Start

### 1. Installation

```bash
# Clone repository
git clone <repository-url>
cd Thesis

# Install dependencies
pip install -r requirements.txt

# Or use conda
conda env create -f environment.yml
conda activate emg-imu
```

### 2. Hardware Setup

**EMG Device (TMSi Porti7):**
- Connect via USB
- Install TMSi drivers
- Test connection: `python tests/test_emg.py`

**IMU Sensors (Dual BMI160):**
- Upload Arduino sketch: `arduino/IMU_sketch/IMU_sketch_I2C/`
- Connect via USB serial
- Test connection: `python tests/test_imu.py`

### 3. Run Trial Collection

```bash
# Configure trial in trial/setup_trial.py
# Then run:
python -m trial.trial_manager

# Or test without hardware:
python -m trial.trial_manager --mock
```

## 📖 Documentation

Comprehensive documentation is available in the `README/` directory:

- **[INDEX.md](README/INDEX.md)** - Documentation overview
- **[README_EMG.md](README/README_EMG.md)** - EMG acquisition guide
- **[README_IMU.md](README/README_IMU.md)** - IMU acquisition guide
- **[README_SIGNAL_ACQUISITION.md](README/README_SIGNAL_ACQUISITION.md)** - Synchronized acquisition
- **[README_PREPROCESSING.md](README/README_PREPROCESSING.md)** - Signal processing
- **[SYNCHRONIZATION_EXPLAINED.md](README/SYNCHRONIZATION_EXPLAINED.md)** - Synchronization deep-dive
- **[trial/README.md](trial/README.md)** - Trial system guide ⭐

## 🎮 Trial Data Collection

The trial system provides a complete solution for structured data collection:

### Features

✅ **Interactive GUI** - Real-time signal visualization
✅ **Keyboard Control** - Simple trial progression (SPACE, N, C, Q)
✅ **Flexible Protocols** - Configurable exercise sequences
✅ **HDF5 Storage** - ML-ready data format
✅ **Comprehensive Metadata** - Participant, trial, device info
✅ **Preprocessing Pipeline** - Bandpass, notch, envelope extraction
✅ **Mock Mode** - Test without hardware

### Quick Trial Setup

1. **Edit Configuration** (`trial/setup_trial.py`):
   ```python
   PARTICIPANT_ID = "P001"
   SESSION_ID = "S001"
   
   TRIAL_EXERCISES = [
       {'name': 'Wrist_Flexion', 'duration': 5.0, ...},
       {'name': 'Hand_Grip', 'duration': 5.0, ...},
       # ... more exercises
   ]
   ```

2. **Run Collection**:
   ```bash
   python -m trial.trial_manager
   ```

3. **Keyboard Controls**:
   - **SPACE**: Start/stop trial
   - **N**: Next trial
   - **C**: Calibrate IMU
   - **Q**: Quit

4. **Data Location**:
   ```
   database/participant_P001/session_S001/
   ├── metadata.json
   ├── trial_001_raw.h5
   ├── trial_001_preprocessed.h5
   └── ...
   ```

See [trial/README.md](trial/README.md) for complete documentation.

## 💾 Data Format

All trial data is stored in HDF5 format, compatible with PyTorch and Keras:

### Loading Data

```python
import h5py
import torch

# Load preprocessed EMG data
with h5py.File('database/P001/S001/trial_001_preprocessed.h5', 'r') as f:
    emg_filtered = torch.from_numpy(f['emg/filtered'][:])
    emg_envelope = torch.from_numpy(f['emg/envelope'][:])
    timestamps = f['emg/timestamps'][:]

print(f"EMG shape: {emg_filtered.shape}")  # (samples, channels)
```

See [database/README.md](database/README.md) for detailed format documentation.

## 🔬 Example Usage

### Simple EMG Acquisition

```python
from src import EMGDevice

with EMGDevice(connection_type='usb') as emg:
    emg.connect()
    emg.configure_emg_differential_pair(pos_channel=1, neg_channel=2)
    emg.start_acquisition(duration=10.0)
    data = emg.get_data()
    emg.export_data('my_recording.npy')
```

### Synchronized EMG + IMU

```python
from src.synchronized_acquisition import SynchronizedAcquisition, SyncConfig

config = SyncConfig()
with SynchronizedAcquisition(config) as acq:
    acq.start()
    
    # Data available in buffers:
    # acq.emg_buffer
    # acq.imu_buffer
    
    time.sleep(10)  # Collect for 10 seconds
    acq.stop()
```

### Signal Processing

```python
from src.emg_processing import EMGPreprocessor

preprocessor = EMGPreprocessor(
    fs=2048,
    bandpass_low=20.0,
    bandpass_high=450.0,
    notch_freq=50.0,
    envelope_cutoff=10.0
)

# Process raw EMG signal
result = preprocessor.process(raw_emg, return_all_stages=True)
filtered = result['filtered']
envelope = result['envelope']
```

## 🧪 Testing

### Basic Tests (No Hardware)

```bash
python trial/test_simple.py
```

### Full Integration Tests

```bash
python trial/test_trial_system.py
```

### Component Tests

```bash
# Test EMG device
python tests/test_emg.py

# Test IMU device
python tests/test_imu.py

# Test synchronization
python setup_scripts/signal_acquisition_testing.py
```

## 🛠️ Development

### Key Modules

- **`src/emg_acquisition.py`** - EMG device interface
- **`src/imu_acquisition.py`** - IMU device interface
- **`src/synchronized_acquisition.py`** - Synchronized acquisition
- **`src/emg_processing.py`** - Signal preprocessing
- **`trial/trial_manager.py`** - Trial orchestration
- **`trial/data_storage.py`** - HDF5 storage

### Adding New Protocols

Edit `trial/trial_protocols.py` or create in `trial/setup_trial.py`:

```python
MY_PROTOCOL = [
    {
        'name': 'Custom_Movement',
        'duration': 5.0,
        'instruction': 'Perform custom movement',
        'rest_after': 3.0,
        'repetitions': 5,
    },
    # ... more exercises
]
```

## 📊 Data Processing Pipeline

```
Raw EMG/IMU Data
      ↓
Synchronized Acquisition (hardware timestamps)
      ↓
Real-time Preprocessing
  - Bandpass filter (20-450 Hz)
  - Notch filter (50/60 Hz)
  - Rectification
  - Envelope extraction
      ↓
HDF5 Storage (raw + preprocessed)
      ↓
ML Training (PyTorch/Keras)
```

## 🎓 Citation

If you use this system in your research, please cite:

```bibtex
@software{emg_imu_system,
  title={EMG + IMU Data Acquisition System},
  author={Your Name},
  year={2026},
  url={https://github.com/your-repo}
}
```

## 📝 License

See [LICENSE](LICENSE) file.

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## 📧 Support

For issues or questions:
- Check documentation in `README/` directory
- Review `trial/README.md` for trial system
- Check `database/README.md` for data format
- Run tests: `python trial/test_simple.py`

## 🔄 Recent Updates

### January 2026
- ✨ **New Trial System**: Complete GUI-based data collection
- 📦 **HDF5 Storage**: ML-ready data format
- 🎮 **Keyboard Controls**: Intuitive trial progression
- 📊 **Real-time Visualization**: Live EMG+IMU plots
- 🧪 **Mock Mode**: Test without hardware
- 📚 **Comprehensive Documentation**: Detailed guides

## 🗺️ Roadmap

- [ ] MVC calibration workflow
- [ ] Advanced artifact detection
- [ ] Video synchronization
- [ ] Web-based interface
- [ ] Real-time EMG-driven feedback
- [ ] Multi-session analysis tools

---

**Ready to collect trial data?** See [trial/README.md](trial/README.md) to get started! 🚀
