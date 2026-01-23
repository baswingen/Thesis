# Trial System - Quick Start Guide

Get started with trial data collection in 5 minutes!

## ⚡ Installation

```bash
# Install dependencies
pip install -r requirements.txt
```

## ⚙️ Configuration

Edit `trial/setup_trial.py`:

```python
# 1. Set participant info
PARTICIPANT_ID = "P001"
SESSION_ID = "S001"

# 2. Configure exercises (or use default)
TRIAL_EXERCISES = [
    {
        'name': 'Wrist_Flexion',
        'duration': 5.0,
        'instruction': 'Flex your wrist and hold',
        'rest_after': 3.0,
        'repetitions': 3,
    },
    # Add more exercises...
]
```

## 🚀 Running Trials

### With Hardware (Real Data Collection)

```bash
python -m trial.trial_manager
```

### Without Hardware (Testing/Demo)

```bash
python -m trial.trial_manager --mock
```

## 🎮 Keyboard Controls

While running:

| Key | Action |
|-----|--------|
| **SPACE** | Start/Stop trial |
| **N** | Next trial |
| **R** | Repeat trial |
| **C** | Calibrate IMU |
| **Q** | Quit session |

## 📊 Viewing Results

After data collection, find your data in:

```
database/
└── participant_P001/
    └── session_S001/
        ├── metadata.json              # Session info
        ├── trial_001_raw.h5          # Raw EMG + IMU
        ├── trial_001_preprocessed.h5  # Filtered EMG
        └── ...
```

## 💻 Loading Data for ML

```python
import h5py
import torch

# Load preprocessed data
with h5py.File('database/P001/S001/trial_001_preprocessed.h5', 'r') as f:
    emg_data = torch.from_numpy(f['emg/filtered'][:])
    timestamps = f['emg/timestamps'][:]

print(f"Shape: {emg_data.shape}")  # (samples, channels)
```

## 🧪 Testing

Test without installing dependencies:

```bash
python trial/test_simple.py
```

Test with full system:

```bash
python trial/test_trial_system.py
```

## 📚 Next Steps

- Read [trial/README.md](README.md) for detailed documentation
- Check [database/README.md](../database/README.md) for data format
- Customize protocols in `trial_protocols.py`
- Configure preprocessing in `setup_trial.py`

## 🆘 Troubleshooting

**"Module not found" errors:**
```bash
pip install -r requirements.txt
```

**Device not connecting:**
- Check USB connections
- Verify drivers installed
- Try `--mock` mode to test software

**GUI not showing:**
- Check display settings
- Try running in windowed mode (set `fullscreen: False`)

## 💡 Tips

1. **Start with mock mode** to learn the interface
2. **Use short trials** (2-3 seconds) for initial testing
3. **Calibrate IMU** at the start of each session
4. **Check plots** to verify signal quality before proceeding
5. **Take breaks** between repetitions

---

**Need help?** Check the full documentation in [README.md](README.md)
