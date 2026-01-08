# Thesis Project

A clean Python project setup optimized for Google Colab and GitHub integration.

## Project Structure

```
Thesis/
├── src/                    # Source code package
│   └── __init__.py
├── notebooks/              # Jupyter notebooks
├── tests/                  # Unit tests
├── requirements.txt        # Python dependencies
├── setup.py                # Package installation configuration
├── .gitignore             # Git ignore rules
├── LICENSE                # License file
└── README.md              # This file
```

## Local Setup

### Prerequisites

- **Python 3.8-3.12** (Python 3.12 recommended for PyTorch, TensorFlow, and Keras 3 compatibility)
- pip

### Installation

#### Option 1: Using Conda (Recommended for Deep Learning)

1. Clone the repository:
```bash
git clone https://github.com/baswingen/Thesis.git
cd Thesis
```

2. Create conda environment with Python 3.12:
```bash
# Using the setup script
./setup_conda_env.sh

# Or manually:
conda env create -f environment.yml
conda activate thesis
```

3. Install the project in editable mode:
```bash
pip install -e .
```

4. Register Jupyter kernel (if using Jupyter notebooks):
```bash
python -m ipykernel install --user --name thesis --display-name "Python (thesis)"
```

#### Option 2: Using venv

1. Clone the repository:
```bash
git clone https://github.com/baswingen/Thesis.git
cd Thesis
```

2. Create a virtual environment with Python 3.12:
```bash
python3.12 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
pip install -e .  # Install project in editable mode
```

4. Install ipykernel for Jupyter:
```bash
pip install ipykernel
python -m ipykernel install --user --name thesis --display-name "Python (thesis)"
```

## Google Colab Setup

Run this in a Colab cell (replace with your repo URL):

```python
REPO_URL = "https://github.com/baswingen/Thesis.git"

import os
import sys

# Clone repository
if not os.path.exists('thesis_project'):
    os.system(f'git clone {REPO_URL} Thesis')
    os.rename('Thesis', 'thesis_project')

# Install dependencies
%pip install -q -r thesis_project/requirements.txt
%pip install -q -e thesis_project/

# Add to path
sys.path.insert(0, '/content/thesis_project')
sys.path.insert(0, '/content/thesis_project/src')

print("✅ Setup complete!")
```

## Dependencies

This project includes the following deep learning frameworks:
- **PyTorch** (>=2.0.0)
- **TensorFlow** (>=2.15.0)
- **Keras 3** (>=3.0.0)

Additional dependencies:
- NumPy (compatible with TensorFlow requirements)
- Pandas
- Matplotlib
- Jupyter/IPython kernel support

## Development

### Adding Dependencies

Add new dependencies to `requirements.txt`:

```bash
# Add package name and version
# Note: Ensure compatibility with PyTorch/TensorFlow if adding ML libraries
numpy>=1.24.0
pandas>=2.0.0
```

Then install:
```bash
pip install -r requirements.txt
```

### Running Tests

```bash
# Add your test framework commands here
# Example with pytest:
pytest tests/
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

See the [LICENSE](LICENSE) file for details.

## Author

Project Link: [https://github.com/baswingen/Thesis](https://github.com/baswingen/Thesis)
