#!/bin/bash
# Setup script for creating a conda environment for the Thesis project

echo "Creating conda environment 'thesis' with Python 3.12..."

# Create conda environment with Python 3.12
conda env create -f environment.yml

echo ""
echo "✅ Conda environment created!"
echo ""
echo "To activate the environment, run:"
echo "  conda activate thesis"
echo ""
echo "To install the project in editable mode:"
echo "  pip install -e ."
echo ""
echo "To register the kernel for Jupyter:"
echo "  python -m ipykernel install --user --name thesis --display-name 'Python (thesis)'"

