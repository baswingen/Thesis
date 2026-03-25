#!/bin/bash
#SBATCH --job-name=run_model
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=4G
#SBATCH --account=education-me-msc-me
#SBATCH --output=logs/run_%j.out
#SBATCH --error=logs/run_%j.err

cd /home/bwingen/thesis/Thesis
source .venv/bin/activate

echo "Running on $(hostname)"
echo "Working directory: $(pwd)"
which python
python --version

srun python -u model/ablation/ablation_cnn_lstm.py