#!/bin/bash
#SBATCH --job-name=run_model
#SBATCH --partition=gpu-a100-small
#SBATCH --time=00:15:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=1G
#SBATCH --account=education-ME-MSc-ME
#SBATCH --output=logs/run_%j.out
#SBATCH --error=logs/run_%j.err

cd /home/bwingen/thesis/Thesis
source .venv/bin/activate

echo "Running on $(hostname)"
echo "Working directory: $(pwd)"
which python
python --version

srun python -u model/run_model.py