#!/bin/bash
#SBATCH --job-name=run_model
#SBATCH --partition=gpu-a100-small
#SBATCH --time=2:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=4G
#SBATCH --account=education-me-msc-me
#SBATCH --output=logs/run_%j.out
#SBATCH --error=logs/run_%j.err

cd /home/bwingen/thesis/Thesis

# ── Environment Setup ──────────────────────────────────────
module load 2024r1
module load miniconda3
conda activate thesis

# ── DelftBlue Environment Check ────────────────────────────
# The model is configured to look for data in /scratch/bwingen/thesis/database
DB_ROOT="/scratch/bwingen/thesis/database"
if [ ! -d "$DB_ROOT" ]; then
    echo "ERROR: Database not found at $DB_ROOT"
    echo "Please run './upload_database.sh' from your local machine to sync data."
    exit 1
fi

echo "Database found at $DB_ROOT"
echo "Using Python: $(which python)"
python --version

srun python -u model/deepshap_analysis.py