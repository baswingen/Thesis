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
module load 2024r1 miniconda3
conda activate /scratch/bwingen/thesis_env

# ── DelftBlue Environment Check ────────────────────────────
# The model is configured to look for data in /scratch/bwingen/thesis/database
# If it's not there, the job will fail with an informative message.

DB_ROOT="/scratch/bwingen/thesis/database"
if [ ! -d "$DB_ROOT" ]; then
    echo "ERROR: Database not found at $DB_ROOT"
    echo "Please run './upload_database.sh' from your local machine to sync data."
    exit 1
fi

echo "Database found at $DB_ROOT"
echo "Running on $(hostname)"
echo "Working directory: $(pwd)"
which python
python --version

srun python -u model/deepshap_analysis.py