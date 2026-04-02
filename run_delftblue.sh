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

module load 2024r1
module load cuda/12.1

ENV_PYTHON="/scratch/bwingen/thesis_env/bin/python"

DB_ROOT="/scratch/bwingen/thesis/database"
if [ ! -d "$DB_ROOT" ]; then
    echo "ERROR: Database not found at $DB_ROOT"
    echo "Please run './upload_database.sh' from your local machine to sync data."
    exit 1
fi

echo "Database found at $DB_ROOT"
echo "Using Python: $ENV_PYTHON"
$ENV_PYTHON --version

srun $ENV_PYTHON -u model/run_model.py