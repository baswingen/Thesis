#!/bin/bash
#SBATCH --job-name=run_model
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=6G
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
# Default pipeline execution (commented out to run sweeps instead):
# srun $ENV_PYTHON -u -m model.run_model "$@"

# Results are checkpointed to sweep_results_partial.csv after every fit;
# if a job is killed, resume with --start_iter <last completed + 1>
# (features/model_arch/generalization modes only).
#
# Recommended: the budgeted full pipeline (feature set + HP/regularization/
# OneCycleLR via successive halving), guaranteed to finish within budget:
#   sbatch run_delftblue.sh --mode full --time_budget_hours 23
#
# Legacy single-stage sweeps (a 60-iteration stage takes ~27.5h > 24h limit,
# so these need a resume submission):
#   sbatch run_delftblue.sh --mode features --n_iter 60 --start_iter 52
#   sbatch run_delftblue.sh --mode generalization --n_iter 60
if [ $# -eq 0 ]; then
    echo "ERROR: No sweep arguments given. Example:"
    echo "  sbatch run_delftblue.sh --mode full --time_budget_hours 23"
    exit 1
fi

echo "================================================================="
echo "SWEEP: $@"
echo "================================================================="
srun $ENV_PYTHON -u model/model_archs/model_hp_opti/sweep_spatio_temporal_transformer3.py "$@"