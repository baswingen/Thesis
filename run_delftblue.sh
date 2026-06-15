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

# ─────────────────────────────────────────────────────────────────────────
# STAGED: full evaluation run of the FULL-SWEEP winner
# (sweep_st_transformer3_full_20260615_054731).
#   - MODEL_TYPE=spatio_temporal_transformer3, CV_STRATEGY=kfold (config_model.py)
#   - 5-fold pooled cross-validation (CV_CONFIG)
#   - all feature-importance methods ON (permutation channel/feature/individual + DeepSHAP)
#   - modality ablation ON (RUN_MODALITY_ABLATION)
#   - joint participant×weight balancing ON (AUGMENTATION_CONFIG)
# RUN_GRID_SEARCH=False, so this runs the pipeline (not a sweep).
# Any args are passed through to run_model (e.g. --run_ablation true/false).
#
# To run a hyperparameter sweep instead, call the sweep script directly:
#   srun $ENV_PYTHON -u model/model_archs/model_hp_opti/sweep_spatio_temporal_transformer3.py --mode full --time_budget_hours 23
echo "================================================================="
echo "RUN_MODEL pipeline (full-sweep winner, kfold CV + ablation + importance)"
echo "Args: $*"
echo "================================================================="
srun $ENV_PYTHON -u -m model.run_model "$@"