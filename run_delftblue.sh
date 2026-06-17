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
# STAGED: SENSOR-PRACTICALITY ABLATION GRID (par-spec ST-transformer3).
# Splits the 20 baseline channels into 5 placement groups (IMU upper/forearm,
# EMG forearm/upper/shoulder) and runs all 31 non-empty combinations.
#   - MODEL_TYPE=spatio_temporal_transformer3, pooled k-fold (within-subject calibration)
#   - limit_folds=3, skip_final_full_train=True  (24h budget; projected 14-16h)
#   - feature importance OFF (reuse cross-val2 baseline DeepSHAP/permutation)
# The driver mutates cfg in-memory only; config_model.py is untouched.
# Pass-through args go to the driver, e.g.:
#   --skip_baseline_combos       (skip all/emg_only/imu_only -> 28 combos)
#   --combos IMUfo-EMGfo,emg_only (restrict to named combos)
#
# PHASE 2 (cross-subject transfer check, run AFTER analysing phase-1 results):
#   srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation --lopo \
#        --combos <top-3 combo names from analysis>
#
# To re-launch the previous baseline pipeline instead, restore the line below:
#   srun $ENV_PYTHON -u -m model.run_model "$@"
echo "================================================================="
echo "SENSOR-PRACTICALITY ABLATION (par-spec kfold, 31 combos, importance OFF)"
echo "Args: $*"
echo "================================================================="
srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation "$@"