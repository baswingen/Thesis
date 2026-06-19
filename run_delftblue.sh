#!/bin/bash
#SBATCH --job-name=final_runs
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=6G
#SBATCH --account=education-me-msc-me
#SBATCH --array=0-1
#SBATCH --output=logs/run_%A_%a.out
#SBATCH --error=logs/run_%A_%a.err

# ─────────────────────────────────────────────────────────────────────────
# FINAL MODELS — two distinct, simultaneous jobs (SLURM array, one GPU each).
#   array task 0 -> THESIS_CV_STRATEGY=participant  (LOPO / cross-subject)
#                   reproduces `final_run`     (WarmupCosine, 17-fold LOPO)
#   array task 1 -> THESIS_CV_STRATEGY=kfold        (par-spec / within-subject)
#                   reproduces `cross-val2`    (OneCycleLR, pooled 5-fold)
#
# Each regime pulls its OWN hyperparameters, LR scheduler, validation strategy
# and feature set from config_model.py via the CV_STRATEGY switch (the env var
# selects it at import — see model/config_model.py). Both regimes already run
# joint participant×weight (participant-class) balancing of the TRAINING set via
# augmentation (AUGMENTATION_CONFIG.balance_participants + balance_weights), and
# report participant-class-balanced macro metrics (model/macro_metrics.py).
#
# The ONLY intended deltas vs the previous saved runs are: (1) the new balanced
# macro metrics, (2) confirmed participant-class training balance. Configs are
# otherwise identical (verified against the saved run_data.json).
# ─────────────────────────────────────────────────────────────────────────

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

# Map the array task index to a regime.
case "$SLURM_ARRAY_TASK_ID" in
    0) export THESIS_CV_STRATEGY="participant"; REGIME="LOPO / cross-subject (final_run)" ;;
    1) export THESIS_CV_STRATEGY="kfold";       REGIME="par-spec / within-subject (cross-val2)" ;;
    *) echo "ERROR: unexpected SLURM_ARRAY_TASK_ID=$SLURM_ARRAY_TASK_ID (expected 0 or 1)"; exit 1 ;;
esac

echo "================================================================="
echo "FINAL RUN — array task $SLURM_ARRAY_TASK_ID"
echo "  Regime              : $REGIME"
echo "  THESIS_CV_STRATEGY  : $THESIS_CV_STRATEGY"
echo "  Database            : $DB_ROOT"
echo "  Python              : $ENV_PYTHON"
$ENV_PYTHON --version
echo "  Extra args          : $*"
echo "================================================================="

srun $ENV_PYTHON -u -m model.run_model "$@"

# ─────────────────────────────────────────────────────────────────────────
# To re-launch the sensor-practicality ablation instead, comment out the
# srun line above and the --array directive, then use:
#   srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation "$@"
# ─────────────────────────────────────────────────────────────────────────
