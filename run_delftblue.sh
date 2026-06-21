#!/bin/bash
#SBATCH --job-name=arch_compare
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=6G
#SBATCH --account=education-me-msc-me
#SBATCH --array=0-9%3
#SBATCH --output=logs/run_%A_%a.out
#SBATCH --error=logs/run_%A_%a.err

# ─────────────────────────────────────────────────────────────────────────
# ARCHITECTURE COMPARISON — 5 baselines × 2 regimes = 10 jobs (SLURM array).
#   %3 throttle = max 3 running simultaneously (24 h each).
#
#   array index = regime_idx * 5 + arch_idx
#     archs   = (lstm gru cnn_lstm cnn_gru transformer)
#     regimes = (participant kfold)
#   tasks 0-4 -> participant (LOPO)  : compare vs final_run_lopo (generalized st3)
#   tasks 5-9 -> kfold (par-spec)    : compare vs cross-val2     (specialized  st3)
#
# Each job trains ONE architecture through the same pipeline/data/CV/balancing
# as the st3 final runs and reports participant-class macro metrics
# (model/macro_metrics.py) in run_data.json for direct comparison. Regime is
# selected via THESIS_CV_STRATEGY (config_model.py reads it at import); the
# architecture + importance-off + single-"all"-modality are handled by the
# driver (model/run_arch_comparison.py). Feature importance is OFF.
#
# Consistency with st3 (handled by the driver):
#   - joint (participant, weight) augmentation balancing for ALL models
#   - LR scheduler per regime: kfold -> OneCycleLR (full 200-epoch budget, early
#     stop off); participant -> ReduceLROnPlateau (patience 3, factor 0.8, es 5)
#
# To also run emg_only/imu_only per arch, add --all_modalities (triples runtime;
# mind the 24 h wall, especially the raw-segment CNNs under LOPO).
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

ARCHS=(lstm gru cnn_lstm cnn_gru transformer)
REGIMES=(participant kfold)
N_ARCH=${#ARCHS[@]}

ARCH_IDX=$(( SLURM_ARRAY_TASK_ID % N_ARCH ))
REGIME_IDX=$(( SLURM_ARRAY_TASK_ID / N_ARCH ))
MODEL_TYPE=${ARCHS[$ARCH_IDX]}
export THESIS_CV_STRATEGY=${REGIMES[$REGIME_IDX]}

if [ -z "$MODEL_TYPE" ] || [ -z "$THESIS_CV_STRATEGY" ]; then
    echo "ERROR: could not map SLURM_ARRAY_TASK_ID=$SLURM_ARRAY_TASK_ID to (arch, regime)."
    exit 1
fi

echo "================================================================="
echo "ARCH COMPARISON — array task $SLURM_ARRAY_TASK_ID"
echo "  Architecture        : $MODEL_TYPE"
echo "  Regime              : $THESIS_CV_STRATEGY"
echo "  Database            : $DB_ROOT"
echo "  Python              : $ENV_PYTHON"
$ENV_PYTHON --version
echo "  Extra args          : $*"
echo "================================================================="

srun $ENV_PYTHON -u -m model.run_arch_comparison --model_type "$MODEL_TYPE" "$@"

# ─────────────────────────────────────────────────────────────────────────
# Previous staging (restore by swapping the --array directive + srun line):
#
#   FINAL st3 MODELS (2 jobs):   #SBATCH --array=0-1
#     case "$SLURM_ARRAY_TASK_ID" in
#       0) export THESIS_CV_STRATEGY="participant" ;;
#       1) export THESIS_CV_STRATEGY="kfold" ;;
#     esac
#     srun $ENV_PYTHON -u -m model.run_model "$@"
#
#   SENSOR-PRACTICALITY ABLATION (single job, no --array):
#     srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation "$@"
# ─────────────────────────────────────────────────────────────────────────
