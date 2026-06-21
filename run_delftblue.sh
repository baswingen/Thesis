#!/bin/bash
#SBATCH --job-name=arch_seeds
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=6G
#SBATCH --account=education-me-msc-me
#SBATCH --array=0-59%3
#SBATCH --output=logs/run_%A_%a.out
#SBATCH --error=logs/run_%A_%a.err

# ─────────────────────────────────────────────────────────────────────────
# MULTI-SEED ARCHITECTURE COMPARISON — significance backing.
#   6 models × 2 regimes × 5 seeds = 60 jobs (SLURM array, %3 = max 3 at once).
#
#   models  = (lstm gru cnn_lstm cnn_gru transformer spatio_temporal_transformer3)
#   regimes = (participant kfold)
#   seeds   = (245 1 2 3 4)      # seed 245 reproduces the single arch_comp run
#
#   index decomposition (NM=6 models, NS=5 seeds):
#     regime_idx = t / (NM*NS)        # 0=participant (0-29), 1=kfold (30-59)
#     rem        = t % (NM*NS)
#     seed_idx   = rem / NM
#     model_idx  = rem % NM
#
# st3 is included so it runs under the IDENTICAL protocol (same scheduler/epochs/
# balancing as the baselines) — giving its own seed distribution for a proper
# mean±std significance test. Each job: importance OFF, fused "all" modality only,
# participant-class macro in run_data.json. Output dirs are self-describing via
# THESIS_RUN_TAG: run_<ts>_<regime>_<model>_s<seed>_t<taskid>.
#
# To use 3 seeds instead of 5: set SEEDS=(245 1 2) and change --array=0-35%3.
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

MODELS=(lstm gru cnn_lstm cnn_gru transformer spatio_temporal_transformer3)
REGIMES=(participant kfold)
SEEDS=(245 1 2 3 4)
NM=${#MODELS[@]}
NS=${#SEEDS[@]}

t=$SLURM_ARRAY_TASK_ID
REGIME_IDX=$(( t / (NM * NS) ))
REM=$(( t % (NM * NS) ))
SEED_IDX=$(( REM / NM ))
MODEL_IDX=$(( REM % NM ))

MODEL_TYPE=${MODELS[$MODEL_IDX]}
export THESIS_CV_STRATEGY=${REGIMES[$REGIME_IDX]}
export THESIS_SEED=${SEEDS[$SEED_IDX]}
export THESIS_RUN_TAG="${MODEL_TYPE}_s${THESIS_SEED}"

if [ -z "$MODEL_TYPE" ] || [ -z "$THESIS_CV_STRATEGY" ] || [ -z "$THESIS_SEED" ]; then
    echo "ERROR: could not map SLURM_ARRAY_TASK_ID=$t to (model, regime, seed)."
    exit 1
fi

echo "================================================================="
echo "MULTI-SEED ARCH COMPARISON — array task $t"
echo "  Architecture        : $MODEL_TYPE"
echo "  Regime              : $THESIS_CV_STRATEGY"
echo "  Seed                : $THESIS_SEED"
echo "  Run tag             : $THESIS_RUN_TAG"
echo "  Python              : $ENV_PYTHON"
$ENV_PYTHON --version
echo "================================================================="

srun $ENV_PYTHON -u -m model.run_arch_comparison --model_type "$MODEL_TYPE" "$@"

# ─────────────────────────────────────────────────────────────────────────
# Previous staging (restore by swapping the --array directive + body):
#   SINGLE-SEED ARCH COMPARISON: #SBATCH --array=0-9%3 ; map t -> (arch, regime)
#     via ai=$((t%5)), ri=$((t/5)) over ARCHS=(lstm gru cnn_lstm cnn_gru transformer).
#   FINAL st3 MODELS (2 jobs): #SBATCH --array=0-1 ; export THESIS_CV_STRATEGY per task,
#     srun ... -m model.run_model "$@"
# ─────────────────────────────────────────────────────────────────────────
