#!/bin/bash
#SBATCH --job-name=ablgen
#SBATCH --partition=gpu-a100
#SBATCH --time=24:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=6G
#SBATCH --account=education-me-msc-me
#SBATCH --array=0-30%3
#SBATCH --output=logs/run_%A_%a.out
#SBATCH --error=logs/run_%A_%a.err

# ─────────────────────────────────────────────────────────────────────────
# GENERALIZED SENSOR-PRACTICALITY ABLATION — LOPO ST-transformer3.
#   The mirror of run_modality_ablation_spec (par-spec / pooled k-fold), but
#   under leave-one-participant-out CV so the gen-vs-spec comparison can enhance
#   visualization/run_modality_ablation/ablation_both_mae_v2.pdf.
#
#   31 combos = 2^5 - 1 non-empty subsets of the 5 sensor-practicality groups
#   (IMUup IMUfo EMGfo EMGup EMGsh). ONE combo per array task — full LOPO over a
#   single combo fits comfortably in 24h, so even the heavy 4-/5-group combos
#   can't blow the wall-time, and a failed combo is isolated (just resrequeue
#   that one task). --array=0-30%3 keeps exactly 3 GPUs busy at once (the full
#   capacity), scheduling the 31 tasks 3-at-a-time.
#
#   Each task self-flips to LOPO via the driver's --lopo flag (only the CV
#   strategy changes; hyperparameters/features stay identical to the par-spec
#   grid, so the spec-vs-gen difference is purely the regime). Importance is OFF
#   (reuse the baseline SHAP/perm); participant class-macro lands in run_data.json.
#
#   Output: each task writes its own timestamped run dir holding the single
#   combo's subfolder, self-describing via THESIS_RUN_TAG=ablgen_<combo>, e.g.
#     run_<ts>_participant_ablgen_<combo>_t<taskid>/<combo>/run_data.json
#   After the array completes, gather the 31 combo subfolders into one grid dir:
#     ./collect_modality_ablation_gen.sh
#   then analyse + plot (see that script's footer).
#
#   To limit LOPO folds (faster, less rigorous), append --limit_folds N to the
#   srun line below (default: full LOPO over all participants).
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

# 31 canonical combo names, in the driver's enumeration order (small→large).
# MUST match model.run_sensor_practicality_ablation.combo_name() exactly.
COMBOS=(
    IMUup IMUfo EMGfo EMGup EMGsh imu_only
    IMUup-EMGfo IMUup-EMGup IMUup-EMGsh
    IMUfo-EMGfo IMUfo-EMGup IMUfo-EMGsh
    EMGfo-EMGup EMGfo-EMGsh EMGup-EMGsh
    IMUup-IMUfo-EMGfo IMUup-IMUfo-EMGup IMUup-IMUfo-EMGsh
    IMUup-EMGfo-EMGup IMUup-EMGfo-EMGsh IMUup-EMGup-EMGsh
    IMUfo-EMGfo-EMGup IMUfo-EMGfo-EMGsh IMUfo-EMGup-EMGsh emg_only
    IMUup-IMUfo-EMGfo-EMGup IMUup-IMUfo-EMGfo-EMGsh IMUup-IMUfo-EMGup-EMGsh
    IMUup-EMGfo-EMGup-EMGsh IMUfo-EMGfo-EMGup-EMGsh all
)
NC=${#COMBOS[@]}

t=$SLURM_ARRAY_TASK_ID
if [ "$t" -ge "$NC" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID=$t out of range (have $NC combos)."
    exit 1
fi
COMBO=${COMBOS[$t]}

export THESIS_CV_STRATEGY="participant"          # leave-one-participant-out (gen)
export THESIS_RUN_TAG="ablgen_${COMBO}"          # self-describing run dir

echo "================================================================="
echo "GENERALIZED SENSOR-PRACTICALITY ABLATION (LOPO) — array task $t"
echo "  Combo               : $COMBO  ($((t+1))/$NC)"
echo "  Regime              : $THESIS_CV_STRATEGY  [LOPO / cross-subject]"
echo "  Run tag             : $THESIS_RUN_TAG"
echo "  Python              : $ENV_PYTHON"
$ENV_PYTHON --version
echo "================================================================="

srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation --lopo --combos "$COMBO" "$@"

# ─────────────────────────────────────────────────────────────────────────
# Previous staging (restore by swapping the --array directive + body):
#   MULTI-SEED ARCH COMPARISON (60 jobs): #SBATCH --array=0-59%3
#     MODELS=(lstm gru cnn_lstm cnn_gru transformer spatio_temporal_transformer3)
#     REGIMES=(participant kfold) ; SEEDS=(245 1 2 3 4) ; NM=6 NS=5
#     regime_idx=t/(NM*NS) ; rem=t%(NM*NS) ; seed_idx=rem/NM ; model_idx=rem%NM
#     export THESIS_CV_STRATEGY=${REGIMES[regime_idx]} ; THESIS_SEED=${SEEDS[seed_idx]}
#     export THESIS_RUN_TAG="${MODEL_TYPE}_s${THESIS_SEED}"
#     srun ... -m model.run_arch_comparison --model_type "$MODEL_TYPE" "$@"
#   PAR-SPEC SENSOR ABLATION (1 job, 31 combos): #SBATCH --array=0-0
#     srun ... -m model.run_sensor_practicality_ablation "$@"   # no --lopo
#   FINAL st3 MODELS (2 jobs): #SBATCH --array=0-1 ; export THESIS_CV_STRATEGY per task,
#     srun ... -m model.run_model "$@"
# ─────────────────────────────────────────────────────────────────────────
