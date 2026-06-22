#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# Collect the per-task LOPO ablation run dirs into ONE grid dir that mirrors
# model/model_results/run_modality_ablation_spec (31 combo subfolders, no
# root-level master run_data.json so the viz scans the subfolders).
#
# run_delftblue.sh (generalized staging) writes one combo per array task to:
#     model/model_results/run_<ts>_participant_ablgen_<combo>_t<id>/<combo>/
# This script moves each <combo>/ subfolder into:
#     model/model_results/run_modality_ablation_gen/<combo>/
#
# Run after the SLURM array has fully completed. Idempotent-ish: re-runs skip
# combos already present in the target. Use --copy to copy instead of move.
# ─────────────────────────────────────────────────────────────────────────
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS="$ROOT/model/model_results"
TARGET="$RESULTS/run_modality_ablation_gen"

MOVE="mv"
[ "${1:-}" = "--copy" ] && MOVE="cp -r"

mkdir -p "$TARGET"
moved=0
shopt -s nullglob
for rundir in "$RESULTS"/run_*_ablgen_*; do
    [ -d "$rundir" ] || continue
    for sub in "$rundir"/*/; do
        [ -f "${sub}run_data.json" ] || continue          # only real combo subfolders
        name="$(basename "$sub")"
        if [ -e "$TARGET/$name" ]; then
            echo "skip  $name (already in target)"
            continue
        fi
        $MOVE "$sub" "$TARGET/$name"
        echo "added $name"
        moved=$((moved+1))
    done
done

n="$(find "$TARGET" -maxdepth 2 -name run_data.json | wc -l | tr -d ' ')"
echo "─────────────────────────────────────────────"
echo "collected $moved new combo(s); $n total in $TARGET (expected 31)"
echo
echo "Next — analyse + plot the generalized grid:"
echo "  python -m model.analyze_sensor_ablation --grid_dir model/model_results/run_modality_ablation_gen"
echo "  python visualization/run_viz.py -r model/model_results/run_modality_ablation_gen \\"
echo "      -o visualization/run_modality_ablation_gen -p modality_ablation_study"
