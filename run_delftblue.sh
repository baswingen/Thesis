#!/bin/sh
#
#SBATCH --job-name="job_name"
#SBATCH --partition=gpu
#SBATCH --time=0:15:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --gpus-per-task=1
#SBATCH --mem-per-cpu=1G
#SBATCH --account=research-ME-BME

module load 2025
module load cuda

srun ./model/run_model.py