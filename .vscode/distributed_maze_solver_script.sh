#!/bin/bash
#
#SBATCH --cpus-per-task=1
#SBATCH --nodes=1
#SBATCH --ntasks=8
#SBATCH --partition=slow
#SBATCH --mem=10G

srun ./curve_area_parallel 20