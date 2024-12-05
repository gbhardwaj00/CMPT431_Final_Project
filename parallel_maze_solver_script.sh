#!/bin/bash
#
#SBATCH --cpus-per-task=4
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --partition=slow
#SBATCH --mem=10G

srun ./parallel_maze_solver 200