#!/bin/bash
# slicer_execute absolute/path/to/slicer/folder absolute/path/to/slicer/executable
#e.g. bash slicer_execute.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer

printf "SLICER EXECUTE FOR LITERT\n"
bash scripts/slicer_preprocess.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
bash scripts/slicer_preprocess_gpu_rq.sh $1 $2
bash scripts/slicer_build_shaders_gpu_rq.sh $1 $2

printf "SLICER EXECUTE FOR HYDRA\n"
bash scripts/slicer_preprocess_hydra_litert.sh $1 $2
bash scripts/slicer_build_shaders_hydra.sh $1 $2