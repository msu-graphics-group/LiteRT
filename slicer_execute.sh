#!/bin/bash
# slicer_execute absolute/path/to/slicer/folder absolute/path/to/slicer/executable
#e.g. bash slicer_execute.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer

bash scripts/slicer_preprocess.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
bash scripts/slicer_preprocess_rtx.sh $1 $2
bash scripts/slicer_build_shaders_rtx.sh $1 $2