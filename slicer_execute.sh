#!/bin/bash
# slicer_execute absolute/path/to/slicer/folder absolute/path/to/slicer/executable
#e.g. bash slicer_execute.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer

bash slicer_preprocess.sh $1 $2
bash slicer_build_shaders.sh $1 $2