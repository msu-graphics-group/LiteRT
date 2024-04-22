#!/bin/bash
# slicer_execute_mini absolute/path/to/slicer/folder absolute/path/to/slicer/executable
#e.g. bash slicer_execute_mini.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer
start_dir=$PWD
cd $1

$2 $start_dir/Renderer/eye_ray.cpp $start_dir/BVH/BVH2Common.cpp \
-mainClass MultiRenderer \
-composInterface ISceneObject \
-composImplementation BVHRT \
-stdlibfolder $PWD/TINYSTL \
-pattern rtv \
-I$PWD/TINYSTL                  ignore \
-I$PWD/apps                     ignore \
-I$PWD/apps/LiteMath            ignore \
-I$start_dir                   process \
-I$start_dir/Renderer          process \
-I$start_dir/BVH               process \
-I$start_dir/sdfScene           ignore \
-shaderCC glsl \
-suffix _GPU \
-megakernel 1 \
-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -DLITERT_MINI -v

cd $start_dir
rm -r shaders_gpu
mkdir shaders_gpu
cd Renderer/shaders_gpu
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_gpu
cd ../..