#!/bin/bash
# slicer_execute absolute/path/to/slicer/folder absolute/path/to/slicer/executable
#e.g. bash slicer_execute.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer
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
-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v \
-DDISABLE_SDF_PRIMITIVE \
-DDISABLE_SDF_OCTREE \
-DDISABLE_SDF_GRID \
-DDISABLE_SDF_FRAME_OCTREE \
-DDISABLE_SDF_SBS \
-DDISABLE_SDF_SVS \
-DDISABLE_RF_GRID \
-DDISABLE_GS_PRIMITIVE

$2 $start_dir/NeuralRT/NeuralRT.cpp \
-mainClass NeuralRT \
-stdlibfolder $PWD/TINYSTL \
-pattern ipv \
-I$PWD/TINYSTL                  ignore \
-I$PWD/apps                     ignore \
-I$PWD/apps/LiteMath            ignore \
-I$start_dir                   process \
-I$start_dir/BVH               process \
-I$start_dir/sdfScene           ignore \
-shaderCC glsl \
-suffix _GPU \
-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v

cd $start_dir
rm -r shaders_gpu
mkdir shaders_gpu
cd Renderer/shaders_gpu
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_gpu
cd ../..
cd NeuralRT/shaders_gpu
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_gpu
cd ../..