#!/bin/bash

#make -j16
#./render_app -benchmark $1 build >> ./saves/${1}/auto_results.txt

cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
make -j16
./render_app -rtx_benchmark $1 sdf_grid           GPU
./render_app -rtx_benchmark $1 sdf_frame_octree   GPU
./render_app -rtx_benchmark $1 sdf_SVS            GPU 
./render_app -rtx_benchmark $1 "sdf_SBS-2-1"      GPU
cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
make -j16
./render_app -rtx_benchmark $1 sdf_grid           RTX
./render_app -rtx_benchmark $1 sdf_frame_octree   RTX
./render_app -rtx_benchmark $1 sdf_SVS            RTX 
./render_app -rtx_benchmark $1 "sdf_SBS-2-1"      RTX
cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
make -j16
./render_app -rtx_benchmark $1 sdf_grid           CPU
./render_app -rtx_benchmark $1 sdf_frame_octree   CPU
./render_app -rtx_benchmark $1 sdf_SVS            CPU 
./render_app -rtx_benchmark $1 "sdf_SBS-2-1"      CPU