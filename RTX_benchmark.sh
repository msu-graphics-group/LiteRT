#!/bin/bash

make -j16
./render_app -benchmark $1 build >> ./saves/${1}/auto_results.txt

cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
make -j16
./render_app -rtx_benchmark $1 sdf_grid
./render_app -rtx_benchmark $1 sdf_frame_octree
./render_app -rtx_benchmark $1 sdf_SVS
./render_app -rtx_benchmark $1 "sdf_SBS-2-1"
cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
make -j16
./render_app -rtx_benchmark $1 sdf_grid
./render_app -rtx_benchmark $1 sdf_frame_octree
./render_app -rtx_benchmark $1 sdf_SVS
./render_app -rtx_benchmark $1 "sdf_SBS-2-1"