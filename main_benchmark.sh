#!/bin/bash
#./render_app -benchmark dragon build >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_grid_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_grid >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_octree_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_octree >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_framed_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_frame_octree >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_sbs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_SBS-2-1 >> ./saves/dragon/auto_results.txt
./render_app -benchmark dragon sdf_SBS-2-2 >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_hp_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_hp_octree >> ./saves/dragon/auto_results.txt

bash scripts/slicer_preprocess_svs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark dragon sdf_SVS >> ./saves/dragon/auto_results.txt