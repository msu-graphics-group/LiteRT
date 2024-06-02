#!/bin/bash

bash scripts/slicer_preprocess_grid_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_grid >> ./saves/bunny/auto_results.txt

bash scripts/slicer_preprocess_octree_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_octree >> ./saves/bunny/auto_results.txt

bash scripts/slicer_preprocess_framed_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_frame_octree >> ./saves/bunny/auto_results.txt

bash scripts/slicer_preprocess_sbs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_SBS-2-1 >> ./saves/bunny/auto_results.txt
./render_app -benchmark bunny sdf_SBS-2-2 >> ./saves/bunny/auto_results.txt

bash scripts/slicer_preprocess_hp_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_hp_octree >> ./saves/bunny/auto_results.txt

bash scripts/slicer_preprocess_svs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark bunny sdf_SVS >> ./saves/bunny/auto_results.txt