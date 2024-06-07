#!/bin/bash

./render_app -benchmark $3 build >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_grid_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_grid >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_octree_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_octree >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_framed_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_frame_octree >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_sbs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_SBS-2-1 >> ./saves/$3/auto_results.txt
./render_app -benchmark $3 sdf_SBS-2-2 >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_hp_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_hp_octree >> ./saves/$3/auto_results.txt

bash scripts/slicer_preprocess_svs_only.sh $1 $2
bash scripts/slicer_build_shaders.sh $1 $2
make -j15
./render_app -benchmark $3 sdf_SVS >> ./saves/$3/auto_results.txt