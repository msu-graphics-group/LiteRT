
start_dir=$PWD
echo "start_dir: $start_dir"
cd $1

$2 $start_dir/Renderer/eye_ray.cpp $start_dir/BVH/BVH2Common.cpp \
-mainClass MultiRenderer \
-composInterface ISceneObject \
-composImplementation BVHRT \
-stdlibfolder $PWD/TINYSTL \
-pattern rtv \
-timestamps 1 \
-I$PWD/TINYSTL                     ignore  \
-I$start_dir/dependencies          ignore  \
-I$start_dir/dependencies/HydraCore3/external          ignore  \
-I$start_dir/dependencies/HydraCore3/external/LiteMath ignore  \
-I$start_dir                       process \
-I$start_dir/Renderer              process \
-I$start_dir/BVH                   process \
-I$start_dir/sdfScene              ignore  \
-shaderCC glsl \
-suffix _GPU \
-megakernel 1 \
-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v \
-DDISABLE_MESH_LOD -DDISABLE_SDF_GRID -DDISABLE_SDF_SVS -DDISABLE_SDF_SBS -DDISABLE_SDF_SBS_ADAPT -DDISABLE_SDF_FRAME_OCTREE -DDISABLE_SDF_FRAME_OCTREE_TEX -DDISABLE_SDF_HP -DDISABLE_RF_GRID -DDISABLE_GS_PRIMITIVE -DDISABLE_NURBS -DDISABLE_RIBBON -DDISABLE_CATMUL_CLARK -DDISABLE_GRAPHICS_PRIM -DDISABLE_OPENVDB 

cd $start_dir