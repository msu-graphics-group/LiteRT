
start_dir=$PWD
echo "start_dir: $start_dir"
cd $1

$2 \
$start_dir/dependencies/HydraCore3/integrator_pt.cpp \
$start_dir/dependencies/HydraCore3/integrator_pt_lgt.cpp \
$start_dir/dependencies/HydraCore3/integrator_pt_mat.cpp \
$start_dir/dependencies/HydraCore3/integrator_rt.cpp \
$start_dir/dependencies/HydraCore3/integrator_spectrum.cpp \
$start_dir/BVH/BVH2Common.cpp \
-mainClass Integrator \
-composInterface ISceneObject \
-composImplementation BVHRT \
-stdlibfolder $PWD/TINYSTL \
-pattern rtv \
-I$PWD/TINYSTL                     ignore  \
-I$PWD/apps/LiteMathAux            ignore  \
-I$start_dir/dependencies/HydraCore3/external/LiteScene ignore  \
-I$start_dir/dependencies/HydraCore3/external/LiteMath  ignore  \
-I$start_dir/dependencies/HydraCore3/external           ignore  \
-I$start_dir/dependencies/HydraCore3/cam_plugin         process \
-I$start_dir                       process \
-I$start_dir/BVH                   process \
-I$start_dir/sdfScene              ignore  \
-shaderCC glsl \
-megakernel 1 \
-enable_ray_tracing_pipeline 0 \
-enable_motion_blur 0 \
-gen_gpu_api 0 \
-DLITERT_RENDERER \
-DKERNEL_SLICER -v \
-DDISABLE_SDF_HP \
-DDISABLE_RF_GRID \
-DDISABLE_GS_PRIMITIVE \
-DDISABLE_SDF_GRID \
-DDISABLE_SDF_SBS_ADAPT \
-DDISABLE_SDF_FRAME_OCTREE_TEX \
-DDISABLE_GRAPHICS_PRIM \
#-DDISABLE_MESH \

cd $start_dir