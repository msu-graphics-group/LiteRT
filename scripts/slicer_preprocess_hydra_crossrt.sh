
start_dir=$PWD
echo "start_dir: $start_dir"
cd $1

$2 \
$start_dir/dependencies/HydraCore3/integrator_pt.cpp \
$start_dir/dependencies/HydraCore3/integrator_pt_lgt.cpp \
$start_dir/dependencies/HydraCore3/integrator_pt_mat.cpp \
$start_dir/dependencies/HydraCore3/integrator_rt.cpp \
$start_dir/dependencies/HydraCore3/integrator_spectrum.cpp \
-mainClass Integrator \
-stdlibfolder $PWD/TINYSTL \
-pattern rtv \
-I$PWD/TINYSTL                     ignore  \
-I$PWD/apps/LiteMath               ignore  \
-I$PWD/apps/LiteMathAux            ignore  \
-I$start_dir/dependencies/HydraCore3/external/LiteScene ignore  \
-I$start_dir/dependencies/HydraCore3/external/LiteMath  ignore  \
-I$start_dir/dependencies/HydraCore3/external/CrossRT   process \
-I$start_dir/dependencies/HydraCore3/cam_plugin         process \
-shaderCC glsl \
-megakernel 1 \
-enable_ray_tracing_pipeline 0 \
-enable_motion_blur 0 \
-gen_gpu_api 0 \
-DKERNEL_SLICER -v \

cd $start_dir