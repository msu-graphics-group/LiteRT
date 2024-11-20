
start_dir=$PWD
echo "start_dir: $start_dir"
cd $1

$2 $start_dir/Renderer/eye_ray.cpp $start_dir/BVH/BVH2Common.cpp \
-mainClass MultiRenderer \
-composInterface ISceneObject \
-composImplementation BVHRT \
-options $start_dir/options.json \
-intersectionShader AbstractObject::Intersect \
-intersectionTriangle AbstractObject::GeomDataTriangle \
-enable_ray_tracing_pipeline 0 \
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
-suffix _gpu_rq \
-megakernel 1 \
-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v \
-DDISABLE_SDF_HP \
-DDISABLE_RF_GRID \
-DDISABLE_GS_PRIMITIVE

cd $start_dir