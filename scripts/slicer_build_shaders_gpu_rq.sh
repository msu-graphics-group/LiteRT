rm -r shaders_gpu_rq
mkdir shaders_gpu_rq
cd Renderer/shaders_gpu_rq
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_gpu_rq
cd ../..