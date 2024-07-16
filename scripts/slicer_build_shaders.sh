rm -r shaders_gpu
mkdir shaders_gpu
cd Renderer/shaders_gpu
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_gpu
cd ../..