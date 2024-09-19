rm -r shaders_rtx
mkdir shaders_rtx
cd Renderer/shaders_rtx
bash build.sh
find -name "*.spv" | xargs cp --parents -t ../../shaders_rtx
cd ../..