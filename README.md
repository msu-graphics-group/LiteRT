# LiteRT

## Build
Clone this repo with all its submodules:

    $ git clone https://github.com/msu-graphics-group/LiteRT.git
    $ git submodule update --init --recursive

### Build (CPU)

    $ cmake -S . -B build -DUSE_VULKAN=OFF -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Release -DUSE_STB_IMAGE=ON
    $ cmake --build build -j8

### Build (GPU)

Install GLFW and glsl tools

    sudo apt-get install libglfw3-dev
    sudo apt-get install glslang-tools

For GPU with compute shaders:

    $ cmake -S . -B build -DUSE_VULKAN=ON -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Release -DUSE_STB_IMAGE=ON 

Then run:

    cmake --build build -j8

## Launch

    $ ./engine -scene <scene_name.xml>
  for launching real-time renderer (available only with -DUSE_VULKAN=ON)

  There are a few xml test scenes in "scenes" folder. Test the command below to make
  sure the engine is working (first launch can take a few minutes due to shaders compilation)
    $ ./engine -scene scenes/01_simple_scenes/bunny.xml
  
  SCom Tree scenes can be obtained through the benchmark. Benchmark is currently available only on CPU.
  Provided a config (.blk file) and a model (.obj file), it will create and .xml file for original model
  and one or more SCom Trees (.bin + .xml) according to presents in config. These files will later be 
  rendered and saved to benchmark/saves/<model_name>/models/SDF_COCTREE_V3/<config_name>.bin/ (.xml)/
  Rendered images can be found in benchmark/saves/<model_name>/MR/CPU/SDF_COCTREE_V3

  Create 3 SCom Trees (depth 6, depth 7, depth 7 with LODs) from a Stanford Bunny model:
    $ ./benchmark_app --conf benchmark/demo_config.blk -m models/Bunny.obj
  
  Render SCom Tree with LODs:
    $ ./engine -scene benchmark/saves/Bunny/models/SDF_COCTREE_V3/d_7l.xml
  
  Level of detail will increase as you move further away, you can set fixed LODs and change it manually
  with a slider in the app.


