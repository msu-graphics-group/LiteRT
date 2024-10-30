# LiteRT

## Build
Clone this repo with all its submodules:

    $ git clone https://github.com/msu-graphics-group/LiteRT.git
    $ git submodule update --init --recursive

### Build (CPU)

    $ cmake CMakeLists.txt -DUSE_VULKAN=OFF -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON
    $ make -j8

### Build (GPU)

Install GLFW

    $ sudo apt-get install libglfw3-dev

Download and build kernel slicer (https://github.com/Ray-Tracing-Systems/kernel_slicer) somewhere outside LiteRT folder (e.g. *~/kernel_slicer*)

Generate GPU-related code and shaders (use your path to slicer folder and executable):

    $ bash slicer_execute.sh ~/kernel_slicer/ ~/kernel_slicer/kslicer 

For GPU with compute shaders:

    $ cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=OFF -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON 

For GPU with RTX (tested only on Nvidia RTX GPUs):

    $ cmake CMakeLists.txt -DUSE_VULKAN=ON -DUSE_RTX=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON

Then run:

    make -j8

## Launch

    $ ./engine
  for launching real-time renderer (available only with -DUSE_VULKAN=ON)

    $ ./render_app -tests_litert
  for different rendering tests,
  
    $ ./render_app -tests_dr
  for differentiable rendering tests.

Performance benchmarks. You mush have a folder in *LiteRT/saves* with file named *mesh.vsgf*

    $ bash main_benchmark.sh <model_folder>
    $ bash RTX_benchmark.sh  <model_folder>
  
results will be put into LiteRT/saves/<model_folder>/results.txt

When you first launch a benchmark, SDFs will be built and saved in
*LiteRT/saves/<model_folder>* (in separate subfolders). Make sure that you benchmark script has something like "./render_app -benchmark $1 build" uncommented
