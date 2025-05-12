# HydraCore3
Modern rendering core: spec, vulkan (by kernel_slicer) and other

# Build (CPU)
1) clone this repo with all its submodules
   * git clone --recurse-submodules git@github.com:Ray-Tracing-Systems/HydraCore3.git
   * or use git submodule init && git submodule apdate after clone
2) Disable Vulkan by '-DUSE_VULKAN=OFF'
3) Build normally with Cmake 

# Build (CPU and GPU) [Linux only for now]
1) clone https://github.com/Ray-Tracing-Systems/kernel_slicer
2) Build kernel_slicer
3) clone this repo with all its submodules
   * git clone --recurse-submodules git@github.com:Ray-Tracing-Systems/HydraCore3.git
   * or use git submodule init && git submodule apdate after clone
4) run kernel_slicer with 'Launch (HydraCore3/GLSL/External)' config
5) build shaders (by calling 'shaders_generated/build.sh')
6) build solution normally with CMake

# Build diff render (Enzyme AD)

1. install llvm-17 (both dev and not dev) and some libs
 * wget https://apt.llvm.org/llvm.sh
 * chmod +x llvm.sh
 * sudo ./llvm.sh 17
 * sudo apt-get install llvm-17-dev
 * sudo touch /usr/lib/llvm-17/bin/yaml-bench 
 * sudo apt-get install libclang-17-dev 
 * sudo apt install clang-17
 * sudo apt install libomp-17-dev
 * sudo apt-get install libzstd-dev (a compression library which is needed for Enzyme)

2. Build Enzyme 
 * Download latest release from https://github.com/EnzymeAD/Enzyme
 * cd /path/to/Enzyme/enzyme
 * mkdir build && cd build
 * cmake -G Ninja .. -DLLVM_DIR=usr/lib/llvm-17/lib/cmake/llvm -DClang_DIR=/usr/lib/cmake/clang-17
 * ninja
 * you should have 'ClangEnzyme-17.so' in 'enzyme/build/Enzyme'. You have to pass this DLL to clang via '-fplugin=...' when compile the project!

3. Build hydra with CMake and clang
 * make sure you set 'ENZYME_PLUGIN_DLL' correctly to your 'ClangEnzyme-17.so'
 * export CC=/usr/bin/clang-17
 * export CXX=/usr/bin/clang++-17
 * cmake -DCMAKE_BUILD_TYPE=Release -DUSE_ENZYME=ON -DCLANG_VERSION=17 .. 

# Development pipeline
1) Select/Find/Make a reference image to you feature
2) Implement it in renderer on CPU
3) run kernel_slicer to get GPU version and be sure that code succesefully transformed to shaders
   * you may work with CPU build only, but this is long ...  
4) Add test to python script 'testing/run_tests.py':
   * You have to clone https://github.com/Ray-Tracing-Systems/HydraAPI-tests
   * You have to run tests from 'HydraAPI-tests' repo to generate scene files
   * If you don't have access to some closed test repo which is used in test, please comment out such tests. Otherwise clone these repos.
   * Currently you have to set two variables in script: 'PATH_TO_HYDRA2_TESTS' and 'PATH_TO_HYDRA3_SCENS' (the last one is currently closed)
   * read [testing script doc](testing/testing_doc.md)
5) Add you feature to specitication in specification: 'HydraAPI-tests/doc/doc_hydra_standart/hydra_spec.tex' 

# Documentation

* [User Guide](user_guide.md)
* [Input scene format (materials)](hydra_xml_materials.md)