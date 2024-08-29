Installation guide (Ubuntu):
1) install submodules:
  git submodule update --init
2) install cmake:
  sudo apt install cmake
3) install dependencies:
  sudo apt install libglfw libsdl2-dev
2) make "build" folder in the project directory:
  mkdir build
  cd ./build 
3) build an application:
  cmake .. -DCMAKE_BUILD_TYPE=Release 
  make -j 8 
4) Application "NURBSViewer" will appear in "build" directory 
5) Run "NURBSViewer" from any working directory:
  <path_to_build_dir...>/NURBSViewer