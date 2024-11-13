Installation guide (Ubuntu):
* install submodules:
  git submodule update --init
* install Intel ISPC compiler:
  1) Download binaries https://ispc.github.io/downloads.html
  2) copy ispc to /usr/local/bin or any another directory from PATH
* install cmake:
  sudo apt install cmake
* install dependencies:
  sudo apt install libglfw3 libglfw3-dev libsdl2-dev libomp-dev
* make "build" folder in the project directory:
  mkdir build
  cd ./build 
* build an application:
  cmake .. -DCMAKE_BUILD_TYPE=Release 
  make -j 8 
* Application "NURBSViewer" will appear in "build" directory 
* Run "NURBSViewer" from any working directory:
  <path_to_build_dir...>/NURBSViewer
