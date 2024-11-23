## Installation guide (Linux/OSX):
* Clone the repository and install submodules:  
  `git submodule update --init`
* Install Intel ISPC compiler:
  1) Download binaries https://ispc.github.io/downloads.html
  2) Copy ispc to /usr/local/bin or any another directory from PATH
* Install brew from https://brew.sh/ (**OSX**)
* Install cmake:  
  **Linux**:  
  `sudo apt install cmake`  
  **OSX**:  
  `brew install cmake`
* Install dependencies:  
  **Linux**:  
  `sudo apt install libglfw3 libglfw3-dev libsdl2-dev libomp-dev`  
  **OSX**:  
  `brew install libomp glfw sdl2`
* build an application:  
  `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`  
  `cmake --build build`
* Application "NURBSViewer" will appear in "build" directory 
* Run "NURBSViewer" from any working directory.  
  For example, from project root directory run:  
  `./build/NURBSViewer`

