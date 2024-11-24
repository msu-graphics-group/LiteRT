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
  `brew install libomp glfw sdl2 embree`  
  *Warning*: Zenity installation might take up to two hours due to the  
  amount of dependencies. You can skip the installation and disable  
  zenity when building the project (see 'Generate build files' part).  
  Disabling zenity is not critical to run the application, it just  
  provides user-friendly file-selection interface.  
  To install zenity run:  
  `brew install zenity`
* Generate build files:  
  `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`  
  Note: if you want to disable zenity, add the correspondig flag  
  to the previous command:  
  `-DUSE_ZENITY=OFF`
* Build the application binary:  
  `cmake --build build`
* Application "NURBSViewer" will appear in "build" directory 
* Run "NURBSViewer" from any working directory.  
  For example, from project root directory run:  
  `./build/NURBSViewer`

