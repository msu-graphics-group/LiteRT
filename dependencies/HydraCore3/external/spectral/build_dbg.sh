#!/bin/bash

cmake -G "Ninja" -DSPECTRAL_LIB_ONLY=0 -DCMAKE_BUILD_TYPE=Debug -B ./build_debug
cd ./build_debug
ninja 