#!/bin/sh
glslangValidator -V CastRaySingleMega.comp -o CastRaySingleMega.comp.spv -DGLSL -I.. -I/home/sammael/kernel_slicer/apps -I/home/sammael/grade/modules/LiteRT/sdfScene -I/home/sammael/kernel_slicer/TINYSTL -I/home/sammael/kernel_slicer/apps/LiteMath 
glslangValidator -V PackXYMega.comp -o PackXYMega.comp.spv -DGLSL -I.. -I/home/sammael/kernel_slicer/apps -I/home/sammael/grade/modules/LiteRT/sdfScene -I/home/sammael/kernel_slicer/TINYSTL -I/home/sammael/kernel_slicer/apps/LiteMath 
