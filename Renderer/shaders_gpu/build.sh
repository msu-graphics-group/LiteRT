#!/bin/sh
glslangValidator -V CastRaySingleMega.comp -o CastRaySingleMega.comp.spv -DGLSL -I.. -I/home/egorf/kernel_slicer/TINYSTL -I/home/egorf/kernel_slicer/apps -I/home/egorf/kernel_slicer/apps/LiteMath -I/home/egorf/LiteRT/sdfScene 
glslangValidator -V PackXYMega.comp -o PackXYMega.comp.spv -DGLSL -I.. -I/home/egorf/kernel_slicer/TINYSTL -I/home/egorf/kernel_slicer/apps -I/home/egorf/kernel_slicer/apps/LiteMath -I/home/egorf/LiteRT/sdfScene 
