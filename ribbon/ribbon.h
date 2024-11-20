#pragma once 

#ifndef KERNEL_SLICER
/*
* HOST Ribbon representation (for preprocessing on CPU)
* Almost all c++ possibilities, but KSlicer has problems with included headers
*/
struct Ribbon
{
  // ribbon data
};
#endif

// Representation for BVH
struct RibbonHeader
{
  /*
  * One object information
  * Offsets in data array (see BVHRT structure)
  * No pointers, vectors, etc
  * Only trivial types: int, float, ..., struct SomeStruct{ int, int }, etc
  */
  unsigned data_offset;
};