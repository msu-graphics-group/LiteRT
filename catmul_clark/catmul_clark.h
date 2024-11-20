#pragma once 

#ifndef KERNEL_SLICER
/*
* HOST Catmul Clark representation (for preprocessing on CPU)
* Almost all c++ possibilities, but KSlicer has problems with included headers
*/
struct CatmulClark
{
  // catmul clark data
};
#endif

// Representation for BVH
struct CatmulClarkHeader
{
  /*
  * One object information
  * Offsets in data array (see BVHRT structure)
  * No pointers, vectors, etc
  * Only trivial types: int, float, ..., struct SomeStruct{ int, int }, etc
  */
  unsigned data_offset;
};