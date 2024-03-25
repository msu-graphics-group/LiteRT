#pragma once

#include <vector>
#include <cstdint>
#include <cassert>

#include "LiteMath.h"

  using LiteMath::float4;
  using LiteMath::float3;
  using LiteMath::uint;
  using LiteMath::uint2;
  using LiteMath::uint4;
  
  constexpr unsigned int LEAF_NORMAL = 0xFFFFFFFF; ///!<
  constexpr unsigned int LEAF_EMPTY  = 0xFFFFFFFD; ///!<
  constexpr unsigned int ESCAPE_ROOT = 0xFFFFFFFE; ///!< when escapeIndex (for stackless traversal) points out of tree

  struct BVHNode 
  {
    float3 boxMin;
    uint   leftOffset; //!< please note that when LEAF_BIT (0x80000000) is set in leftOffset, this node is a leaf
    float3 boxMax;
    uint   escapeIndex;
  };

  struct BVHNodePair
  { 
    BVHNode left;
    BVHNode right;
  };

#ifndef KERNEL_SLICER
  enum FMT{ BVH2_LEFT_OFFSET = 1, //!< Children: (leftOffset, leftOffset+1); 'escapeIndex' is a true escapeIndex;
            BVH2_LEFT_RIGHT  = 2, //!< Children: (leftOffset, rightOffset is 'escapeIndex'); actual escapeIndex is not stored;
            BVH4_LEFT_OFFSET = 3, //!< Children: (leftOffset, leftOffset+1,leftOffset+2,leftOffset+3); escapeIndex is a true escapeIndex;
            BVH2_LEFT_ROPES  = 4, //!< Children: (leftOffset, unknown); 'escapeIndex' is a true escapeIndex; This format is ONLY for stackless traversal
            };

  enum TLAYOUT{ LAYOUT_DFS         = 0, ///<! Depth First
                LAYOUT_ODFS        = 1, ///<! Ordered Depth First
                LAYOUT_BFS         = 2, ///<! Breadth First
                LAYOUT_COLBVH_TRB1 = 3, ///<! Treelet: analogue to [AK10] 
                LAYOUT_COLBVH_TRB2 = 4, ///<! Treelets Super: [AK10]+[YM06]
                LAYOUT_COLBVH_TRB3 = 5, ///<! Treelets Super Super: [AK10]+[YM06]
                LAYOUT_CALBVH      = 6, ///<! our CALBVH implementation, please note that 'BuildBVHFat' function don't compress bvh itself!
                LAYOUT_COLBVH_YM06 = 7, ///<! [YM06] clusters approach
              };
  enum class BVHQuality
  {
    LOW = 0,
    MEDIUM = 1,
    HIGH = 2
  };
  struct BuilderPresets
  {
    FMT   format      = BVH2_LEFT_OFFSET;
    BVHQuality quality = BVHQuality::HIGH;  
    int   primsInLeaf = 2;                      ///<! recomended primitives in leaf
  };

  struct LayoutPresets
  {
    TLAYOUT layout = LAYOUT_COLBVH_TRB3; ///<! 
    int     grSzId = 1;                  ///<! Group Size Id; 0,1,2,3,4; Please note this is just id, rel group size could be evaluated via 'CalcActualGroupSize' function
    int     grSzXX = 0;                  ///<! Group Size Internal; This in internal parameter, don't use it please!
  };

  enum CBVH_FORMATS
  {
    FMT_BVH2Node32_Interval32_Static = 1,  //!< Children: (leftOffset, leftOffset+1); escapeIndex is true escapeIndex;
    FMT_BVH2Node32_Interval32_Dynamic = 2, //!< Children: (leftOffset, rightOffset == escapeIndex); true escapeIndex is not stored;
    FMT_BVH4Node32_Interval32_Static = 3,  //!< Children: (leftOffset, leftOffset+1,leftOffset+2,leftOffset+3); escapeIndex is true escapeIndex;

    FMT_BVH_UNKNOWN = -1, //!< if passed to builder via 'BVHPresets', builder will select some format according to other settings
  };

  struct BVHPresets
  {
    int  primsInLeaf  = 4;                             ///<! recomended primitives in leaf
    int  childrenNum  = 4;                             ///<! (2,4,8,16,32) 
    int  maxThreads   = 0;                             ///<! 0 means unbounden number of threads; please note that current implementation is single threaded!
    BVHQuality quality = BVHQuality::HIGH;
    CBVH_FORMATS     desiredFormat = FMT_BVH4Node32_Interval32_Static; ///<! desired format of BVH; if required format is not possiable for current presets (not compatible with childrenNum or not implemented for example), different format will be selected.
  };

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  struct BVHTreeCommon
  {
    BVHTreeCommon(){}
    BVHTreeCommon(const std::vector<BVHNode>& a_nodes, const std::vector<uint32_t>& a_indices) : nodes(a_nodes), indices(a_indices) { } 
    std::vector<BVHNode>  nodes;
    std::vector<uint32_t> indices;
  };

  struct BVHTreeFat
  {
    BVHTreeFat(){}
    BVHTreeFat(const std::vector<BVHNodePair>& a_nodes, const std::vector<uint32_t>& a_indices) : nodes(a_nodes), indices(a_indices) { }  
    
    std::vector<BVHNodePair>  nodes;
    std::vector<uint32_t>    indices;
  };

  struct Interval
  {
    Interval() : start(0), count(0) {}
    Interval(uint a, uint b) : start(a), count(b) {}

    uint start;
    uint count;
  };

  /**
  \brief Linearized layout bvh tree
  */
  struct BVHTree
  {
    BVHTree(){}
    BVHTree(const std::vector<BVHNode>&  a_nodes, 
            const std::vector<Interval>& a_intervals,
            const std::vector<uint32_t>& a_indices) : nodes(a_nodes), intervals(a_intervals), indicesReordered(a_indices) {}


    std::vector<BVHNode>  nodes;            //!< nodes stored strongly in bread-first order
    std::vector<Interval> intervals;        //!< 
    std::vector<uint32_t> indicesReordered; //!< reordered index buffer. may not used for TLAS and custom user objects, i.e. indicesReordered.size() will be equal to zero
                                            //!< in this case, reordering of objects in memory is not happened
                                            //!< for triangle meshes indicesReordered.size() == numTris*3, for custom objects indicesReordered.size() == a_objNum or zero (if reordering didn't happened).

    unsigned int          geomID;           //!< identifier that builder writes, but user could overwrite it if wants

    CBVH_FORMATS          format;           //!< actual format according to specification (you need to treat 'escapeIndex' field differently for 'FMT_BVH4Node32_Interval32_Static' format)
    unsigned int          leavesNumber = 0; //!< filled by the LBVH builder only

    void Print(std::ostream& out);          //!< for debug needs
  };

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  BuilderPresets BuilderPresetsFromString(const char* a_str);
  LayoutPresets  LayoutPresetsFromString(const char* a_str);

  /**
  \brief Main Builder C++ interface, Build both TLAS (Top Level Acceleration Structure) and BLAS for custom user input.

  \param  a_nodes   - input bounding boxes with 2 custom fields (leftOffset and escapeIndex) wich are unused in general
  \param  a_objNum  - input bounding boxes number
  \param  a_presets - input builder presets

  \return linearized layout bvh tree

  User may pack his custom data inside two integer fields of BVHNode: (leftOffset and escapeIndex).
  These fields are ignored by the builder. 

  */
  BVHTree BuildBVH(const BVHNode* a_nodes, size_t a_objNum, BuilderPresets a_presets = BuilderPresets());

  /**
  \brief Main Builder C++ interface, build BLAS (Bottom Level Acceleration Structure) for input triangle mesh.

  \param  a_vertices    - input triangle vertices; 
  \param  a_vertNum     - input verices count
  \param  a_vByteStride - input stride in bytes between vertices in 'a_vpos3f' (12 for float3, 16 flor float4 and e.t.c.)

  \param  a_indices     - input index buffer
  \param  a_indexNum    - input indices number
  \param  a_presets     - input builder presets

  \return linearized layout bvh tree in two arrays 
  */
  BVHTreeCommon BuildBVH(const float* a_vpos3f,     size_t a_vertNum, size_t a_vByteStride, 
                         const uint32_t* a_indices, size_t a_indexNum, BuilderPresets a_presets = BuilderPresets());

  /**
  \brief Main Builder C++ interface, build BLAS (Bottom Level Acceleration Structure) for input triangle mesh.

  \param  a_vertices    - input triangle vertices; 
  \param  a_vertNum     - input verices count
  \param  a_vByteStride - input stride in bytes between vertices in 'a_vpos3f' (12 for float3, 16 flor float4 and e.t.c.)
  \param  a_indices     - input index buffer
  \param  a_indexNum    - input indices number

  \param  a_presets     - input builder presets
  \param  a_layout      - input tree layout

  \return linearized layout bvh tree in two arrays 
  */
  BVHTreeFat BuildBVHFat(const float* a_vpos3f,     size_t a_vertNum,  size_t a_vByteStride, 
                         const uint32_t* a_indices, size_t a_indexNum, BuilderPresets a_presets, LayoutPresets a_layout);

  /**
  \brief Build BLAS (Bottom Level Acceleration Structure) for custom geometry.
  \return linearized layout bvh tree in two arrays 
  */
  BVHTreeFat BuildBVHFatCustom(const BVHNode* a_nodes, size_t a_objNum, BuilderPresets a_presets, LayoutPresets a_layout);

  BVHTree BuildBVHEmbree(const float4 *a_vertices, size_t a_vertNum, const uint *a_indices, size_t a_indexNum, BVHPresets a_presets);
#endif
