#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>
#include <memory>

#include "LiteMath.h"

using LiteMath::cross;
using LiteMath::dot;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::int2;
using LiteMath::inverse4x4;
using LiteMath::normalize;
using LiteMath::sign;
using LiteMath::to_float3;
using LiteMath::uint;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::Box4f;

#include "../ISceneObject.h"
#include "../raytrace_common.h"
#include "cbvh.h"

// main class
//
struct BVHRT : public ISceneObject
#ifndef KERNEL_SLICER  
, public ISdfSceneFunction
, public ISdfOctreeFunction
, public ISdfGridFunction
#endif
{
  //overiding ISceneObject interface
  BVHRT(const char* a_buildName = nullptr, const char* a_layoutName = nullptr) : 
    m_buildName(a_buildName != nullptr ? a_buildName : ""), 
    m_layoutName(a_layoutName != nullptr ? a_layoutName : "") { }
  ~BVHRT() override {}

  const char* Name() const override { return "BVH2Fat"; }
  const char* BuildName() const override { return m_buildName.c_str(); };

  void ClearGeom() override;

  uint32_t AddGeom_Triangles3f(const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel, size_t vByteStride) override;
#ifndef KERNEL_SLICER  
  uint32_t AddGeom_SdfScene(SdfSceneView scene, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_SdfGrid(SdfGridView grid, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_RFScene(RFScene grid, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_SdfOctree(SdfOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_SdfSVS(SdfSVSView octree, BuildOptions a_qualityLevel = BUILD_HIGH) override;
  uint32_t AddGeom_SdfSBS(SdfSBSView octree, BuildOptions a_qualityLevel = BUILD_HIGH) override;

  void set_debug_mode(bool enable) override;
#endif

  //common functions for a few Sdf...Function interfaces
#ifndef KERNEL_SLICER 
  float eval_distance(float3 pos) override;
#endif
  //overiding SdfSceneFunction interface
#ifndef KERNEL_SLICER 
  void init(SdfSceneView scene) override; 
#endif

  //overiding SdfOctreeFunction interface
#ifndef KERNEL_SLICER
  void init(SdfOctreeView octree) override; 
  float eval_distance_level(float3 pos, unsigned max_level) override;
  std::vector<SdfOctreeNode> &get_nodes() override;
  const std::vector<SdfOctreeNode> &get_nodes() const override;
#endif

  //overiding SdfGridFunction interface
#ifndef KERNEL_SLICER 
  void init(SdfGridView scene) override; 
#endif

  void ClearScene() override;
  virtual void CommitScene(BuildOptions a_qualityLevel) override;

  uint32_t AddInstance(uint32_t a_geomId, const float4x4 &a_matrix) override;
  uint32_t AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber) override
  { return AddInstance(a_geomId, a_matrices[0]); }

  void UpdateInstance(uint32_t a_instanceId, const float4x4 &a_matrix) override;

  CRT_Hit RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar) override;
  CRT_Hit RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time) override
  { return RayQuery_NearestHit(posAndNear, dirAndFar); }

  bool    RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar) override;
  bool    RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time = 0.0f) override
  { return RayQuery_AnyHit(posAndNear, dirAndFar); }
  
  uint32_t GetGeomNum() const override { return uint32_t(m_geomBoxes.size()); }
  uint32_t GetInstNum() const override { return uint32_t(m_instBoxes.size()); }
  const LiteMath::float4* GetGeomBoxes() const override { return (const LiteMath::float4*)m_geomBoxes.data(); }
  
//protected:

  void IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                    float tNear, uint32_t instId, uint32_t geomId,
                                    uint32_t a_start, uint32_t a_count,
                                    CRT_Hit *pHit);

  void IntersectAllSdfsInLeaf(const float3 ray_pos, const float3 ray_dir,
                              float tNear, uint32_t instId, uint32_t geomId,
                              uint32_t a_start, uint32_t a_count,
                              CRT_Hit *pHit);

  void IntersectRFInLeaf(const float3 ray_pos, const float3 ray_dir,
                              float tNear, uint32_t instId, uint32_t geomId,
                              uint32_t a_start, uint32_t a_count,
                              CRT_Hit *pHit);

  void RayGridIntersection(float3 ray_pos, float3 ray_dir, float3 bbMin, float3 bbMax, uint gridSize, float3 p, float3 lastP, float diff, float &throughput, float3 &colour);
  void lerpCell(const int idx0, const int idx1, const float t, float memory[28]);

  void IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit);
                                   
  void OctreeNodeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                           float tNear, uint32_t instId, uint32_t geomId,
                           uint32_t a_start, uint32_t a_count,
                           CRT_Hit *pHit);

  virtual void BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear, 
                               uint32_t instId, uint32_t geomId, uint32_t stack[STACK_SIZE], bool stopOnFirstHit,
                               CRT_Hit *pHit);

  virtual void AppendTreeData(const std::vector<BVHNodePair>& a_nodes, const std::vector<uint32_t>& a_indices, 
                              const uint32_t *a_triIndices, size_t a_indNumber);

#ifndef KERNEL_SLICER  
  std::vector<BVHNode> GetBoxes_RFGrid(RFScene grid);
  std::vector<BVHNode> GetBoxes_SdfGrid(SdfGridView grid);
  std::vector<BVHNode> GetBoxes_SdfOctree(SdfOctreeView octree);
  std::vector<BVHNode> GetBoxes_SdfFrameOctree(SdfFrameOctreeView octree);
#endif

  //helper functions to calculate SDF
  //It is a copy-past of sdfScene functions with the same names
  //Slicer is weak and can't handle calling external functions  ¯\_(ツ)_/¯
  virtual float2 box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir);
  virtual bool is_leaf(unsigned offset);
  virtual float eval_dist_trilinear(const float values[8], float3 dp);
  virtual bool need_normal();

#ifndef LITERT_MINI
  virtual float eval_dist_prim(unsigned prim_id, float3 p);
  virtual float sdf_octree_sample_mipskip_3x3(unsigned octree_id, float3 p, unsigned max_level);
  virtual float sdf_octree_sample_mipskip_closest(unsigned octree_id, float3 p, unsigned max_level);
  virtual float sdf_octree_sample_closest(unsigned octree_id, float3 p, unsigned max_level);
  virtual float eval_dist_sdf_conjunction(unsigned conj_id, float3 p);
#endif

  virtual float eval_distance_sdf_grid(unsigned grid_id, float3 p);
  virtual float eval_distance_sdf_octree(unsigned octree_id, float3 p, unsigned max_level);
  virtual float eval_distance_sdf_frame_octree(unsigned octree_id, float3 p);

  virtual float eval_distance_sdf(unsigned type, unsigned prim_id, float3 p);
  virtual SdfHit sdf_sphere_tracing(unsigned type, unsigned prim_id, const float3 &min_pos, const float3 &max_pos,
                                    const float3 &pos, const float3 &dir, bool need_norm);    
  //for each model in scene  
  std::vector<Box4f>    m_geomBoxes;
  std::vector<uint2>    m_geomOffsets; //means different things for different types of geometry
  std::vector<uint32_t> m_bvhOffsets;
  std::vector<uint32_t> m_geomTypeByGeomId;

  //SDFs data
#ifndef LITERT_MINI
  std::vector<float> m_SdfParameters;
  std::vector<SdfObject> m_SdfObjects;
  std::vector<SdfConjunction> m_SdfConjunctions;
  std::vector<NeuralProperties> m_SdfNeuralProperties;
  std::vector<uint32_t> m_ConjIndices; //conjunction index for each leaf node in Fat BVH related to SDF
#endif

  //SDF grid data
#ifndef LITERT_MINI
  std::vector<float> m_SdfGridData;       //raw data for all SDF grids
  std::vector<uint32_t> m_SdfGridOffsets; //offset in m_SdfGridData for each SDF grid
  std::vector<uint3> m_SdfGridSizes;      //size for each SDF grid
#endif

  // RF grid data
  std::vector<float> m_RFGridData;       //raw data for all RF grids
  std::vector<uint32_t> m_RFGridOffsets; //offset in m_SdfGridData for each RF grid
  std::vector<size_t> m_RFGridSizes;      //size for each RF grid
  std::vector<float> m_RFGridScales;      //size for each RF grid

  //SDF octree data
#ifndef LITERT_MINI
  std::vector<SdfOctreeNode> m_SdfOctreeNodes;//nodes for all SDF octrees
  std::vector<uint32_t> m_SdfOctreeRoots;     //root node ids for each SDF octree
#endif

  //SDF frame octree data
  std::vector<SdfFrameOctreeNode> m_SdfFrameOctreeNodes;//nodes for all SDF octrees
  std::vector<uint32_t> m_SdfFrameOctreeRoots;     //root node ids for each SDF octree
  std::vector<BVHNode> m_origNodes;

  //SDF Sparse Voxel Sets
  std::vector<SdfSVSNode> m_SdfSVSNodes;//nodes for all SDF Sparse Voxel Sets
  std::vector<uint32_t> m_SdfSVSRoots;     //root node ids for each SDF Sparse Voxel Set

  //SDF Sparse Brick Sets
#ifndef LITERT_MINI
  std::vector<SdfSBSNode>   m_SdfSBSNodes;   //nodes for all SDF Sparse Brick Sets
  std::vector<uint32_t>     m_SdfSBSData;    //raw data for all Sparse Brick Sets
  std::vector<uint32_t>     m_SdfSBSRoots;   //root node ids for each SDF Sparse Voxel Set
  std::vector<SdfSBSHeader> m_SdfSBSHeaders; //header for each SDF Sparse Voxel Set
  std::vector<uint2>        m_SdfSBSRemap;   //primId->nodeId, required as each SBS node can have >1 bbox in BLAS
#endif

  //for each instance in scene
  std::vector<Box4f> m_instBoxes;
  std::vector<uint32_t> m_geomIdByInstId;
  std::vector<float4x4> m_instMatricesInv; ///< inverse instance matrices
  std::vector<float4x4> m_instMatricesFwd; ///< instance matrices

  //meshes data
  std::vector<float4>   m_vertPos;
  std::vector<uint32_t> m_indices;
  std::vector<uint32_t> m_primIndices;

  //Top Level Acceleration Structure
  std::vector<BVHNode>    m_nodesTLAS;

  //Bottom Level Acceleration Structure
  std::vector<BVHNodePair> m_allNodePairs;


  // Format name the tree build from
  const std::string m_buildName;
  const std::string m_layoutName;

  bool m_firstSceneCommit = true;
  bool debug_cur_pixel = false;
};
