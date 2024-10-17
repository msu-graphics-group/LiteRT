#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>
#include <memory>

#include "CrossRT.h"
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
#include "nurbs/nurbs_common.h"
#include "harmonic_function/any_polygon_common.h"
#include "graphics_primitive/graphics_primitive_common.h"

float3 tricubicInterpolationDerrivative(const float *m_SdfGridData, const uint vox_u[3], const float dp[3], const uint32_t off, const uint size[3]);
float tricubicInterpolation(const float *m_SdfGridData, const uint vox_u[3], const float dp[3], const uint32_t off, const uint size[3]);

struct BVHRT;
struct GeomData
{
  float4 boxMin;
  float4 boxMax;

  uint32_t bvhOffset;
  uint32_t type; // enum GeomType
  uint2 offset;
};

struct AbstractObject
{
  static constexpr uint32_t TAG_NONE             = 0;    // !!! #REQUIRED by kernel slicer: Empty/Default impl must have zero both m_tag and offset
  static constexpr uint32_t TAG_TRIANGLE         = 1; 
  static constexpr uint32_t TAG_SDF_GRID         = 2; 
  static constexpr uint32_t TAG_SDF_NODE         = 3; 
  static constexpr uint32_t TAG_SDF_BRICK        = 4;
  static constexpr uint32_t TAG_RF               = 5;
  static constexpr uint32_t TAG_GS               = 6;
  static constexpr uint32_t TAG_SDF_ADAPT_BRICK  = 7;
  static constexpr uint32_t TAG_NURBS            = 8; 
  static constexpr uint32_t TAG_GRAPHICS_PRIM    = 9;
  static constexpr uint32_t TAG_ANY_POLYGON      = 10;

  AbstractObject(){}  // Dispatching on GPU hierarchy must not have destructors, especially virtual   
  virtual uint32_t GetTag()   const  { return TAG_NONE; }; // !!! #REQUIRED by kernel slicer
  virtual uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, CRT_Hit* pHit, BVHRT* pData)   const  { return 0; }; // !!! #REQUIRED by kernel slicer
  //uint64_t vtable is here!
  uint32_t m_tag;  // !!! #REQUIRED by kernel slicer with this specific name!
  uint32_t geomId; //geometry Id (index in the list of all objects on scene, regardless of the type)
};

// common data for arch instance on scene
struct InstanceData
{
  float4 boxMin;
  float4 boxMax;
  uint32_t geomId;
  uint32_t instStart;
  uint32_t instEnd;
  uint32_t _pad[5];
  float4x4 transform;
  float4x4 transformInv;
  float4x4 transformInvTransposed; //for normals
};

// main class
//
struct BVHRT : public ISceneObject
#ifndef KERNEL_SLICER  
, public ISdfGridFunction
#endif
{
  //overiding ISceneObject interface
  BVHRT(const char* a_buildName = nullptr, const char* a_layoutName = nullptr) : 
    m_buildName(a_buildName != nullptr ? a_buildName : ""), 
    m_layoutName(a_layoutName != nullptr ? a_layoutName : "") 
    {
      m_preset = getDefaultPreset();
    }
  ~BVHRT() override {}

  const char* Name() const override { return "BVH2Fat"; }

  void ClearGeom() override;

  uint32_t AddCustomGeom_FromFile(const char *geom_type_name, const char *filename, ISceneObject *fake_this) override;

  uint32_t AddGeom_Triangles3f(const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride) override;

  //generic version, should not be used, use simplier version below
  uint32_t AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;
  void     UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;

#ifndef KERNEL_SLICER  
  uint32_t AddGeom_Triangles3f(const float* a_vpos3f, const float* a_vnorm3f, size_t a_vertNumber, const uint32_t* a_triIndices, 
                               size_t a_indNumber, BuildOptions a_qualityLevel, size_t vByteStride);
  uint32_t AddGeom_SdfGrid(SdfGridView grid, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_RFScene(RFScene grid, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_GSScene(GSScene grid, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_SdfSVS(SdfSVSView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_SdfSBS(SdfSBSView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_SdfSBSAdapt(SdfSBSAdaptView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_SdfFrameOctreeTex(SdfFrameOctreeTexView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_NURBS(const RawNURBS &nurbs, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_GraphicsPrim(const GraphicsPrimView &nurbs, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);
  uint32_t AddGeom_AnyPolygon(const AnyPolygon &poly, ISceneObject *fake_this, BuildOptions a_qualityLevel = BUILD_HIGH);

  void set_debug_mode(bool enable);
#endif

  void SetPreset(const MultiRenderPreset& a_preset){ m_preset = a_preset; }
  MultiRenderPreset GetPreset() const { return m_preset; }

  //common functions for a few Sdf...Function interfaces
#ifndef KERNEL_SLICER 
  float eval_distance(float3 pos) override;
#endif

  //overiding SdfGridFunction interface
#ifndef KERNEL_SLICER 
  void init(SdfGridView scene) override; 
#endif

  void ClearScene() override;
  virtual void CommitScene(uint32_t a_qualityLevel) override;

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

  void IntersectGSInLeaf(const float3& ray_pos, const float3& ray_dir,
                         float tNear, uint32_t instId,
                         uint32_t geomId, uint32_t a_start,
                         uint32_t a_count, CRT_Hit* pHit);

  void IntersectNURBS(const float3& ray_pos, const float3& ray_dir,
                      float tNear, uint32_t instId,
                      uint32_t geomId, CRT_Hit* pHit);

  void IntersectGraphicPrims(const float3& ray_pos, const float3& ray_dir,
                             float tNear, uint32_t instId,
                             uint32_t geomId, uint32_t a_start, uint32_t a_count, CRT_Hit* pHit);

  void IntersectAnyPolygon(const float3& ray_pos, const float3& ray_dir,
                           float tNear, uint32_t instId,
                           uint32_t geomId, uint32_t a_start, uint32_t a_count, CRT_Hit* pHit);

  void RayGridIntersection(float3 ray_dir, uint gridSize, float3 p, float3 lastP, uint4 ptrs, uint4 ptrs2, float &throughput, float3 &colour);
  void lerpCell(const uint idx0, const uint idx1, const float t, float memory[28]);

  void IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit);
                                   
  void OctreeNodeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                           float tNear, uint32_t instId, uint32_t geomId,
                           uint32_t a_start, uint32_t a_count,
                           CRT_Hit *pHit);

  void RayNodeIntersection(uint32_t type, const float3 ray_pos, const float3 ray_dir, 
                           float tNear, uint32_t geomId, uint32_t bvhNodeId, 
                           float values[8], uint32_t &primId, uint32_t &nodeId, float &d, 
                           float &qNear, float &qFar, float2 &fNearFar, float3 &start_q);

  void LocalSurfaceIntersection(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                float values[8], uint32_t nodeId, uint32_t primId, float d, float qNear, 
                                float qFar, float2 fNearFar, float3 start_q,
                                CRT_Hit *pHit);
  
  void OctreeBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                            float tNear, uint32_t instId, uint32_t geomId,
                            uint32_t a_start, uint32_t a_count,
                            CRT_Hit *pHit);
  
  void OctreeAdaptBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                 float tNear, uint32_t instId, uint32_t geomId,
                                 uint32_t a_start, uint32_t a_count,
                                 CRT_Hit *pHit);

  virtual void BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear, 
                               uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                               CRT_Hit *pHit);

  virtual void AppendTreeData(const std::vector<BVHNodePair>& a_nodes, const std::vector<uint32_t>& a_indices, 
                              const uint32_t *a_triIndices, size_t a_indNumber);

#ifndef KERNEL_SLICER  
  std::vector<BVHNode> GetBoxes_RFGrid(RFScene grid, std::vector<float>& sparseGrid, std::vector<uint4>& sparsePtrs);
  std::vector<BVHNode> GetBoxes_GSGrid(const GSScene& grid);
  std::vector<BVHNode> GetBoxes_SdfGrid(SdfGridView grid);
  std::vector<BVHNode> GetBoxes_SdfFrameOctree(SdfFrameOctreeView octree);
  std::vector<BVHNode> GetBoxes_SdfFrameOctreeTex(SdfFrameOctreeTexView octree);
#endif

  //helper functions to calculate SDF
  //It is a copy-past of sdfScene functions with the same names
  //Slicer is weak and can't handle calling external functions  ¯\_(ツ)_/¯
  virtual float2 box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir);
  virtual float eval_dist_trilinear(const float values[8], float3 dp);
  virtual bool need_normal();
  virtual float2 encode_normal(float3 n);
  float load_distance_values(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, float values[8]);
  float tricubicInterpolation(const uint32_t vox_u[3], const float dp[3], const uint32_t off, const uint32_t size[3]);

#ifndef DISABLE_SDF_GRID
  virtual float eval_distance_sdf_grid(unsigned grid_id, float3 p);
#endif 
#ifndef DISABLE_SDF_FRAME_OCTREE
  virtual float eval_distance_sdf_frame_octree(unsigned octree_id, float3 p);
#endif
  virtual float eval_distance_sdf(unsigned type, unsigned prim_id, float3 p);
  virtual SdfHit sdf_sphere_tracing(unsigned type, unsigned prim_id, const float3 &min_pos, const float3 &max_pos,
                                    float tNear, const float3 &pos, const float3 &dir, bool need_norm);    

  //SDF grid data
#ifndef DISABLE_SDF_GRID
  std::vector<float> m_SdfGridData;       //raw data for all SDF grids
  std::vector<uint32_t> m_SdfGridOffsets; //offset in m_SdfGridData for each SDF grid
  std::vector<uint3> m_SdfGridSizes;      //size for each SDF grid
#endif

  // RF grid data
#ifndef DISABLE_RF_GRID
  std::vector<float> m_RFGridData;       //raw data for all RF grids
  std::vector<uint4> m_RFGridPtrs;       //raw data for all RF grids
  std::vector<uint32_t> m_RFGridOffsets; //offset in m_SdfGridData for each RF grid
  std::vector<size_t> m_RFGridSizes;      //size for each RF grid
  std::vector<float> m_RFGridScales;      //size for each RF grid
  std::vector<uint32_t> m_RFGridFlags;      //size for each RF grid
  #ifdef DISABLE_SDF_FRAME_OCTREE
    std::vector<BVHNode> m_origNodes;
  #endif
#endif

  // GS data
#ifndef DISABLE_GS_PRIMITIVE
  std::vector<float4x4> m_gs_data_0{};
  std::vector<float4x4> m_gs_conic{};
#endif

  //SDF frame octree data
#ifndef DISABLE_SDF_FRAME_OCTREE
  std::vector<SdfFrameOctreeNode> m_SdfFrameOctreeNodes;//nodes for all SDF octrees
  std::vector<uint32_t> m_SdfFrameOctreeRoots;     //root node ids for each SDF octree
  std::vector<BVHNode> m_origNodes;
#endif

  //SDF Sparse Voxel Sets
#ifndef DISABLE_SDF_SVS
  std::vector<SdfSVSNode> m_SdfSVSNodes;//nodes for all SDF Sparse Voxel Sets
  std::vector<uint32_t> m_SdfSVSRoots;     //root node ids for each SDF Sparse Voxel Set
#endif

  //SDF Sparse Brick Sets
#ifndef DISABLE_SDF_SBS
  std::vector<SdfSBSNode>   m_SdfSBSNodes;   //nodes for all SDF Sparse Brick Sets
  std::vector<uint32_t>     m_SdfSBSData;    //raw data for all Sparse Brick Sets
  std::vector<float>        m_SdfSBSDataF;    //raw float data for all indexed Sparse Brick Sets
  std::vector<uint32_t>     m_SdfSBSRoots;   //root node ids for each SDF Sparse Voxel Set
  std::vector<SdfSBSHeader> m_SdfSBSHeaders; //header for each SDF Sparse Voxel Set
#endif

  //SDF Adaptive Sparse Brick Sets
#ifndef DISABLE_SDF_SBS_ADAPT
  std::vector<SdfSBSAdaptNode>   m_SdfSBSAdaptNodes;   //nodes for all SDF Sparse Brick Sets
  std::vector<uint32_t>          m_SdfSBSAdaptData;    //raw data for all Sparse Brick Sets
  std::vector<float>             m_SdfSBSAdaptDataF;   //raw float data for all indexed Sparse Brick Sets
  std::vector<uint32_t>          m_SdfSBSAdaptRoots;   //root node ids for each SDF Sparse Voxel Set
  std::vector<SdfSBSAdaptHeader> m_SdfSBSAdaptHeaders; //header for each SDF Sparse Voxel Set
#endif

  //SDF textured frame octree data
#ifndef DISABLE_SDF_FRAME_OCTREE_TEX
  std::vector<SdfFrameOctreeTexNode> m_SdfFrameOctreeTexNodes;//nodes for all SDF octrees
  std::vector<uint32_t> m_SdfFrameOctreeTexRoots;          //root node ids for each SDF octree
  #ifdef DISABLE_SDF_FRAME_OCTREE
    #ifdef DISABLE_RF_GRID
      std::vector<BVHNode> m_origNodes;
    #endif
  #endif
#endif

  //NURBS data
  std::vector<float> m_NURBSData;
  std::vector<NURBSHeader> m_NURBSHeaders;

  // Graphic primitives data
#ifndef DISABLE_GRAPHICS_PRIM
  std::vector<float4> m_GraphicsPrimPoints;
  std::vector<uint32_t> m_GraphicsPrimRoots;
  std::vector<GraphicsPrimHeader> m_GraphicsPrimHeaders;
#endif

  // Non-planar polygons data
#ifndef DISABLE_ANY_POLYGON
  std::vector<float3> m_AnyPolygonVertices;
  std::vector<float3> m_AnyPolygonTriangles;
  std::vector<AnyPolygonDataHeader> m_AnyPolygonHeaders;
#endif // !defined(DISABLE_ANY_POLYGON)

  //meshes data
  std::vector<float4>   m_vertPos;
  std::vector<float4>   m_vertNorm;
  std::vector<uint32_t> m_indices;
  std::vector<uint32_t> m_primIndices;

  //geometric data, indexed by geomId
  std::vector<GeomData> m_geomData;
  std::vector<AbstractObject>   m_abstractObjects;
  std::vector<AbstractObject *> m_abstractObjectPtrs;
  std::vector<uint2>    startEnd;

  //instance data
  std::vector<InstanceData> m_instanceData;

  //AABB data, indexed by globalAABBId
  std::vector<uint32_t> m_primIdCount; //for each AABB stores start index and number of primitives stored in this AABB

  //Top Level Acceleration Structure
  std::vector<BVHNode>    m_nodesTLAS;

  //Bottom Level Acceleration Structure
  std::vector<BVHNodePair> m_allNodePairs;


  // Format name the tree build from
  const std::string m_buildName;
  const std::string m_layoutName;

  bool m_firstSceneCommit = true;
  bool debug_cur_pixel = false;
  
  MultiRenderPreset m_preset;
};

static bool need_normal(MultiRenderPreset preset)
{
  return preset.render_mode == MULTI_RENDER_MODE_LAMBERT_NO_TEX || 
         preset.render_mode == MULTI_RENDER_MODE_NORMAL  ||
         preset.render_mode == MULTI_RENDER_MODE_PHONG_NO_TEX ||
         preset.render_mode == MULTI_RENDER_MODE_LAMBERT ||
         preset.render_mode == MULTI_RENDER_MODE_PHONG;
}

struct EmptyGeomData : public AbstractObject
{
  EmptyGeomData() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_NONE; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* pData)   const override
  {
    return TAG_NONE;
  }
};

struct GeomDataTriangle : public AbstractObject
{
  GeomDataTriangle() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_TRIANGLE; }  

  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt) const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectAllTrianglesInLeaf(ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_TRIANGLE;
  }  
};

struct GeomDataSdfGrid : public AbstractObject
{
  GeomDataSdfGrid() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_SDF_GRID; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectAllSdfsInLeaf(ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_SDF_GRID;
  }
};

struct GeomDataSdfNode : public AbstractObject
{
  GeomDataSdfNode() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_SDF_NODE; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt) const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);
    uint32_t type = bvhrt->m_geomData[geometryId].type;

    bvhrt->OctreeNodeIntersect(type, ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_SDF_NODE;
  }
};

struct GeomDataSdfBrick : public AbstractObject
{
  GeomDataSdfBrick() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_SDF_BRICK; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);
    uint32_t type = bvhrt->m_geomData[geometryId].type;

    bvhrt->OctreeBrickIntersect(type, ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_SDF_BRICK;
  }
};

struct GeomDataSdfAdaptBrick : public AbstractObject
{
  GeomDataSdfAdaptBrick() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_SDF_ADAPT_BRICK; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);
    uint32_t type = bvhrt->m_geomData[geometryId].type;

    bvhrt->OctreeAdaptBrickIntersect(type, ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_SDF_ADAPT_BRICK;
  }
};

struct GeomDataRF : public AbstractObject
{
  GeomDataRF() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_RF; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
#ifndef DISABLE_RF_GRID
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectRFInLeaf(ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
#endif
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_RF;
  }
};

struct GeomDataGS : public AbstractObject
{
  GeomDataGS() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_GS; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
#ifndef DISABLE_GS_PRIMITIVE
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectGSInLeaf(ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
#endif
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_GS;
  }
};

struct GeomDataNURBS : public AbstractObject
{
  GeomDataNURBS() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_NURBS; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    //uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    //uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    //uint32_t a_start = EXTRACT_START(start_count_packed);
    //uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectNURBS(ray_pos, ray_dir, tNear, info.instId, geometryId, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_GS;
  }
};

struct GeomDataGraphicsPrim : public AbstractObject
{
  GeomDataGraphicsPrim() {m_tag = GetTag();} 

  uint32_t GetTag() const override { return TAG_GRAPHICS_PRIM; }  
  uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, 
                     CRT_Hit* pHit, BVHRT* bvhrt)   const override
  {
    float3 ray_pos = to_float3(rayPosAndNear);
    float3 ray_dir = to_float3(rayDirAndFar);
    float tNear    = rayPosAndNear.w;
    uint32_t geometryId = geomId;
    uint32_t globalAABBId = bvhrt->startEnd[geometryId].x + info.aabbId;
    uint32_t start_count_packed = bvhrt->m_primIdCount[globalAABBId];
    uint32_t a_start = EXTRACT_START(start_count_packed);
    uint32_t a_count = EXTRACT_COUNT(start_count_packed);

    bvhrt->IntersectGraphicPrims(ray_pos, ray_dir, tNear, info.instId, geometryId, a_start, a_count, pHit);
    return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_GRAPHICS_PRIM;
  }
};

struct GeomDataAnyPolygon : public AbstractObject {
    GeomDataAnyPolygon() { m_tag = this->GetTag(); }

    uint32_t GetTag() const override { return TAG_ANY_POLYGON; }

    uint32_t Intersect(float4 rayPosAndNear, float4 rayDirAndFar, CRT_LeafInfo info, CRT_Hit* pHit,
                       BVHRT* bvhrt) const override
    {
        auto const ray_pos = to_float3(rayPosAndNear);
        auto const ray_dir = to_float3(rayDirAndFar);
        auto const t_near = rayPosAndNear.w;
        auto const geometry_id = geomId;
        auto const globalAABBId = bvhrt->startEnd[geometry_id].x + info.aabbId;
        auto const start_count_packed = bvhrt->m_primIdCount[globalAABBId];
        auto const a_start = EXTRACT_START(start_count_packed);
        auto const a_count = EXTRACT_COUNT(start_count_packed);

        bvhrt->IntersectAnyPolygon(ray_pos, ray_dir, t_near, info.instId, geometry_id, a_start, a_count, pHit);

        return pHit->primId == 0xFFFFFFFF ? TAG_NONE : TAG_ANY_POLYGON;
    }
};
