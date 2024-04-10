#pragma once

#include <cstdint>
#include <cstddef>
#include <unordered_set>

#include "LiteMath.h"
#include "sdfScene/sdf_scene.h"

enum BuildQuality
{
  BUILD_LOW    = 0, ///< Prefer Fast Build
  BUILD_MEDIUM = 1, ///< Standart sweep builder
  BUILD_HIGH   = 2, ///< Enable Advanced techniques like Split BVH or Early Split Clipping
  BUILD_REFIT  = 3, ///< Don't change hirarchy, recompute bouding boxes.
};

/**
\brief API to ray-scene intersection on CPU
*/
struct CRT_Hit 
{
  float    t;         ///< intersection distance from ray origin to object
  uint32_t primId; 
  uint32_t instId;
  uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
  float    coords[4]; ///< custom intersection data; for triangles coords[0] and coords[1] stores baricentric coords (u,v)
                      // coords[2] and coords[3] stores normal.xy
};

static constexpr unsigned SH_TYPE = 28; //4 bits for type

static constexpr unsigned TYPE_MESH_TRIANGLE       = 0;
static constexpr unsigned TYPE_SDF_PRIMITIVE       = 1;
static constexpr unsigned TYPE_SDF_GRID            = 2;
static constexpr unsigned TYPE_SDF_OCTREE          = 3;
static constexpr unsigned TYPE_SDF_FRAME_OCTREE    = 4;
static constexpr unsigned TYPE_RF_GRID             = 5;

//enum SdfOctreeSampler
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_3X3 = 0; //go to the deepest level possible, resampling larger nodes
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_CLOSEST = 1; //go deeper while resampling is not needed, then sample
static constexpr unsigned SDF_OCTREE_SAMPLER_CLOSEST = 2;

//enum SdfFrameOctreeBLAS
static constexpr unsigned SDF_FRAME_OCTREE_BLAS_NO = 0; //use trivial BLAS with 2 bboxes and full sphere tracing later on
static constexpr unsigned SDF_FRAME_OCTREE_BLAS_DEFAULT = 1; //use BVH with one leaf for every non-empty leaf node of octree

//enum SdfFrameOctreeIntersect
static constexpr unsigned SDF_FRAME_OCTREE_INTERSECT_DEFAULT = 0; //sphere tracing + octree traversal
static constexpr unsigned SDF_FRAME_OCTREE_INTERSECT_ST = 1;// onnly with SDF_FRAME_OCTREE_BLAS_DEFAULT! Sphere tracing inside node
static constexpr unsigned SDF_FRAME_OCTREE_INTERSECT_ANALYTIC = 2;// onnly with SDF_FRAME_OCTREE_BLAS_DEFAULT! Explicitly finding ray/sdf intersection inside node

//enum VisualizeStatType 
static constexpr unsigned VISUALIZE_STAT_NONE = 0;
static constexpr unsigned VISUALIZE_STAT_SPHERE_TRACE_ITERATIONS = 1;
struct TracerPreset
{
  unsigned need_normal;
  unsigned sdf_octree_sampler; //enum SdfOctreeSampler
  unsigned visualize_stat; //enum VisualizeStatType 
  unsigned sdf_frame_octree_blas; //enum SdfFrameOctreeBLAS
  unsigned sdf_frame_octree_intersect; //enum SdfFrameOctreeIntersect
};
/**
\brief API to ray-scene intersection on CPU
*/
struct ISceneObject
{
  ISceneObject(){}
  virtual ~ISceneObject(){} 
 
  /**
  \brief get implementation name  
  */
  virtual const char* Name() const = 0;

  /**
  \brief get the format name the tree build from
  */
  virtual const char* BuildName() const { return NULL; };

  /**
  \brief clear everything 
  */
  virtual void ClearGeom() = 0; 

  /**
  \brief Add geometry of type 'Triangles' to 'internal geometry library' of scene object and return geometry id
  \param a_vpos3f       - input vertex data;
  \param a_vertNumber   - vertices number. The total size of 'a_vpos4f' array is assumed to be qual to 4*a_vertNumber
  \param a_triIndices   - triangle indices (standart index buffer)
  \param a_indNumber    - number of indices, shiuld be equal to 3*triaglesNum in your mesh
  \param a_qualityLevel - bvh build quality level (low -- fast build, high -- fast traversal) 
  \param vByteStride    - byte offset from each vertex to the next one; if 0 or sizeof(float)*3 then data is tiny packed
  \return id of added geometry
  */
  virtual uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
  /**
  \brief Update geometry for triangle mesh to 'internal geometry library' of scene object and return geometry id
  \param a_geomId - geometry id that should be updated. Please refer to 'AddGeom_Triangles4f' for other parameters
  
  Updates geometry. Please note that you can't: 
   * change geometry type with this fuction (from 'Triangles' to 'Spheres' for examples). 
   * increase geometry size (no 'a_vertNumber', neither 'a_indNumber') with this fuction (but it is allowed to make it smaller than original geometry size which was set by 'AddGeom_Triangles3f')
     So if you added 'Triangles' and got geom_id == 3, than you will have triangle mesh on geom_id == 3 forever and with the size you have set by 'AddGeom_Triangles3f'.
  */
  virtual void UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
#ifndef KERNEL_SLICER 
  virtual uint32_t AddGeom_SdfScene(SdfSceneView scene, BuildQuality a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfGrid(SdfGridView grid, BuildQuality a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_RFScene(RFScene grid, BuildQuality a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfOctree(SdfOctreeView octree, BuildQuality a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, BuildQuality a_qualityLevel = BUILD_HIGH) = 0;
#endif

  /**
  \brief Clear all instances, but don't touch geometry
  */
  virtual void ClearScene() = 0; ///< 

  /**
  \brief Finish instancing and build top level acceleration structure
  */
  virtual void CommitScene(BuildQuality a_qualityLevel = BUILD_MEDIUM) = 0; ///< 
  
  /**
  \brief Add instance to scene
  \param a_geomId     - input if of geometry that is supposed to be instanced
  \param a_matrixData - float4x4 matrix, default layout is column-major

  */
  virtual uint32_t AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix) = 0;
  
  /**
  \brief Add instance to scene
  \param a_instanceId
  \param a_matrixData - float4x4 matrix, the layout is column-major
  */
  virtual void     UpdateInstance(uint32_t a_instanceId, const LiteMath::float4x4& a_matrix) = 0; 
 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  /**
  \brief Find nearest intersection of ray segment (Near,Far) and scene geometry
  \param posAndNear   - ray origin (x,y,z) and t_near (w)
  \param dirAndFar    - ray direction (x,y,z) and t_far (w)
  \return             - closest hit surface info
  */
  virtual CRT_Hit RayQuery_NearestHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) = 0;

  /**
  \brief Find any hit for ray segment (Near,Far). If none is found return false, else return true;
  \param posAndNear   - ray origin (x,y,z) and t_near (w)
  \param dirAndFar    - ray direction (x,y,z) and t_far (w)
  \return             - true if a hit is found, false otherwaise
  */
  virtual bool    RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) = 0;

  virtual uint32_t GetGeomNum() const  { return 0; };
  virtual uint32_t GetInstNum() const  { return 0; };
  virtual const LiteMath::float4* GetGeomBoxes() const { return nullptr; };

  void SetPreset(const TracerPreset& a_preset){ m_preset = a_preset; }

  TracerPreset m_preset;
};
