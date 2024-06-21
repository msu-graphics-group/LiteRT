#pragma once

#include <cstdint>
#include <cstddef>
#include <unordered_set>

#include "LiteMath.h"
#include "sdfScene/sdf_scene.h"
#include "utils/radiance_field.h"
#include "utils/gaussian_field.h"
#include "render_settings.h"

enum BuildOptions 
{
  NONE                   = 0x00000000, 
  BUILD_LOW              = 0x00000001,
  BUILD_MEDIUM           = 0x00000002,
  BUILD_HIGH             = 0x00000004,
  BUILD_REFIT            = 0x00000008,  
  MOTION_BLUR            = 0x00000010,
  BUILD_OPTIONS_MAX_ENUM = 0x7FFFFFFF
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
  virtual uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
  /**
  \brief Update geometry for triangle mesh to 'internal geometry library' of scene object and return geometry id
  \param a_geomId - geometry id that should be updated. Please refer to 'AddGeom_Triangles4f' for other parameters
  
  Updates geometry. Please note that you can't: 
   * change geometry type with this fuction (from 'Triangles' to 'Spheres' for examples). 
   * increase geometry size (no 'a_vertNumber', neither 'a_indNumber') with this fuction (but it is allowed to make it smaller than original geometry size which was set by 'AddGeom_Triangles3f')
     So if you added 'Triangles' and got geom_id == 3, than you will have triangle mesh on geom_id == 3 forever and with the size you have set by 'AddGeom_Triangles3f'.
  */
  virtual void UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
#ifndef KERNEL_SLICER 
  virtual uint32_t AddGeom_Triangles3f(const float* a_vpos3f, const float* a_vnorm3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  virtual uint32_t AddGeom_SdfScene(SdfSceneView scene, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfGrid(SdfGridView grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_RFScene(RFScene grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_GSScene(GSScene grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfOctree(SdfOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfSVS(SdfSVSView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfSBS(SdfSBSView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfHpOctree(SdfHPOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;

  virtual uint32_t AddGeom_SdfFrameOctreeTex(SdfFrameOctreeTexView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;

  virtual void set_debug_mode(bool enable) { };
#endif

  /**
  \brief Clear all instances, but don't touch geometry
  */
  virtual void ClearScene() = 0; ///< 

  /**
  \brief Finish instancing and build top level acceleration structure
  */
  virtual void CommitScene(uint32_t a_qualityLevel = BUILD_MEDIUM) = 0; ///< 
  
  /**
  \brief Add instance to scene
  \param a_geomId     - input if of geometry that is supposed to be instanced
  \param a_matrixData - float4x4 matrix, default layout is column-major

  */
  virtual uint32_t AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix) = 0;
  
  /**
  \brief Add moving instance to scene
  \param a_geomId       - input id of geometry that is supposed to be instanced
  \param a_matrices     - array of float4x4 matrices, default layout is column-major
  \param a_matrixNumber - size of matrices array

  */
  virtual uint32_t AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber) = 0; 
  
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
  \brief Find nearest intersection of ray segment (Near,Far) and scene geometry
  \param posAndNear   - ray origin (x,y,z) and t_near (w)
  \param dirAndFar    - ray direction (x,y,z) and t_far (w)
  \param time         - time in [0, 1] interval between first and last timesteps
  \return             - closest hit surface info
  */
  virtual CRT_Hit RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time) = 0;

  /**
  \brief Find any hit for ray segment (Near,Far). If none is found return false, else return true;
  \param posAndNear   - ray origin (x,y,z) and t_near (w)
  \param dirAndFar    - ray direction (x,y,z) and t_far (w)
  \return             - true if a hit is found, false otherwaise
  */
  virtual bool    RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) = 0;

  /**
  \brief Find any hit for ray segment (Near,Far). If none is found return false, else return true;
  \param posAndNear   - ray origin (x,y,z) and t_near (w)
  \param dirAndFar    - ray direction (x,y,z) and t_far (w)
  \param time         - time in [0, 1] interval between first and last timesteps
  \return             - true if a hit is found, false otherwaise
  */
  virtual bool    RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time = 0.0f) = 0;

  virtual uint32_t GetGeomNum() const  { return 0; };
  virtual uint32_t GetInstNum() const  { return 0; };
  virtual const LiteMath::float4* GetGeomBoxes() const { return nullptr; };

  void SetPreset(const MultiRenderPreset& a_preset){ m_preset = a_preset; }
  MultiRenderPreset GetPreset() const { return m_preset; }

  MultiRenderPreset m_preset;
};
ISceneObject* CreateSceneRT(const char* a_impleName);
void DeleteSceneRT(ISceneObject* a_pScene);