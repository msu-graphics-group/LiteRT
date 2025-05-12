#pragma once

#include <cstdint>
#include <cstddef>
#include "LiteMath.h"
using LiteMath::float4;

enum BuildOptions 
{
  NONE                   = 0x00000000, //
  BUILD_LOW              = 0x00000001, //
  BUILD_MEDIUM           = 0x00000002, //
  BUILD_HIGH             = 0x00000004, //
  BUILD_REFIT            = 0x00000008, //
  MOTION_BLUR            = 0x00000010, //
  BUILD_NOW              = 32,         // this is for internal usage normally 
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
};

/// @brief custom objects via aabb flag
static constexpr unsigned int CRT_GEOM_MASK_AABB_BIT    = 0x80000000; // 1000 0000 ... 
static constexpr unsigned int CRT_GEOM_MASK_AABB_BIT_RM = 0x7fffffff; // 0111 1111 ... 

struct CRT_AABB
{
  float4 boxMin;
  float4 boxMax;
};

/**
\brief API to ray-scene intersection on CPU
*/
struct ISceneObject
{
  ISceneObject(){}
  virtual ~ISceneObject(){} 

  virtual const char*   Name() const { return ""; }
  virtual ISceneObject* UnderlyingImpl(uint32_t a_implId) { return this; }

  /**
  \brief clear everything 
  */
  virtual void ClearGeom() = 0; 

  /**
  \brief Read given file and add geometry of a custom type to 'internal geometry library' and return geometry id
  */
  virtual uint32_t AddCustomGeom_FromFile(const char *geom_type_name, const char *filename, ISceneObject *fake_this) { return 0; }

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
  virtual uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber,
                                       uint32_t a_flags = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
  /**
  \brief Update geometry for triangle mesh to 'internal geometry library' of scene object and return geometry id
  \param a_geomId - geometry id that should be updated. Please refer to 'AddGeom_Triangles4f' for other parameters
  
  Updates geometry. Please note that you can't: 
   * change geometry type with this fuction (from 'Triangles' to 'Spheres' for examples). 
   * increase geometry size (no 'a_vertNumber', neither 'a_indNumber') with this fuction (but it is allowed to make it smaller than original geometry size which was set by 'AddGeom_Triangles3f')
     So if you added 'Triangles' and got geom_id == 3, than you will have triangle mesh on geom_id == 3 forever and with the size you have set by 'AddGeom_Triangles3f'.
  */
  virtual void UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber,
                                      uint32_t a_flags = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  
  /**
  \brief Add geometry of type 'AxisAlignedBoundingBox' with some custom geometry tag/type (a_typeId), return geometry id
  \param a_typeId          - internal geometry typeId (called 'tag' sometimes)
  \param boxMinMaxF8       - bounding box array
  \param a_boxNumber       - bounding box count (array size); if a_customPrimCount is not 0, a_boxNumber must be a multiple of a_customPrimCount
  \param a_customPrimPtrs  - array of pointers of type, corresponding to a_typeId.
  \param a_customPrimCount - size of array pointer. It is allowd to represent each primitive with several AABB on input. For example, if we want 4 boxes per primitive, for a_customPrimCount equals to 10, a_boxNumber must be 40 
  \return id of added geometry
  */
  virtual uint32_t AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs = nullptr, size_t a_customPrimCount = 0) { return 0; }
  
  virtual void     UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs = nullptr, size_t a_customPrimCount = 0) {}

  /**
  \brief Clear all instances, but don't touch geometry
  */
  virtual void ClearScene() = 0; ///< 

  /**
  \brief Finish instancing and build top level acceleration structure
  */
  virtual void CommitScene(uint32_t options = BUILD_HIGH) = 0; ///< 
  
  /**
  \brief Add instance to scene
  \param a_geomId     - input id of geometry that is supposed to be instanced
  \param a_matrix     - float4x4 matrix, default layout is column-major

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
  \param a_matrix - float4x4 matrix, the layout is column-major
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

};

/**
\brief 'fixed-function' intersection data which is passed to custom intersection function
*/
struct CRT_LeafInfo 
{
  uint32_t aabbId; ///<! id of aabb/box  inside BLAS
  uint32_t primId; ///<! id of primitive inside BLAS
  uint32_t instId; ///<! instance id
  uint32_t geomId; ///<! end-to-end index of custom geometry processerd with AABB
  uint32_t rayxId; ///<! unique ray id (x coord on image)
  uint32_t rayyId; ///<! unique ray id (y coord on image)
};


ISceneObject* CreateEmbreeRT();
//ISceneObject* CreateVulkanRTX(VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_transferQId, uint32_t a_graphicsQId);

ISceneObject* CreateSceneRT(const char* a_impleName); 
void          DeleteSceneRT(ISceneObject* a_pScene);
