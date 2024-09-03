#pragma once

#include <cstdint>
#include <cstddef>
#include <unordered_set>

#include "LiteMath.h"
#include "dependencies/CrossRT/CrossRT.h"
#include "sdfScene/sdf_scene.h"
#include "utils/radiance_field.h"
#include "utils/gaussian_field.h"
#include "render_settings.h"

static constexpr unsigned SH_TYPE = 28; //4 bits for type

/**
\brief API to ray-scene intersection on CPU
*/
struct ISceneObject2 : public ISceneObject
{
  ISceneObject2(){}
  virtual ~ISceneObject2(){} 
 
  /**
  \brief get the format name the tree build from
  */
  virtual const char* BuildName() const { return NULL; }

  virtual void SetProxy(std::shared_ptr<ISceneObject> a_proxyImpl) { m_proxyAS = a_proxyImpl; }

#ifndef KERNEL_SLICER 
  virtual uint32_t AddGeom_Triangles3f(const float* a_vpos3f, const float* a_vnorm3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildOptions a_qualityLevel = BUILD_HIGH, size_t vByteStride = sizeof(float)*3) = 0;
  virtual uint32_t AddGeom_SdfGrid(SdfGridView grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_RFScene(RFScene grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_GSScene(GSScene grid, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfOctree(SdfOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfSVS(SdfSVSView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfSBS(SdfSBSView octree, bool single_bvh_node = false, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;
  virtual uint32_t AddGeom_SdfFrameOctreeTex(SdfFrameOctreeTexView octree, BuildOptions a_qualityLevel = BUILD_HIGH) = 0;

  virtual void set_debug_mode(bool enable) { };
#endif

  virtual uint32_t GetGeomNum() const  { return 0; }
  virtual uint32_t GetInstNum() const  { return 0; }
  virtual const LiteMath::float4* GetGeomBoxes() const { return nullptr; }

  void SetPreset(const MultiRenderPreset& a_preset){ m_preset = a_preset; }
  MultiRenderPreset GetPreset() const { return m_preset; }
  MultiRenderPreset m_preset;

  std::shared_ptr<ISceneObject> m_proxyAS;
};

std::shared_ptr<ISceneObject2> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName);
