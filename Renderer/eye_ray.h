#pragma once

#include <cstdint>
#include <memory>
#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>

#include "LiteMath.h"
#include "../ISceneObject.h"
#include "../IRenderer.h"
#include "LiteScene/cmesh4.h"
#include "Image2d.h"
#include "BVH/BVH2Common.h"
#include "harmonic_function/any_polygon_common.h"

using LiteMath::uint;
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;

struct SparseOctreeSettings;

//enum MultiRendererMaterialType
static constexpr unsigned MULTI_RENDER_MATERIAL_TYPE_COLORED  = 0;
static constexpr unsigned MULTI_RENDER_MATERIAL_TYPE_TEXTURED = 1;

static constexpr unsigned DEFAULT_MATERIAL = 0u;
static constexpr unsigned DEFAULT_TEXTURE = 0u;

static constexpr unsigned MULTI_RENDER_MAX_TEXTURES = 16;

struct MultiRendererMaterial
{
  unsigned type;
  unsigned texId; // valid if type == MULTI_RENDER_MATERIAL_TYPE_TEXTURED
  unsigned _pad[2];
  float4 base_color; // valid if type == MULTI_RENDER_MATERIAL_TYPE_COLORED
};

//enum LightType
static constexpr unsigned LIGHT_TYPE_DIRECT   = 0;
static constexpr unsigned LIGHT_TYPE_POINT    = 1;
static constexpr unsigned LIGHT_TYPE_AMBIENT  = 2;

struct Light
{
  float3 space; //position or direction
  unsigned type;
  float3 color; //intensity included
  unsigned _pad;
};

static Light create_direct_light(float3 dir, float3 color)
{
  return {normalize(dir), LIGHT_TYPE_DIRECT, color, 0};
}

static Light create_point_light(float3 pos, float3 color)
{
  return {pos, LIGHT_TYPE_POINT, color, 0};
}

static Light create_ambient_light(float3 color)
{
  return {float3(0,0,0), LIGHT_TYPE_AMBIENT, color, 0};
}


class MultiRenderer : public IRenderer
{
public:

  //a bunch of functions extending IRenderer to make working with MultiRenderer easier
#ifndef KERNEL_SLICER 
  void SetScene(const cmesh4::SimpleMesh &scene);
  void SetScene(SdfGridView scene);
  void SetScene(SdfFrameOctreeView scene);
  void SetScene(SdfSVSView scene);
  void SetScene(SdfSBSView scene);
  void SetScene(SdfSBSAdaptView scene);

  void SetScene(SdfFrameOctreeTexView scene);
  void SetScene(const RawNURBS &nurbs);
  void SetScene(GraphicsPrimView scene);
  void SetScene(AnyPolygon const& poly);
#endif // !defined(KERNEL_SLICER)

  void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, 
              const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj,
              MultiRenderPreset preset = getDefaultPreset(), int a_passNum = 1);
  void RenderFloat(LiteMath::float4* imageData, uint32_t a_width, uint32_t a_height, 
                   const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj,
                   MultiRenderPreset preset = getDefaultPreset(), int a_passNum = 1);

  void SetPreset(const MultiRenderPreset& a_preset);

  //functions implementing IRenderer interface
  MultiRenderer(uint32_t maxPrimitives); 
  const char* Name() const override;
  
  //required by slicer!
  virtual void SceneRestrictions(uint32_t a_restrictions[4]) const
  {
    uint32_t maxMeshes            = 1024;
    uint32_t maxTotalVertices     = m_maxPrimitives;
    uint32_t maxTotalPrimitives   = m_maxPrimitives;
    uint32_t maxPrimitivesPerMesh = m_maxPrimitives;

    a_restrictions[0] = maxMeshes;
    a_restrictions[1] = maxTotalVertices;
    a_restrictions[2] = maxTotalPrimitives;
    a_restrictions[3] = maxPrimitivesPerMesh;
  }

  bool LoadScene(const char* a_scenePath) override;

  void Clear (uint32_t a_width, uint32_t a_height, const char* a_what) override;
  void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum = 1) override;
  void RenderFloat(LiteMath::float4* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum = 1);
  void SetViewport(int a_xStart, int a_yStart, int a_width, int a_height) override;

  void SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct) override 
  { 
    m_pAccelStruct  = a_customAccelStruct;
  }
  std::shared_ptr<ISceneObject> GetAccelStruct() override { return m_pAccelStruct; }

  void GetExecutionTime(const char* a_funcName, float a_out[4]) override;

  void CommitDeviceData() override {}

  void UpdateCamera(const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj) override;
  
  //default one, loading already existing stuff
  bool LoadSceneHydra(const std::string& a_path);

  //load hydra scene with some meshes, convert them to SDF with given settings and load to MultiRenderer
  bool CreateSceneFromHydra(const std::string& a_path, unsigned type, SparseOctreeSettings so_settings);  

  uint32_t AddTexture(const Image2D<float4> &image);
  uint32_t AddMaterial(const MultiRendererMaterial &material);
  void     SetMaterial(uint32_t matId, uint32_t geomId);

  void SetLights(const std::vector<Light>& lights);
  
  uint32_t AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix);
  
#ifndef KERNEL_SLICER 
  void add_mesh_internal(const cmesh4::SimpleMesh& mesh, unsigned geomId);
  void add_SdfFrameOctreeTex_internal(SdfFrameOctreeTexView scene, unsigned geomId);
#endif

  LiteMath::float4x4 getProj() { return m_proj; }
  LiteMath::float4x4 getWorldView() { return m_worldView; }

  const std::vector<MultiRendererMaterial>& getMaterials() { return m_materials; }
  const std::vector<std::shared_ptr<ICombinedImageSampler>> &getTextures() { return m_textures; }

  void setSeed(uint32_t seed) { m_seed = seed; }
  uint32_t getSeed() const { return m_seed; }

protected:

  virtual void PackXYBlock(uint tidX, uint tidY, uint a_passNum);
  virtual void PackXY(uint tidX, uint tidY);
  virtual void kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY);
  
  virtual void CastRaySingle(uint32_t tidX, uint32_t* out_color __attribute__((size("tidX"))));
  virtual void CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses = 1);

  virtual void CastRayFloatSingle(uint32_t tidX, LiteMath::float4* out_color __attribute__((size("tidX"))));
  virtual void CastRayFloatSingleBlock(uint32_t tidX, LiteMath::float4* out_color, uint32_t a_numPasses = 1);

  void kernel_InitEyeRay(uint32_t tidX, float2 d, LiteMath::float4* rayPosAndNear, LiteMath::float4* rayDirAndFar);
  LiteMath::float4 kernel_RayTrace(uint32_t tidX, const LiteMath::float4* rayPosAndNear, const LiteMath::float4* rayDirAndFar);

  uint32_t encode_RGBA8(LiteMath::float4 c);
  LiteMath::float4 decode_RGBA8(uint32_t c);
  LiteMath::float3 decode_normal(float2 v);
  LiteMath::float3 rand3(uint32_t x, uint32_t y, uint32_t iter);
  LiteMath::float2 rand2(uint32_t x, uint32_t y, uint32_t iter);

  uint32_t m_width;
  uint32_t m_height;
  MultiRenderPreset m_preset;
  uint32_t m_seed;

  LiteMath::float4x4 m_proj;
  LiteMath::float4x4 m_worldView;
  LiteMath::float4x4 m_projInv;
  LiteMath::float4x4 m_worldViewInv;

  std::shared_ptr<ISceneObject>  m_pAccelStruct;
  std::vector<uint32_t>          m_packedXY;

  //duplicating data for meshes if we want to visualize them with textures
#ifndef DISABLE_MESH_TEX
  std::vector<float4> m_vertices; //.w is tc.x
  std::vector<float4> m_normals;  //.w is tc.y
  std::vector<uint32_t> m_indices;
  std::vector<uint2> m_geomOffsets;
#endif

  //materials and textures if at least one textured type is enabled
//#if !defined(DISABLE_MESH_TEX) || !defined(DISABLE_SDF_TEX)
  std::vector<MultiRendererMaterial> m_materials;
  std::vector< std::shared_ptr<ICombinedImageSampler> > m_textures;
  std::vector<uint32_t> m_matIdbyPrimId;
  std::vector<uint2> m_matIdOffsets; //for every geometry, start and size of it's part of m_matIdbyPrimId
  unsigned active_textures_count = 0;

  std::vector<Light> m_lights;
//#endif
  std::vector<LiteMath::float4x4> m_instanceTransformInvTransposed;

  // color palette to select color for objects based on mesh/instance id
  static constexpr uint32_t palette_size = 20;
  static constexpr uint32_t m_palette[palette_size] = {
    0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
    0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
    0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
    0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
    0xff808000, 0xffffd8b1, 0xff000080, 0xff808080
  };

  //statistics
  std::unordered_map<std::string, float> timeDataByName;
  uint64_t m_totalTris         = 0;
  uint64_t m_totalTrisVisiable = 0;

  uint32_t m_maxPrimitives; //required in constructor to allocate enough memory in Vulkan
};

std::shared_ptr<MultiRenderer> CreateMultiRenderer(const char* a_name, uint32_t maxPrimitives = 10'000'000);
